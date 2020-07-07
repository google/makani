% Copyright 2020 Makani Technologies LLC
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%      http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

function [time, flight_constants, aerodata, rtt_commanded, debug] = GetAeroCoeffs(h5file_loc, rotor_json_db, varargin)
% GetAeroCoeffs computes 6DOF forces and moments of the M600 kite in stability and wind axis
% [time, flight_constants, aerodata, rtt_commanded, debug] = GetAeroCoeffs(h5file_loc, rotor_json_db, varargin)
%
% Description:
%   Uses the h5 file ControllerA and motor debug nodes to calculate aerodynamic coefficients for an M600 test by
%   using accelerometers to get the total forces and moments acting on the kite, then decomposes the sum of the
%   contributing forces and moments on the kite from rotor table generated thrust and gyroscopic moments,
%   tether tension and bridle location, and weight of the kite then treating the residual between the total forces
%   and moments and calculated contributing forces and moments as the total kite aerodynamic forces and moments.
%   Any instrumentation error or modelling error is implicitly lumped into this residual value.
%
% Arguments:
%   h5file_loc:     string;        the h5 flight database file string location
%   rotor_json_db:  json database; the rotor tables from a particular rotor design
%   varargin:       integer;       allows filtering of data based on flight mode number
%
% Return Values:
%   time:             vector; contains the time in seconds for the data
%   flight_constants: struct; containing the meta data for the flight
%                             WARNING: the cg and bridle position are averaged over all flight modes
%                                      unless varargin is used to filter the flight data by flight mode
%   aerodata:         struct; aerodynamic data struct
%   rtt_commanded:    struct; rotors, thrust, torque *commanded* from the controller
%   debug:            struct; debug information
%
% Required Toolboxes:
%   Signal Processing Toolbox
%
% Future Work:
%   - The section for rotor tables and lookup can be broken into their own folder and functions
%   - The input arguments and outputs should be simplified
%
%% Begin
%
% track wall time
tic

% Define the motor labels for the configuration to be analyzed
motor_lbl = {'MotorSbo'; 'MotorSbi'; 'MotorPbi'; 'MotorPbo'; ...
             'MotorPto'; 'MotorPti'; 'MotorSti'; 'MotorSto'};

%=======================================================================================================================
%% Data Input from h5file
%=======================================================================================================================
% Break out h5 file into convenient sub-structs
disp(h5file_loc);
disp(pwd);
h5data        = h5read(h5file_loc, '/messages/kAioNodeControllerA/kMessageTypeControlDebug');
%
% Important variables:
%   system_params: system parameters, e.g. nondimensional constants
%   controller:    what the controller is telling the kite to perform
%   state_est:     the estimated state of the kite read from the controller
system_params = h5read(h5file_loc, '/parameters/system_params');
controller    = h5data.message;       % controller telemetry
state_est     = controller.state_est; % control state estimator

% if filtering based on flight mode
if varargin{1} > 3
  flight_mode_flag = varargin{1};
  index_list = find(controller.flight_mode' == flight_mode_flag);
else
    index_list = [1:length(controller.time)];
end

%=====================================
% FLIGHT TIME VECTOR INFO
%  NOTE:  based on *controller* time and mode 2
%=====================================
disp('GetAeroCoeffs: Time');
mode_flag = 2;
synced_time = GetFlightControllerSyncTime(controller, mode_flag);
time        = synced_time(index_list);
dt          = system_params.ts;
sample_freq = 1/dt;

%=====================================
% Flight Conditions
%=====================================
disp('GetAeroCoeffs: Flight conditions');
rho          = system_params.phys.rho; % [kg/m^3] - density of air measured on day of flight
gravity      = system_params.phys.g;   % [m/s^2]  - gravitational constant
s_ref        = system_params.wing.A;   % [m^2]    - reference area
c_ref        = system_params.wing.c;   % [m]      - reference chord
b_ref        = system_params.wing.b;   % [m]      - reference wing span
kite_mass    = system_params.wing.m;   % [kg]     - mass of kite
kite_inertia = system_params.wing.I.d; % [kg-m^2] - moments of inertia of kite
%
% tso note : this is a questionable operation based on whether the info is filtered or not
%
cg_posn      = [system_params.wing.center_of_mass_pos.x; ...
                system_params.wing.center_of_mass_pos.y; ...
                system_params.wing.center_of_mass_pos.z]; %[m, m, m]'
bridle_posn  = [mean(system_params.wing.bridle_pos.x); ...
                mean(system_params.wing.bridle_pos.y); ...
                mean(system_params.wing.bridle_pos.z)]; %[m, m, m]'
%
flight_constants = struct('rho', rho, 'g', gravity,...
                          'Sref', s_ref, 'cbar', c_ref, 'b', b_ref,...
                          'mass', kite_mass, 'wing_inertia', kite_inertia',...
                          'cg', cg_posn, 'bridle_posn', bridle_posn);
%
altitude = -state_est.Xg.z(index_list);

%================================
%  VELOCITIES
%================================
disp('GetAeroCoeffs: Velocities');
% apparent wind vector in body basis
wind_u = state_est.apparent_wind.vector.x(index_list);
wind_v = state_est.apparent_wind.vector.y(index_list);
wind_w = state_est.apparent_wind.vector.z(index_list);
% compute magnitude of wind velocity
wind_vmag = sqrt(wind_u.^2+wind_v.^2+wind_w.^2);

% kite velocity components in body basis
%   filtered by the state estimator
body_vx = state_est.Vb_f.x(index_list)';
body_vy = state_est.Vb_f.y(index_list)';
body_vz = state_est.Vb_f.z(index_list)';

%================================
%  ACCELERATIONS
%================================
disp('GetAeroCoeffs: Accelerations');
% Note: The accelerations coming from the estimator are in the inertial frame, expressed in the body basis
% Store as 3 x n_timestamps matrix, giving a column vector for position for the nth time
accFilt = designfilt('lowpassfir', 'PassbandFrequency', 0.001, 'StopbandFrequency', 4, 'SampleRate', sample_freq);
delay = mean(grpdelay(accFilt));

acc_bod_x = filter(accFilt, [state_est.Ab_f.x(index_list);zeros(delay,1)]);
acc_bod_y = filter(accFilt, [state_est.Ab_f.y(index_list);zeros(delay,1)]);
acc_bod_z = filter(accFilt, [state_est.Ab_f.z(index_list);zeros(delay,1)]);

acc_bod_x = acc_bod_x(delay+1:end);
acc_bod_y = acc_bod_y(delay+1:end);
acc_bod_z = acc_bod_z(delay+1:end);

acc_body = [acc_bod_x'; acc_bod_y'; acc_bod_z'];
% the signal directly from the state estimator is too noisy for future derivatives
%{
acc_body = [state_est.Ab_f.x(index_list)';...
            state_est.Ab_f.y(index_list)';...
            state_est.Ab_f.z(index_list)'];
%}
%================================
% BODY PITCH, ROLL, YAW RATES
%================================
disp('GetAeroCoeffs: Body pitch, roll, yaw rates');
% Additional filter for p, q, and r signals
pqrFilt = designfilt('lowpassfir', 'PassbandFrequency', 0.001, 'StopbandFrequency', .1, 'SampleRate', sample_freq);
delay = mean(grpdelay(pqrFilt));

pfilt = filter(pqrFilt,[state_est.pqr.x(index_list);zeros(delay,1)]);
qfilt = filter(pqrFilt,[state_est.pqr.y(index_list);zeros(delay,1)]);
rfilt = filter(pqrFilt,[state_est.pqr.z(index_list);zeros(delay,1)]);

p_body = pfilt(delay+1:end)';
q_body = qfilt(delay+1:end)';
r_body = rfilt(delay+1:end)';
% the signal directly from the state estimator is too noisy for future derivatives
%{
  p_body = state_est.pqr.x(index_list)';
  q_body = state_est.pqr.y(index_list)';
  r_body = state_est.pqr.z(index_list)';
%}
omega_body = [p_body; q_body; r_body];
omega_body_deg_per_second = rad2deg(omega_body);

%=======================================================================================================================
%% Input and process rotor data
%=======================================================================================================================
disp('GetAeroCoeffs: Rotor data');
% Create truncated struct to pass onto GetRotorsThrustTorque
rtt_commanded=struct();                                 % Named for _r_otors, _t_hrust, and _t_orque, commanded
rtt_commanded.apparent_wind  = [wind_u wind_v wind_w]'; % [3 x m] apparent wind vector for the rotors

% controller timestamp for syncing motor data to the log computer
rtt_commanded.ctime = double(h5data.capture_header.tv_sec(index_list)) + ...
                      double(h5data.capture_header.tv_usec(index_list))*1e-6;
%
% rtt_commanded
%   .rotors_axis: struct of the x,y,z vectors where each index is the rotor basis z-axis in the body basis
%   .rotors_posn: struct of the x,y,z vectors where each index is the rotor x,y,z position [m] in the body basis
%   .rotors_I: the moment of inertia of the rotor about its Z-axis, i.e. Izz [kg-m2]
%
rtt_commanded.rotors_axis  = [system_params.rotors.axis.x, system_params.rotors.axis.y, system_params.rotors.axis.z];
rtt_commanded.rotors_posn  = [system_params.rotors.pos.x, system_params.rotors.pos.y, system_params.rotors.pos.z];
rtt_commanded.rotors_I     = system_params.rotors.I;
%
% Create individual motor struct data
%  rTT_commanded
%    .motor_lbl
%      .mtime - motor controller log computer time
%      .omega - rotor speed about its axis
%      .iq    - rotor current
%      .id    - rotor current, pt. 2
%      .Vfreestream - velocity of freestream along rotor axis of rotation
%
for i=1:length(motor_lbl)

  % Struct is labelled by rotor name
  rtt_commanded.(motor_lbl{i}) = [];
  % h5 file motor debug info
  motorname = strcat('/messages/kAioNode', motor_lbl{i}, '/kMessageTypeMotorDebug');
  motordebug = h5read(h5file_loc,motorname);
  % each motor has its own asynchronous processor, so we use logging computer time
  motor_time = double(motordebug.capture_header.tv_sec) + double(motordebug.capture_header.tv_usec)*1e-6;
  % want to work in ControllerA time world, overlay 2 timelines and find the ones that match up
  %   motors are 2kHz, controller is 100HZ
  %   tso note - not sure if this is necessary as they are both pulling from the logging computer time
  i_begin = find(motor_time > rtt_commanded.ctime(1), 1, 'first');
  i_end   = find(motor_time < rtt_commanded.ctime(length(rtt_commanded.ctime)), 1, 'last') + 1;
  % store motor info
  rtt_commanded.(motor_lbl{i}).mtime = motor_time(i_begin:i_end);
  rtt_commanded.(motor_lbl{i}).omega = motordebug.message.omega(i_begin:i_end);
  rtt_commanded.(motor_lbl{i}).iq    = motordebug.message.iq(i_begin:i_end);
  rtt_commanded.(motor_lbl{i}).id    = motordebug.message.id(i_begin:i_end);
  % dot to get the components of the wind in the rotor axial direction
  rtt_commanded.(motor_lbl{i}).Vfreestream = -rtt_commanded.apparent_wind'*[rtt_commanded.rotors_axis(i,:)]';
  %
  % define a rotation matrix from airframe body to rotor axis, assuming
  %   no roll relative to airframe (ie rotor axis is arrived at only by a yaw
  %   and pitch relative to airframe)
  %
  % tso note - why can't body just pitch, or is this a gimble thing?
  xy_hypoteneuse = sqrt(rtt_commanded.rotors_axis(i,1)^2 + rtt_commanded.rotors_axis(i,2)^2);
  sinpsi         =  rtt_commanded.rotors_axis(i,2) / xy_hypoteneuse; % 0 for the m600
  cospsi         =  rtt_commanded.rotors_axis(i,1) / xy_hypoteneuse; % 180 degree in yaw
  sintheta       = -rtt_commanded.rotors_axis(i,3);                  % 3 degrees in z
  costheta       = xy_hypoteneuse;
  %
  % store rotation matrix
  rtt_commanded.(motor_lbl{i}).bod2rotor = [costheta*cospsi, costheta*sinpsi,   -sintheta; ...
                                                    -sinpsi,          cospsi,           0; ...
                                            sintheta*cospsi, sintheta*sinpsi,    costheta];
end

%=======================================================================================================================
%% Set up for and calculate body axis system forces and moments based on state estimator values of speed and rotation
%=======================================================================================================================
% angular accelerations by averaging out the forward- and backward- facing
%  d/dt's (generalized center difference)
p_body_dot = DiffLpf(p_body', sample_freq, .01)';
q_body_dot = DiffLpf(q_body', sample_freq, .01)';
r_body_dot = DiffLpf(r_body', sample_freq, .01)';

%{
% generalized differencing on raw signal
omega_body_dot_fwd = zeros(3,length(time));
omega_body_dot_bkd = zeros(3,length(time));
% forward differencing on omega_body
for i=1:length(time)-1
  omega_body_dot_fwd(:,i)=[(omega_body(1,i+1) - omega_body(1,i))/(time(i+1) - time(i)); ...
                           (omega_body(2,i+1) - omega_body(2,i))/(time(i+1) - time(i)); ...
                           (omega_body(3,i+1) - omega_body(3,i))/(time(i+1) - time(i))];
end
% backward differencing on omega_body
for i=2:length(time)
  omega_dot_fwd(:,i)=[(omegabod(1,i) - omegabod(1,i-1))/(time(i) - time(i-1)); ...
                      (omegabod(2,i) - omegabod(2,i-1))/(time(i) - time(i-1));...
                      (omegabod(3,i) - omegabod(3,i-1))/(time(i) - time(i-1))];
end
% generalized central difference method, except edges of signal
omega_body_dot        = (omega_dot_fwd + omega_dot_bkd)/2;
omega_body_dot(:,1)   = omega_dot_fwd(:,1);
omega_body_dot(:,end) = omega_dot_bkd(:,end);

% compile to our variables
p_body_dot = omega_body_dot(1,:);
q_body_dot = omega_body_dot(2,:);
r_body_dot = omega_body_dot(3,:);
%}

%
% store Force as 3 x n_timesteps matrix and angular acc in same manner
%   the angular accelerations need to convert to inertial reference frame by taking airframe rotation into account

% Forces comprised by accelerations in the inertial frame but in body basis
forces_body = kite_mass * acc_body;

% Moment vector
%{
%terms for accelerations when acc_body taken to be non-inertial reference frame accelerations in body
%axis system, to which the terms for rotating axis system must be added
                      [q .* body_vz - r .* body_vy;...
                       r .* body_vx - p .* body_vz;...
                       p .* body_vy - q .* body_vx]);
%}
%
%Schmidt, Introduction to Flight Dynamics, Eq'n 4.30
moment_fixed_frame = [kite_inertia(1,1) * p_body_dot - kite_inertia(1,3) * r_body_dot;...
                      kite_inertia(2,2) * q_body_dot; ...
                      kite_inertia(3,3) * r_body_dot - kite_inertia(1,3) * p_body_dot];
%
moment_L_body = q_body .* r_body * (kite_inertia(3,3) - kite_inertia(2,2)) -  p_body .* q_body * kite_inertia(1,3);
moment_M_body = p_body .* r_body * (kite_inertia(1,1) - kite_inertia(3,3)) +...
               (p_body .^ 2 - r_body .^ 2) * kite_inertia(1,3);
moment_N_body = p_body .* q_body * (kite_inertia(2,2) - kite_inertia(1,1)) +  q_body .* r_body * kite_inertia(1,3);
moment_rotating_frame = [moment_L_body; moment_M_body; moment_N_body];
%
% completed moments acting on body in inertial frame 
moments_body = moment_fixed_frame + moment_rotating_frame;
%
%
%=======================================================================================================================
%% remove weight, tether, and propulsion, and gyro effects to arrive at aero forces
%=======================================================================================================================
%=====================================
%  Weight
%=====================================
% ground-to-body rotation matrix
ground_to_body_R = state_est.dcm_g2b.d(:,:,index_list);
%
weight_body   = ones(3, length(time));
weight_ground = kite_mass*gravity*[0;0;1];
for i = 1:length(time)
  %ground-to-body basis rotation matrix is from C code; for Matlab, must be transposed here
  weight_body(:,i) = ground_to_body_R(:,:,i)' * weight_ground;
end
%
%=====================================
%  Tether
%=====================================
% Take advantage of flight_mode being equal to 12 for off tether to create boolean for on or off tether
%   this is to force Ftether to 0 when off-tether as residuals can be large
%on_tether = controller.flight_mode(index_list)==12;
on_tether = -1*(fix(double(controller.flight_mode(index_list))/12)-1)';
%
% State estimator for force
tether_force = state_est.tether_force_b.vector_f;
%
% compute tether force on the body
force_tether_body = [tether_force.x(index_list)' .* on_tether; ...
                     tether_force.y(index_list)' .* on_tether; ...
                     tether_force.z(index_list)' .* on_tether];
%
% calculate the tether moment on the body
moment_tether_body = bsxfun(@cross, bridle_posn - cg_posn, force_tether_body);

%=====================================
%  Propulsion
%=====================================
% Rotor thrust and torque in rotor axis system
[mtime_sync, iq, id, omega_rotor, omegadot_rotor, Vfreestream_rotor, rotor_thrust, aero_torque, motor_torque] = ...
  GetRotorThrustTorque(rtt_commanded, rotor_json_db);

%tso - check matrix multiplication vs. no matrix multiplication
%  Change to body axis and sum in x, y, and z directions
table_rho = 1.225; % density the rotor tables are created at

%
%  thrust tables output sea level thrust and must be corrected for rho
thrust_body = rtt_commanded.rotors_axis' * rotor_thrust * rho/table_rho;
moment_proptorque_body = rtt_commanded.rotors_axis' * aero_torque;

%Calculate moments due to rotor thrust
moment_thrust_body = zeros(3,length(time));
for i=1:length(motor_lbl)
  %moment = posn_vector x thrust_vector(collected in matrix with time on colum dimension)
  %thrust_vector = axis_vector * thrust_rotor (scalar collected in time vector)
  %thus, moment = posn_vector x (axis_vector * rotor_thrust)
  rtt_commanded.(motor_lbl{i}).thrust_body = bsxfun(@cross,rtt_commanded.rotors_posn(i,:)' - cg_posn, thrust_body);
  moment_thrust_body = moment_thrust_body + rtt_commanded.(motor_lbl{i}).thrust_body;

  %record rotor values for posterity
  rtt_commanded.(motor_lbl{i}).thrust_rotax  = rotor_thrust(i,:) * rho/table_rho;
  rtt_commanded.(motor_lbl{i}).thrust_bod    = rtt_commanded.rotors_axis(i,:)' * rotor_thrust(i,:) * rho/table_rho;
  rtt_commanded.(motor_lbl{i}).aero_torque   = aero_torque(i,:);
  rtt_commanded.(motor_lbl{i}).motor_torque  = motor_torque(i,:);
  rtt_commanded.(motor_lbl{i}).iq            = iq(i,:);
  rtt_commanded.(motor_lbl{i}).id            = id(i,:);
  rtt_commanded.(motor_lbl{i}).omega_rotor   = omega_rotor(i,:);
  rtt_commanded.(motor_lbl{i}).Vfreestream   = Vfreestream_rotor;
  rtt_commanded.(motor_lbl{i}).mtime_sync    = mtime_sync;
end

%=====================================
%  Gyroscpopic moments
%=====================================
%NOTE: motor omega is opposite from rotor sign convention!!
%Initialize matrices
moment_rotorgyro_body = zeros(3,length(time));

for i=1:length(motor_lbl)
  omega_rotor_axis = rtt_commanded.(motor_lbl{i}).bod2rotor * omega_body;  %angular velocity of rotor axis sytem

  %For rotor axis system with x perpendiuclar to disk and positive in
  %positive thrust, and rotation allowed only about x axis, Euler Eq's for
  %gyroscopic motion reduce to:
  rtt_commanded.(motor_lbl{i}).gyro_mom_x =  rtt_commanded.rotors_I(i) * omegadot_rotor(i,:);
  rtt_commanded.(motor_lbl{i}).gyro_mom_y =  rtt_commanded.rotors_I(i) * omega_rotor(i,:) .* omega_rotor_axis(3,:);
  rtt_commanded.(motor_lbl{i}).gyro_mom_z = -rtt_commanded.rotors_I(i) * omega_rotor(i,:) .* omega_rotor_axis(2,:);
  %
  moment_rotorgyro_body = moment_rotorgyro_body + rtt_commanded.(motor_lbl{i}).bod2rotor' * ...
                          [rtt_commanded.(motor_lbl{i}).gyro_mom_x;...
                           rtt_commanded.(motor_lbl{i}).gyro_mom_y;...
                           rtt_commanded.(motor_lbl{i}).gyro_mom_z];
end

%=======================================================================================================================
%% COMPUTE AERODYNAMIC FORCES AND MOMENTS
% Subtract out all the above contributions to forces and moments to extract aero
%=======================================================================================================================
aeroforces_body  = forces_body - weight_body - force_tether_body - thrust_body;
aeromoments_body = moments_body - moment_tether_body - moment_proptorque_body - moment_rotorgyro_body;

debug=struct('abod',acc_body,'vel_body',[body_vx; body_vy; body_vz],'u',wind_u,'v',wind_v,'w',wind_w,...
             'Fb',forces_body,'Wb',weight_body,'Ftether',force_tether_body,'Thrust',thrust_body,...
             'Mb',moments_body,'Mfixed_frame',moment_fixed_frame,'Mrotating_frame',moment_rotating_frame,...
             'Mproptorque',moment_proptorque_body,'Mrotorgyro',moment_rotorgyro_body,'Mtether',moment_tether_body,...
             'p', p_body, 'q', q_body, 'r', r_body, 'pdot', p_body_dot, 'qdot', q_body_dot, 'rdot', r_body_dot);

%=======================================================================================================================
%%  Rotations to stability and wind axes
%=======================================================================================================================
alpha_rad = atan2(-wind_w, -wind_u);
beta_rad  = atan2(-wind_v,  wind_vmag);
alpha_deg = rad2deg(alpha_rad);
beta_deg  = rad2deg(beta_rad);

forces_stability  = zeros(3,length(time));
moments_stability = zeros(3,length(time));
forces_wind       = zeros(3,length(time));
moments_wind      = zeros(3,length(time));

for i=1:length(time)
  %rotation through alpha gives stability axis
  bod2stab_R = [[ cos(alpha_rad(i)),  0,  sin(alpha_rad(i))];...
                [                 0,  1,                  0];...
                [-sin(alpha_rad(i)),  0,  cos(alpha_rad(i))]];

  %rotation through alpha and beta gives wind axis
  stab2wind_r = [[  cos(beta_rad(i)), sin(beta_rad(i)), 0];...
                 [ -sin(beta_rad(i)), cos(beta_rad(i)), 0];...
                 [                 0,                0, 1]];

  %body to wind rotation matrix
  bod2wind_R = stab2wind_r * bod2stab_R;

  % forces and moments in stability
  forces_stability(:,i)    = bod2stab_R * aeroforces_body(:,i);
  moments_stability(:,i)   = bod2stab_R * aeromoments_body(:,i);

  % forces and moments in wind
  forces_wind(:,i)  = bod2wind_R * aeroforces_body(:,i);
  moments_wind(:,i) = bod2wind_R * aeromoments_body(:,i);

  % debug output



  debug.forces_stability(:,i) = bod2stab_R * forces_body(:,i);
  debug.weight_stability(:,i) = bod2stab_R * weight_body(:,i);
  debug.thrust_stability(:,i) = bod2stab_R * thrust_body(:,i);
  debug.tether_force_stability(:,i) = bod2stab_R * force_tether_body(:,1);
end

%normalization coefficients
dynamic_pressure = 0.5 * rho * wind_vmag.^2;

%coefficient of lift, drag, lateral force in stability basis
cl_stability = -forces_stability(3,:)'./(dynamic_pressure * s_ref);
cd_stability = -forces_stability(1,:)'./(dynamic_pressure * s_ref);
cy_stability =  forces_stability(2,:)'./(dynamic_pressure * s_ref);

%coefficient of Roll, Pitch, Yaw moments in stability basis about c.g.
crm_stability_cg = moments_stability(1,:)'./(dynamic_pressure * s_ref * b_ref);
cpm_stability_cg = moments_stability(2,:)'./(dynamic_pressure * s_ref * c_ref);
cym_stability_cg = moments_stability(3,:)'./(dynamic_pressure * s_ref * b_ref);

%coefficient of Roll, Pitch, Yaw moments in stability basis about \vec{x}=0
crm_stability_0 = crm_stability_cg - (cg_posn(2)/b_ref)*cl_stability - (cg_posn(3)/b_ref) * cy_stability;
cpm_stability_0 = cpm_stability_cg + (cg_posn(1)/c_ref)*cl_stability - (cg_posn(3)/c_ref) * cd_stability;
cym_stability_0 = cym_stability_cg + (cg_posn(1)/b_ref)*cy_stability + (cg_posn(2)/b_ref) * cd_stability;

% body force coefficients in body basis
cx_body = aeroforces_body(1,:)'./(dynamic_pressure * s_ref);
cy_body = aeroforces_body(2,:)'./(dynamic_pressure * s_ref);
cz_body = aeroforces_body(3,:)'./(dynamic_pressure * s_ref);

% coefficient of moment in roll, pitch, yaw in body coordinates
% arzamora - fixed permutation from 3,1,2 to 1,2,3 on 4/17
crm_body = aeromoments_body(1,:)'./(dynamic_pressure * s_ref * b_ref);
cpm_body = aeromoments_body(2,:)'./(dynamic_pressure * s_ref * c_ref);
cym_body = aeromoments_body(3,:)'./(dynamic_pressure * s_ref * b_ref);

% coefficients in relative wind axis basis
cx_wind = forces_wind(1,:)'./(dynamic_pressure * s_ref);
cy_wind = forces_wind(2,:)'./(dynamic_pressure * s_ref);
cz_wind = forces_wind(3,:)'./(dynamic_pressure * s_ref);

% coefficient of moment in roll, pitch, yaw in wind coordinates
% arzamora - fixed permutation from 3,1,2 to 1,2,3 on 4/17
crm_wind = moments_wind(1,:)'./(dynamic_pressure * s_ref * b_ref);
cpm_wind = moments_wind(2,:)'./(dynamic_pressure * s_ref * c_ref);
cym_wind = moments_wind(3,:)'./(dynamic_pressure * s_ref * b_ref);


% output to debug struct
debug.cdprops  = -debug.thrust_stability(1,:)'./(dynamic_pressure * s_ref);
debug.cdweight = -debug.weight_stability(1,:)'./(dynamic_pressure * s_ref);
debug.cdtether = -debug.tether_force_stability(1,:)' ./ (dynamic_pressure * s_ref);
debug.cdtot    = -debug.forces_stability(1,:)'./(dynamic_pressure * s_ref);

% sort the data (flight mode, rates below threshold, etc...)
%=======================================================================================================================
%% create output struct
%=======================================================================================================================
aerodata=struct('Vinf', wind_vmag, 'Pdyn', dynamic_pressure, 'h', altitude, 'alphad', alpha_deg, 'betad', beta_deg, ...
                 'CLSA', cl_stability, 'CDSA', cd_stability, 'CYSA', cy_stability, ...
                 'CPMSACG', cpm_stability_cg, 'CYMSACG', cym_stability_cg, 'CRMSACG', crm_stability_cg, ...
                 'CPMSA0', cpm_stability_0, 'CYMSA0', cym_stability_0, 'CRMSA0', crm_stability_0, ...
                 'CXbody', cx_body, 'CYbody', cy_body, 'CZbody', cz_body, ...
                 'CRMbody', crm_body, 'CPMbody', cpm_body, 'CYMbody', cym_body, ...
                 'CXwind', cx_wind, 'CYwind', cy_wind, 'CZwind', cz_wind, ...
                 'CRMwind', crm_wind, 'CPMwind', cpm_wind, 'CYMwind', cym_wind);

             %
% end wall time track
toc
end