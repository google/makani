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

% DataOverlayCSV.m
%
% This is a Matlab script to load Makani flight data files and export a CSV
% used to overlay the data onto flight videos
%
% Inputs: logDataFileName = name of the wing recorder flight data output log file in h5 format
% Outputs:
%       - csvFilename = path and name of the CSV to export
%       - printout of table useful for building the flight observations
%       sheets
%
% 2017-08-04

%%


%% ToDos
% - add in calcs for htail pitot alpha, etc
% - fix bug where some flights (RPX06 wing logs) have error w/ get aero coeff when using varagin=0

%%
clear
% close all
% format compact
tic

% Set the .h5 data file name.
% add paths as needed depending on your specific matlab and local disk setup

% logDataFileName = '20161214-rpx-02_Wing'; % RPX02 full set of data
% logDataFileName = '20170203-rpx03_Wing'; % RPX03 full set of data
% logDataFileName = '20170525-rpx04_Wing'; % RPX04 full set of data
% logDataFileName = '20170627-rpx05_Wing_TimeSync'; % RPX05 full set of data
logDataFileName = '20170807-rpx06_Wing'; % RPX06 full set of data

%

% csvFilename= ['../../../../../Google Drive/Flight data and analysis/Active files/' logDataFileName '.csv'];
csvFilename= ['../../Google Drive/Flight data and analysis/Active files/' logDataFileName '.csv'];

interpmethod = 'linear';

%% pull out aero coefficients
% doing this first so the subroutine doesn't have to load the full h5file
% with the full matfile already in memory

h5file_loc= [logDataFileName,'.h5'];
rotor_json_db= 'rotor_rev4.json';
varargin = [7]; % some of the aerocoeff only make sense in crosswind, but exporting the whole flight let's you view the data for trans-in/out too
% varargin = 0;
[aerodataS.time, aerodataS.flight_constants, aerodataS.aerodata, aerodataS.rtt_commanded, aerodataS.debug] = GetAeroCoeffs(h5file_loc, rotor_json_db, varargin);

toc

%%
h5_filename = [logDataFileName,'.h5'];
inC= h5read(h5_filename,'/messages/kAioNodeControllerA/kMessageTypeControlDebug');
inCsA= h5read(h5_filename,'/messages/kAioNodeCsA/kMessageTypeTetherDown');

C = inC;
c = C.message;
A = inCsA; % b/c we need to pass the header too...


%% pull out the data of interest

% note: much of this isn't exported, but vestigle from original code used
% to export the CSV and do a bunch of other analysis; leaving it in here
% for now since it's relatively cheap and would make it easier to add more
% variables later desired

d.time = c.time;								% time (sec)
d.fltMode = c.flight_mode;						% controller flight mode
d.posGround = s2m(c.state_est.Xg);				% x,y,z ground position (m)
d.wingAz = atan(d.posGround(2,:)./d.posGround(1,:))';
d.wingEl = -atan(d.posGround(3,:)./((d.posGround(1,:).^2+d.posGround(2,:).^2).^0.5))';
d.wind_g_speed_f = c.state_est.wind_g.speed_f; % wind speed at tower
d.wind_g_dir_f = c.state_est.wind_g.dir_f;      % wind direction at tower
if isfield(c.control_input,'weather')
    d.temperature_amb = c.control_input.weather.temperature; % ambient temp
end
d.velGround = s2m(c.state_est.Vg);				% x,y,z ground velocity est (m)
gs_vert_ref = -14.0;                          	% zg of tether origin (m)
d.radiusEst = sqrt(d.posGround(1,:).^2 + ...  	% current distance from ground station (m)
                   d.posGround(2,:).^2 + ...
                  (d.posGround(3,:) - gs_vert_ref).^2)';
d.horDistEst = sqrt(d.posGround(1,:).^2 + ...  	% current hor. distance from ground station (m)
                    d.posGround(2,:).^2);
d.vertDistEst = -(d.posGround(3,:) - gs_vert_ref); % vert. distance from gs, m
d.Xg_gps = c.estimator.gps.Xg.x;              	% xg nx2 for the two gps receivers (m)
d.Yg_gps = c.estimator.gps.Xg.y;              	% yg nx2 for the two gps receivers (m)
d.Zg_gps = c.estimator.gps.Xg.z;              	% zg nx2 for the two gps receivers (m)
d.Vxg_gps = c.estimator.gps.Vg.x;              	% vxg nx2 for the two gps receivers (m)
d.Vyg_gps = c.estimator.gps.Vg.y;              	% vyg nx2 for the two gps receivers (m)
d.Vzg_gps = c.estimator.gps.Vg.z;              	% vzg nx2 for the two gps receivers (m)
d.vgps1 = sqrt(c.estimator.gps.Vg.x(1,:).^2 + ...
               c.estimator.gps.Vg.y(1,:).^2 + ...
               c.estimator.gps.Vg.z(1,:).^2)';
d.altEst = -d.posGround(3,:)';                  % estimated altitude (m)
d.dcm_b2g = c.state_est.dcm_g2b.d;            % body to ground dir cos matrix (note the transpose from the received name)
d.rotorSpeedMeas = c.control_input.rotors;	% measured rotor speeds(rad/sec)
d.rotorSpeedCmd = c.control_output.rotors;	% rotor speed command (rad/sec)
d.thrustCmd = c.thrust_moment_avail.thrust;   % thrust command out of the mixer (N)
d.thrustMomentCmd = c.thrust_moment_avail.moment; % thrust moment command out of the mixer (Nm)
d.normAccel = c.state_est.acc_norm_f;			% total accel (m/s/s)
d.axb1 = c.control_input.imus.acc.x(1,:)';    % imu1 x accel (m/s/s)
d.ayb1 = c.control_input.imus.acc.y(1,:)';    % imu1 y accel (m/s/s)
d.azb1 = c.control_input.imus.acc.z(1,:)';    % imu1 z accel (m/s/s)
d.pb1 = c.control_input.imus.gyro.x(1,:)';    % imu1 x gyro rate (rad/sec)
d.qb1 = c.control_input.imus.gyro.y(1,:)';    % imu1 y gyro rate (rad/sec)
d.rb1 = c.control_input.imus.gyro.z(1,:)';    % imu1 z gyro rate (rad/sec)
d.hover_angles = c.hover.angles;              % .x,.y.,.z = roll, pitch, yaw hover angles (rad)
d.hover_angles_cmd = c.hover.angles_cmd;      % .x,.y.,.z = roll, pitch, yaw hover angle cmds (rad)
d.alt_cmd_hover = -c.hover.wing_pos_g_cmd.z;  % altitude command in hover (m)
d.r_cmd = c.crosswind.pqr_cmd.z;              % commanded yaw rate in cw (rad/sec)
d.gsgAz = c.control_input.gsg.azi(1,:)' + ...
    c.control_input.perch.perch_azi(1,:)';            % ground station gimbal azimuth (rotation about vertical relative to radial) angle (rad)
d.gsgperchAz = c.control_input.gsg.azi(1,:)';          % ground station gimbal azimuth (rotation about vertical relative to perch) angle (rad)
d.perchAz = c.control_input.perch.perch_azi(1,:)';            % ground station perch azimuth (rotation about vertical) angle (rad)
d.gsgEl = c.control_input.gsg.ele(1,:)';            % ground station gimbal elevation angle (rad)
d.flapCmd = c.control_output.flaps;           % flap angle cmds: A1 A2 A4 A5 A7 A8 elevator rudder (rad)
d.cw_phi = c.crosswind.eulers_cw.x;           % roll angle of body wrt cw axes (rad)
d.cw_theta = c.crosswind.eulers_cw.y;         % pitch angle of body wrt cw axes (rad)
d.ti_euler = c.trans_in.eulers_ti2b;          % x,y,z trans-in Euler angles (rad)
d.ti_euler_cmd = c.trans_in.eulers_ti2cmd;    % x,y,z trans-in Euler angle cmds (rad)
d.tether_roll = c.state_est.tether_force_b.sph.roll;  % estimated tether roll angle from load cells (rad)
d.tether_pitch = c.state_est.tether_force_b.sph.pitch;  % estimated tether pitch angle from load cells (rad)
d.tether_roll_cmd = c.crosswind.tether_roll_cmd;      % commaned tether roll angle (rad)
d.tether_tension = c.state_est.tether_force_b.sph.tension;  % estimated non-filtered tether tension (N)
d.tether_tension_f = c.state_est.tether_force_b.tension_f;  % estimated and filtered tether tension (N)
d.tether_tension_h = ...                      % horizontal tension (N)
    sqrt(c.state_est.tether_force_b.vector_f.x.^2 + ...
    c.state_est.tether_force_b.vector_f.y.^2);
d.tether_tension_v = ...
    c.state_est.tether_force_b.vector_f.z;  % vertical tension (N)
d.tether_tension_h_cmd = ...                  % horizontal tension command (N)
    c.hover.horizontal_tension_cmd;
d.tether_tension_cmd = ...                    % tension command (N)
    c.hover.tension_cmd;
d.loop_angle = c.crosswind.loop_angle;        % angle in cw loop:  0 = 9 o'clock looking from ground; up:  3pi/2 (rad)
d.airspeed = c.state_est.apparent_wind.sph_f.v;       % estimated airspeed (m/s)
d.airspeed_cmd = c.crosswind.airspeed_cmd;            % commanded airspeed (m/s)
d.alpha = c.state_est.apparent_wind.sph.alpha;        % estimated angle of attack (rad)
d.alpha_cmd = c.crosswind.alpha_cmd;                  % commanded angle of attack (rad)
d.beta = c.state_est.apparent_wind.sph.beta;          % estimated sideslip angle (rad)
d.beta_cmd = c.crosswind.beta_cmd;                    % commanded sideslip angle (rad)
d.xfp_center = c.crosswind.path_center_g.x;           % tlm xg component of flight path center vector, in g (m)
d.yfp_center = c.crosswind.path_center_g.y;           % tlm yg component of flight path center vector, in g (m)
d.zfp_center = c.crosswind.path_center_g.z;           % tlm zg component of flight path center vector, in g (m)
d.cw_pos_x = c.crosswind.current_pos_cw.x;            % crosswind x location (m)
d.cw_pos_y = c.crosswind.current_pos_cw.y;            % crosswind y location (m)
d.crosswind_pathtype = c.crosswind.path_type;         % crosswind flag indicating shift to transout

d.ail1_angle_cmd = c.control_output.flaps(1,:)';       % a1 command (rad)
d.ail1_angle_meas = c.control_input.flaps(1,:)';       % a1 meas (rad)
d.ail2_angle_cmd = c.control_output.flaps(2,:)';       % a2 command (rad)
d.ail2_angle_meas = c.control_input.flaps(2,:)';       % a2 meas (rad)
d.ail4_angle_cmd = c.control_output.flaps(3,:)';       % a4 command (rad)
d.ail4_angle_meas = c.control_input.flaps(3,:)';       % a4 meas (rad)
d.ail5_angle_cmd = c.control_output.flaps(4,:)';       % a5 command (rad)
d.ail5_angle_meas = c.control_input.flaps(4,:)';       % a5 meas (rad)
d.ail7_angle_cmd = c.control_output.flaps(5,:)';       % a7 command (rad)
d.ail7_angle_meas = c.control_input.flaps(5,:)';       % a7 meas (rad)
d.ail8_angle_cmd = c.control_output.flaps(6,:)';       % a8 command (rad)
d.ail8_angle_meas = c.control_input.flaps(6,:)';       % a8 meas (rad)
d.elv_angle_cmd = c.control_output.flaps(7,:)';        % elv command (rad)
d.elv_angle_meas = c.control_input.flaps(7,:)';        % elv meas (rad)
d.rud_angle_cmd = c.control_output.flaps(8,:)';        % rud command (rad)
d.rud_angle_meas = c.control_input.flaps(8,:)';        % rud meas (rad)

% Compute the transpose of the body-to-ground dcm
d.dcm_g2b = d.dcm_b2g;
for k = 1:size(d.dcm_b2g, 3)
  d.dcm_g2b(:, :, k) = d.dcm_g2b(:, :, k)';
end

% Compute the ground to kite body Euler angles from the dcm
% use standard matlab function if available, otherwise using function found in 'GIT\makani\nav\matlab\attitude'
if license('test','aerospace_toolbox')
    [d.yaw,d.pitch,d.roll] = dcm2angle(d.dcm_g2b);               % Euler angles (rad)
else
    d.yaw = nan(size(d.dcm_g2b,3),1);
    d.pitch = nan(size(d.dcm_g2b,3),1);
    d.roll = nan(size(d.dcm_g2b,3),1);
    for ii=1:size(d.dcm_g2b,3)
        e_angles = dcm_to_euler(d.dcm_g2b(:,:,ii));
        d.yaw(ii,1) = e_angles(1);
        d.pitch(ii,1) = e_angles(2);
        d.roll(ii,1) = e_angles(3);
    end
end


% Set time = 0 at the first time that FltMode = 2 (HoverPerchAscend)
% (unless this variable was defined manually earlier)
if isfield(c,'time0')
    d.time0 = c.time0;
else
    d.time0ii = find(d.fltMode == 2,1,'first');
    d.time0 = d.time(d.time0ii);
end
d.time = d.time - d.time0;


if isfield(c.state_est,'experimental_crosswind_config')
d.xwind_config_select= int32(c.state_est.experimental_crosswind_config);
d.xwind_config_engaged= int32(c.state_est.joystick.pitch_f < -0.5);
d.xwind_config_case= d.xwind_config_select.*d.xwind_config_engaged;
end
d.loop_count= (cumsum([0;diff(d.loop_angle)]>6)+(d.fltMode>=7)).*((d.fltMode==7)|(d.fltMode==8));




d.loop_periods = diff(d.time(diff(d.loop_count)==1));

if min(d.loop_periods)<0.2 % alt def
    d.loop_angle = pi-atan2(c.crosswind.current_pos_cw.y,-c.crosswind.current_pos_cw.x);
    d.loop_count = (cumsum([0;diff(d.loop_angle)]>6)+(d.fltMode>=7)).*((d.fltMode==7)|(d.fltMode==8));
    d.loop_periods = diff(d.time(diff(d.loop_count)==1));
end



%% build up flight modes table and other data used to make the data observation tables
% https://docs.google.com/spreadsheets/d/1mWFrhsatKJuNJhtbeY8dK5kneD-mmyom7hf4IdnkZmc/edit#gid=1387741595

d.epochT0 = double(C.capture_header.tv_sec(d.time0ii))+double(C.capture_header.tv_usec(d.time0ii))*1e-6;
disp(['~Epoch time at T0: ' sprintf('%0.6f', d.epochT0)])
[d.flight_modes, d.crosswind_cases] = get_modes_and_cases(C);

% Get the flight mode labels, specify rpx # for flights before rpx-08.
[~, d.flight_modes.fltModeDef] = get_flight_mode_labels;

if length(d.flight_modes.all_modes) == length(fields(d.flight_modes.time_in_mode))
    for ii= 1:length(d.flight_modes.all_modes)
        d.flight_modes.Tstart(ii,1) = d.flight_modes.time_in_mode.(d.flight_modes.all_modes{ii})(1);
        d.flight_modes.all_modes_names{ii,1} = d.flight_modes.fltModeDef{d.fltMode(d.flight_modes.indices.(d.flight_modes.all_modes{ii})(1))+1};
        d.fltModeNames(d.flight_modes.indices.(d.flight_modes.all_modes{ii})(1):d.flight_modes.indices.(d.flight_modes.all_modes{ii})(2),:)= {[d.flight_modes.all_modes{ii}, ': ',d.flight_modes.all_modes_names{ii,1}]};
    end
    % table(d.flight_modes.all_modes, d.flight_modes.Tstart)
    table(d.flight_modes.all_modes_names, d.flight_modes.Tstart)
else
    warning('bug w/ get_modes_and_cases... skipping table output here for now; only needed for data obs file, not for dataoverlay CSV')
end

if ~isempty(fieldnames(d.crosswind_cases))
    d.xwindCaseNames{length(d.fltModeNames),1}=[];
    for ii= 1:length(d.crosswind_cases.all_cases)
        d.crosswind_cases.Tstart(ii,1) = d.crosswind_cases.time_in_case.(d.crosswind_cases.all_cases{ii})(1);
        d.xwindCaseNames(d.crosswind_cases.indices.(d.crosswind_cases.all_cases{ii})(1):d.crosswind_cases.indices.(d.crosswind_cases.all_cases{ii})(2),:)= {[d.crosswind_cases.all_cases{ii}]};
    end
    table(d.crosswind_cases.all_cases, d.crosswind_cases.Tstart)
end



%% set up export structure

overlayData.time = d.time;
overlayData.airspeed = d.airspeed;
% overlayData.alpha_deg = interp1(aerodataS.time,aerodataS.aerodata.alphad,d.time,interpmethod,NaN);
% overlayData.beta_deg = interp1(aerodataS.time,aerodataS.aerodata.betad,d.time,interpmethod,NaN);
overlayData.alpha_deg = rad2deg(d.alpha);
overlayData.beta_deg = rad2deg(d.beta);
overlayData.CDSA = interp1(aerodataS.time,aerodataS.aerodata.CDSA,d.time,interpmethod,NaN);
overlayData.CYSA = interp1(aerodataS.time,aerodataS.aerodata.CYSA,d.time,interpmethod,NaN);
overlayData.CLSA = interp1(aerodataS.time,aerodataS.aerodata.CLSA,d.time,interpmethod,NaN);
overlayData.CRMSACG = interp1(aerodataS.time,aerodataS.aerodata.CRMSACG,d.time,interpmethod,NaN);
overlayData.CPMSACG = interp1(aerodataS.time,aerodataS.aerodata.CPMSACG,d.time,interpmethod,NaN);
overlayData.CYMSACG = interp1(aerodataS.time,aerodataS.aerodata.CYMSACG,d.time,interpmethod,NaN);
overlayData.posGroundY_m = -d.posGround(2,:)';
overlayData.height_m = -d.posGround(3,:)';
overlayData.ail1_angle_meas_deg = rad2deg(d.ail1_angle_meas);
overlayData.ail2_angle_meas_deg = rad2deg(d.ail2_angle_meas);
overlayData.ail4_angle_meas_deg = rad2deg(d.ail4_angle_meas);
overlayData.ail5_angle_meas_deg = rad2deg(d.ail5_angle_meas);
overlayData.ail7_angle_meas_deg = rad2deg(d.ail7_angle_meas);
overlayData.ail8_angle_meas_deg = rad2deg(d.ail8_angle_meas);
overlayData.elv_angle_meas_deg = rad2deg(d.elv_angle_meas);
overlayData.rud_angle_meas_deg = rad2deg(d.rud_angle_meas);
overlayData.flight_mode = double(d.fltMode);
if ~isempty(fieldnames(d.crosswind_cases))
    overlayData.xwind_config_case = double(d.xwind_config_case);
else
    overlayData.xwind_config_case = zeros(size(overlayData.flight_mode));
end
overlayData.loop_count = d.loop_count;
overlayData.tether_tension_kN = d.tether_tension*0.001;
overlayData.pitch_deg = rad2deg(d.pitch);
overlayData.roll_deg = rad2deg(d.roll);
overlayData.tether_pitch_deg = rad2deg(d.tether_pitch);
overlayData.tether_roll_deg = rad2deg(d.tether_roll);
overlayData.loop_angle_deg = rad2deg(d.loop_angle);
overlayData.gsgEl_deg = rad2deg(d.gsgEl);
overlayData.gsgAz_deg = rad2deg(d.gsgAz);

toc

%% export to csv

overlayTable = struct2table(overlayData);
write(overlayTable,csvFilename); % could be sped up by reducing precision w/ dlm write, but need a clean way to write headers to file

display('export complete')
toc
