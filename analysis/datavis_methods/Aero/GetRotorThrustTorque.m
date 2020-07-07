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

function [mtime_sync,iq, id, omega_rotor_axis, omegadot_rotor_axis, ...
          vFreestream, rotor_thrust, aero_torque, motor_torque] = GetRotorThrustTorque(rTT, rotor_json_db)
  % GetRotorThrustTorque gets the rotor thrust and torque from a rotor json file
  %
  % Description
  % - Calculate the thrust produced by each rotor using the rotor tables.
  % - Calculate the aero torque using the motor torque current iq
  %
  % Modified CalcRotorsThrustTorque for use with aero_coeffs.m
  %
  % Inputs:
  %   rTT - rotor thrust and torque commanded
  %   rotor_json_db - the specific rotor design json file
  %
  % Outputs:
  %   mtime_sync - the time sync between rotor time and controller time
  %   iq - current
  %   id - another form of current
  %   omega_rotor_axis    - angular speed of rotor about its axis
  %   omegadot_rotor_axis - angular acceleration of rotor about its axis
  %   vFreestream  - rotor axial freestream velocity
  %   rotor_thrust - thrust the rotor is expected to produce
  %   aero_torque  - moment that the rotor experiences
  %   motor_torque - amount of torque on the motor
  %
  %% Initialization
  % Load the json rotor database
  rotor_db = loadjson(rotor_json_db);
  %
  % Get the motor labels
  motor_label = {'MotorSbo'; 'MotorSbi'; 'MotorPbi'; 'MotorPbo'; ...
                 'MotorPto'; 'MotorPti'; 'MotorSti'; 'MotorSto'};
  %
  % Initialize output variables
  number_rotor = length(motor_label);
  length_time_vector = length(rTT.ctime);
  iq = zeros(number_rotor, length_time_vector);
  id = zeros(number_rotor, length_time_vector);
  rotor_thrust = zeros(number_rotor, length_time_vector);
  motor_torque = zeros(number_rotor, length_time_vector);
  aero_torque  = zeros(number_rotor, length_time_vector);
  vFreestream  = zeros(number_rotor, length_time_vector);
  rotor_omega  = zeros(number_rotor, length_time_vector);
  omega_rotor_axis    = zeros(number_rotor, length_time_vector);
  omegadot_rotor_axis = zeros(number_rotor, length_time_vector);
  %
  mtime_sync = zeros(number_rotor, length_time_vector);
  %
  %%
  for i = 1:number_rotor
      %% Rotor parameters and local consitions
      time_ctrl = rTT.ctime;
      time_motor = rTT.(motor_label{i}).mtime;

      %re-breakpoint motor values into the controller time regime
      omega_motor = interp1(time_motor, rTT.(motor_label{i}).omega,time_ctrl,'linear','extrap');
      iq(i,:) = interp1(time_motor, rTT.(motor_label{i}).iq,time_ctrl,'linear','extrap')';
      id(i,:) = interp1(time_motor, rTT.(motor_label{i}).id,time_ctrl,'linear','extrap')';
      mtime_sync(i,:) = interp1(time_motor, time_motor,time_ctrl,'linear','extrap')';

      % Angular acceleration (LPF + central difference)
      domega_motor = DiffLpf(omega_motor, 1/(time_ctrl(2)-time_ctrl(1)), 10);

      % Motor omega and rotor omega are opposite sign convention
      omega_rotor_axis(i,:)    = -omega_motor';
      omegadot_rotor_axis(i,:) = -domega_motor;
      %
      % Axial freestream:
          % Projects the local apparent wind vector on the rotor axis to
          % calculate the apparent wind speed (freestream velocity) used in the
          % rotor aerodynamics database.
          % Neglect the apparent speed due to the kite rotations.
      vFreestream(i,:) = -rTT.apparent_wind'*[rTT.rotors_axis(i,:)]';

      % Current rotor tables don't include negative advance ratios.
      % If VFreestream is negative, saturate.
      i_saturate = find(vFreestream(i,:) < 0);
      vFreestream(i,i_saturate) = rotor_db.v_freestreams(1);

      %% Calculate Torque
      %Torque current
      % Yasa 2.3 best fit curve
      motor_torque(i,:) = 3.72849*iq(i,:) - 0.0005336*iq(i,:).^2 - 4.09426e-06*iq(i,:).^3;

      % Inertial torque
      inertial_torque = rTT.rotors_I(i)*domega_motor';

      % Aero torque
      aero_torque(i, :) = motor_torque(i,:) - inertial_torque;

      % Interpolate linearly
      % rotor_db.thrust (N_omegas, N_freestreams)
      rotor_thrust(i, :) = interp2(rotor_db.v_freestreams, rotor_db.omegas, ...
                                   rotor_db.thrust, vFreestream(i,:), abs(omega_rotor_axis(i,:)));

      % If vFreestream or omega are outside of the bounds of the table,
      % then extrapolate using a spline interpolation.
       j_isnan = find(isnan(rotor_thrust(i, :)));
       rotor_thrust(i, j_isnan) =interp2(rotor_db.v_freestreams, rotor_db.omegas, rotor_db.thrust, ...
                                         vFreestream(i,j_isnan), rotor_omega(i, j_isnan), 'spline');
  end
end
