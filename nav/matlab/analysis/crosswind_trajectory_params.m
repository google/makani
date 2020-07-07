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

function [p] = crosswind_trajectory_params()
% crosswind_trajectory_params -- Generate the default trajectory parameters.
%
% p = crosswind_trajectory_params()

d2r = pi/180;
p = struct();

% Define the wind direction as a rotation about the north-east-down z-axis,
% where 0 degrees points north, and 90 degrees points east.
p.wind_direction = 270 * d2r;  % [rad]

% Define the crosswind trajectory loop radius.
p.loop_radius = 100;  % [m]

% Define the average angular rate to velocity scale factor for the top of the
% loop. Generally, this parameter should be on [0, 1]. A value of zero sets the
% velocity at the top of the loop to equal zero. A value of one makes the
% velocity constant throughout the loop.
p.loop_velocity_scale = 0.8;  % [#]

% Define the amount of time it takes the kite to fly one loop.
p.loop_period = 20;  % [s]

% Define the virtual hub height.
p.hub_height = 200;  % [m]

% Define the tether length. The trajectory generator assumes that the tether
% remains taut.
p.tether_length = 450;  % [m]

% Define the kite angle of attack about the pitch axis.
p.kite_alpha = 4 * d2r;  % [rad]

% Define the inertial sensor offset from the body frame origin.
p.sensor_offset = [0; 0; 0.5];  % [m]

% Define the inertial sensor rotation from sensor to body frame.
p.dcm_sensor_to_body = eye(3);

% Define the gravity vector in north-east-down coordinates.
p.g_n = [0; 0; 9.80665];

% Define the magnetic field vector in north-east-down coordinates.
p.field_n = [0.231583; 0.050742; 0.417817];
end
