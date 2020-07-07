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

function [x, u, m] = ins_states()
deg = 180 / pi;
g = 9.80665;
mg = 1000 / g;

% States.
x = struct();
x = add_index(x, 3, 'theta_bi', 'Attitude', 'deg', deg);
x = add_index(x, 3, 'v_ib_e', 'ECEF velocity', 'm/s', 1);
x = add_index(x, 3, 'r_ib_e', 'ECEF position', 'm', 1);
x = add_index(x, 3, 'accel_bias', 'Accel bias instability', 'mg', mg);
x = add_index(x, 3, 'accel_walk', 'Accel turn-on bias', 'mg', mg);
x = add_index(x, 3, 'accel_ortho', 'Accel nonorthogonality', 'deg', deg);
x = add_index(x, 3, 'accel_scale', 'Accel scale factor error', 'ppm', 1e6);
x = add_index(x, 3, 'accel_offset_y', 'Accel y-axis offset', 'mm', 1000);
x = add_index(x, 3, 'accel_offset_z', 'Accel z-axis offset', 'mm', 1000);
x = add_index(x, 3, 'theta_sa', 'Body-to-sensor misalignment', 'deg', deg);
x = add_index(x, 3, 'gyro_bias', 'Gyro bias instability', 'deg/hr', 3600*deg);
x = add_index(x, 3, 'gyro_walk', 'Gyro turn-on bias', 'deg/hr', 3600*deg);
x = add_index(x, 3, 'gyro_ortho', 'Gyro nonorthogonality', 'deg', deg);
x = add_index(x, 3, 'gyro_scale', 'Gyro scale factor error', 'ppm', 1e6);
x = add_index(x, 9, 'gyro_g_sense', 'Gyro g-sensitivity', 'deg/s/g', deg/g);
x = add_index(x, 3, 'theta_ag', 'Gyro-to-accel misalignment', 'deg', deg);
x = add_index(x, 4, 'wheel_radius', 'Wheel radius error', 'mm', 1000);
x = add_index(x, 1, 'cb_phase', 'GPS clock phase error', 'm', 1);
x = add_index(x, 1, 'cf_bias', 'GPS clock frequency instability', 'm/s', 1);
x = add_index(x, 1, 'cf_walk', 'GPS clock frequency random walk', 'm/s', 1);
x = add_index(x, 32, 'gps_pr', 'GPS pseudo-range bias', 'm', 1);

% Inputs.
u = struct();
u = add_index(u, 3, 'accel_n', 'Velocity random walk');
u = add_index(u, 3, 'accel_b', 'Accel bias instability');
u = add_index(u, 3, 'accel_k', 'Accel bias random walk');
u = add_index(u, 3, 'accel_ortho', 'Accel nonorthogonality instability');
u = add_index(u, 3, 'accel_scale', 'Accel scale factor error instability');
u = add_index(u, 3, 'accel_misalign', 'Accel misalignment instability');
u = add_index(u, 3, 'accel_offset_y', 'Accel y-axis offset instability');
u = add_index(u, 3, 'accel_offset_z', 'Accel z-axis offset instability');
u = add_index(u, 3, 'gyro_n', 'Angular random walk');
u = add_index(u, 3, 'gyro_b', 'Gyro bias instability');
u = add_index(u, 3, 'gyro_k', 'Gyro bias random walk');
u = add_index(u, 3, 'gyro_ortho', 'Gyro nonorthogonality instability');
u = add_index(u, 3, 'gyro_scale', 'Gyro scale factor error instability');
u = add_index(u, 3, 'gyro_misalign', 'Gyro misalignment instability');
u = add_index(u, 9, 'gyro_g_sense', 'Gyro g-sensitivity instability');
u = add_index(u, 4, 'wheel_radius', 'Wheel radius instability');
u = add_index(u, 1, 'clock_n', 'GPS clock white noise');
u = add_index(u, 1, 'clock_b', 'GPS clock bias instability');
u = add_index(u, 1, 'clock_k', 'GPS clock bias random walk');
u = add_index(u, 32, 'gps_pr', 'GPS pseudo-range bias random walk');

% Measurements.
m = struct();
m = add_index(m, 4, 'wheel_vel', 'Wheel angular velocity', 'deg/s', deg);
m = add_index(m, 1, 'zomega', 'Zero angular rate', 'deg/s', deg);
m = add_index(m, 1, 'zaccel', 'Zero acceleration', 'm/s^2', 1);
m = add_index(m, 1, 'zvel', 'Zero velocity', 'm/s', 1);
m = add_index(m, 1, 'vertical', 'Vertical constraint', 'm/s', 1);
m = add_index(m, 32, 'gps_pr', 'GPS pseudo-range', 'm', 1);
m = add_index(m, 32, 'gps_dr', 'GPS delta-range', 'm', 1);
m = add_index(m, 32, 'gps_do', 'GPS doppler', 'm/s', 1);
