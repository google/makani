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

function [p] = ins_param(data, init_azimuth)
if ~exist('init_azimuth', 'var') || isempty(init_azimuth)
  init_azimuth = 0;
end

p = struct();
p.geometry = get_geometry(data.config);
p.gps = get_gps_param(data);
p.clock = get_clock_param();
p.accel = get_accel_param();
p.gyro = get_gyro_param();
p.wheel = get_wheel_param(data);
p.init = get_init_param(init_azimuth);
p.stationary = get_stationary_param();
[p.states, p.inputs, p.meas] = ins_states();

% Use select_index() to down select states from ins_states.m.

% Select wheel error states.
wheels = [p.wheel.WHEEL_RL, p.wheel.WHEEL_RR];
p.states = select_index(p.states, p.states.WHEEL_RADIUS, wheels);
p.inputs = select_index(p.inputs, p.inputs.WHEEL_RADIUS, wheels);
p.meas = select_index(p.meas, p.meas.WHEEL_VEL, wheels);

% Select GPS satellites in data set (to improve computation performance).
p.states = select_index(p.states, p.states.GPS_PR, p.gps.prns);
p.inputs = select_index(p.inputs, p.inputs.GPS_PR, p.gps.prns);
p.meas = select_index(p.meas, p.meas.GPS_PR, p.gps.prns);
p.meas = select_index(p.meas, p.meas.GPS_DR, p.gps.prns);
p.meas = select_index(p.meas, p.meas.GPS_DO, p.gps.prns);

% These error sources tend to be small.
p.states = select_index(p.states, p.states.THETA_AG, []);
p.states = select_index(p.states, p.states.ACCEL_ORTHO, []);
p.states = select_index(p.states, p.states.ACCEL_OFFSET_Y, []);
p.states = select_index(p.states, p.states.ACCEL_OFFSET_Z, []);
p.states = select_index(p.states, p.states.GYRO_ORTHO, []);
p.states = select_index(p.states, p.states.GYRO_G_SENSE, []);

fprintf(1, 'Selected error states:\n');
for i = 1:length(p.states.select)
  fprintf(1, '  - %s\n', p.states.name{p.states.select(i)});
end

% Lag INS by a fixed number of IMU samples. This number times the sampling
% period should be greater than the maximum GPS measurement latency.
p.lag_imu_samples = 30;


function [p] = get_geometry(config)
% Recalculate position vectors such that they are all relative to the IMU
% in body frame.
p = struct();

% Chauffeur defines vehicle coordinates x-forward, y-left, and z-up. Rotate
% them to x-forward, y-right, z-down.
R = [1 0 0; 0 -1 0; 0 0 -1];
imu_pos = R * reshape(config.extrinsics.imu_pos, 3, 1);
gps_pos = R * reshape(config.extrinsics.gps_pos, 3, 1);
wheel_pos = R * reshape(config.extrinsics.wheel_pos, 3, 1);

% The wheel_pos appears to represent the rear differential position and
% relates to the resolver speed output. We assume that the location of this
% position is equal distance between the two rear wheels. We calculate the
% position of each wheel using the track_width and wheel_base.
% See PoseEstimator::PoseEstimator() in pose_estimator.cc.
p.track_width = config.extrinsics.track_width;
p.wheel_base = config.extrinsics.wheelbase;
p.wheel_radius = config.extrinsics.wheel_radius;
p.r_bwheelfl_b = [p.wheel_base; -p.track_width/2; 0] - imu_pos;
p.r_bwheelfr_b = [p.wheel_base; p.track_width/2; 0] - imu_pos;
p.r_bwheelrl_b = [0; -p.track_width/2; 0] - imu_pos;
p.r_bwheelrr_b = [0; p.track_width/2; 0] - imu_pos;
p.r_bresolver_b = (p.r_bwheelrl_b + p.r_bwheelrr_b) / 2;

% GPS antenna position.
p.r_bgps_b = gps_pos - imu_pos;

% Compute sensor-to-body rotation. For some reason, we require an additional
% rotation about the x-axis to obtain the correct roll and pitch angles.
R = euler_to_dcm([-pi; 0; 0]);
p.R_bs = euler_to_dcm(config.extrinsics.rot * pi / 180) * R;


function [p] = get_gps_param(data)
p = struct();
p.prns = unique(data.obs.prn);
p.prns = p.prns(1 <= p.prns);
p.prns = p.prns(p.prns <= 32);
p.timeout = diff(data.obs.t_obs);
p.timeout = p.timeout(p.timeout > 0);
p.timeout = max(p.timeout(p.timeout <= 1)) * 1.1;  % [s]
p.sigma_pr = 1;  % [m]
p.sigma_do = 0.3;  % [m/s]
p.mask_angle = 10 * pi/180;  % [rad]
p.pr_bias_tau = 3600;  % [s]


function [p] = get_clock_param()
% Our u-blox clock appears to be less stable than described in their timing
% app note. Dataset 20151028_195957_C00516, for example, has a significant
% change in frequency drift near t=225 seconds. Using values from the app
% note do not capture this instability.
% https://www.u-blox.com/sites/default/files/products/documents/Timing_AppNote_(GPS.G6-X-11007).pdf
c = 300e6;
p = struct();
p.sigma_N = c * 5e-10;
p.sigma_B = c * 5e-10 / sqrt(2*log(2)/pi);
p.B_tau = 100;
p.sigma_K = c * 1e-10;
[p.qc_freq, p.tc_freq] = ins_bi_model(p.sigma_B, p.B_tau);


function [p] = get_accel_param()
deg = pi / 180;
mg = 1000 / 9.80665;
hr = 3600;

p = struct();
p.sigma_N = 0.1 * 1/mg;
p.sigma_B = 0.01 / sqrt(2*log(2)/pi) * 1/mg;
p.B_tau = 100;
p.sigma_K = 1e-6;

% These parameters specify the stability of each quantity. Set parameters in
% ins_coarse_align.m to specify the initial uncertainty of each quantity.
[p.qc_bias, p.tc_bias] = ins_bi_model(p.sigma_B, p.B_tau);
[p.qc_scale, p.tc_scale] = ins_bi_model(1e-6, 3600);
[p.qc_misalign, p.tc_misalign] = ins_bi_model(1e-6 * deg, 3600);
[p.qc_ortho, p.tc_ortho] = ins_bi_model(1e-6 * deg, 3600);
[p.qc_offset, p.tc_offset] = ins_bi_model(1e-6, 3600);


function [p] = get_gyro_param()
deg = pi / 180;
hr = 3600;
g = 9.80665;

p = struct();
p.sigma_N = [11, 11, 0.5] * deg/sqrt(hr);
p.sigma_B = [4, 4, 0.1] / sqrt(2*log(2)/pi) * deg/hr;
p.B_tau = [100, 100, 500];
p.sigma_K = [1e-8, 1e-8, 1e-8];

% These parameters specify the stability of each quantity. Set parameters in
% ins_coarse_align.m to specify the initial uncertainty of each quantity.
[p.qc_bias, p.tc_bias] = ins_bi_model(p.sigma_B, p.B_tau);
[p.qc_scale, p.tc_scale] = ins_bi_model(1e-6, 3600);
[p.qc_misalign, p.tc_misalign] = ins_bi_model(1e-6 * deg, 3600);
[p.qc_ortho, p.tc_ortho] = ins_bi_model(1e-6 * deg, 3600);
[p.qc_g_sense, p.tc_g_sense] = ins_bi_model(1e-6 * deg/g, 3600);


function [p] = get_wheel_param(data)
p = struct();
p.WHEEL_FL = 1;  % Array indices.
p.WHEEL_FR = 2;
p.WHEEL_RL = 3;
p.WHEEL_RR = 4;
p.sigma_omega = 1;  % [rad/s]
p.update_period = 0.1;
p.has_wheel = length(data.wheel.t) > 0;
[p.qc_radius, p.tc_radius] = ins_bi_model(1e-3, 300);


function [p] = get_init_param(init_azimuth)
p = struct();
% TODO: Determine reasonable thresholds.
p.satellites = 5;
p.dvsf_var = 0.1;
p.dvsf_err = 0.05;
p.phi_var = 1e-3;
p.phi_err = 1e-3;
p.azimuth = init_azimuth;

function [p] = get_stationary_param()
p.max_velocity = 0.001;
p.update_period = 0.1;
