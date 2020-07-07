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

function [xhat] = ins_xhat_initialize(p, start_time, init)
earth = earth_wgs84();
xhat = struct();

% Inertial inputs.
xhat.t = start_time;

% Orientation parameters.
xhat.R_bs = p.geometry.R_bs;

% Accelerometer error model parameters.
xhat.b_a = zeros(3, 1);      % Accel bias instability.
xhat.c_a = zeros(3, 1);      % Accel turn-on bias and random walk.
xhat.k_a = zeros(3, 1);      % Accel scale factor error.
xhat.mrp_sa = zeros(3, 1);   % Accel misalignment error (accel-to-sensor).
xhat.no_a = zeros(3, 1);     % Accel non-orthogonality error.
xhat.r_ba_ay = zeros(3, 1);  % Accel y-axis offset.
xhat.r_ba_az = zeros(3, 1);  % Accel z-axis offset.

% Gyro error model parameters.
xhat.b_g = zeros(3, 1);     % Gyro bias instability.
xhat.c_g = zeros(3, 1);     % Gyro turn-on bias and random walk.
xhat.k_g = zeros(3, 1);     % Gyro scale factor error.
xhat.mrp_ag = zeros(3, 1);  % Gyro misalignment error (gyro-to-accel).
xhat.no_g = zeros(3, 1);    % Gyro non-orthogonality error.
xhat.F_g = zeros(3, 3);     % Gyro g-sensitivity.

% Wheel encoder parameters.
xhat.dradius = zeros(4, 1);

% GPS error model parameters.
xhat.cb_phase = 0;  % GPS receiver clock phase (multipled by c).
xhat.cf_bias = 0;   % GPS receiver clock frequency bias (multipled by c).
xhat.cf_walk = 0;   % GPS receiver clock frequency random walk (multipled by c).
xhat.gps_pr = zeros(32, 1);  % GPS pseudo-range bias.

% ECEF navigation state.
xhat.q_eb = q_null();              % Body to ECEF rotation.
xhat.R_eb = q_to_dcm(xhat.q_eb);   % Body to ECEF rotation.
xhat.r_ix_e = int32(zeros(3, 1));  % Fixed point ECEF position.
xhat.dr_xb_e = zeros(3, 1);        % Fractional ECEF position.
xhat.v_ib_e = zeros(3, 1);         % ECEF velocity.
xhat.g_ib_e = zeros(3, 1);         % ECEF plumb gravity vector.
xhat.omega_ie_e = [0; 0; earth.omega];  % Earth rotation rate.

% Initialze.
if exist('init', 'var') && ~isempty(init)
  % Quasi-stationary assumptions. We are not accelerating and therefore the
  % specific force measurement aligns with plumb gravity to define vertical.
  a_ib_e = zeros(3, 1);

  % Compute corrected inertial measurements using our error models.
  dt = init.imu.dt;
  [dvsf_b, phi_b] = ins_correct_imu(xhat, init.imu);

  % Compute GPS antenna position and velocity from observable data. An
  % alternate implementation might assume the GPS solution output.
  [r_igps_e, xhat.cb_phase, drho, v_igps_e, xhat.cf_walk] = ...
      gps_point(init.eph, init.iono, init.obs);

  % Use GPS pseudo-range residuals to initialize the pseudo-range biases.
  xhat.gps_pr(cellfun(@(x) x.prn, init.obs)) = drho;

  % Compute local gravity using the GPS antenna's location. This vector
  % defines plumb gravity (vertical).
  [xhat.g_ib_e, xhat.dg_ib_e] = earth_gravity(r_igps_e);

  % Define an initial azimuth rotation to study estimation performance for
  % arbitrary azimuth angles.
  R_azi = [cos(p.init.azimuth), -sin(p.init.azimuth), 0;
           sin(p.init.azimuth), cos(p.init.azimuth), 0;
           0, 0, 1];

  % Compute the local level frame rotation to relate our initial azimuth
  % rotation to our ECEF attitude.
  [~, ~, ~, R_le] = ecef_to_llh(r_igps_e);

  % Estimate attitude. Since we do not have a magnetometer, we assume due
  % north as our initial azimuth (before applying rotation R_azi).
  f_ib_b = dvsf_b / dt;
  f_ib_e = a_ib_e - xhat.g_ib_e + 2 * cross(xhat.omega_ie_e, v_igps_e);
  x_b = [1; 0; 0];
  x_e = R_le' * R_azi * [1; 0; 0];
  xhat.q_eb = q_subopt(f_ib_b, f_ib_e, x_b, x_e);
  xhat.R_eb = q_to_dcm(xhat.q_eb);

  % Now that we know our GPS antenna position and attitude, we can compute
  % the corresponding INS position.
  r_ib_e = r_igps_e - xhat.R_eb * p.geometry.r_bgps_b;
  xhat.r_ix_e = int32(fix(r_ib_e));
  xhat.dr_xb_e = r_ib_e - double(xhat.r_ix_e);

  % Compute INS velocity.
  omega_ib_b = phi_b / dt;
  omega_eb_b = omega_ib_b - xhat.R_eb' * xhat.omega_ie_e;
  xhat.v_ib_e = v_igps_e - xhat.R_eb * cross(omega_eb_b, p.geometry.r_bgps_b);
end
