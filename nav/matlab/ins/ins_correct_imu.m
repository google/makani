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

function [dvsf_b, phi_b] = ins_correct_imu(xhat, imu)
% Compute raw inertial measurements.
ya_a = imu.dvsf' / imu.dt;
yg_g = imu.phi' / imu.dt;

% Compute useful forms of states.
z_a = ya_a - xhat.b_a - xhat.c_a;
z_g = yg_g - xhat.b_g - xhat.c_g;
K_a = diag(1 + xhat.k_a);
K_g = diag(1 + xhat.k_g);
R_ba = xhat.R_bs * mrp_to_dcm(xhat.mrp_sa);
R_ag = mrp_to_dcm(xhat.mrp_ag);
T_aa = ins_ortho(xhat.no_a);
T_gg = ins_ortho(xhat.no_g);

% Compute specific force experienced by each accelerometer sensor.
f_ia_b = R_ba * T_aa * inv(K_a) * z_a;

% Compute angular rate, where we assume that the location of each gyro sensor
% corresponds to a respective accelerometer sensor.
omega_ib_b = R_ba * R_ag * T_gg * inv(K_g) * z_g - xhat.F_g * f_ia_b;

% Compute finite size correction.
f_ba_bx = [0; 0; 0];
f_ba_by = vcross(omega_ib_b) * vcross(omega_ib_b) * R_ba * xhat.r_ba_ay;
f_ba_bz = vcross(omega_ib_b) * vcross(omega_ib_b) * R_ba * xhat.r_ba_az;
f_ba_b = [f_ba_bx(1); f_ba_by(2); f_ba_bz(3)];
f_ib_b = f_ia_b - f_ba_b;

% Output increments.
dvsf_b = f_ib_b * imu.dt;
phi_b = omega_ib_b * imu.dt;
