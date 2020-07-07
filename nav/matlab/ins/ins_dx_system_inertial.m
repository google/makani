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

function [A, B, Q, inputs] = ins_dx_system_inertial(p, xhat, imu)
% Compute general inertial error model.
s = p.states;
u = p.inputs;
I3 = eye(3);
I9 = eye(9);

% Compute raw inertial measurements.
ya_a = imu.dvsf' / imu.dt;
yg_g = imu.phi' / imu.dt;

% Compute helpful quantities.
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
Omega_ib_b = vcross(omega_ib_b);
omega_eb_b = omega_ib_b - xhat.R_eb' * xhat.omega_ie_e;

% Compute finite size correction.
f_ba_bx = [0; 0; 0];
f_ba_by = vcross(omega_ib_b) * vcross(omega_ib_b) * R_ba * xhat.r_ba_ay;
f_ba_bz = vcross(omega_ib_b) * vcross(omega_ib_b) * R_ba * xhat.r_ba_az;
f_ba_b = [f_ba_bx(1); f_ba_by(2); f_ba_bz(3)];
f_ib_b = f_ia_b - f_ba_b;

% Compute helpful matrices for accelerometer nonothogonality and scale factor
% error relationships.
dT_aa = [0, z_a(3)/(1 + xhat.k_a(3)), -z_a(2)/(1 + xhat.k_a(2));
         -z_a(3)/(1 + xhat.k_a(3)), 0, 0;
         0, 0, 0];
dK_a = diag(z_a ./ (1 + xhat.k_a).^2);

% Compute helpful matrices for gyro nonothogonality, scale factor error,
% and g-sensitivity relationships.
dT_gg = [0, z_g(3)/(1 + xhat.k_g(3)), -z_g(2)/(1 + xhat.k_g(2));
         -z_g(3)/(1 + xhat.k_g(3)), 0, 0;
         0, 0, 0];
dK_g = diag(z_g ./ (1 + xhat.k_g).^2);
dF_g = -[f_ia_b', zeros(1, 3), zeros(1, 3);
         zeros(1, 3), f_ia_b', zeros(1, 3);
         zeros(1, 3), zeros(1,3), f_ia_b'];

% Compute specific force f_ia_b error model.
A_fia = zeros(3, s.count);
A_fia(:, s.THETA_SA) = R_ba * vcross(T_aa * inv(K_a) * z_a);
A_fia(:, s.ACCEL_BIAS) = -R_ba * T_aa * inv(K_a);
A_fia(:, s.ACCEL_WALK) = A_fia(:, s.ACCEL_BIAS);
A_fia(:, s.ACCEL_ORTHO) = R_ba * dT_aa;
A_fia(:, s.ACCEL_SCALE) = -R_ba * T_aa * dK_a;

% Compute angular rate omega_ib_b error model.
A_wib = zeros(3, s.count);
A_wib(:, s.THETA_SA) = R_ba * vcross(R_ag * T_gg * inv(K_g) * z_g);
A_wib(:, s.THETA_AG) = R_ba * R_ag * vcross(T_gg * inv(K_g) * z_g);
A_wib(:, s.GYRO_BIAS) = -R_ba * R_ag * T_gg * inv(K_g);
A_wib(:, s.GYRO_WALK) = A_wib(:, s.GYRO_BIAS);
A_wib(:, s.GYRO_ORTHO) = R_ba * R_ag * dT_gg;
A_wib(:, s.GYRO_SCALE) = -R_ba * R_ag * T_gg * dK_g;
A_wib(:, s.GYRO_G_SENSE) = dF_g;
A_wib = A_wib - xhat.F_g * A_fia;
B_wib = A_wib(:, s.GYRO_BIAS);  % Angular random walk.

% Compute angular rate omega_eb_b error model.
A_web = A_wib;
A_web(:, s.THETA_BI) = -vcross(xhat.R_eb' * xhat.omega_ie_e);
B_web = A_web(:, s.GYRO_BIAS);  % Angular random walk.

% Compute specific force f_ba_b(x) finite size error model.
A_fbax = zeros(3, s.count);

% Compute specific force f_ba_b(y) finite size error model.
A_fbay = zeros(3, s.count);
A_fbay(:, s.THETA_SA) = Omega_ib_b * Omega_ib_b * R_ba * vcross(xhat.r_ba_ay);
A_fbay(:, s.ACCEL_OFFSET_Y) = Omega_ib_b * Omega_ib_b * R_ba;
A_fbay = A_fbay - vcross(Omega_ib_b * R_ba * xhat.r_ba_ay) * A_wib;
A_fbay = A_fbay - Omega_ib_b * vcross(R_ba * xhat.r_ba_ay) * A_wib;

% Compute specific force f_ba_b(z) finite size error model.
A_fbaz = zeros(3, s.count);
A_fbaz(:, s.THETA_SA) = Omega_ib_b * Omega_ib_b * R_ba * vcross(xhat.r_ba_az);
A_fbaz(:, s.ACCEL_OFFSET_Z) = Omega_ib_b * Omega_ib_b * R_ba;
A_fbaz = A_fbaz - vcross(Omega_ib_b * R_ba * xhat.r_ba_az) * A_wib;
A_fbaz = A_fbaz - Omega_ib_b * vcross(R_ba * xhat.r_ba_az) * A_wib;

% Compute specific force f_ib_b error model.
A_fib = A_fia - [A_fbax(1, :); A_fbay(2, :); A_fbaz(3, :)];
B_fib = A_fib(:, s.ACCEL_BIAS);  % Velocity random walk.

% Compute system model.
A = zeros(s.count, s.count);
B = zeros(s.count, u.count);
Q = zeros(u.count, u.count);

% Compute accelerometer misalignment error state.
A(s.THETA_SA, s.THETA_SA) = -I3 * diag(1 ./ p.accel.tc_misalign);
B(s.THETA_SA, u.ACCEL_MISALIGN) = I3;
Q(u.ACCEL_MISALIGN, u.ACCEL_MISALIGN) = I3 * diag(p.accel.qc_misalign.^2);

% Compute accelerometer nonorthogonality error state.
A(s.ACCEL_ORTHO, s.ACCEL_ORTHO) = -I3 * diag(1 ./ p.accel.tc_ortho);
B(s.ACCEL_ORTHO, u.ACCEL_ORTHO) = I3;
Q(u.ACCEL_ORTHO, u.ACCEL_ORTHO) = I3 * diag(p.accel.qc_ortho.^2);

% Compute accelerometer bias instability error state.
A(s.ACCEL_BIAS, s.ACCEL_BIAS) = -I3 * diag(1 ./ p.accel.tc_bias);
B(s.ACCEL_BIAS, u.ACCEL_B) = I3;
Q(u.ACCEL_B, u.ACCEL_B) = I3 * diag(p.accel.qc_bias.^2);

% Compute accelerometer turn-on bias and random walk error state.
B(s.ACCEL_WALK, u.ACCEL_K) = I3;
Q(u.ACCEL_K, u.ACCEL_K) = I3 * diag(p.accel.sigma_K.^2);

% Compute accelerometer scale factor error state.
A(s.ACCEL_SCALE, s.ACCEL_SCALE) = -I3 * diag(1 ./ p.accel.tc_scale);
B(s.ACCEL_SCALE, u.ACCEL_SCALE) = I3;
Q(u.ACCEL_SCALE, u.ACCEL_SCALE) = I3 * diag(p.accel.qc_scale.^2);

% Compute accelerometer offset (finite size) error state.
A(s.ACCEL_OFFSET_Y, s.ACCEL_OFFSET_Y) = -I3 * diag(1 ./ p.accel.tc_offset);
B(s.ACCEL_OFFSET_Y, u.ACCEL_OFFSET_Y) = I3;
Q(u.ACCEL_OFFSET_Y, u.ACCEL_OFFSET_Y) = I3 * diag(p.accel.qc_offset.^2);
A(s.ACCEL_OFFSET_Z, s.ACCEL_OFFSET_Z) = -I3 * diag(1 ./ p.accel.tc_offset);
B(s.ACCEL_OFFSET_Z, u.ACCEL_OFFSET_Z) = I3;
Q(u.ACCEL_OFFSET_Z, u.ACCEL_OFFSET_Z) = I3 * diag(p.accel.qc_offset.^2);

% Compute gyro misalignment error state.
A(s.THETA_AG, s.THETA_AG) = -I3 * diag(1 ./ p.gyro.tc_misalign);
B(s.THETA_AG, u.GYRO_MISALIGN) = I3;
Q(u.GYRO_MISALIGN, u.GYRO_MISALIGN) = I3 * diag(p.gyro.qc_misalign.^2);

% Compute gyro nonorthogonality error state.
A(s.GYRO_ORTHO, s.GYRO_ORTHO) = -I3 * diag(1 ./ p.gyro.tc_ortho);
B(s.GYRO_ORTHO, u.GYRO_ORTHO) = I3;
Q(u.GYRO_ORTHO, u.GYRO_ORTHO) = I3 * diag(p.gyro.qc_ortho.^2);

% Compute gyro bias instability error state.
A(s.GYRO_BIAS, s.GYRO_BIAS) = -I3 * diag(1 ./ p.gyro.tc_bias);
B(s.GYRO_BIAS, u.GYRO_B) = I3;
Q(u.GYRO_B, u.GYRO_B) = I3 * diag(p.gyro.qc_bias.^2);

% Compute gyro turn-on bias and random walk error state.
B(s.GYRO_WALK, u.GYRO_K) = I3;
Q(u.GYRO_K, u.GYRO_K) = I3 * diag(p.gyro.sigma_K.^2);

% Compute gyro scale factor error state.
A(s.GYRO_SCALE, s.GYRO_SCALE) = -I3 * diag(1 ./ p.gyro.tc_scale);
B(s.GYRO_SCALE, u.GYRO_SCALE) = I3;
Q(u.GYRO_SCALE, u.GYRO_SCALE) = I3 * diag(p.gyro.qc_scale.^2);

% Compute gyro g-sensitivity error state.
A(s.GYRO_G_SENSE, s.GYRO_G_SENSE) = -I9 * diag(1 ./ p.gyro.tc_g_sense);
B(s.GYRO_G_SENSE, u.GYRO_G_SENSE) = I9;
Q(u.GYRO_G_SENSE, u.GYRO_G_SENSE) = I9 * diag(p.gyro.qc_g_sense.^2);

% Compute attitude error state.
A(s.THETA_BI, :) = A_wib;
A(s.THETA_BI, s.THETA_BI) = -vcross(omega_ib_b);
B(s.THETA_BI, u.GYRO_N) = B_wib;
Q(u.GYRO_N, u.GYRO_N) = I3 * diag(p.gyro.sigma_N.^2);

% Compute velocity error state.
r_ib_e = double(xhat.r_ix_e) + xhat.dr_xb_e;
A(s.V_IB_E, :) = xhat.R_eb * A_fib;
A(s.V_IB_E, s.THETA_BI) = -xhat.R_eb * vcross(f_ib_b);
A(s.V_IB_E, s.V_IB_E) = -2 * vcross(xhat.omega_ie_e);
A(s.V_IB_E, s.R_IB_E) = xhat.dg_ib_e;
B(s.V_IB_E, u.ACCEL_N) = xhat.R_eb * B_fib;
Q(u.ACCEL_N, u.ACCEL_N) = I3 * diag(p.accel.sigma_N.^2);

% Compute position error state.
A(s.R_IB_E, s.V_IB_E) = I3;

% Store system inputs.
inputs = struct();
inputs.f_ib_b = f_ib_b;
inputs.H_f_ib_b = A_fib;
inputs.omega_ib_b = omega_ib_b;
inputs.H_omega_ib_b = A_wib;
inputs.omega_eb_b = omega_eb_b;
inputs.H_omega_eb_b = A_web;
