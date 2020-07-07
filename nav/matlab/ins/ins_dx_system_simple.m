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

function [A, B, Q, inputs] = ins_dx_system_simple(p, xhat, imu)
s = p.states;
u = p.inputs;

% Allocate memory.
A = zeros(s.count, s.count);
B = zeros(s.count, u.count);
Q = zeros(u.count, u.count);
I3 = eye(3);

% Raw inertial measurements.
ya_a = imu.dvsf' / imu.dt;
yg_g = imu.phi' / imu.dt;

% Compute helpful quantities.
r_ib_e = double(xhat.r_ix_e) + xhat.dr_xb_e;
f_ib_b = xhat.R_bs * (ya_a - xhat.b_a - xhat.c_a);
omega_ib_b = xhat.R_bs * (yg_g - xhat.b_g - xhat.c_g);
omega_eb_b = omega_ib_b - xhat.R_eb' * xhat.omega_ie_e;

% Attitude.
A(s.THETA_BI, s.THETA_BI) = -vcross(omega_ib_b);
A(s.THETA_BI, s.GYRO_BIAS) = -xhat.R_bs;
A(s.THETA_BI, s.GYRO_WALK) = -xhat.R_bs;
B(s.THETA_BI, u.GYRO_N) = -xhat.R_bs;
Q(u.GYRO_N, u.GYRO_N) = I3 * diag(p.gyro.sigma_N.^2);

% Velocity.
A(s.V_IB_E, s.THETA_BI) = -xhat.R_eb * vcross(f_ib_b);
A(s.V_IB_E, s.V_IB_E) = -2 * vcross(xhat.omega_ie_e);
A(s.V_IB_E, s.R_IB_E) = xhat.dg_ib_e;
A(s.V_IB_E, s.ACCEL_BIAS) = -xhat.R_eb * xhat.R_bs;
A(s.V_IB_E, s.ACCEL_WALK) = -xhat.R_eb * xhat.R_bs;
B(s.V_IB_E, u.ACCEL_N) = -xhat.R_eb * xhat.R_bs;
Q(u.ACCEL_N, u.ACCEL_N) = I3 * diag(p.accel.sigma_N.^2);

% Position.
A(s.R_IB_E, s.V_IB_E) = I3;

% Accelerometer bias instability.
A(s.ACCEL_BIAS, s.ACCEL_BIAS) = -I3 * diag(1 ./ p.accel.tc_bias);
B(s.ACCEL_BIAS, u.ACCEL_B) = I3;
Q(u.ACCEL_B, u.ACCEL_B) = I3 * diag(p.accel.qc_bias.^2);

% Accelerometer random walk.
B(s.ACCEL_WALK, u.ACCEL_K) = I3;
Q(u.ACCEL_K, u.ACCEL_K) = I3 * diag(p.accel.sigma_K.^2);

% Gyro bias instability.
A(s.GYRO_BIAS, s.GYRO_BIAS) = -I3 * diag(1 ./ p.gyro.tc_bias);
B(s.GYRO_BIAS, u.GYRO_B) = I3;
Q(u.GYRO_B, u.GYRO_B) = I3 * diag(p.gyro.qc_bias.^2);

% Gyro random walk.
B(s.GYRO_WALK, u.GYRO_K) = I3;
Q(u.GYRO_K, u.GYRO_K) = I3 * diag(p.gyro.sigma_K.^2);

% Store system inputs.
inputs = struct();

inputs.f_ib_b = f_ib_b;
inputs.H_f_ib_b = zeros(3, s.count);
inputs.H_f_ib_b(:, s.ACCEL_BIAS) = -xhat.R_bs;
inputs.H_f_ib_b(:, s.ACCEL_WALK) = -xhat.R_bs;

inputs.omega_ib_b = omega_ib_b;
inputs.H_omega_ib_b = zeros(3, s.count);
inputs.H_omega_ib_b(:, s.GYRO_BIAS) = -xhat.R_bs;
inputs.H_omega_ib_b(:, s.GYRO_WALK) = -xhat.R_bs;

inputs.omega_eb_b = omega_eb_b;
inputs.H_omega_eb_b = zeros(3, s.count);
inputs.H_omega_eb_b(:, s.GYRO_BIAS) = -xhat.R_bs;
inputs.H_omega_eb_b(:, s.GYRO_WALK) = -xhat.R_bs;
inputs.H_omega_eb_b(:, s.THETA_BI) = -vcross(xhat.R_eb' * xhat.omega_ie_e);
