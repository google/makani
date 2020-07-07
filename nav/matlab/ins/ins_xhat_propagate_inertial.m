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

function [xhat] = ins_xhat_propagate_inertial(p, xhat_1, imu)
xhat = struct();
xhat.t = imu.t;
dt = imu.dt;

% Copy constant orientation parameters.
xhat.R_bs = xhat_1.R_bs;

% Propagate accelerometer misalignment navigation state.
xhat.mrp_sa = diag(1 - dt ./ p.accel.tc_misalign) * xhat_1.mrp_sa;

% Propagate accelerometer nonorthogonality navigation state.
xhat.no_a = diag(1 - dt ./ p.accel.tc_ortho) * xhat_1.no_a;

% Propagate accelerometer bias instability error state.
xhat.b_a = diag(1 - dt ./ p.accel.tc_bias) * xhat_1.b_a;

% Propagate accelerometer turn-on bias and random walk navigation state.
xhat.c_a = xhat_1.c_a;

% Propagate accelerometer scale factor error navigation state.
xhat.k_a = diag(1 - dt ./ p.accel.tc_scale) * xhat_1.k_a;

% Propagate accelerometer offset (finite size) navigation state.
xhat.r_ba_ay = diag(1 - dt ./ p.accel.tc_offset) * xhat_1.r_ba_ay;
xhat.r_ba_az = diag(1 - dt ./ p.accel.tc_offset) * xhat_1.r_ba_az;

% Propagate gyro misalignment navigation state.
xhat.mrp_ag = diag(1 - dt ./ p.gyro.tc_misalign) * xhat_1.mrp_ag;

% Propagate gyro nonorthogonality navigation state.
xhat.no_g = diag(1 - dt ./ p.gyro.tc_ortho) * xhat_1.no_g;

% Propagate gyro bias instability navigation state.
xhat.b_g = diag(1 - dt ./ p.gyro.tc_bias) * xhat_1.b_g;

% Propagate gyro turn-on bias and random walk navigation state.
xhat.c_g = xhat_1.c_g;

% Propagate gyro scale factor error navigation state.
xhat.k_g = diag(1 - dt ./ p.gyro.tc_scale) * xhat_1.k_g;

% Propagate gyro g-sensitivity navigation state.
xhat.F_g = diag(1 - dt ./ p.gyro.tc_g_sense) * xhat_1.F_g;

% Compute corrected inertial measurements.
[dvsf_b, phi_b] = ins_correct_imu(xhat, imu);

% Update Earth state.
r_ib_e_1 = double(xhat_1.r_ix_e) + xhat_1.dr_xb_e;
[xhat.g_ib_e, xhat.dg_ib_e] = earth_gravity(r_ib_e_1);
xhat.omega_ie_e = xhat_1.omega_ie_e;

% Integrate attitude.
% q^{bk}_{bk-1}.
phi2 = dot(phi_b, phi_b);
dq_b.s = taylor_series_trig(phi2 / 4, 0);
dq_b.v = -taylor_series_trig(phi2 / 4, 1) * 0.5 * phi_b;

% q^{ek}_{ek-1}.
zeta_e = xhat_1.omega_ie_e * dt;
zeta2 = dot(zeta_e, zeta_e);
dq_e.s = taylor_series_trig(zeta2 / 4, 0);
dq_e.v = taylor_series_trig(zeta2 / 4, 1) * 0.5 * zeta_e;

% q^{ek}_{bk} = q^{ek}_{ek-1} (X) q^{ek-1}_i (X) q^i_{bk-1} (X) q^{bk-1}_{bk}.
xhat.q_eb = q_mult(dq_e, q_mult(xhat_1.q_eb, dq_b));
xhat.R_eb = q_to_dcm(xhat.q_eb);

% Compute gravitational and Coriolis increment.
dGCor_e_1 = (xhat_1.g_ib_e - 2 * cross(xhat_1.omega_ie_e, xhat_1.v_ib_e) ...
    - cross(xhat_1.omega_ie_e, cross(xhat_1.omega_ie_e, r_ib_e_1))) * dt;
dGCor_e = xhat.R_eb * xhat_1.R_eb' * dGCor_e_1;

% Integrate velocity.
dv_ib_e = dGCor_e + xhat_1.R_eb * dvsf_b;
xhat.v_ib_e = xhat_1.v_ib_e + dv_ib_e;

% Integrate position (trapezoidal integration).
xhat.dr_xb_e = xhat_1.dr_xb_e + (xhat_1.v_ib_e + 0.5 * dv_ib_e) * dt;
xhat.r_ix_e = xhat_1.r_ix_e + int32(fix(xhat.dr_xb_e));
xhat.dr_xb_e = xhat.dr_xb_e - fix(xhat.dr_xb_e);
