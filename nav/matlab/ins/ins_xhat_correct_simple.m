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

function [xhat] = ins_xhat_correct_simple(p, xhat_1, dx_plus)
s = p.states;
xhat = struct();
xhat.t = xhat_1.t;

% Copy constant orientation parameters.
xhat.R_bs = xhat_1.R_bs;

% Copy unused states from full model.
xhat.mrp_sa = xhat_1.mrp_sa;
xhat.no_a = xhat_1.no_a;
xhat.k_a = xhat_1.k_a;
xhat.r_ba_ay = xhat_1.r_ba_ay;
xhat.r_ba_az = xhat_1.r_ba_az;
xhat.mrp_ag = xhat_1.mrp_ag;
xhat.no_g = xhat_1.no_g;
xhat.k_g = xhat_1.k_g;
xhat.F_g = xhat_1.F_g;

% Correct attitude error.
% q^e_b = q^e_ehat (X) q^ehat_i (X) q^i_bhat (X) q^bhat_b.
% q^e_b = q^e_bhat (X) q^bhat_b.
dq_bbhat = mrp_to_q(dx_plus(s.THETA_BI) / 4);
xhat.q_eb = q_normalize(q_mult(xhat_1.q_eb, q_conj(dq_bbhat)));
xhat.R_eb = q_to_dcm(xhat.q_eb);

% Correct velocity error.
xhat.v_ib_e = xhat_1.v_ib_e + dx_plus(s.V_IB_E);

% Correct position error.
xhat.dr_xb_e = xhat_1.dr_xb_e + dx_plus(s.R_IB_E);
xhat.r_ix_e = xhat_1.r_ix_e + int32(fix(xhat.dr_xb_e));
xhat.dr_xb_e = xhat.dr_xb_e - fix(xhat.dr_xb_e);

% Correct accelerometer bias instability error.
xhat.b_a = xhat_1.b_a + dx_plus(s.ACCEL_BIAS);

% Correct accelerometer turn-on bias and random walk error.
xhat.c_a = xhat_1.c_a + dx_plus(s.ACCEL_WALK);

% Correct gyro bias instability error.
xhat.b_g = xhat_1.b_g + dx_plus(s.GYRO_BIAS);

% Correct gyro turn-on bias and random walk error.
xhat.c_g = xhat_1.c_g + dx_plus(s.GYRO_WALK);

% Update Earth quantities.
xhat.g_ib_e = earth_gravity(double(xhat.r_ix_e) + xhat.dr_xb_e);
xhat.omega_ie_e = xhat_1.omega_ie_e;
