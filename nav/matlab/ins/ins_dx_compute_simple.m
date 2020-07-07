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

function [dx] = ins_dx_compute_simple(p, xhat0, xhat1);
s = p.states;
dx = zeros(s.count, 1);

% Compute attitude error.
% q^e_b = q^e_ehat (X) q^ehat_i (X) q^i_bhat (X) q^bhat_b.
% q^e_b = q^e_bhat (X) q^bhat_b.
% q^e_b (X) q^b_bhat = q^e_bhat.
% q^b_bhat = q_conj(q^e_b) (X) q^e_bhat.
dq_bbhat = q_mult(q_conj(xhat1.q_eb), xhat0.q_eb);
dx(s.THETA_BI) = q_to_mrp(dq_bbhat) * 4;

% Compute velocity error.
dx(s.V_IB_E) = xhat1.v_ib_e - xhat0.v_ib_e;

% Compute position error.
dx(s.R_IB_E) = double(xhat1.r_ix_e - xhat0.r_ix_e) ...
    + (xhat1.dr_xb_e - xhat0.dr_xb_e);

% Compute accelerometer bias instability error.
dx(s.ACCEL_BIAS) = xhat1.b_a - xhat0.b_a;

% Compute accelerometer turn-on bias and random walk error.
dx(s.ACCEL_WALK) = xhat1.c_a - xhat0.c_a;

% Compute gyro bias instability error.
dx(s.GYRO_BIAS) = xhat1.b_g - xhat0.b_g;

% Compute gyro turn-on bias and random walk error.
dx(s.GYRO_WALK) = xhat1.c_g - xhat0.c_g;
