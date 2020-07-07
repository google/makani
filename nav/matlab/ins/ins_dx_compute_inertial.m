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

function [dx] = ins_dx_compute_inertial(p, xhat0, xhat1)
s = p.states;

% Compute error between two navigation state estimates, or truth source.
dx = zeros(s.count, 1);

% Compute attitude error (scaled to full angles).
% q^e_b = q^e_ehat (X) q^ehat_i (X) q^i_bhat (X) q^bhat_b.
% q^e_b = q^e_bhat (X) q^bhat_b.
% q^e_b (X) q^b_bhat = q^e_bhat.
% q^b_bhat = q_conj(q^e_b) (X) q^e_bhat.
dq_bbhat = q_mult(q_conj(xhat1.q_eb), xhat0.q_eb);
dx(s.THETA_BI) =  q_to_mrp(dq_bbhat) * 4;

% Compute velocity error.
dx(s.V_IB_E) = xhat1.v_ib_e - xhat0.v_ib_e;

% Compute position error.
dx(s.R_IB_E) = double(xhat1.r_ix_e - xhat0.r_ix_e) ...
    + (xhat1.dr_xb_e - xhat0.dr_xb_e);

% Compute accelerometer misalignment error (scaled to full angles).
% p^s_ahat = p^s_a (X) p^a_ahat.
% p^a_ahat = mrp_conj(p^s_a) (X) p^s_ahat.
dx(s.THETA_SA) = mrp_mult(mrp_conj(xhat1.mrp_sa), xhat0.mrp_sa) * 4;

% Compute accelerometer nonorthogonality error.
dx(s.ACCEL_ORTHO) = xhat1.no_a - xhat0.no_a;

% Compute accelerometer bias instability error.
dx(s.ACCEL_BIAS) = xhat1.b_a - xhat0.b_a;

% Compute accelerometer turn-on bias and random walk error.
dx(s.ACCEL_WALK) = xhat1.c_a - xhat0.c_a;

% Compute accelerometer scale factor error.
dx(s.ACCEL_SCALE) = xhat1.k_a - xhat0.k_a;

% Compute accelerometer offset (finite size) error.
dx(s.ACCEL_OFFSET_Y) = xhat1.r_ba_ay - xhat0.r_ba_ay;
dx(s.ACCEL_OFFSET_Z) = xhat1.r_ba_az - xhat0.r_ba_az;

% Compute gyro misalignment error (scaled to full angles).
% p^a_ghat = p^a_g (X) p^g_ghat.
% p^g_ghat = mrp_conj(p^a_g) (X) p^a_ghat.
dx(s.THETA_AG) = mrp_mult(mrp_conj(xhat1.mrp_ag), xhat0.mrp_ag) * 4;

% Compute gyro nonorthogonality error.
dx(s.GYRO_ORTHO) = xhat1.no_g - xhat0.no_g;

% Compute gyro bias instability error.
dx(s.GYRO_BIAS) = xhat1.b_g - xhat0.b_g;

% Compute gyro turn-on bias and random walk error.
dx(s.GYRO_WALK) = xhat1.c_g - xhat0.c_g;

% Compute gyro scale factor error.
dx(s.GYRO_SCALE) = xhat1.k_g - xhat0.k_g;

% Compute gyro g-sensitivity error.
dx(s.GYRO_G_SENSE) = reshape(xhat1.F_g - xhat0.F_g, 9, 1);
