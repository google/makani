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

function [out] = ins_log_state(ins, index, out)
xhat = ins_xhat_correct(ins.param, ins.xhat, ins.dx.dx_minus);
r_ib_e = double(xhat.r_ix_e) + xhat.dr_xb_e;
[lat, lon, height, R_le] = ecef_to_llh(r_ib_e);
R_lb = R_le * xhat.R_eb;

% Output time/status.
out.iter(index, :) = ins.iter;
out.init(index, :) = ins.init;

% Output position.
out.r_ib_e(index, :) = r_ib_e';
out.lat(index, :) = lat;
out.lon(index, :) = lon;
out.height(index, :) = height;

% Output GPS antenna position.
antenna_r_igps_e = ins_gps_antenna(ins.param, xhat, ins.inputs);
[antenna_lat, antenna_lon, antenna_height] = ecef_to_llh(antenna_r_igps_e);
out.antenna_r_igps_e(index, :) = antenna_r_igps_e';
out.antenna_lat(index, :) = antenna_lat;
out.antenna_lon(index, :) = antenna_lon;
out.antenna_height(index, :) = antenna_height;

% Output velocity.
out.v_ib_e(index, :) = xhat.v_ib_e';
out.v_ib_l(index, :) = (R_le * xhat.v_ib_e)';
out.v_ib_b(index, :) = (xhat.R_eb' * xhat.v_ib_e)';

% Output acceleration.
a_ib_b = ins.inputs.f_ib_b ...
    + xhat.R_eb' * (xhat.g_ib_e - 2 * cross(xhat.omega_ie_e, xhat.v_ib_e));
out.a_ib_e(index, :) = (xhat.R_eb * a_ib_b)';
out.a_ib_l(index, :) = (R_lb * a_ib_b)';
out.a_ib_b(index, :) = a_ib_b';

% Output inertial inputs.
out.f_ib_b(index, :) = ins.inputs.f_ib_b';
out.omega_ib_b(index, :) = ins.inputs.omega_ib_b';

% Output attitude.
q_be = q_conj(xhat.q_eb);
out.mrp_be(index, :) = q_to_mrp(q_be)';
out.theta_be(index, :) = out.mrp_be(index, :)' * 4;
out.q_eb(index, :) = q_struct2vector(xhat.q_eb)';
out.q_lb(index, :) = q_struct2vector(dcm_to_q(R_lb))';
out.euler(index, :) = dcm_to_euler(R_lb')';
x_b = [1; 0; 0];
x_l = R_lb * x_b;
out.azimuth(index, :) = atan2(x_l(2), x_l(1));

% Output accelerometer parameters.
out.b_a(index, :) = xhat.b_a';
out.c_a(index, :) = xhat.c_a';
out.k_a(index, :) = xhat.k_a';
out.mrp_sa(index, :) = xhat.mrp_sa';
out.no_a(index, :) = xhat.no_a';
out.r_ba_ay(index, :) = xhat.r_ba_ay';
out.r_ba_az(index, :) = xhat.r_ba_az';

% Output gyro parameters.
out.b_g(index, :) = xhat.b_g';
out.c_g(index, :) = xhat.c_g';
out.k_g(index, :) = xhat.k_g';
out.mrp_ag(index, :) = xhat.mrp_ag';
out.no_g(index, :) = xhat.no_g';
out.F_g(index, :) = reshape(xhat.F_g, 9, 1)';

% Output wheel encoder parameters.
out.radius(index, :) = ins.param.geometry.wheel_radius + xhat.dradius';

% Output GPS parameters.
out.cb_phase(index, :) = xhat.cb_phase';
out.cf_bias(index, :) = xhat.cf_bias';
out.cf_walk(index, :) = xhat.cf_walk';
out.gps_pr(index, :) = xhat.gps_pr';

% Output covariance.
out.sigma_xx(index, :) = sqrt(diag(ins.dx.Pxx_minus))';
