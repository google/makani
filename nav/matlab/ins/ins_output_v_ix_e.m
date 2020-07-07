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

function [H, R, dy] = ins_output_v_ix_e(p, xhat, inputs, r_bx_b, y, sigma)
s = p.states;

H = zeros(3, s.count);
H(:, s.THETA_BI) = -xhat.R_eb * vcross(cross(inputs.omega_eb_b, r_bx_b));
H(:, s.V_IB_E) = eye(3);
H = H - xhat.R_eb * vcross(r_bx_b) * inputs.H_omega_eb_b;

R = sigma * eye(3) * sigma';

yhat = xhat.v_ib_e + xhat.R_eb * cross(inputs.omega_eb_b, r_bx_b);
dy = y - yhat;
