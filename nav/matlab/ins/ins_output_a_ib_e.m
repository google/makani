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

function [H, R, dy] = ins_output_a_ib_e(p, xhat, inputs, y, sigma)
s = p.states;

H = zeros(3, s.count);
H(:, s.THETA_BI) = -xhat.R_eb * vcross(inputs.f_ib_b);
H(:, s.V_IB_E) = -2 * vcross(xhat.omega_ie_e);
H(:, s.R_IB_E) = xhat.dg_ib_e;
H = H + xhat.R_eb * inputs.H_f_ib_b;

R = sigma * eye(3) * sigma';

yhat = xhat.R_eb * inputs.f_ib_b + xhat.g_ib_e ...
    - 2 * cross(xhat.omega_ie_e, xhat.v_ib_e);
dy = y - yhat;
