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

function [H, R, dy] = ins_output_omega_eb_b(p, xhat, inputs, y, sigma)
s = p.states;

H = zeros(3, s.count);
H(:, s.THETA_BI) = -vcross(xhat.R_eb' * xhat.omega_ie_e);
H = H + inputs.H_omega_ib_b;

R = sigma * eye(3) * sigma';

yhat = inputs.omega_eb_b;
dy = y - yhat;
