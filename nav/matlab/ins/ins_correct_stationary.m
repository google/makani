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

function [dx, out] = ins_correct_stationary(p, xhat, dx, inputs, out)

y = zeros(3, 1);
sigma = 0.005;
[H, R, dy] = ins_output_omega_eb_b(p, xhat, inputs, y, sigma);
[dx, out] = ins_dx_correct(p, dx, H, R, dy, 3*3, xhat.t, out, p.meas.ZOMEGA);

%y = zeros(3, 1);
%sigma = 0.1;
%[H, R, dy] = ins_output_a_ib_e(p, xhat, inputs, y, sigma);
%[dx, out] = ins_dx_correct(p, dx, H, R, dy, 3*3, xhat.t, out, p.meas.ZACCEL);

y = zeros(3, 1);
r_bx_b = zeros(3, 1);
sigma = 0.1;
[H, R, dy] = ins_output_v_ix_b(p, xhat, inputs, r_bx_b, y, sigma);
[dx, out] = ins_dx_correct(p, dx, H, R, dy, 3*3, xhat.t, out, p.meas.ZVEL);
