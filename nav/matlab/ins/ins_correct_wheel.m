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

function [dx, out] = ins_correct_wheel(p, xhat, dx, inputs, data, out, wheel)

%r_bwheels_b = [p.geometry.r_bwheelfl_b,
%               p.geometry.r_bwheelfr_b,
%               p.geometry.r_bwheelrl_b,
%               p.geometry.r_bwheelrr_b];
%r_bw_b = r_bwheels_b(:, wheel);
%mm = p.meas.WHEEL_VEL(wheel);

%[H, R, dy] = ins_output_wheel(p, xhat, inputs, r_bw_b, data.omega, wheel);
%[dx, out] = ins_dx_correct(p, dx, H, R, dy, 3*3, data.t, out, mm);
