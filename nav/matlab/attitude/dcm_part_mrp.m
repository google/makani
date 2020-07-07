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

function [dRv_dp, dRtv_dp] = dcm_part_mrp(p, v)

dpxpxv_dp = [p(2)*v(2) + p(3)*v(3),   p(1)*v(2) - 2*p(2)*v(1), p(1)*v(3) - 2*p(3)*v(1);
             p(2)*v(1) - 2*p(1)*v(2), p(1)*v(1) + p(3)*v(3),   p(2)*v(3) - 2*p(3)*v(2);
             p(3)*v(1) - 2*p(1)*v(3), p(3)*v(2) - 2*p(2)*v(3), p(1)*v(1) + p(2)*v(2)];

dsx1_dp = 8 * cross(p, v) * p' / (1 + p'*p)^2;
dsx2_dp = 16 * (1 - p'*p) * cross(p, v) * p' / (1 + p'*p)^3;
dsx3_dp = 4 * (1 - p'*p) * vcross(v) / (1 + p'*p)^2;
dsx_dp = dsx1_dp + dsx2_dp + dsx3_dp;

dcx1_dp = -32 * vcross(p)^2 * v * p' / (1 + p'*p)^3;
dcx2_dp = 8 * dpxpxv_dp / (1 + p'*p)^2;
dcx_dp = dcx1_dp + dcx2_dp;

dRv_dp = dsx_dp + dcx_dp;
dRtv_dp = -dsx_dp + dcx_dp;