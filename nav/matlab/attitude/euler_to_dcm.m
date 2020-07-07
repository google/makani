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

function [R] = euler_to_dcm(euler)
cphi = cos(euler(1));
sphi = sin(euler(1));
ctheta = cos(euler(2));
stheta = sin(euler(2));
cpsi = cos(euler(3));
spsi = sin(euler(3));

R = zeros(3, 3);
R(1, 1) = ctheta * cpsi;
R(1, 2) = ctheta * spsi;
R(1, 3) = -stheta;
R(2, 1) = cpsi * stheta * sphi - cphi * spsi;
R(2, 2) = cphi * cpsi + stheta * sphi * spsi;
R(2, 3) = ctheta * sphi;
R(3, 1) = cphi * cpsi * stheta + sphi * spsi;
R(3, 2) = -cpsi * sphi + cphi * stheta * spsi;
R(3, 3) = ctheta * cphi;
