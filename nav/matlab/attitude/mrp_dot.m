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

function [pdot, dpdot_dp, dpdot_domega] = mrp_dot(p, omega)
B = ((1 - p'*p) * eye(3) + 2 * vcross(p) + 2*p*p') / 4;
pdot = B * omega;
dpdot_dp = (p' * omega * eye(3) - omega * p' - vcross(omega) + p * omega') / 2;
dpdot_domega = B;
