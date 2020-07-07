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

function [r_eci, v_eci] = ecef_to_eci(r_ecef, theta, v_ecef, omega)
T_eci2ecef = [cos(theta),  sin(theta), 0;
              -sin(theta), cos(theta), 0;
              0,           0,          1];
r_eci = T_eci2ecef'*r_ecef;

if nargin > 2
  Tdot_eci2ecef = [-omega*sin(theta), omega*cos(theta),  0;
                   -omega*cos(theta), -omega*sin(theta), 0;
                   0,                 0,                 0];
  v_eci = T_eci2ecef'*v_ecef + Tdot_eci2ecef'*r_ecef;
end
