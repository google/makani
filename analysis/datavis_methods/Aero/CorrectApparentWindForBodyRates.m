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

function [apparent_wind_out] = CorrectApparentWindForBodyRates(apparent_wind, pitot_position, p, q, r)
% CorrectApparentWindForBodyRates -- body rate correction to pitot apparent wind.
% apparent_wind_out = CorrectApparentWindForBodyRates(apparent_wind, p, q, r)
%
% Arguments-
%
% apparent_wind  -- [3 x n] apparent wind vector [m/s]
% pitot_position -- [1 x 3] position of pitot in body coordinates [m]
% p              -- [1 x 3] roll rate,  about x-axis [rad/s]
% q              -- [1 x 3] pitch rate, about y-axis [rad/s]
% r              -- [1 x 3] yaw rate,   about z-axis [rad/s]
%
% Output Values-
%
% apparent_wind_out -- [3 x n] apparent wind vector [m/s]
%                              with body rate corrections
%

% Collect body rates for body rotation matrix [3 x n].
rotation_rate = [p; q; r];

% Apply correction for body rates experienced at pitot position
apparent_wind_out = apparent_wind - cross(pitot_position .* ones(size(rotation_rate)), rotation_rate);