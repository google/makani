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

function [speed, alpha, beta] = ApparentWindCartToSph(apparent_wind)
% ApparentWindCartToSph -- apparent wind vector to speed, alpha and beta.
% [speed, alpha, beta] = ApparentWindCartToSph(apparent_wind)
% Based on namesake function in control/sensor_util.c
% Incidence angles follow conventions from Etkin (pg. 11):
%
%   alpha = arctan(w / u)
%   beta  = arcsin(v / |V|)
%
% where (u, v, w) are the cartesian components of the body velocity,
% i.e. opposite the apparent wind.
%
% Arguments-
% apparent_wind: [3 x n] apparent wind vector [Vx;Vy;Vz] [m/s]
%
% Output Values-
% speed: [1 x n]: speed [m/s]
% alpha: [1 x n]: angle of attack [rad]
% beta : [1 x n]: angle of sideslip [rad]

speed = sqrt(sum(apparent_wind.^2));
alpha = atan2(-apparent_wind(3,:), -apparent_wind(1,:));

% The following is equivalent to asin(-apparent_wind(2,:) / speed) but
% avoids the potential for division by zero at zero airspeed.
beta  = atan2(-apparent_wind(2,:), hypot(apparent_wind(1,:), apparent_wind(3,:)));
