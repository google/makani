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

function [wgs84] = earth_wgs84()

wgs84.a = 6378137.0;            % Semi-major axis [m].
wgs84.inv_f = 298.257223563;    % Flattening [#].
wgs84.omega = 7.2921151467e-5;  % Angular velocity [rad/s]
wgs84.mu = 3986004.418e8;       % Gravitational constant [m^3/s^2].
wgs84.e = 8.1819190842622e-2;   % First eccentricity [#].
wgs84.e2 = 6.69437999014e-3;    % First eccentricity squared [#].
wgs84.c = 299792458;            % Speed of light [m/s].
