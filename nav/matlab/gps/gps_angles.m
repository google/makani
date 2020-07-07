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

function [lat, lon, height, elev, azi] = gps_angles(r_u_ecefeci, r_s_eci)

% See IS-GPS-200D Figure 20-4.

% Geodetic position (WGS-84).
[lat, lon, height, R_le] = ecef_to_llh(r_u_ecefeci);

% User and satellite positions in local level coordinate system.
r_u_level = R_le * r_u_ecefeci;
r_s_level = R_le * r_s_eci;
dr = r_s_level - r_u_level;

% Elevation angle between user and satellite.
elev = atan2(-dr(3), norm(dr(1:2)));

% Azimuth angle between user and satellite, CW positive from true north.
azi = atan2(dr(2), dr(1));
