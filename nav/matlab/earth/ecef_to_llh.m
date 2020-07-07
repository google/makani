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

function [lat, lon, height, R_e2l] = ecef_to_llh(ecef)
earth = earth_wgs84();
x = ecef(1);
y = ecef(2);
z = ecef(3);

% Initial conditions.
h = 0;
N = earth.a;
p = norm([x, y]);
lambda = 0;

% Iterate.
for iter = 1:10
  s_lambda = z / (N * (1.0 - earth.e2) + h);
  lambda = atan2((z + earth.e2 * N * s_lambda), p);
  N = earth.a / sqrt(1.0 - earth.e2 * s_lambda^2);
  h = p / cos(lambda) - N;
end
lat = lambda;
lon = atan2(y, x);
height = h;

% Compute Earth to level rotation.
R_e2l = [-sin(lat)*cos(lon), -sin(lat)*sin(lon), cos(lat);
         -sin(lon), cos(lon), 0;
         -cos(lat)*cos(lon), -cos(lat)*sin(lon), -sin(lat)];
