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

function [ecef] = llh_to_ecef(lat, lon, h)
earth = earth_wgs84();

N = earth.a / sqrt(1 - earth.e2 * sin(lat)^2);
ecef = [(N + h) * cos(lat) * cos(lon);
        (N + h) * cos(lat) * sin(lon);
        (N * (1 - earth.e2) + h) * sin(lat)];
