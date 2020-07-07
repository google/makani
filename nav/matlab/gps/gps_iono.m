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

function [T_iono] = gps_iono(iono, lat, lon, elev, azi, t_gps)

% See IS-GPS-200D Figure 20-4.

% Geodetic position in semi-circles.
phi_u = lat / pi;
lambda_u = lon / pi;

% Elevation angle between user and satellite (semi-circles).
E = elev / pi;

% Azimuth angle between user and satellite, CW positive from true north
% (semi-circles).
A = azi / pi;

% Earth's central angle between user position and the earth project of
% ionospheric intersection point (semi-circles).
Psi = 0.0137 / (E + 0.11) - 0.022;

% Geodetic latitude of the earth projection of the ionospheric intersection
% point (semi-circles).
phi_i = sat(phi_u + Psi * cos(A * pi), 0.416);

% Geodetic longitude of the earth projection of the ionospheric intersection
% point (semi-circles).
lambda_i = lambda_u + Psi * sin(A * pi) / cos(phi_i * pi);

% Geomagnetic latitude of the earth projection of the ionospheric
% intersection point (semi-circles).
phi_m = phi_i + 0.064 * cos((lambda_i - 1.617) * pi);

% Local time (sec).
t = 4.32e4 * lambda_i + t_gps;
if t >= 86400
  t = t - 86400;
elseif t < 0
  t = t + 86400;
end

% Obliquity factor (dimensionless).
F = 1.0 + 16.0 * (0.53 - E)^3;

% Period.
PER = iono.beta0 * phi_m^0 + iono.beta1 * phi_m^1 + iono.beta2 * phi_m^2 ...
    + iono.beta3 * phi_m^3;
if PER < 72000
  PER = 72000;
end

% Phase (radians).
x = 2 * pi * (t - 50400) / PER;

% Amplitude of delay.
AMP = iono.alpha0 * phi_m^0 + iono.alpha1 * phi_m^1 + iono.alpha2 * phi_m^2 ...
    + iono.alpha3 * phi_m^3;
if AMP < 0
  AMP = 0;
end

% L1 ionosphere correction.
if abs(x) < 1.57
  T_iono = F * (5e-9 + AMP * (1 - x^2/2 + x^4/24));
else
  T_iono = F * (5e-9);
end
