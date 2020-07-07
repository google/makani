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

function [g_ib_e, dg_ib_e] = earth_gravity(r_ib_e)
earth = earth_wgs84();
x = r_ib_e(1);
y = r_ib_e(2);
z = r_ib_e(3);

% Earth EGM96 constants.
omega_ie_e = [0; 0; earth.omega];
mu = earth.mu;
a = earth.a;
cbar20 = -0.484165371736e-03;
cbar21 = -0.186987635955e-09;
sbar21 = 0.119528012031e-08;
cbar22 = 0.243914352398e-05;
sbar22 = -0.140016683654e-05;

% Denormalize spherical harmonic coefficients.
c20 = cbar20 / legendre_normalization(2, 0);
c21 = cbar21 / legendre_normalization(2, 1);
s21 = sbar21 / legendre_normalization(2, 1);
c22 = cbar22 / legendre_normalization(2, 2);
s22 = sbar22 / legendre_normalization(2, 2);

% Compute geocentric spherical position.
r = max(norm([x, y, z]), earth.a * 0.75);
phi_p = asin(z / r);
lambda = atan2(y, x);

% Partials of gravitational potential w.r.t. geocentric coordinates.
dV_dr = - mu/r^2 - 3*mu/r^2 * (a/r)^2 * ( ...
  1/2 * (3 * sin(phi_p)^2 - 1) * c20 ...
  + 3 * sin(phi_p) * cos(phi_p) * (c21 * cos(lambda) + s21 * sin(lambda)) ...
  + 3 * cos(phi_p)^2 * (c22 * cos(2*lambda) + s22 * sin(2*lambda)));
dV_dphi = mu/r * (a/r)^2 * ( ...
  3 * sin(phi_p) * cos(phi_p) * c20 ...
  + 3 * cos(2 * phi_p) * (c21 * cos(lambda) + s21 * sin(lambda)) ...
  - 6 * sin(phi_p) * cos(phi_p) * (c22 * cos(2*lambda) + s22 * sin(2*lambda)));
dV_dlambda = mu/r * (a/r)^2 * ( ...
  3 * sin(phi_p) * cos(phi_p) * (s21 * cos(lambda) - c21 * sin(lambda)) ...
  + 6 * cos(phi_p)^2 * (s22 * cos(2*lambda) - c22 * sin(2*lambda)));

% Partials of geocentric coordinates w.r.t. ECEF coordinates.
dr_dx = x / r;
dr_dy = y / r;
dr_dz = z / r;
if x == 0
  dphi_dx = 0;
  dlambda_dy = 0;
else
  dphi_dx = - x * z / (r^2 * sqrt(x^2 + y^2));
  dlambda_dy = x / (x^2 + y^2);
end
if y == 0
  dphi_dy = 0;
  dlambda_dx = 0;
else
  dphi_dy = - y * z / (r^2 * sqrt(x^2 + y^2));
  dlambda_dx = - y / (x^2 + y^2);
end
dphi_dz = sqrt(x^2 + y^2) / r^2;
dlambda_dz = 0;

% Partials of gravitational potential w.r.t. ECEF coordinates.
dV_dx = dV_dr * dr_dx + dV_dphi * dphi_dx + dV_dlambda * dlambda_dx;
dV_dy = dV_dr * dr_dy + dV_dphi * dphi_dy + dV_dlambda * dlambda_dy;
dV_dz = dV_dr * dr_dz + dV_dphi * dphi_dz + dV_dlambda * dlambda_dz;

% Gravitational acceleration in ECEF coordinates.
dV = [dV_dx; dV_dy; dV_dz];

% Plumb gravity.
g_ib_e = dV - cross(omega_ie_e, cross(omega_ie_e, r_ib_e));

% Plumb gravity position partial (spherical approximation).
ddV_dr = earth.mu / r^5 * (3 * r_ib_e * r_ib_e' - eye(3) * r^2);
dg_ib_e = ddV_dr - vcross(omega_ie_e) * vcross(omega_ie_e);



function [f] = legendre_normalization(n, m)
k = 1 + (m > 0);
f = sqrt(factorial(n + m) / (factorial(n - m) * (2 * n + 1) * k));
