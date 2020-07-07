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

function [r_ecef, v_ecef, E_k, Edot_k] = gps_position(eph, t_transmit)
earth = earth_wgs84();

% See IS-GPS-200D Table 20-IV.

% WGS84 value of the earth's gravitational constant for GPS user.
mu = 3.986005e14;

% Semi-major axis.
A = eph.sqrt_a^2;

% Time from ephemeris reference epoch.
t_k = wrap_tgps(t_transmit - eph.t_oe);

% Computed mean motion (rad/s).
n_0 = sqrt(mu/A^3);

% Corrected mean motion.
n = n_0 + eph.delta_n;

% Mean anomaly.
M_k = eph.m_0 + n*t_k;

% Solve Kepler's equation for eccentric anomaly.
E_k = kepler_anomaly(eph.ecc, M_k);

% True anomaly.
v_k = atan2(sqrt(1 - eph.ecc^2) * sin(E_k), cos(E_k) - eph.ecc);

% Argument of latitude.
Phi_k = v_k + eph.omega;

% Second harmonic perturbation.
delta_u_k = eph.c_us * sin(2 * Phi_k) + eph.c_uc * cos(2 * Phi_k);
delta_r_k = eph.c_rs * sin(2 * Phi_k) + eph.c_rc * cos(2 * Phi_k);
delta_i_k = eph.c_is * sin(2 * Phi_k) + eph.c_ic * cos(2 * Phi_k);

% Corrected argument of latitude.
u_k = Phi_k + delta_u_k;

% Corrected radius.
r_k = A * (1 - eph.ecc * cos(E_k)) + delta_r_k;

% Corrected inclination.
i_k = eph.i_0 + delta_i_k + eph.i_dot * t_k;

% Positions in orbital plane.
x_k_prime = r_k * cos(u_k);
y_k_prime = r_k * sin(u_k);

% Corrected longitude of ascending node.
Omega_k = eph.omega_0 + (eph.omega_dot - earth.omega)*t_k ...
    - earth.omega*eph.t_oe;

% Earth-fixed coordinates.
x_k = x_k_prime * cos(Omega_k) - y_k_prime * cos(i_k) * sin(Omega_k);
y_k = x_k_prime * sin(Omega_k) + y_k_prime * cos(i_k) * cos(Omega_k);
z_k = y_k_prime * sin(i_k);
r_ecef = [x_k; y_k; z_k];

% Satellite velocity estimation.
% Benjamin W. Remondi, February 2004
% http://www.ngs.noaa.gov/gps-toolbox/bc_velo/bc_velo.c

% Derivative of mean anomaly.
Mdot_k = n;

% Derivative of eccentric anomaly.
Edot_k = Mdot_k/(1.0 - eph.ecc * cos(E_k));

% Derivative of true anomaly.
vdot_k = sin(E_k) * Edot_k * (1 + eph.ecc * cos(v_k)) ...
    / (sin(v_k) * (1 - eph.ecc * cos(E_k)));

% Derivative of corrected argument of latitude.
udot_k = vdot_k + 2 * (eph.c_us * cos(2 * u_k) - eph.c_uc * sin(2 * u_k)) ...
    * vdot_k;

% Derivative of corrected radius.
rdot_k = A * eph.ecc * sin(E_k) * n / (1.0 - eph.ecc * cos(E_k)) ...
    + 2 * (eph.c_rs * cos(2 * u_k) - eph.c_rc * sin(2 * u_k)) * vdot_k;

% Derivative of corrected inclination.
idot_k = eph.i_dot + (eph.c_is * cos(2 * u_k) - eph.c_ic * sin(2 * u_k)) ...
    * 2 * vdot_k;

% Derivative of positions in orbital plane.
xdot_k_prime = rdot_k * cos(u_k) - y_k_prime * udot_k;
ydot_k_prime = rdot_k * sin(u_k) + x_k_prime * udot_k;

% Derivative of corrected longitude of ascending node.
OmegaDot_k = eph.omega_dot - earth.omega;

% Derivative of earth-fixed coordinates.
xdot_k = (xdot_k_prime - y_k_prime * cos(i_k) * OmegaDot_k) * cos(Omega_k) ...
    - (x_k_prime * OmegaDot_k + ydot_k_prime * cos(i_k) ...
    - y_k_prime * sin(i_k) * idot_k) * sin(Omega_k);
ydot_k = (xdot_k_prime - y_k_prime * cos(i_k) * OmegaDot_k) * sin(Omega_k) ...
    + (x_k_prime * OmegaDot_k + ydot_k_prime * cos(i_k) - y_k_prime ...
    * sin(i_k) * idot_k) * cos(Omega_k);
zdot_k = ydot_k_prime * sin(i_k) + y_k_prime * cos(i_k) * idot_k;
v_ecef = [xdot_k; ydot_k; zdot_k];
