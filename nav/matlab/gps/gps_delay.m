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

function [dt, r_u_eci, v_u_eci, r_s_eci, v_s_eci, azi, elev] = ...
    gps_delay(eph, iono, r_u_ecef, v_u_ecef, t_user)
earth = earth_wgs84();

E_k = eph.m_0;
Edot_k = 0;
dt_path = 0.075;
iterations = 10;
for iter = 1:iterations
  dt_path_old = dt_path;

  % Compute the satellite transmition time.
  t_sv = t_user - dt_path;

  % Compute GPS system time.
  % See IS-GPS-200H: 20.3.3.3.3.1 User Algorithm for SV Clock Correction.
  [dt_clock, dt_clockdot] = gps_clock(eph, E_k, Edot_k, t_sv);
  t_gps = t_sv - dt_clock;

  % Compute satellite position given GPS system time.
  [r_s_ecef, v_s_ecef, E_k, Edot_k] = gps_position(eph, t_gps);

  % Compute satellite and user position in ECI coordinates to estimate range.
  theta = earth.omega * dt_path;
  [r_s_eci, v_s_eci] = ecef_to_eci(r_s_ecef, theta, v_s_ecef, earth.omega);
  [r_u_eci, v_u_eci] = ecef_to_eci(r_u_ecef, 0, v_u_ecef, earth.omega);

  % Compute satellite angles.
  [lat, lon, height, elev, azi] = gps_angles(r_u_eci, r_s_eci);

  % Compute transmission delay estimates from models.
  dt_geo = norm(r_s_eci - r_u_eci) / earth.c;
  dt_iono = gps_iono(iono, lat, lon, elev, azi, t_gps);
  dt_tropo = 0;  % TBD.

  % Compute total transmition delay (assuming L1-only receiver).
  % See IS-GPS-200H: Figure 20-3 Sample Application of Correction Parameters.
  dt_path = dt_geo + dt_iono + dt_tropo + eph.t_gd;

  % Check for convergence (1 ns ~= 0.3 m).
  if abs(dt_path - dt_path_old) < 1e-16
    break;
  end
end

% Output structure.
dt = struct();
dt.clock = dt_clock;
dt.clockdot = dt_clockdot;
dt.path = dt_path;
dt.geo = dt_geo;
dt.iono = dt_iono;
dt.tropo = dt_tropo;
dt.gd = eph.t_gd;
