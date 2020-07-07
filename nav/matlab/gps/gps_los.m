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

function [sv] = gps_los(eph, iono, r_u_ecef, b_u, v_u_ecef, f_u, t_recv)
earth = earth_wgs84();

% Compute user time.
t_user = t_recv - b_u / earth.c;

% Compute user and satellite position in ECI coordinates.
[dt, r_u_eci, v_u_eci, r_s_eci, v_s_eci, azi, elev] = ...
    gps_delay(eph, iono, r_u_ecef, v_u_ecef, t_user);

% Compute line-of-sight unit vector.
dr_eci = r_s_eci - r_u_eci;
los_eci = dr_eci' / norm(dr_eci);
rho = norm(dr_eci);

% Compute pseudo-range and carrier phase measurement estimates.
pr = rho + earth.c * (-dt.clock + dt.iono + dt.tropo + dt.gd) + b_u;
cp = rho + earth.c * (-dt.clock - dt.iono + dt.tropo + dt.gd) + b_u;

% Compute derivative of line-of-sight unit vector.
dv_eci = v_s_eci - v_u_eci;
dlos_eci = dv_eci' / norm(dr_eci) ...
    - (dr_eci' * dv_eci * dr_eci') / (dr_eci' * dr_eci)^(3/2);
rhodot = dot(dv_eci, los_eci);

% Compute Doppler measurement estimate.
do = rhodot + earth.c * (-dt.clockdot) + f_u;

% Output structure.
sv = struct();
sv.prn = eph.prn;
sv.delay = dt;
sv.elev = elev;
sv.azimuth = azi;

% Output structure (position).
sv.r_u_eci = r_u_eci;
sv.r_s_eci = r_s_eci;
sv.dr_eci = dr_eci;
sv.los_eci = los_eci;
sv.rho = rho;
sv.pr = pr;
sv.cp = cp;
sv.ura = eph.ura;

% Output structure (velocity).
sv.v_u_eci = v_u_eci;
sv.v_s_eci = v_s_eci;
sv.dv_eci = dv_eci;
sv.dlos_eci = dlos_eci;
sv.rhodot = rhodot;
sv.do = do;
