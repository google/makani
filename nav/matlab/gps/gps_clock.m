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

function [dt_clock, dt_clockdot] = gps_clock(eph, E_k, Edot_k, t_gps)

% See IS-GPS-200H: 20.3.3.3.3.1 User algorithm for SV clock correction.

% F = -2*sqrt(mu)/c^2.
F = -4.442807633e-10;

% Compute Relativistic correction.
dt_R = F * eph.ecc * eph.sqrt_a * sin(E_k);
dt_Rdot = F * eph.ecc * eph.sqrt_a * cos(E_k) * Edot_k;

% Compute SV PRN code phase offset.
dt_oc = wrap_tgps(t_gps - eph.t_oc);
dt_clock = eph.a_f0 + eph.a_f1 * dt_oc + eph.a_f2 * dt_oc^2 + dt_R;
dt_clockdot = eph.a_f1 + 2 * eph.a_f2 * dt_oc + dt_Rdot;
