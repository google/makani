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

function [r_u_ecef, v_u_ecef] = ins_gps_antenna(p, xhat, inputs)
r_bgps_b = p.geometry.r_bgps_b;

% Compute antenna position.
r_ib_e = double(xhat.r_ix_e) + xhat.dr_xb_e;
r_u_ecef = r_ib_e + xhat.R_eb * r_bgps_b;

% Compute antenna velocity.
v_u_ecef = xhat.v_ib_e + xhat.R_eb * cross(inputs.omega_eb_b, r_bgps_b);
