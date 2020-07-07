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

function [dx] = ins_dx_compute(p, xhat0, xhat1)
s = p.states;

% Compute error between two navigation state estimates, or truth source.
dx = ins_dx_compute_inertial(p, xhat0, xhat1);

% Compute wheel encoder error states.
dx(s.WHEEL_RADIUS) = xhat1.dradius - xhat0.dradius;

% Compute GPS error states.
dx(s.CB_PHASE) = xhat1.cb_phase - xhat0.cb_phase;
dx(s.CF_BIAS) = xhat1.cf_bias - xhat0.cf_bias;
dx(s.CF_WALK) = xhat1.cf_walk - xhat0.cf_walk;
dx(s.GPS_PR) = xhat1.gps_pr - xhat0.gps_pr;
