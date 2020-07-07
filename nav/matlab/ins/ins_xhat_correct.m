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

function [xhat] = ins_xhat_correct(p, xhat_1, dx_plus)
s = p.states;

% Correct general inertial error states.
xhat = ins_xhat_correct_inertial(p, xhat_1, dx_plus);

% Correct wheel encoder error states.
xhat.dradius = xhat_1.dradius + dx_plus(s.WHEEL_RADIUS);

% Correct GPS error states.
xhat.cb_phase = xhat_1.cb_phase + dx_plus(s.CB_PHASE);
xhat.cf_bias = xhat_1.cf_bias + dx_plus(s.CF_BIAS);
xhat.cf_walk = xhat_1.cf_walk + dx_plus(s.CF_WALK);
xhat.gps_pr = xhat_1.gps_pr + dx_plus(s.GPS_PR);
