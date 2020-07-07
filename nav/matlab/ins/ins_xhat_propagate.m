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

function [xhat] = ins_xhat_propagate(p, xhat_1, gps, imu)
xhat = ins_xhat_propagate_inertial(p, xhat_1, imu);
dt = imu.dt;

% Propagate wheel encoder error states.
xhat.dradius = diag(1 - dt ./ p.wheel.tc_radius) * xhat_1.dradius;

% Propagate GPS error states.
xhat.cb_phase = xhat_1.cb_phase + (xhat_1.cf_bias + xhat_1.cf_walk) * dt;
xhat.cf_bias = xhat_1.cf_bias * (1 - dt / p.clock.tc_freq);
xhat.cf_walk = xhat_1.cf_walk;
xhat.gps_pr = diag(1 - dt ./ gps.tc_ura) * xhat_1.gps_pr;
