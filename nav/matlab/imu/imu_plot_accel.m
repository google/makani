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

function [fig_i] = imu_plot_accel(fig_i, time, data, model)
if ~exist('model', 'var')
  model = [];
end

name = 'Accel';
grav = 9.80665;
fig_i = imu_plot_ts(fig_i, time, data, 1 / grav, name, 'g');
fig_i = imu_plot_psd(fig_i, time, data, name, model);
fig_i = imu_plot_avar(fig_i, time, data, 1000 / grav, name, 'mg', model);
