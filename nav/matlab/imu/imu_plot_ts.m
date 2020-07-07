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

function [fig_i] = imu_plot_ts(fig_i, time, data, scale, name, units)
names = {};
for i = 1:size(data, 2)
  names{i} = sprintf('%s-%d', name, i);
end

plot_title = sprintf('%s time series', name);
[fig_i, h] = inc_figure(fig_i, plot_title);
plot(time, data * scale);
grid on;
xlabel('Tau (s)');
ylabel(sprintf('%s (%s)', name, units));
title(plot_title);
legend(names);
