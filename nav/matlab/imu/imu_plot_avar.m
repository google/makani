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

function [fig_i] = imu_plot_avar(fig_i, dt, data, scale, name, units, model)
res = 0.001;
names = {};
plot_title = sprintf('%s AVAR', name);
[fig_i, h] = inc_figure(fig_i, plot_title);
hold on;
for i = 1:size(data, 2)
  names{i} = sprintf('%s-%d', name, i);
  [tau, sigma] = imu_calc_avar(data(:, i), dt, res);
  plot(tau, sigma * scale)
end
if exist('model', 'var') && ~isempty(model)
  names = [names, 'Ideal'];
  plot(model.tau, model.sigma * scale);
end
hold off;
set(gca, 'XScale', 'log', 'YScale', 'log');
grid on;
xlabel('Tau (s)');
ylabel(sprintf('Sigma (%s)', units));
title(plot_title);
legend(names);
