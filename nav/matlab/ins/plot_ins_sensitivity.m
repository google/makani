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

function [fig_i] = plot_ins_sensitivity(sense, fig_i)
if ~exist('fig_i', 'var') || isempty(fig_i)
  fig_i = 0;
end
r2d = 180 / pi;

labels = {'x', 'y', 'z'};
for l = 1:length(labels)
  label = labels{l};
  plot_title = sprintf('ECEF %s position sensitivity', upper(label));
  fig_i = inc_figure(fig_i, plot_title);
  pxv_name = sprintf('pxv_%s', label);
  plot(sense.t, sqrt(abs(sense.(pxv_name)(:, sense.select))));
  set(gca, 'YScale', 'log');
  grid on;
  title(plot_title);
  ylabel('1 \sigma error (m)');
  xlabel('Time (s)');
  legend(sense.name_escaped{sense.select}, 'Location', 'SouthWest');
end

plot_title = 'Azimuth sensitivity';
fig_i = inc_figure(fig_i, plot_title);
plot(sense.t, sqrt(abs(sense.pxv_azi(:, sense.select))) * r2d);
set(gca, 'YScale', 'log');
grid on;
title(plot_title);
ylabel('1 \sigma error (deg)');
xlabel('Time (s)');
legend(sense.name_escaped{sense.select}, 'Location', 'SouthWest');
