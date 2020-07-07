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

function [fig_i] = plot_ins_azimuth_sensitivity(out, fig_i)
if ~exist('fig_i', 'var') || isempty(fig_i)
  fig_i = 0;
end
r2d = 180 / pi;

fig_i = inc_figure(fig_i, 'Azimuth estimate');
names = {};
hold on;
for i = 1:length(out)
  ii = find(out{i}.init > 0);
  plot(out{i}.t(ii), out{i}.azimuth(ii, :) * r2d);
  names{i} = sprintf('%.0f deg', out{i}.param.init.azimuth * r2d);
end
hold off;
axis tight;
grid on;
xlabel('Time (s)');
ylabel('Azimuth estimate (deg)');
title('Azimuth convergence for different initial azimuth angles');
legend(names);

fig_i = inc_figure(fig_i, 'Position estimate');
names = {};
hold on;
for i = 1:length(out)
  ii = find(out{i}.init > 0);
  plot(out{i}.lon(ii) * r2d, out{i}.lat(ii, :) * r2d);
  names{i} = sprintf('%.0f deg', out{i}.param.init.azimuth * r2d);
end
hold off;
axis tight;
grid on;
xlabel('Longitude (deg)');
ylabel('Latitude (deg)');
title('Position convergence for different initial azimuth angles');
legend(names);
