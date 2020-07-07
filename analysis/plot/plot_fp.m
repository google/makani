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

function plot_fp(t, x, y, c, r0)
% plot_fp -- Displays wing path in flight plane with a color variable.
%
% plot_fp(t, x, y, c, r0)
%
% Example:
%   plot_fp(c.time, -c.crosswind.Xp_curr.x, -c.crosswind.Xp_curr.y, ...
%           c.state_est.tether_force_b.sph.tension);

set(gcf, 'Renderer', 'OpenGL');
if nargin < 4
  plot3(t, x, y, 'b');
elseif isstr(c)
  plot3(t, x, y, c);
else
  plot3c(t, x, y, c);
  jet0 = jet();
  jet0(1, :) = [1, 1, 1];
  colormap(jet0);
  colorbar;
end
if nargin < 5
  r0 = 47;
end
ind2sec = find(t - t(1) >= 1, 1);
if isempty(ind2sec), ind2sec = 250; end
plot3(t(1:ind2sec:end), x(1:ind2sec:end), y(1:ind2sec:end), 'k.');
t_fine = linspace(t(1), t(end), 10*length(t));
plot3(t_fine, r0*cos(2*pi*t_fine), r0*sin(2*pi*t_fine), 'k');
axis equal;
set(gca, 'CameraTargetMode', 'manual', ...
    'CameraPositionMode', 'manual', ...
    'CameraTarget', [0, 0, -r0/2], ...
    'CameraPosition', [max(t), 0, -r0/2], ...
    'YLimMode', 'manual', 'ZLimMode', 'manual', ...
    'YLim', 1.5*[-r0, r0], 'ZLim', 1.5*[-r0, r0]);
