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

function plot3c(x, y, z, c)
% plot3c -- Plot like plot3 but with color that can change along line.
%
% function plot3c(x, y, z, c)
% Plots a line in 3-space through the points whose coordinates are the
% elements of x, y and z, and color varies based on c, where x, y, z,
% and c are vectors of the same length.

% We would prefer to use "line" here but "line" doesn't have access to
% the individual colors of each connecting line, so we use patch
% instead.
%
% Patch connects the first and last points.  To fix this, we set the
% end color to NaN to make the last line element have no color.
c(1) = NaN;
c(end) = NaN;

% This used to have "'CDataMapping', 'direct', ..." as a parameter;
% however this caused MATLAB 2013a to crash.
h = patch('XData', x, 'YData', y, 'ZData', z, ...
          'FaceVertexCData', c, ...
          'Facecolor', 'none', ...
          'EdgeColor', 'flat', ...
          'Linewidth', 1);
