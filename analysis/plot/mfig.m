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

function mfig(fig_name, xlabel_str, ylabel_str, zlabel_str, link_x_axis)
% mfig -- Make figure with standard Makani style.
%
% mfig(fig_name, xlabel_str, ylabel_str, zlabel_str, link_x_axis)
% Creates a named figure, docks the figure, labels the axes, and turns the
% grid on.  If link_x_axis is set, the axes of the figure are appended to a
% global list of axes.

if nargin < 5, link_x_axis = true; end
if nargin < 4, zlabel_str = ''; end
if nargin < 3, ylabel_str = ''; end
if nargin < 2, xlabel_str = ''; end
subsequent_plot = 'Replace';

h = findobj('type', 'figure', 'name', fig_name);
if (isempty(h) ...
    || strcmp(subsequent_plot, 'Replace') ...
    || strcmp(subsequent_plot, 'NewFig'))
  if strcmp(subsequent_plot, 'Replace')
    close(h);
  end
  figure('Name', fig_name, 'WindowStyle', 'docked');
  axes();  % Makes it so that "gca" works below.
elseif strcmp(subsequent_plot, 'OnTop')
  figure(h(end));
end

hold on;

grid on;
zoom('out');
zoom('on');
xlabel(xlabel_str);
ylabel(ylabel_str);
zlabel(zlabel_str);

if link_x_axis
  evalin('caller', 'if ~exist(''ax''), ax=[]; end; ax(end + 1) = gca;');
end
