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

function [ retstruct ] = NicePlot(x_data, y_data)
  %NICEPLOT Plots x_data vs y_data between mode_1 and mode_2 in a new figure window
  %
  % Usage:
  %   [ retstruct ] = ModeSegmentPlot(x_data, y_data, mode_data, mode_1, mode_2, newfig )
  %
  % Description:
  %   This function will create a standard plot for the time_data and y_data
  %   and will also plot vertical lines for the time_data where flight mode_1 occurs as well
  %   and hopefully a reasonably decent fit and sensible scale on the y_data
  %
  % Inputs:
  %   x_data    - the independent time data
  %   y_data    - the data to be plotted
  %
  % Outputs:
  %   retstruct is a struct that contains
  %    .fig     - the handle for the figure in which the plot occurs
  %    .datplot - the handle for the plot in which the data is plotted
  %    .ax      - the handle for the plot axes
  %
  %%====================================================================================================================
  % new figure window
  retfig=figure();
  %
  % plot the data
  retplot=plot(x_data,y_data,'LineWidth',2.0);
  data_axes=gca;
  % keep it together
  hold on;
  %
  xmin = min(x_data);
  %
  % demarkate second instance of mode in a later time
  xmax = max(x_data);
  %
  %x axis limits
  xrange=[xmin-0.1*abs(xmax-xmin)  xmax+0.1*abs(xmax-xmin)];
  xlim(data_axes,xrange);
  set(data_axes,'XMinorTick','on','XMinorTick','on');
  %
  % y axis limits
  %  figure out the data spread and the order of magnitude of the edge cases
  %  round to the appropriate order of magnitude in y
  %  get 5 major ticks of the appropriate scaling
  %
  % get order of magnitude of limits of data *we care about*
  %
  % y-axis
  ymax = max(max(y_data));
  ymin = min(min(y_data));
  %
  data_order=GetDataDeltaO(ymin,ymax);
  %
  % cut down and round
  ymax_new = ymax-sign(ymax)*(mod(abs(ymax),data_order)-data_order);
  ymin_new = ymin-sign(ymin)*(mod(abs(ymin),data_order)+data_order);
  %
  %
  y_ticks=linspace(ymin_new,ymax_new,5);
  ylim(data_axes,[y_ticks(1) y_ticks(end)]);
  %
  set(data_axes,'Ytick',y_ticks);
  set(data_axes,'YtickLabel',y_ticks);
  set(data_axes,'YMinorTick','on','YMinorTick','on');
  %
  % get the axes
  retaxes = data_axes;
  %
  %build return struct
  retstruct=struct('fig',retfig,'datplot',retplot,'ax',retaxes);
end