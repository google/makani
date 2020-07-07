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

function [ retstruct ] = ModeSegmentPlot(x_data, y_data, mode_data, mode_1, mode_2)
  %MODESEGMENTPLOT Plots x_data vs y_data between mode_1 and mode_2 in a new figure window
  %
  % Usage:
  %   [ retstruct ] = ModeSegmentPlot(x_data, y_data, mode_data, mode_1, mode_2, newfig )
  %
  % Description:
  %   This function will create a standard plot for the time_data and y_data
  %   and will also plot vertical lines for the time_data where flight mode_1 occurs as well
  %   as a vertical line at time_data where mode_2 occurs with a 20% width overage on the x-axis
  %   and hopefully a reasonably decent fit and sensible scale on the y_data
  %
  % Inputs:
  %   x_data    - the independent time data
  %   y_data    - the data to be plotted
  %   mode_data - the data array containing the mode switches for x_data
  %   mode_1    - the first flight mode line to be plotted vertically
  %   mode_2    - the second flight mode line to be plotted vertically
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
  % demarkate first instance of mode in time
  index1 = find(mode_data>=mode_1);
  xmin   = x_data(index1(1));
  %
  % demarkate second instance of mode in a later time
  index2 = find(mode_data>=mode_2)
  if(isempty(index2))
    xmax = x_data(end);
  else
    xmax = x_data(index2(1));
  end
  %
  %x axis limits
  xrange=[xmin-0.2*abs(xmax-xmin)  xmax+0.2*abs(xmax-xmin)];
  xlim(data_axes,xrange);
  set(data_axes,'XMinorTick','on','XMinorTick','on');
  %
  % y axis limits
  %  figure out the data spread and the order of magnitude of the edge cases
  %  round to the appropriate order of magnitude in y
  %  get 5 major ticks of the appropriate scaling
  %
  % get order of magnitude of limits of data *we care about*
  if(isempty(index2))
    disp('still empty')
    msk = ones(size(x_data));
  else
    msk = x_data >= x_data(index1(1)) & x_data <= x_data(index2(1));
  end
  %
  % y-axis
  ymax_raw = max(max(y_data(msk)));
  ymin_raw = min(min(y_data(msk)));
  raw_delta_y  = ymax_raw - ymin_raw;
  % get most significant order of the data
  order_ymax = 10^ceil(log10(abs(ymax_raw)));
  order_ymin = 10^ceil(log10(abs(ymin_raw)));
  max_order = max([order_ymax; order_ymin]);
  % control the small data spreads vs. order of data
  while(max_order>raw_delta_y)
    max_order=max_order/10;
  end
  % cut down and round
  ymax_new = ymax_raw-sign(ymax_raw)*(mod(abs(ymax_raw),max_order)-max_order);
  ymin_new = ymin_raw-sign(ymin_raw)*(mod(abs(ymin_raw),max_order)-max_order);
  %
  %plot demarcated lines
  plot([xmin, xmin],[ymin_new, ymax_new],'k-.'); %demarcate mode1
  plot([xmax, xmax],[ymin_new, ymax_new],'k--'); %demarcate mode2
  %
  y_ticks=linspace(ymin_new,ymax_new,5)
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
