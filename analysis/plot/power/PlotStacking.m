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

function [t0] = PlotStacking(filename, motor_list, timebase, plot_func, ...
                             plot_str, t0, rescale_time)
% PlotStacking -- Plot data related to stacked power and powertrains.
%
% [t0] = PlotStacking(filename, motor_list, timebase, plot_str, t0)
% Generates a series of linked plots that display variables related to stacking
% and powertrain debugging and analysis.
%
% Arguments
%
% filename: Name of hdf5 file with data to be plotted
% motor_list: Array with motors to plot
% timebase: Source of time: 'aio_sequence' uses aio sequence numbers and
%           'capture_header' will plot using the time data is logged.
% plot_str: Listing of plots groups to generate
% t0: Base time for aligning plots - use zero to assign zero to first
%     data point
% rescale_time: Optional boolean to correct noisy time data
%
% Return values
% t0: If discovered, will return timestamp of flight start, else zero
%
% Required toolboxes: Filter toolbox

%% Set up defaults
% Default to standard stack
if (nargin < 2)
    motor_list = 1:8;
end

% Choices are 'aio_sequence' or 'capture_header'
if (nargin < 3)
    timebase = 'aio_sequence';
end

if strcmp(timebase, 'capture_header')
  t_aio = 0;
else
  t_aio = 1;
end

if nargin < 4 || isempty(plot_func)
  plot_func = 'StackList';
end

if (nargin < 5)
  plot_str = 'all';
end

if nargin < 7
  rescale_time = false;
end

% Turn on rpx special mode plotting option if set to 1
rpx_special_in_use = 0;

% Initialize axes
ax = [];

if (contains({'flight_mode'}, plot_str))
  if (nargin < 6) || isempty(t0)
    [t0, cmessage, t] = GetFlightStartTime(filename, rescale_time);
    if t0 == 0  % If no flighttime available, use the first time stamp as zero.
      t0 = t(1);
    end
  else
    [~, cmessage, t] = GetFlightStartTime(filename, rescale_time);
  end
  ax = PlotMotorData({t - t0}, {cmessage}, 'flight mode', 'flight_mode', ...
                   1, 'Flight Mode', ax);

  % Special flight mode starting in RPX-04
  % Replaced by play book.  Turn on above if
  % looking at older data sets.
  if rpx_special_in_use
    var_math = struct('math', '@(x,y) double(x < -0.5) .* double(y)', ...
      'input', {{'state_est.joystick.pitch_f', 'state_est.experimental_crosswind_config'}}, ...
                          'output', 'active_mode_enabled', 'header', []);

    cmessage = GenerateDerivedValue({cmessage}, var_math, ...
                            1);

    ax = PlotMotorData({t - t0}, cmessage, 'active mode', 'active_mode_enabled', ...
      1, 'Special Mode', ax);
  end

elseif (nargin < 6) || isempty(t0)
  t0 = GetFlightStartTime(filename, rescale_time);
end

% Load in list of plots to perform
GenPlotList = str2func(plot_func);
plot_struct = GenPlotList(plot_str);

datatype_list = {'kMessageTypeMotorIsrLog', 1/15000; ...
                 'kMessageTypeMotorDebug', 1e-3*t_aio; ...
                 'kMessageTypeMotorStatus', 1e-2*t_aio; ...
                 'kMessageTypeSlowStatus', 1e-0*t_aio}'; % Transpose to work in for loop
for dtype = datatype_list
  first_time = 1;
  for ps = plot_struct
    if (strcmp(ps.message_type, dtype{1}))
      if (first_time)
        [messages, t] = GetMotorMessages(filename, dtype{1}, ...
                  dtype{2}, t0, motor_list, rescale_time);
        if all(cellfun(@isempty, t))
          disp(['Unable to find data of type ', dtype{1}, ' in ', filename])
          break;
        end
        first_time = 0;
        disp(['Plotting data from: ', filename, ' of datatype ', dtype{1}])
      end
      messages = GenerateDerivedValue(messages, ps.var_math, ...
                            motor_list);
      if ps.deal_plots
        for mi = motor_list
          ps_i = ps;
          ps_i.title_string = [ps.title_string, ' (motor # ', num2str(mi), ')'];
          ax = PlotData(t, messages, mi, ps_i, ax);
        end
      else
        ax = PlotData(t, messages, motor_list, ps, ax);
      end
    end
  end
end

if ~isempty(ax)
  linkaxes(ax, 'x');
end

end

function ax = PlotData(time, messages, motor_list, plot_struct, ax)
% PlotData - Parsing function to better organize plotting
  switch lower(plot_struct.plot_function)
    case 'plotmotordata'
      ax = PlotMotorData(time, messages, plot_struct.title_string, plot_struct.var_name, ...
           motor_list, plot_struct.y_string, ax, plot_struct.plot_type);
    case 'plotmotorsumdata'
      ax = PlotMotorSumData(time, messages, plot_struct.title_string, ...
         plot_struct.var_name, motor_list, plot_struct.y_string, ax, plot_struct.plot_type);
    case 'plotmotordiffdata'
      ax = PlotMotorDiffData(time, messages, plot_struct.title_string, ...
         plot_struct.var_name, motor_list, plot_struct.y_string, ax);
    case 'plotmotortempdata'
      ax = PlotMotorTempData(time, messages, plot_struct.title_string, plot_struct.var_name, ...
         motor_list, plot_struct.y_string, ax);
    case 'plotmotorarraydata'
      ax = PlotMotorArrayData(time, messages, plot_struct.title_string, plot_struct.var_name, ...
         plot_struct.labels, motor_list, plot_struct.y_string, ax);
    case 'generateonly'
      if isempty(plot_struct.var_math)
        disp('Generate called with empty variable.')
      else
        disp(['Generating variable: ', plot_struct.var_math.output]);
      end
    otherwise
      disp(['Unknown plot function for plot: ', plot_struct.title_string])
  end
end

% Main plotting function - encapsulating generic routine
function ax = PlotMotorData(time, messages, plot_name, plot_var, ...
                            motor_list, units, ax, plottype, array_index)
% plottypes:
%  0 - nominal
%  1 - filter
%  2 - difference
%  3 - filtered difference
%  4 - 2 variable different
%  5 - 3 phase plot
%  6 - abs(diff)
%  7 - filter of abs(diff)
%  8 - subcommutate on first variable

   if (~exist('array_index', 'var') || isempty(array_index))
     array_index = 1;
   end

  % Verify that plot_var is member of struct
  if (isa(plot_var, 'cell'))
    vars = cell(length(plot_var), length(motor_list));
    i = 0;
    for plot_var_i = plot_var
      i = i+1;
      for j = motor_list
        [stat, vars{i,j}] = IsDeepField(messages{j}, ['message.',plot_var_i{1}]);
        if ~stat
          disp(['Field not available: ', plot_var_i{1}]);
%           ax = ax;
          return;
        end

        vars{i,j} = vars{i,j}(:, array_index);
      end
    end

  else
    vars = cell(max(motor_list), 1);

    for j = motor_list
      [stat, vars{j}] = IsDeepField(messages{j}, ['message.', plot_var]);
      if ~stat
        disp(['Field not available: ', plot_var]);
        return;
      end
    end

  end

  % Set defaults
  if (~exist('plottype', 'var') || isempty(plottype))
    plottype = 0;
  end

  % Build filter if necessary
  if (plottype == 1 || plottype == 3 || plottype == 7)
    if (~exist('butter', 'file') || ~exist('filtfilt', 'file'))
      plottype = plottype - 1;
      disp(['Filter toolbox does not exist: demoting plot to unfiltered: ', ...
             plot_name]);
    else
      [b, a] = butter(1, 10/500);
    end
  end

  % Set up docked figure
  if (plottype ~= 5)
    figure('Name', plot_name, 'WindowStyle', 'docked');
  end

  % Set up expectation for number of motors
%   if (strcmp(stacktype, '4x2')),
%     n = 8;
%   else
%     n = 4;
%   end

  colors = GetPlotColors();

  % Initialize holder for plines
  if iscell(plot_var)
    plines = gobjects(length(plot_var), max(motor_list));
  else
    [r, c] = cellfun(@size, vars);
    if min(max(r),max(c)) > 1
      plines = gobjects(max(r), max(motor_list));
    else
      plines = gobjects(1, max(motor_list));
    end
  end


  % Plot data and apply filter if necessary
  if (plottype == 5)
    motor_names = GetMotorList();
    ax_tmp = [];
    for i_motor = motor_list
      plot_name_with_motor = [plot_name, ': ', motor_names{i_motor}];
      figure('Name', plot_name_with_motor, 'WindowStyle', 'docked');
      hold on;
      grid('on');

      for i_var = 1:length(plot_var)
        plines(:, i_var) = plot(time{i_motor}, vars{i_var, i_motor}, ...
                           '-', 'color', colors(i_var, :));
      end

      if (length(plot_var) == 3)
        [~, hObj] = legend('a', 'b', 'c', 'location', 'northeast');
        set(hObj, 'LineWidth', 0.75);
      elseif (length(plot_var) == 4)
        [~, hObj] = legend('a', 'b', 'c', 'b_nom', 'location', 'northeast');
        set(hObj, 'LineWidth', 0.75);
      end

      title(plot_name_with_motor);
      xlabel('Time (sec)');
      ylabel(units);
      ax_tmp = [ax_tmp, gca];  %#ok.
    end
    ax = [ax, ax_tmp];
  elseif (plottype == 4)  % cell array but for difference two variables
    % Deal with uniqueness
    vv = cell(1, length(motor_list));
    for i = motor_list
      %v = messages{i}.message.(plot_var{1}) - messages{i}.message.(plot_var{2});
      vv{i} = vars{1,i} - vars{2,i};

      plines(:, i) = plot(time{i}, vv{i}, 'color', colors(i, :));
      hold on;

    end
  elseif (plottype == 8) % subcommutate second variable using first
    if (~isa(plot_var, 'cell'))
      disp(['Unable to subcommutate ', plot_var, ': Not cell array']);
    else
      for i = motor_list
        for j = min(vars{1,i}):max(vars{1,i})
          plines(:, i) = plot(time{i}(vars{1,i} == j), vars{2,i}(vars{1,i} == j), ...
                           'color', colors(i, :));
          hold all;
        end
      end
    end
  elseif (isa(plot_var, 'cell'))
    line_type = '-';
    celli = 0;
    for plot_var_i = plot_var
      celli = celli + 1;

      for i = motor_list
        if (plottype == 1)  % Filtered plot
          plines(:, i) = plot(time{i}, filtfilt(b, a, ...
                             double(vars{celli,i})), ...
                             line_type, 'color', colors(i, :));
        elseif (plottype == 2)  % Delta plot
          plines(:, i) = plot(time{i}(1:end-1), ...
                             diff(double(vars{celli,i}')), ...
                             line_type, 'color', colors(i, :));
        elseif (plottype == 3)  % Filtered delta plot
          plines(:, i) = plot(time{i}(1:end-1), filtfilt(b,a, ...
                             diff(double(vars{celli,i}'))), ...
                             line_type, 'color', colors(i, :));
        elseif (plottype == 6)  % abs(Delta) plot
          plines(:, i) = plot(time{i}(1:end-1), ...
                             abs(diff(double(vars{celli,i}'))), ...
                             line_type, 'color', colors(i, :));
        elseif (plottype == 7)  % filtered abs(Delta) plot
          plines(:, i) = plot(time{i}(1:end-1), filtfilt(b,a, ...
                             abs(diff(double(vars{celli,i}')))), ...
                             line_type, 'color', colors(i, :));
        else  % Standard plot
          plines(:, i) = plot(time{i}, vars{celli, i}, ...
               line_type, 'color', colors(i, :));
        end
        hold all;
      end
      hold on;
      line_type = '-.';
    end
  else
    stackset = motor_list;  % Allow overwriting stacktype for legend
    for i = stackset
      if (plottype == 1)  % Filtered plot
        plines(:, i) = plot(time{i}, ...
                           filtfilt(b, a, double(vars{i})), ...
                           'color', colors(i, :));
      elseif (plottype == 2)  % Delta plot
        if size(vars{i}, 1) == 8
          for j = 1:8
            plines(:, j) = plot(time{i}(1:end-1), ...
                mod(diff(double(vars{i}(j,:)')), 65536), ...
                'color', colors(j,:));
            hold all;
          end
          motor_list = 1:8;
        else
          plines(:, i) = plot(time{i}(1:end-1), ...
              mod(diff(double(vars{i}')), 65536), ...
              'color', colors(i,:));
        end
      elseif (plottype == 3)  % Filtered delta plot
        plines(:, i) = plot(time{i}(1:end-1), filtfilt(b,a, ...
                           diff(double(vars{i}'))), ...
                           'color', colors(i, :));
      elseif (plottype == 6)  % abs(Delta) plot
        plines(:, i) = plot(time{i}(1:end-1), ...
              abs(diff(double(vars{i}'))), ...
              'color', colors(i,:));
      elseif (plottype == 7)  % filtered abs(Delta) plot
        plines(:, i) = plot(time{i}(1:end-1), filtfilt(b,a, ...
              abs(diff(double(vars{i}')))), ...
              'color', colors(i,:));
      else  % Standard plot
        plines(:, i) = plot(time{i}, vars{i}, ...
                           'color', colors(i, :));
      end
      hold all;
    end
  end

  if (plottype ~= 5)
    % Apply legend string
    legend_options = GetMotorList();
    [~, hObj] = legend(plines(1, motor_list), legend_options{motor_list}, ...
           'location', 'northwest');
    set(hObj, 'LineWidth', 0.75)
    grid('on');

    % Build axis vector for linking axes
    ax = [ax,gca];

    % Apply title and axes labels
    title(plot_name);
    xlabel('Time (sec)');
    ylabel(units);
  end
end

% Sum data and plot
function ax = PlotMotorSumData(time, messages, plot_name, plot_var, ...
                               motor_list, units, ax, plottype)
% PlotMotorSumData -- Various forms of data summing
%
% ax = PlotMotorSumData(time, messages, plot_name, plot_var, ...
%                       motor_list, units, ax, plottype)
% Plot different forms of summed data, providing resampling as necessary to
% realign timestamps.
%
% Arguments
%
% time: Cell array of time data corresponding to indeces in motor_list.
% messages: aio message cell array
% plot_name: Text string for title and figure labels.
% plot_var: Name of variable to plot.
% motor_list: Array of motor indeces, indicating which motors to extract (1-8).
% units: Text string of units for y axis.
% ax: axes returned for this plot.
% plottype: What type of summing operation to perform.
%
% Return values
%
% ax: axes for this plot tacked on to the axes in the function input.
%
% Required toolboxes: None.

% plot_type meanings:
%   0 (default) - Sum all and divide by 4 / length(motor_list).  Good for
%                 plotting voltage data
%   1 - Gate sum with motor_status (2 or 8).  Sum all and divide by 2.
%   2 - Sum pairs in stack only.
%   3 - Sum all and filter
%   4 - Sum all bottoms (1-4) and tops (5-8)
%   5 - Sum all Starboard (1,2,7,8) and Port (3,4,5,6)
%   6 - Sum double reverse rainbow (1,4,6,7) and (2,3,5,8)
%   7 - Sum all and take difference
%   8 - Sum all

if nargin < 8
  plottype = 0;
end

  vars = cell(1, length(motor_list));

  for j = motor_list
    [stat, vars{j}] = IsDeepField(messages{j}, ['message.', plot_var]);
    if ~stat
      disp(['Field not available: ', plot_var]);
%       ax = ax;
      return;
    end
  end

figure('Name', plot_name, 'WindowStyle', 'docked');

% Step 1: build new time vector
tt = time(~cellfun(@isempty, time));
tmin = max(cellfun(@min, tt));
tmax = min(cellfun(@max, tt));
t = tmin:.001:tmax;

% Step 2: No longer needed - uniqueness and inversions dealth with in data
% collection.

% Step 3: resample variable of interest
v = zeros(length(t),8);
for i = motor_list
  if plottype == 1
    gate = (messages{i}.message.motor_status == 2) | ...
           (messages{i}.message.motor_status == 8);
    v(:,i) = interp1(time{i}, vars{i}.*gate, t);
  else
    v(:,i) = interp1(time{i}, vars{i}, t, 'linear', 'extrap');
  end
end


% Step 4: sum and plot
if (plottype == 2) % Sum cross pairs
  cross_legend = {'sbo+pto','sbi+pti','pbi+sti','pbo+sto'};
  legend_str = cell(1); % start with empty array
  i_lgnd = 0;
  for i = 1:4
    if all(ismember([i, i+4], motor_list))
      plot(t, v(:,i) + v(:, i+4));
      hold all;
      i_lgnd = i_lgnd + 1;
      legend_str{i_lgnd} = cross_legend{i};
    end
  end
  if i_lgnd
    [~, hObj] = legend(legend_str);
    set(hObj, 'LineWidth', 0.75);
  end
elseif (plottype == 3) % Sum all
  [b,a]= butter(1, 20/500);
  plot(t, filtfilt(b, a, sum(v,2)));
elseif (plottype == 8) % Sum all (no filter)
  plot(t, sum(v,2));
elseif (plottype == 4) % Sum bottoms and tops
  cross_legend = {'wing bottom', 'wing top'};
  legend_str = cell(1); % start with empty array
  i_lgnd = 0;
  [b,a]= butter(1, 20/500);
  for i = 1:2
    if all(ismember(((i - 1) * 4 + 1):((i - 1) * 4 + 4), motor_list))
      temp = filtfilt(b, a, sum(v(:, ((i - 1) * 4 + 1):((i - 1) * 4 + 4)),2));
      plot(t, temp);
      hold all;
      i_lgnd = i_lgnd + 1;
      legend_str{i_lgnd} = cross_legend{i};
    end
  end
  if i_lgnd
    [~, hObj] = legend(legend_str);
    set(hObj, 'LineWidth', 0.75);
  end
elseif (plottype == 5) % Sum Starboard and Port
  cross_legend = {'Starboard', 'Port'};
  legend_str = cell(1); % start with empty array
  i_lgnd = 0;
  [b, a]= butter(1, 20/500);
  for i = 1:2
    if all(ismember(mod((i * 4 + 2):(i * 4 + 5), 8) + 1, motor_list))
      temp = filtfilt(b, a, sum(v(:, mod((i * 4 + 2):(i * 4 + 5), 8) + 1),2));
      plot(t, temp);
      hold all;
      i_lgnd = i_lgnd + 1;
      legend_str{i_lgnd} = cross_legend{i};
    end
  end
  if i_lgnd
    [~, hObj] = legend(legend_str);
    set(hObj, 'LineWidth', 0.75);
  end
elseif (plottype == 6) % Double Reverse Rainbow
  cross_legend = {'Rainbow 1 4 6 7', 'Reverse 2 3 5 8', 'Net'};
  rainbow = [1 4 6 7; 2 3 5 8];
  legend_str = cell(1); % start with empty array
  i_lgnd = 0;
  [b, a]= butter(1, 20/500);
  for i = 1:2
    if all(ismember(rainbow(i,:), motor_list))
      temp = filtfilt(b, a, sum(v(:, rainbow(i,:)),2));
      plot(t, temp);
      hold all;
      i_lgnd = i_lgnd + 1;
      legend_str{i_lgnd} = cross_legend{i};
    end
  end
  temp = filtfilt(b, a, sum(v(:, motor_list), 2));
  plot(t, temp);
  i_lgnd = i_lgnd + 1;
  legend_str{i_lgnd} = cross_legend{i};
  if i_lgnd
    [~, hObj] = legend(legend_str);
    set(hObj, 'LineWidth', 0.75);
  end
elseif (plottype == 7) % Sum all
  [b,a]= butter(1, 1/500);
  plot(t(1:end-1), filter(b, a, diff(sum(v,2))/.001)); % Convert to rate
else  % Sum and try to scale for 4 stack
  plot(t, sum(v,2) * 4 / length(motor_list));
end

ax = [ax,gca];
title(plot_name);
xlabel('Time (sec)');
ylabel(units);
grid('on');

end


function ax = PlotMotorDiffData(time, messages, plot_name, plot_var, ...
                                motor_list, units, ax)
% PlotMotorDiffData -- Plot differences between motor pairs
%
% ax = PlotMotorDiffData(time, messages, plot_name, plot_var, ...
%                       motor_list, units, ax)
% Plot difference between stacked motor pairs.
%
% Arguments
%
% time: Cell array of time data corresponding to indeces in motor_list.
% messages: aio message cell array
% plot_name: Text string for title and figure labels.
% plot_var: Name of variable to plot.
% motor_list: Array of motor indeces, indicating which motors to extract (1-8).
% units: Text string of units for y axis.
% ax: axes returned for this plot.
%
% Return values
%
% ax: axes for this plot tacked on to the axes in the function input.
%
% Required toolboxes: None.

% Verify field in struct
vars = cell(1, length(motor_list));

for j = motor_list
  [stat, vars{j}] = IsDeepField(messages{j}, ['message.', plot_var]);
  if ~stat
    disp(['Field not available: ', plot_var]);
    return;
  end
end

% Prep filter if available
if (exist('butter', 'file'))
  [bb, aa] = butter(1, 50/500);
else
  % Around 1 Hz filter for 1 kHz sampling
  bb = [.003 .003];
  aa = [1.0 -.994];
end


figure('Name', plot_name, 'WindowStyle', 'docked');

colors = GetPlotColors();

% Step 1: build new time vector
tt = time(~cellfun(@isempty, time));
tmin = max(cellfun(@min, tt));
tmax = min(cellfun(@max, tt));
t = tmin:.001:tmax;

% Step 2: No longer needed, handled in data collection

% Step 3: resample variable of interest
v = zeros(length(t),8);
for i = motor_list
  v(:,i) = interp1(time{i}, vars{i}, t);
  v(:,i) = filter(bb, aa, v(:,i));
end

% Step 4: sum and plot
cross_legend = {'sbo-pto','sbi-pti','pbi-sti','pbo-sto'};
legend_str = cell(1); % start with empty array
i_lgnd = 0;
for i = 1:4
  if all(ismember([i, i+4], motor_list))
    plot(t, v(:,i) - v(:, i+4),'color', colors(i,:));
    hold all;
    i_lgnd = i_lgnd + 1;
    legend_str{i_lgnd} = cross_legend{i};
  end
end

[~, hObj] = legend(legend_str);
set(hObj, 'LineWidth', 0.75);

grid on

ax = [ax,gca];
title(plot_name);
xlabel('Time (sec)');
ylabel(units);

end

% Temperature plotting function
function ax = PlotMotorTempData(time, messages, plot_name, plot_var, ...
                                motor_list, units, ax)

  thermal_channels = {'kMotorThermalChannelBoard', ...
  'kMotorThermalChannelColdJunction', 'kMotorThermalChannelStator1', ...
  'kMotorThermalChannelStator2', 'kMotorThermalChannelStator3', ...
  'kMotorThermalChannelRotor', 'kMotorThermalChannelCoolant', ...
  'kMotorThermalChannelController1', 'kMotorThermalChannelController2', ...
  'kMotorThermalChannelController3'};

  vars = cell(max(motor_list), 1);

  for j = motor_list
    [stat, vars{j}] = IsDeepField(messages{j}, ['message.', plot_var]);
    if ~stat
      disp(['Field not available: ', plot_var]);
      return;
    end
  end

  % Set up docked figure
  figure('Name', plot_name, 'WindowStyle', 'docked');

%   colors = [0.800   0.000   0.000    % dark red
%              0.150   0.550   0.150    % dark green
%              0.000   0.447   0.741    % dark blue
%              0.600   0.184   0.700    % dark purple
%              1.000   0.500   0.500    % light red
%              0.200   0.800   0.200    % green
%              0.600   0.700   1.000    % light blue
%              0.900   0.000   1.000    % light purple
%              0.929   0.694   0.125    % orange
%              1.000   1.000   0.000];  % magenta maybe

  colors = varycolor(10);


  set(gca, 'ColorOrder', colors);

  hold all;  % Make sure the color map sticks
  % Plot data and apply filter if necessary
  stackset = motor_list;  % Allow overwriting stacktype for legend
  for i = stackset
    plot(time{i}, vars{i}); %, 'color', colors);
    hold all;
  end

  % Apply legend string
  %legend_options = {'sbo','sbi','pbi','pbo','pto','pti','sti','sto'};
  %legend(legend_options{stacktype});
  [~, hObj] = legend(thermal_channels);
  set(hObj, 'LineWidth', 0.75);

  grid on;

  % Build axis vector for linking axes
  ax = [ax,gca];

  % Apply title and axes labels
  title(plot_name);xlabel('Time (sec)');ylabel(units);
end

% Array plotting function
function ax = PlotMotorArrayData(time, messages, plot_name, plot_var, ...
                                 labels, motor_list, units, ax)

  vars = cell(max(motor_list), 1);

  for j = motor_list
    [stat, vars{j}] = IsDeepField(messages{j}, ['message.', plot_var]);
    if ~stat
      disp(['Field not available: ', plot_var]);
      return;
    end
  end

  % Set up docked figure
  figure('Name', plot_name, 'WindowStyle', 'docked');

%   colors = [0.800   0.000   0.000    % dark red
%              0.150   0.550   0.150    % dark green
%              0.000   0.447   0.741    % dark blue
%              0.600   0.184   0.700    % dark purple
%              1.000   0.500   0.500    % light red
%              0.200   0.800   0.200    % green
%              0.600   0.700   1.000    % light blue
%              0.900   0.000   1.000    % light purple
%              0.929   0.694   0.125    % orange
%              1.000   1.000   0.000];  % magenta maybe


  % Get array size
  sz = 0;
  for i = motor_list
      sz = max(sz, size(vars{i},1));
  end

  colors = varycolor(sz);


  set(gca, 'ColorOrder', colors);

  hold all;  % Make sure the color map sticks
  % Plot data and apply filter if necessary
  stackset = motor_list;  % Allow overwriting stacktype for legend
  for i = stackset
    plot(time{i}, vars{i}); %, 'color', colors);
    hold all;
  end

  % Apply legend string
  %legend_options = {'sbo','sbi','pbi','pbo','pto','pti','sti','sto'};
  %legend(legend_options{stacktype});
  [~, hObj] = legend(labels);
  set(hObj, 'LineWidth', 0.75);

  grid on;

  % Build axis vector for linking axes
  ax = [ax,gca];

  % Apply title and axes labels
  title(plot_name);xlabel('Time (sec)');ylabel(units);
end

function [motors] = GetMotorList()
  motors = {'sbo','sbi','pbi','pbo','pto','pti','sti','sto', ...
            'dsbo','dsbi','dpbi','dpbo','dpto','dpti','dsti','dsto'};
end

function [colors] = GetPlotColors()
  colors = [0.800   0.000   0.000    % dark red
            0.125   0.500   0.125    % dark green
            0.000   0.447   0.741    % dark blue
            0.600   0.184   0.700    % dark purple
            1.000   0.500   0.500    % light red
            0.300   0.800   0.300    % green
            0.600   0.700   1.000    % light blue
            0.900   0.000   1.000    % light purple
            0.850   0.550   0.000    % dark orange
            0.500   0.650   0.000    % dark mustard green
            0.000   0.650   0.700    % dark aquamarine
            0.666   0.000   0.620    % another dark purple
            1.000   0.800   0.400    % light orange
            0.800   1.000   0.100    % mustard green
            0.200   0.950   1.000    % light aquamarine
            1.000   0.400   0.950];  % another light purple
end

% Test if varString indicates a variable in the structure
function [status, var] = IsDeepField(dataStruct, varString)
  status = false;
  var = [];
  tokens = strsplit(varString, '.');
  testStruct = dataStruct;
  for t = tokens
    if (~isfield(testStruct, t{1}))
      return;
    end
    testStruct = testStruct.(t{1});
  end
  status = true;
  var = testStruct;
  return;
end
