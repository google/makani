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

function [] = PlotIsrDiag(filename, motor_index)
% PlotIsrDiag -- Plot data recorded in MotorIsrDiagMessage.
%
% [] = PlotIsrDiag(filename, motor_index).
%
% Plot the MotorIsrDiagMessage data stored in the file filename corresponding to
% the specified motor indices. An additional figure is plotted if dropped or
% missing data is detected.
%
% Arguments:
%
% filename: Optional. Full or relative path of the HDF5 file to use. If empty or
%           not specified, a file picker is used.
% motor_index: Optional. Array of motor indices to plot. The convention followed
%              by AppConfigGetIndex() is used except that the index starts at 1
%              instead of 0, e.g. sbo is 1, pbo is 4, pto is 5, and sto is 8.
%              Default: 1:8.

  % If no motor index is given, try all possibilities.
  if (~exist('motor_index', 'var'))
    motor_index = 1:8;
  end

  % If filename is not given or is empty, start Matlab's file picker.
  if (~exist('filename', 'var') || isempty(filename))
    [filename, pathname] = uigetfile('*.h5', 'Open ISR Diagnostic Log');
    if (ischar(filename))
      filename = [pathname, filename];
    else  % No file was selected.
      return;
    end
  end

  % Start the performance timer.
  tic;

  motor_names = GetMotorShortNames(1:8);
  ax = zeros(10, 1);

  % Loop over motor indices and plot each separately. For long logs, plotting
  % multiple motors on the same axis would potentially make the plot almost
  % unusably slow.
  for i_motor = motor_index
    motor_name = motor_names{i_motor};
    [t, data, dropped] = LoadIsrDiag(filename, i_motor);

    % Empty arrays indicate that data from motor_name is not present in the log.
    if (isempty(t))
      fprintf('Skipping %s; data not available.\n', motor_name);
      continue;
    end

    % Reset the axis index.
    i_ax = 1;

    % Plot Motor Errors.
    if (isfield(data, 'errors'))  % Test to be backwards compatible.
      ax(i_ax) = FormatFigure(motor_name, 'Errors');
      plot(t, data.errors, 'r');
      i_ax = i_ax + 1;
    end

    % Plot Phase Currents.
    ax(i_ax) = FormatFigure(motor_name, 'i_a, i_b, i_c [A]');
    plot(t, data.ia, 'r');
    plot(t, data.ib, 'color', [0, 0.5, 0]);
    plot(t, data.ic, 'b');
    plot(t, (data.ia + data.ib + data.ic)/3, 'c');
    legend('i_a', 'i_b', 'i_c', 'i_0', 'location', 'northwest');
    i_ax = i_ax + 1;

    % Plot Bus Voltage.
    ax(i_ax) = FormatFigure(motor_name, 'Bus Voltage [V]');
    plot(t, data.vbus, 'k');
    i_ax = i_ax + 1;

    % Plot Bus Current.
    ax(i_ax) = FormatFigure(motor_name, 'Bus Current [A]');
    plot(t, data.ibus, 'k');
    i_ax = i_ax + 1;

    % Plot Sin / Cos Measurements.
    ax(i_ax) = FormatFigure(motor_name, 'sin / cos bins');
    plot(t, data.sin, 'r');
    plot(t, data.cos, 'b');
    i_ax = i_ax + 1;

    % Plot dropped samples iff dropped samples are present.
    if (any(dropped))
      warning('Detected %d dropped samples.\n', sum(dropped));
      ax(i_ax) = FormatFigure(motor_name, 'dropped');
      plot(t, dropped, 'r');
      i_ax = i_ax + 1;
    end

    % Link axes of a particular motor. Plots from different motors are not
    % linked due to performance reasons and the fact that the time stamp is not
    % necessarily aligned.
    linkaxes(ax(1 : i_ax - 1), 'x');

    fprintf('Finished %s. Time elapsed: %f s\n', motor_name, toc);
  end
end

function [ax] = FormatFigure(motor_name, y_label)
% Perform simple and repetitive formating tasks for figures.
%
% Arguments:
%
% motor_name: String with the short name of the motor.
% y_label: Argument for ylabel (y_label is also used in the figure name).
%
% Return Values:
%
% ax: Handle to the current figure axes.
  figure('Position', [20, 20, 1200, 350], ...
         'Name', ['(' motor_name '): ' y_label]);
  hold on;
  grid('on');

  xlabel('time (s)');
  ylabel(y_label);
  ax = gca;
end
