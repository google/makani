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

function plot_telem(plot_names, data, time_base)
%  PLOT_TELEM   Makes a standard set of plots from the telemetry data.
%   PLOT_TELEM(PLOT_NAMES, DATA, TIME_BASE) makes a standard set of plots
%   from the telemetry data.
%
%   Inputs are:
%   PLOT_NAMES  : cell array of strings describing what to plot
%   DATA        : data structure containing flight and parameter data
%   TIME_BASE   : selects the time base for different plots
%
%   time_base descriptions: (default is 'sync_zero')
%       'native'     : native time for each source.
%       'native_zero': native time starting at zero.
%       'sync'       : synchronized gs time.
%       'sync_zero'  : synchronized gs time starting at zero. [Default]
%
%   Example:
%       plot_telem({'fm', 'pos', 'tension'});
%       plot_telem({'gs', 'control', 'rf'}, data, 'sync');
%
  persistent ax;
  if nargin < 3
    % Use native time if using the simulator because the simulator runs faster
    % than real-time
    if isempty(data.Simulator.SimTelemetry)
      time_base = 'sync_zero';
    else
      time_base = 'native_zero';
    end
  end

  make_naming_shortcuts(data);

  switch time_base
    case {'native', 'native_zero'},
      t_c = C.message.time;
      t_s = S.message.time;
    case {'sync', 'sync_zero'},
      t_c = float(C.capture_header.tv_sec) ...
            + 1e-6*float(C.capture_header.tv_usec);
      t_s = float(C.capture_header.tv_sec) ...
            + 1e-6*float(C.capture_header.tv_usec);
    otherwise
      error(['Invalid time_base. Please use ''native'', ' ...
             '''sync'', or ''sync_zero'''])
  end

  if strcmp(time_base(end-3:end), 'zero')
    t_c = t_c - t_c(1);
    t_s = t_s - t_s(1);
  end

  ax_c = plot_control(plot_names, data, t_c);
  ax_s = plot_sim(plot_names, data, t_s);
  ax_fd = plot_faults(plot_names, data, t_c);

  ax = [ax, ax_s, ax_c, ax_fd];
  if strcmp(time_base(1:4), 'sync')
    ax = [ax, ax_s];
  end
  linkaxes(ax(ishandle(ax)), 'x');
end
