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

function data = RunAnalysis(search_path)
% RunAnalysis -- Analyze specified set of H5 Logs from a motor test run.
%
% data = RunAnalysis(search_path)
%
% Arguments
%
% search_path: Parent folder containing log data.
%
% Return Values

% data: Structure that contains binned data.

% Default input variables.
if nargin < 1
 search_path = pwd;
end

% TODO: Make these values configurable.
ref_path = '/messages/kAioNodeMotorSto/kMessageTypeMotorDebug';
ref_cmd = 'torque_cmd';
torque_bins = 20:20:600;  % [Start:Step:End] Nm
transient_time = 7.0;     % seconds
time_step = 12.0;         % seconds
message_types = {'kMessageTypeMotorDebug', 'kMessageTypeTorqueCell'};

% Input Struct.
input_params = [];
input_params.search_path = search_path;
input_params.ref_path = ref_path;
input_params.ref_cmd = ref_cmd;
input_params.torque_bins = torque_bins;
input_params.transient_time = transient_time;
input_params.time_step = time_step;
input_params.message_types = message_types;

% TODO: Distinguish and separate input parameters and input arguments.
data = FilterData(input_params);

% Extract data.
ibus_raw = GetResult(data, 'kMessageTypeMotorDebug', ...
                     'kAioNodeMotorSbo', 'bus_current');
vbus_raw = GetResult(data, 'kMessageTypeMotorDebug', ...
                     'kAioNodeMotorSbo', 'bus_voltage');
[omega_raw, omega_avg] = GetResult(data, 'kMessageTypeMotorDebug', ...
                                   'kAioNodeMotorSbo', 'omega');
[torque_raw, torque_avg] = GetResult(data, 'kMessageTypeTorqueCell', ...
                                     'kAioNodeTorqueCell', 'torque');
omega_time = GetResult(data, 'kMessageTypeMotorDebug', ...
                       'kAioNodeMotorSbo', 'time');
torque_time = GetResult(data, 'kMessageTypeTorqueCell', ...
                        'kAioNodeTorqueCell', 'time');

% Calculate average bus power.
% For each (omega, torque) pair, the number of bus current and bus voltage data
% points should be equal.
bus_power = cellfun(@(x, y) mean(x .* y), ibus_raw, vbus_raw, ...
                    'UniformOutput', false);
bus_power = ConvertCell(bus_power);

% Interpolate torque at sampling times of omega.
torque_interp = cellfun(@(x, y, z) interp1(x, y, z, 'nearest', 'extrap'), ...
                        torque_time, torque_raw, omega_time, ...
                        'UniformOutput', false);

% Calculate average shaft power.
shaft_power = cellfun(@(x, y) mean(abs(x) .* y), torque_interp, omega_raw, ...
                      'UniformOutput', false);
shaft_power = ConvertCell(shaft_power);

% Calculate powertrain efficiency.
efficiency = shaft_power ./ bus_power * 100;

torque_avg = ConvertCell(torque_avg);
omega_avg = ConvertCell(omega_avg);

% TODO: Add more analyses beyond simple powertrain efficiency.

% 1) Contour: Powertrain Efficiency.
figure;
contourf(omega_avg, abs(torque_avg), efficiency, 'ShowText', 'on');
colormap(jet);
colorbar;

% TODO: Make this less case specific.
title('Contours of powertrain efficiency: Yasa Gen2 + 1200V-Controller.');
xlabel('Omega (rad/s).');
ylabel('Torque (Nm).');

% 2) Surface: Powertrain Efficiency.
figure;
surf(omega_avg, abs(torque_avg), efficiency);
grid('on');
colormap(jet);
colorbar;

% TODO: Make this less case specific.
title('Surface of powertrain efficiency: Yasa Gen2 + 1200V-Controller.');
xlabel('Omega (rad/s).');
ylabel('Torque (Nm).');
zlabel('Efficiency (%).');
end

function matrix = ConvertCell(cell)
% ConvertCell -- Pad empty cells in cell array with NaN and convert to matrix.
%
% matrix = ConvertCell(cell)
%
% Arguments
%
% cell: Cell array that may contain empty elements.
%
% Return Values
%
% matrix: Ordinary array converted from cell array.

cell(cellfun('isempty', cell)) = {NaN};
matrix = cell2mat(cell);
end
