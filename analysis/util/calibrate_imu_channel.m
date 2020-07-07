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

function [A, b] = ...
  calibrate_imu_channel(log_file, node, signal_name, t_start, t_end)
% calibrate_imu_channel -- Calculate sensor calibration from an HDF5 log.
%
% calibrate_imu_channel(log_file, node, signal_name)
% calibrate_imu_channel(log_file, node, signal_name, t_start, t_end)
% [A, b] = calibrate_imu_channel(log_file, node, signal_name)
% [A, b] = calibrate_imu_channel(log_file, node, signal_name, t_start, t_end)
%
% Arguments
%
% log_file: Path to an HDF5 log.
% node: Node name (e.g. 'FcA') which transmits FlightComputerImu messages.
% signal_name: One of 'acc' or 'mag'.
% t_start, t_end: Optional parameters specifying the interval of data
%     to use.  These times are relative to the start of the log file
%     and based on the capture_header.  If they are omitted, the user
%     will be prompted with a plot to click on to specify the
%     interval.
%
% Return Values
%
% A, b: See calibrate_three_axis_sensor.  If these return values are
%     omitted, a Python dictionary with the calibration information
%     will be printed.

if nargin < 3,
  error('At least three arguments required.');
end

% Load data.
data = load_h5log(log_file);
node = getfield(data, node);

phys_params = data.parameters.system_params.phys;
if strcmp(signal_name, 'acc')
  capture_header = node.FlightComputerImu.capture_header;
  signal = node.FlightComputerImu.message.raw.acc;
  sig_norm = phys_params.g;
  if abs(sig_norm - 9.81) > 0.03
    error('Implausible gravity constant in the parameters.');
  end
elseif strcmp(signal_name, 'mag')
  capture_header = node.FlightComputerSensor.capture_header;
  signal = node.FlightComputerSensor.message.aux.mag;
  sig_norm = norm([phys_params.mag_ned.x
                   phys_params.mag_ned.y
                   phys_params.mag_ned.z]);
else
  error('signal must be one of ''acc'' or ''mag''.');
end

time = double(capture_header.tv_sec) ...
       + 1e-6 * double(capture_header.tv_usec);
time = time - time(1);

% Allow the user to select a time period.
if nargin < 5,
    plot(time, signal);
    [t1, ~] = ginput(1);
    [t2, ~] = ginput(1);
    t_start = min(t1, t2);
    t_end = max(t1, t2);
    fprintf('Using t_start=%g, t_end=%g\n', t_start, t_end);
end

sel = time >= t_start & time <= t_end;
if strcmp(signal_name, 'acc')
  sel = sel & selection_accelerometer_data(signal, sig_norm);
  plot(time, signal, time(sel), signal(:, sel), 'k*')
  drawnow;
  pause(1);
end

% Actually calibrate the sensor.
[A, b] = calibrate_three_axis_sensor(signal(:, sel)', sig_norm);

% Print Python compatible calibration line.
if nargout < 1,
  fprintf('''%s_cal'': [\n', signal_name);
  for i = 1:3,
    fprintf('{''scale'': %g, ''bias'': %g, ''bias_count'': 0},\n', ...
            A(i,i), b(i));
  end
  fprintf(']\n');
end

end

function [sel] = selection_accelerometer_data(acc, sig_norm)
% Helper function which identifies periods where the IMU was likely still.
%
% Includes a data-point if the last W samples differ from an N
% point moving average of the signal by less than a threshold.
  threshold = 0.005 * sig_norm;
  N = 100;
  W = 10;
  acc_err = filter(ones(N, 1) / N, 1, acc, [], 2) - acc;
  sel = all(abs([hankel(acc_err(1, 1:W), acc_err(1, W:end))
                 hankel(acc_err(2, 1:W), acc_err(2, W:end))
                 hankel(acc_err(3, 1:W), acc_err(3, W:end))]) ...
            < threshold)';
  sel = [zeros(W - 1, 1); sel];
end
