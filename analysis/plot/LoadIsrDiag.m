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

function [t, data, dropped] = LoadIsrDiag(filename, motor)
% LoadIsrDiag -- Load and chronologically reconstruct MotorIsrDiagMessage data.
%
% [t, data, dropped] = LoadIsrDiag(filename, motor).
%
% Load the ISR diagnostic data corresponding to motor from filename. The
% buffered data is reconstructed into linear arrays with monotonically
% increasing time and missing data is reported in the dropped array.
%
% Arguments:
%
% filename: Full or relative path of the HDF5 file to load.
% motor: Scalar index of motor to load or string with motor short name, e.g.
%        'Sbo'.  Also, can be full aio path string to support dyno motors.
%
% Return Values:
%
% t: Vector of times since the start of the first sample.
% data: Struct with fields corresponding to the members of MotorIsrDiagMessage.
%       Each field is a vector corresponding to t. Missing data is padded with
%       zeros.
% dropped: Vector of booleans corresponding to t. True indicates that the
%          corresponding data is missing at that time.

  dt = 66.8e-6;  % Sample step size.

  % Convert motor index or short name to aio node path.
  if (~ischar(motor))
    motor = GetMotorShortNames(motor);
    motor = Capitalize(motor{1});
  end

  % If motor is a full path, extract path from string, otherwise assume
  % short form.  This makes us compatible with dyno motors in H5Plotter.
  s = string(motor);
  if (~s.contains('messages'))
    aio_node_path = ['/messages/kAioNodeMotor' motor];
  else
    temp = s.extractBefore(strfind(s,'/kMessageTypeMotorIsrDiag'));
    aio_node_path = temp.char;
  end

  % Load data from disk.
  info_struct = h5info(filename, aio_node_path);
  if (~isempty(info_struct.Datasets) ...
      && any(strcmp({info_struct.Datasets.Name}, 'kMessageTypeMotorIsrDiag')))
    % The HDF5 log contains a MotorIsrDiagMessage from motor_name.
    msg = h5read(filename, [aio_node_path '/kMessageTypeMotorIsrDiag']);
    msg = msg.message;
  else
    % If a dataset is not available in an HDF5 log, Matlab's h5read prints a
    % fair bit of warning information before throwing an error. Instead, bypass
    % h5read and simply return empty datasets.
    t = [];
    data = [];
    dropped = [];
    return;
  end

  % Sort messages to deal with out-of-order and dropped packets.
  % Eventually, overflow of the sample count will need to be considered, but
  % it's a 32 bit unsigned integer; at 15 kHz it only wraps once every
  % 79.5 hours.
  [msg.total, i_sort] = sort(msg.total);

  % Sort all data fields in msg.
  field_names = fieldnames(msg);
  field_names = field_names(~strcmp(field_names, 'total') ...
                            & ~strcmp(field_names, 'num_samples'));
  num_fields = length(field_names);
  for i_name = 1:num_fields
    name = field_names{i_name};
    msg.(name) = msg.(name)(:, i_sort);
  end
  msg.num_samples = msg.num_samples(i_sort);

  % Calculate the cumulative number of samples. Note that both msg.total and
  % cum_samples refer to the start of the message interval.
  num_samples = msg.num_samples;
  cum_samples = msg.total - msg.total(1);
  n_samples = cum_samples(end) + num_samples(end);
  t = double(0 : n_samples - 1)' * dt;

  % Initialize the output data struct.
  data = cell2struct(cell(length(fieldnames(msg)), 1), fieldnames(msg), 1);
  data.total = zeros(n_samples, 1);
  data.num_samples = zeros(n_samples, 1);

  % Setup the mapping from the raw data matrices to output vectors.
  ind_select = false(size(msg.(field_names{1})));
  map_to = uint32(zeros(size(ind_select)));
  for i = 1:length(num_samples)
    ind_select(1:num_samples(i), i) = true;
    map_to_i = (cum_samples(i) + 1) : (cum_samples(i) + num_samples(i));
    map_to(1:num_samples(i), i) = map_to_i;

    % Also expand the total and num_samples fields while map_to_i is
    % available.
    data.total(map_to_i) = msg.total(i);
    data.num_samples(map_to_i) = msg.num_samples(i);
  end
  ind_select = reshape(ind_select, numel(ind_select), 1);  % Vectorize
  map_to = reshape(map_to, numel(map_to), 1);  % Vectorize
  map_to = map_to(ind_select);  % Remove entries without data.

  % Transfer data to the output struct.
  for i_name = 1:num_fields
    name = field_names{i_name};
    field = reshape(msg.(name), numel(ind_select), 1);
    data.(name)(map_to) = field(ind_select);
  end

  % Record which elements in the output vectors are not set, i.e. which samples
  % are dropped.
  dropped = true(n_samples, 1);
  dropped(map_to) = false;
end

function [str] = Capitalize(str)
% Capitalize the first letter in str.
  str = [upper(str(1)), str(2:end)];
end
