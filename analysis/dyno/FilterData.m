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

function filtered_data = FilterData(input_params)
% FilterData -- Bin dyno .h5 test data.
%
% filtered_data = FilterData(input_params)
%
% Arguments
%
% input params: Structure containing necessary binning parameters.
%               .search_path -- parent folder containing the .h5 logs.
%               .ref_path -- h5 path of the reference node.
%               .ref_cmd -- reference bin dataset.
%               .torque_bins = array containing torque bin commands.
%               .transient_time = duration of transient event at start of each
%               time step in seconds.
%               .time_step = duration of each bin in seconds.
%               .message_types = cell array containing desired message types.
%
% Return Values
%
% filtered_data: Structure containing binned data for all files.

search_path = input_params.search_path;
ref_path = input_params.ref_path;
ref_cmd = input_params.ref_cmd;
bins = input_params.torque_bins;
transient_time = input_params.transient_time;
time_step = input_params.time_step;
message_types = input_params.message_types;

% Recursively open and process only .h5 files that do not begin with "format" in
% search path.
all_files = get_files(search_path);
h5_indices = ~cellfun(@isempty, regexp(all_files, '(?<!format.*)\.h5'));
h5_files = all_files(h5_indices);
filtered_data = struct();
start = tic();

for i = 1:length(h5_files)
  tic();
  f = h5_files{i};
  try
    q = regexp(f, '(?<=omega_)\d*\.\d*(?=_rps)', 'match');
    omega_val = str2double(q{1});
    nominal_omega = sprintf('nominal_omega_%d', omega_val);
    file_data = ExtractData(f);
    filtered_data.(nominal_omega) = file_data;
    duration = toc();
    fprintf('\tTime elapsed: %.2f seconds.\n', duration);
  catch
    fprintf('\nInvalid file: %s. Ignored.\n\n', f);
    continue
  end
end

runtime = toc(start);
if ~isempty(fieldnames(filtered_data))
  fprintf('\nSuccess! Total processing time: %.2f seconds.\n', runtime);
else
  fprintf('\nNo files processed.\n');
end


function file_data = ExtractData(file_name)
  % ExtractData -- Pull out only relevant data from a .h5 file.
  %
  % file_data = ExtractData(file_name)
  %
  % Arguments
  %
  % file_name: Filename of H5 log.
  %
  % Return Values
  %
  % file_data: Structure containing binned data for current file.

  fprintf('\nCurrent file: %s.\n', file_name);
  fprintf('\tReading H5 log ');

  % Get AIO message info.
  info = h5info(file_name);
  for l = 1:length(info.Groups)
    if strcmp(info.Groups(l).Name, '/messages')
      info = info.Groups(l);
      break;
    end
  end

  % Catch invalid H5 log.
  if isempty(info)
    error('\nInvalid H5 log.\n');
  end

  % Read desired datasets from H5 log.
  datasets = containers.Map();
  for m = 1:length(info.Groups)
    % Check if node is present.
    for n = 1:length(info.Groups(m).Datasets)
      if any(arrayfun(@(msg) strcmp(info.Groups(m).Datasets(n).Name, ...
          msg), message_types))
        fprintf('.');
        dataset_path = strcat(info.Groups(m).Name, '/', ...
          info.Groups(m).Datasets(n).Name);
        datasets(dataset_path) = h5read(file_name, dataset_path);
      end
    end
  end
  fprintf(' Done.\n');

  % Bin reference dataset.
  datapaths = keys(datasets);
  data_struct = AddTime(datasets(ref_path));
  bin_times = BinReference(data_struct, ref_cmd, bins, time_step);

  % Bin data.
  file_data = struct();
  for k = 1:length(datapaths)
    path = datapaths{k};
    fprintf('\t%s: Processing ... ', path);
    d = datasets(path);
    d_struct = AddTime(d);
    split_path = strsplit(path, '/');
    msg_type = split_path{end};
    node = split_path{end-1};
    file_data.(msg_type).(node) = BinData(d_struct, bin_times, ...
      bins, time_step, transient_time);
    fprintf('Done.\n')
  end
end


function d_path = AddTime(d_path)
  % AddTime -- Add relative time points to `message` datasets.
  %
  % d_path = AddTime(d_path)
  %
  % Arguments
  %
  % d_path: Structure containing data in current H5 path.
  %
  % Return Values
  %
  % d_path: Input structure with relative time points added.

  sec = double(d_path.capture_header.tv_sec);
  usec = double(d_path.capture_header.tv_usec);
  t = (sec - sec(1)) + (usec - usec(1)) * 1e-6;
  d_path.message.time = t;
end


function bin_times = BinReference(data_struct, ref_cmd, bins, time_step)
  % BinReference -- Extract useful data ranges from reference dataset.
  %
  % bin_times = BinReference(data_struct, ref_cmd, bins, time_step)
  %
  % Arguments
  %
  % data_struct: Structure of raw data to be binned.
  % ref_cmd: String of reference command to use for binning.
  % bins: Cell array of values to bin.
  % time_step: Value of interval for each bin level.
  %
  % Return Values:
  %
  % bin_times: Map container of starting time points for each bin.

  fprintf('\tBinning reference dataset ... ');
  ref_data = data_struct.message.(ref_cmd);

  % The start index is defined as the index of the first reference data value
  % that matches the first bin.
  start_index = find(abs(ref_data - bins(1)) < 2 * eps('single'), 1);
  start_time = data_struct.message.time(start_index);
  times = start_time + (0:length(bins) - 1) * time_step;
  bin_times = containers.Map(bins, times);
  fprintf('Done.\n')
end


function binned_data = ...
    BinData(d_struct, bin_times, bins, time_step, transient_time)
  % BinData -- Extract useful data ranges from given dataset.
  %
  % binned_data = BinData(d_struct, bin_times, bins, time_step, transient_time)
  %
  % Arguments
  %
  % d_struct: Structure of raw data to be binned.
  % bin_times: Map container of starting time points for each bin.
  % bins: Cell array of values to bin.
  % time_step: Value of interval for each bin level.
  % transient_time: Value of time interval to filter out transients.
  %
  % Return Values
  %
  % binned_data: Structure containing binned data for current file.

  t = d_struct.message.time;

  for s = 1:length(bins)
    bin = bins(s);
    start_time = bin_times(bin);

    % Find the index of the raw data time value nearest to the reference
    % data bin time value.
    [~, first_idx] = min(abs(t - start_time));
    [~, stop_idx] = min(abs(t - (start_time + time_step)));
    stop_idx = stop_idx - 1;  % Remove index of transition point to next bin.

    % Filter Transients.
    t_slice = t(first_idx:stop_idx);
    base_time = t_slice(1);
    start_idx = find(t - base_time > transient_time, 1);
    data_fields = fieldnames(d_struct.message);
    nominal_bin = sprintf('nominal_torque_%s', num2str(bin));

    % Assign filtered file data.
    for h = 1:length(data_fields)
      field = char(data_fields(h));
      try
        binned_data.(nominal_bin).(field) = ...
          d_struct.message.(field)(start_idx:stop_idx);
      catch
        continue
      end
    end
  end
end
end
