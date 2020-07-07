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

function [flight_modes, crosswind_cases] = get_modes_and_cases(control_dataset)
% get_modes_and_cases -- extract timestamps of flight modes and crosswind cases.
% [flight_modes crosswind_cases] = get_modes_and_cases(control_dataset)
%
% Crosswind cases are numeric identifiers for various configurations tested
% during CrosswindNormal flight mode. For repeated cases, an alphabet is
% appended to case name (case 6a, case 6b ....).
%
% Arguments-
%
% control_dataset: [struct] dataset from controller h5 file at
%                           messages.kAioNodeControllerA.kMessageTypeControlDebug.
% RPX id:          [dbl, optional] Specify RPX id, e.g. 5 for RPX-05, 11 for RPX-11.
%
% Return Values-
% flight_modes   : [struct] information about all flight modes
%                  .all_modes    - list of all flight modes during the flight
%                  .time_in_mode - start and end time for each flight mode
%                  .indices      - start and end array index for each mode
% crosswind_cases: [struct] information about all crosswind cases
%                  .all_cases    - list of all crosswind during the crosswind
%                  .time_in_case - start and end time for each crosswind case
%                  .indices      - start and end array index for each case
%

% Check if correct number of arguments are supplied.
if nargin ~= 1
  error('Maximum one arguments expected.')
end

% Check if the user has supplied correct data structure.
if isstruct(control_dataset) && isfield(control_dataset, 'message')
  if isfield(control_dataset.message, 'flight_mode')
    data_read = control_dataset;
  end
else
  error ('get_modes_and_cases error, check dataset.');
end

% Get the flight mode labels.
[ids, labels] = get_flight_mode_labels;
ids_by_name = containers.Map(labels, ids);
names_by_id = containers.Map(ids, labels);

% Find the identifier corresponding to crosswind mode.
cw_mode_id = ids_by_name('CrosswindNormal');

% Complete array of flight modes.
flight_mode = data_read.message.flight_mode;

% Get the controller timestamp.
cont_time = get_node_time(data_read);
cont_time = cont_time(:);

% Find t=0 from flight mode change from Perched to HoverAscend.
start_t0_idx = find(flight_mode == ids_by_name('HoverAscend'));
t0_index = start_t0_idx(1);
t0_value = cont_time(t0_index);

% Offset controller time relative to t = 0.
cont_time = cont_time - t0_value;
clear start_t0_idx t0_index t0_value;

% Find unique flight mode.
flight_mode_u = unique(flight_mode);
flight_mode_u = flight_mode_u(:);

% Generate labels for modes.
for ii = 1:size(flight_mode_u, 1)
  mode_id{ii, 1} = names_by_id(flight_mode_u(ii));
end
clear ii;

% complete array of crosswind cases.
if isfield (data_read.message.state_est, 'experimental_crosswind_config')
  flight_case = int32(data_read.message.state_est.experimental_crosswind_config).*...
                int32(data_read.message.state_est.joystick.pitch_f < -0.5);
  % Find unique cases.
  flight_case_u = unique(flight_case);
  flight_case_u = flight_case_u(:);
  % Generate labels for cases.
  case_id = strcat('case', num2str(flight_case_u));
else
  flight_case = [];
  flight_case_u = [];
  case_id = [];
end

% Alphabets will be sequentially appended in case of repeated mode or case.
sub_id = 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ';

% find the index for flight modes.
for ii = 1:size(mode_id, 1)
  idx_temp = find(flight_mode == flight_mode_u(ii));

  % Check if there are multiple identical modes in one flight.
  check_mul = find(diff(idx_temp) ~= 1);
  if ~isempty(check_mul)
    for jj = 1:length(check_mul) + 1
      id_temp = [mode_id{ii}, sub_id(jj)];
      if jj == 1
        idx_mode.(id_temp) = [idx_temp(1), idx_temp(check_mul(jj) - 1)];
      elseif jj == length(check_mul) + 1
        idx_mode.(id_temp) = [idx_temp(check_mul(jj-1) + 1), idx_temp(end)];
      else
        idx_mode.(id_temp) = [idx_temp(check_mul(jj-1) + 1), idx_temp(check_mul(jj) - 1)];
      end
      time_in_mode.(id_temp) = cont_time(idx_mode.(id_temp));
    end
    clear jj;
  else
    id_temp = mode_id{ii};
    idx_mode.(id_temp) = [idx_temp(1), idx_temp(end)];
  end
  time_in_mode.(id_temp) = cont_time(idx_mode.(id_temp));
end
clear ii idx_temp check_mul id_temp;

% Find the index for crosswind cases.
for ii = 1:size(case_id, 1)
  idx_temp = find(flight_mode == cw_mode_id & flight_case == flight_case_u(ii));

  % Check if there are repeated crosswind cases in one flight.
  check_mul = find(diff(idx_temp) ~= 1);
  if ~isempty(check_mul)
    for jj = 1:length(check_mul) + 1
      id_temp = [strrep(case_id(ii,:), ' ', ''), sub_id(jj)];
      if jj == 1
        idx_case.(id_temp) = [idx_temp(1), idx_temp(check_mul(jj) - 1)];
      elseif jj == length(check_mul) + 1
        idx_case.(id_temp) = [idx_temp(check_mul(jj-1) + 1), idx_temp(end)];
      else
        idx_case.(id_temp) = [idx_temp(check_mul(jj-1) + 1), idx_temp(check_mul(jj) - 1)];
      end
      time_in_case.(id_temp) = cont_time(idx_case.(id_temp));
    end
    clear jj;
  else
    id_temp = strrep(case_id(ii,:), ' ', '');
    idx_case.(id_temp) = [idx_temp(1), idx_temp(end)];
  end
  time_in_case.(id_temp) = cont_time(idx_case.(id_temp));
end
clear ii idx_temp check_mul id_temp sub_id flight_mode flight_case case_id mode_id;

% Get labels for all the modes and write the output data structure.
all_modes = fieldnames(idx_mode);
flight_modes.all_modes       = all_modes;
flight_modes.time_in_mode    = time_in_mode;
flight_modes.indices         = idx_mode;

% Get labels for all the cases and write the output data structure.
if isfield (data_read.message.state_est, 'experimental_crosswind_config')
  all_cases = fieldnames(idx_case);
  crosswind_cases.all_cases    = all_cases;
  crosswind_cases.time_in_case = time_in_case;
  crosswind_cases.indices      = idx_case;
else
  % simply pass an empty structure if crosswind cases were not used in this flight
  crosswind_cases = struct();
end
