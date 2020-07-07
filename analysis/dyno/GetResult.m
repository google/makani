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

function [raw_data, avg_data] = GetResult(data, msg_type, node, dataset)
% GetResult -- Extract useful data ranges from reference dataset.
%
% [raw_data, avg_data] = GetResult(data, msg_type, node, dataset)
%
% Arguments
%
% data: Structure containing binned data.
% msg_type: String of message type of requested data.
% node: String of node type of requested data.
% data_set: String of specific dataset of requested data.
%
% Return Values
%
% raw_data: 2D cell array of raw data for all (omega, torque) pairs.
% avg_data: 2D cell array of averaged data for all (omega, torque) pairs.

omegas = fieldnames(data);
num_omegas = numel(omegas);

% Find maximum number of torque bins.
% This step is added for future compatibility with binned datasets that do not
% necessarily have the same number of torque bins for each omega.
max_num_torques = 0;
for i = 1:num_omegas
  omega = omegas{i};
  torques = fieldnames(data.(omega).(msg_type).(node));
  num_torques = numel(torques);
  if num_torques > max_num_torques
    max_num_torques = num_torques;
  end
end

% Preallocate cell arrays.
raw_data = cell(num_omegas, max_num_torques);
avg_data = cell(num_omegas, max_num_torques);

% Populate cell arrays with binned data.
for i = 1:num_omegas
  omega = omegas{i};
  torques = fieldnames(data.(omega).(msg_type).(node));
  for j = 1:numel(torques)
    torque = torques{j};
    vals = data.(omega).(msg_type).(node).(torque).(dataset);
    % Assign raw_data.
    raw_data{i, j} = vals;
    % Assign avg_data.
    avg_data{i, j} = mean(vals);
  end
end
