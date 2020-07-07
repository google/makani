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

function [time_zero] = get_time_zero(control_dataset, info_dataset)
% get_time_zero -- get the zero reference time.
% [time_zero] = get_time_zero(control_dataset, info_dataset)
%
% The reference time is defined as transition from mode 1 to 2 in flight.
% The function returns time in seconds from beginning of first test day in the
% dataset. This can be subtracted from all time arrays to reference time to
% common zero.
% In case partial dataset are supplied or if transition criterion is not meet,
% a zero value is returned as output. This function can then be called in a loop
% for partial logs and first non-zero value can be chosen as reference.
%
% Suggested usage-
% control_dataset = h5read(h5file, '/messages/kAioNodeControllerA/kMessageTypeControlDebug');
% info_dataset    = h5read(h5file, '/info');
% time_zero = get_time_zero(control_dataset, info_dataset);
% ctime = get_node_time(control_dataset, info_dataset);
% flight_time = ctime - time_zero;
%
% Arguments-
%
% control_dataset: [struct] messages.kAioNodeControllerA.kMessageTypeControlDebug
%                           ControlDebug dataset from h5 file
% info_dataset:    [struct] dataset containing GPS sync '/info'
%                           Optional if only capture time zero is desired
%
% Output Values-
%
% time_zero: [dbl] time be subtracted from all time arrays to reference time to
%                  common zero.
%

if nargin == 1
  info_dataset = struct();
end

% get time array
controller_time = get_node_time(control_dataset, info_dataset);

% flight modes from control dataset
flight_mode = control_dataset.message.flight_mode;

% index where transition criterion is satisfied
mode_2_index = find(flight_mode == 2, 1, 'first');

% if mode 2 exists, use the index to get the reference time, provided it is
% preceded by mode 1. In all the other cases, set it to zero.
% Note: this will also return zero if the one log ends at mode 1 and
% the next log starts at mode 2.
if ~isempty(mode_2_index)
  if mode_2_index > 1 & flight_mode(mode_2_index - 1) == 1
    time_zero = controller_time(mode_2_index);
  end
else
  time_zero = 0;
end