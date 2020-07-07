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

function [t0, control_data, t] = GetFlightStartTime(filename, rescale_time)
% GetFlightStartTime -- Extract flight start time from control telemetry.
%
% [t0, control_message, t] = GetFlightStartTime(filename)
% Find and return the time of flight initiation as defined by the transition of
% flight_mode from 1 to a different flight_mode.  Initially searches for
% kMessageTypeControlDebug and falls back to kMessageTypeControlTelemetry.
% Returns zero if no data is available or if no appropriate flight_mode
% transition is available.  Uses first transition discovered if multiple
% available.  Time is based on local data capture time.
%
% Arguments
%
% filename: Name of H5 file with flight data.
% recale_time: Optional boolean to select linearize time to clean up noise.
%
% Return values
%
% t0: Time of first sample after transition from flight_mode == 1.
% control_data: A message with flight_mode in it - ground or flight recorder
% t: time vector (has not had t0 subtracted out, yet)
%
% Required toolboxes: None.

if nargin < 2
  rescale_time = false;
end

% Try debug message first.
control_data = GetUniqueData(filename, 'ControllerA', ...
                             'kMessageTypeControlDebug');

% Extract time vectors.
if ~isfield(control_data, 'null')
  t = GetTimeFromStruct(control_data, 'raw', 0.0, rescale_time);
else
  % Try again with control telemetry message.
  control_data = GetUniqueData(filename, 'ControllerA', ...
                               'kMessageTypeControlTelemetry');
  if ~isfield(control_data, 'null')
    t = GetTimeFromStruct(control_data, 'raw', 0.0, rescale_time);
  else
    disp('No valid flight_mode signal found');
    t0 = 0; % Use local file start time.
    return;
  end
end

% Find all transitions from 1 to another flight_mode.
flight_mode_changes = (control_data.message.flight_mode(1:(end - 1)) == 1) & ...
  (control_data.message.flight_mode(2:end) ~= 1);

% Shift to get first time of new mode
candidate_zeros = t([false; flight_mode_changes]);

% Pick out the first transition if available
if (~isempty(candidate_zeros))
  t0 = candidate_zeros(1);
else
  disp('No flight_mode start condition found');
  t0 = 0.0;
end

end

