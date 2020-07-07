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

function [load_flight_modes] = GetLoadFlightModeValues(h5file_loc)
  % GetLoadTimeSyncValues reads the LOADS h5 file to sync to controller time
  %
  % Usage:
  %   [load_flight_modes] = GetLoadFlightModeValues(h5file_loc)
  %
  % Uses the h5 file components in the file located at h5file_loc to extract time
  %
  % Inputs:
  %   h5file_loc - 'string' path for the location of the LOADS h5 file
  %
  % Outputs:
  %   flight_modes - [dbl] vector describing the state of the controller

  load_flight_modes = h5read(h5file_loc,'/messages/kAioNodeController/kMessageTypeImu/aio_header/flight_mode');
end