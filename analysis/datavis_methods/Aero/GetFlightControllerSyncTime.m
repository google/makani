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

function [ synced_time ] = GetFlightControllerSyncTime(flight_controller, mode_flag)
  % GetFlightControllerSyncTime - returns common standard t=0 time vector from flight_controller
  %
  % Uses flight controller and tracks the integer switch when flight achieves mode_flag
  %  and shifts the time vector to t=0 based on this occurance
  %
  % Inputs:
  %   flight_controller - the struct derived from the M600 H5 file for the flight controller
  %   mode_flag         - [int] the first mode of flight on negative side of t=0
  %
  % Outputs:
  %   sync_time - [s] n-by-1 array of time with time shifted to the Makani t=0 convention
  zero_indx = find(flight_controller.flight_mode>=mode_flag);
  t0 = flight_controller.time(zero_indx(1));
  synced_time = flight_controller.time-t0;
end
