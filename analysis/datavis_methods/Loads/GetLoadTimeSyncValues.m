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

function [time] = GetLoadTimeSyncValues(h5file_loc, zoc_id)
  % GetLoadTimeSyncValues reads the LOADS h5 file to sync to controller time
  %
  % Uses the h5 file components in the file located at h5file_loc to extract time
  % Note the time is for a specific kAioMessageType, this function is to get pressure data timestamp
  %
  % Inputs:
  %   h5file_loc - 'string' path for the location of the LOADS h5 file
  %   zoc_id     - Optional 'string', choose 'psi' or 'h2o', Default is 'psi'
  %
  % Outputs:
  %   time - [seconds] vector containing the time synced to controller zero
  %

  % incase of only one input, default to psi zoc module
  if nargin == 1;
    zoc_id = 'psi';
  elseif nargin == 2 && isempty(zoc_id);
    zoc_id = 'psi';
  end

  % correct h2o module label, the config file has zero instead of 'o' as label
  if nargin == 2 && strcmp(zoc_id, 'h2o');
    zoc_id = 'h20';
  end

  % zoc data path
  data_path = ['/messages/kAioNodeWing/kMessageTypePressure', upper(zoc_id),'/P5000/capture_header/tdelta'];

  time = h5read(h5file_loc, data_path);
  time = transpose (time);
end