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

function [xc_location, pressure_val] = GetZocBoxPressureTapInfo(h5file_loc, zoc_id, unit)
  % GetZocBoxPressureInfo collects ZocBox pressure tap location and pressure data
  %
  % Uses the h5 file components in the file located at h5file_loc to extract time, pressure, and location data
  %
  % Inputs:
  %   h5file_loc - 'string' path for the location of the LOADS h5 file
  %   zoc_id     - Optional 'string', choose 'psi' or 'h2o', Default is 'psi'
  %   unit       - Optional 'string', choose 'psi' or 'Pa', Default is 'Pa'
  %
  % Outputs:
  %   xc_location  - [n x 1] chordwise x/c position of the pressure taps
  %   pressure_val - [n x m_timestep] measured pressure fron ZocBox converted to Pa or psi,
  %                                   where m_timestep is the number of time samples
  %

  % set defaults as 'psi' zoc module and 'Pa' pressure_val output units
  if nargin == 1;
    zoc_id = 'psi';
    unit   = 'Pa';
  elseif nargin == 2;
    if isempty(zoc_id);
      zoc_id = 'psi';
    end
    unit   = 'Pa';
  elseif nargin == 3;
    if isempty(zoc_id);
      zoc_id = 'psi';
    end
    if isempty(unit);
      unit   = 'Pa';
    end
  end

  % correct h2o module label, the config file has zero instead of 'o' as label
  if nargin >= 2 && strcmp(zoc_id, 'h2o');
    zoc_id = 'h20';
  end

  %=====================================================================================================================
  %% Data Extraction from h5file
  %=====================================================================================================================
  % zoc data path
  data_path = ['/messages/kAioNodeWing/kMessageTypePressure', upper(zoc_id),'/P5000/message'];

  % read the info at the path and find no of port
  data_info = h5info(h5file_loc, data_path);
  no_ports = numel(data_info.Groups);

  % initialize outputs
  xc_location  = [];
  pressure_val = [];

  for ii = 1:no_ports;
    % locations
    h5_loc = h5read(h5file_loc, [data_info.Groups(ii).Name, '/loc_xc']);
    xc_location = [xc_location; h5_loc];

    % pressures, as measured in psi
    h5_press_psi = h5read(h5file_loc, [data_info.Groups(ii).Name, '/value']);
    pressure_val = [pressure_val; h5_press_psi];
  end

  % convert the pressure from psi to Pa
  if strcmp(unit, 'Pa');
    pressure_val = pressure_val * 6894.75729;
  end

end