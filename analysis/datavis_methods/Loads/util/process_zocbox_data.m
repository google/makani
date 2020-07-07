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

function [zoc_data] = process_zocbox_data(zocbox_struct)
% process_zocbox_data -- collate the zocbox data. 
% [zoc_data] = process_zocbox_data(zocbox_structure)
% location data is non-dimensional [x/c] amd
% pressure data is [psi]
%
% Arguments-
%
% zocbox_struct: Input structure containing zocbox data
%                e.g. messages.kAioNodeWing.kMessageTypePressurePSI
%                     messages.kAioNodeWing.kMessageTypePressureH2O
% 
% Return Values-
%
% zoc_data: output structure; 
%           zoc_data.upper.loc
%           zoc_data.upper.psi
%           zoc_data.lower.loc
%           zoc_data.lower.psi
%

% initialize output arrays
upper_loc = [];
upper_psi = [];
lower_loc = [];
lower_psi = [];

% find the spanwise location of pressure ports
loc = fieldnames(zocbox_struct);

% all the port data
all_port_data = getfield(getfield (zocbox_struct, loc{1,1}), 'message');

% get the port labels
port_id = fieldnames(all_port_data);

% loop through each port data and collate them in the output variables
% separate them into upper and lower surface ports

for ii = 1:numel(port_id);
  % port specific data
  port_data = getfield(all_port_data, port_id{ii,1});
  % separate into upper and lower surface
  if ~isempty(strfind(port_id{ii,1}, 'U'));
    upper_loc = [upper_loc; port_data.loc_xc];
    upper_psi = [upper_psi, port_data.value];
  elseif ~isempty(strfind(port_id{ii,1}, 'L'));
    lower_loc = [lower_loc; port_data.loc_xc];
    lower_psi = [lower_psi, port_data.value];
  end
end

% write the output data structure
zoc_data.upper.loc = upper_loc;
zoc_data.upper.psi = upper_psi;

zoc_data.lower.loc = lower_loc;
zoc_data.lower.psi = lower_psi;
