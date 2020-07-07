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

function [info_zocbox, data_zocbox] = read_zocbox_data (data_raw, line_read1, line_read2);
% read_zocbox_data -- Reads and formats data from zocbox data file
% [info_zocbox, data_zocbox] = read_zocbox_data (data_raw, line_read1, line_read2)
%
% Arguments-
%
% data_raw:   csv data with zocbox data 
% line_read1: header line 1 from data file, contains channel label information
% line_read2: header line 2 from data file, contains taps location information
% 
% Return Values-
%
% info_zocbox: 
% info_zocbox.timestamp
% info_zocbox.tempC
% info_zocbox.type
% data_zocbox: data sorted into channels based on pressure tap location

% sort into channel labels
labels = textscan(line_read1,'%s','delimiter',',');

% pressure tap locations on the wing mainplane
loc_info = textscan(line_read2,'%s','delimiter',',');
loc_grid = regexp (loc_info{1,1}, '\d{1,1}.\d{3,3}', 'match');

% the first column is instrument specific timestamp
labels{1,1}{1,1} = 'timestamp';
labels{1,1}{end-3,1} = 'tempC';

% then loop through the columns, 
for ii = 1:size(data_raw,2)-3;
	vrnm = genvarname([labels{1,1}{ii, 1}]);

	if ii == 1 || ii == size(data_raw,2)-3;
		    	
    else
    % break variable name for identification
		ch_ids = strsplit(vrnm, '_');
		vrnm = (['data_zocbox.' ch_ids{1} '.value']);

		vrnm_loc = (['data_zocbox.' ch_ids{1} '.loc_xc']);
		eval([vrnm_loc '= str2num(loc_grid{ii,1}{1,1});']);
	end

  eval([vrnm '= data_raw(:,ii);']);
end

% write the information about the zoc module
info_zocbox.timestamp = timestamp;
info_zocbox.tempC = tempC;
info_zocbox.type = ch_ids{2}; 
