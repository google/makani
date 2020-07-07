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

function [calibrated_data] = apply_calibration (data_kMessageTypeStrain, calibration_file, path_to_file)
% apply_calibration -- spplies the calibration as per the calibration_file. 
% [calibrated_data] = apply_calibration (data_kMessageTypeStrain, calibration_file, path_to_file)
%
% Arguments-
%
% data_kMessageTypeStrain: strain data  
% calibration_file: string specifying the .cal file to be read 
% path_to_file: Optional, absolute path to the file. Default is working directory
%
% Return Values-
%
% calibrated_data: calibrated load from strain using .cal file 
% calibrated_data.kAioNode<kite_comp_type>.kMessageType<sensor_type>.<sensor_location>
% ... <sensor_location>.message
%                      .message.<data_channels> -- all the data channels with proper labels
%                      .aio_header
%                      .aio_header.sample_rate -- the rate at which the data under message was recorded
%                      .aio_header.units -- engineering units of the data under message
%                      .aio_header.timestamp_utc -- utc clock of the sensor, sec from beginning of the day
%                      .aio_header.date_utc -- utc date as specified in the data file, YYYYMMDD format
%                      .aio_header.cal_data -- calibration file, exists only for derived data e.g. strains --> loads
%                      .capture_header
%                      .capture_header.tv_sec -- utc time synced with controller time, sec  
%                      .capture_header.tv_usec -- utc time synced with controller time, micro sec
%                      .capture_header.tdelta -- delta time (sec), t = 0 corresponds to flight mode switch from 1 to 2

%% sample .cal file
%  % Fuselage strain gauge calibration file
%  % Date: 20160617
%  % gauge_loc = N2000
%  % units = kN
%  % units = kNm
%  % start of cal data
%  Fy,	-0.011098*10^6, B_IP_F
%  Fz, -0.009599*10^6, B_OP_F
%  Mx,  0.038034*10^6, T_F
%  My, -0.066393*10^6, B_OP_F
%  Mz,  0.076765*10^6, B_IP_F
%  Mx_r, -0.01527*10^6, R_P45_Q
%  Mx_r,  0.01527*10^6, R_N45_Q
%  % end of cal data

% read the calibration file
cal_data = fileread([path_to_file, calibration_file]);

gauge_loc = strrep(regexp(cal_data,'gauge_loc = \w*','match'), 'gauge_loc = ', '');

units = strrep(regexp(cal_data,'units = \w*','match'), 'units = ', '');

sensor_loc = fieldnames (data_kMessageTypeStrain);

for ii = 1:numel(sensor_loc);
	if strcmp (sensor_loc{ii}, gauge_loc{1}) == 1;
		data_in =  eval (['data_kMessageTypeStrain.' sensor_loc{ii} '.message;']);
		data_out = [];

		% create a sandox to work the calibrations
		ch_names = fieldnames (data_in);

		% load all the variables under the message
		for jj = 1:numel(ch_names);
			eval ([ch_names{jj} '= data_in.' ch_names{jj} ';']);
		end
		clear jj;

		% read line -by-line and evaluate statements to apply calibration
		fid = fopen ([path_to_file, calibration_file]);
		ll = '%';
		while isempty (strfind (ll, 'end')) == 1;
			ll = fgetl(fid);
			if isempty (strfind (ll, '%')) == 1;
				xx = strsplit(ll, ',');

				% for evaluating a multi variable calibration,
				if exist (xx{1}) == 1
					cal_expr = [xx{1} '=' xx{1} ' + ' xx{2} ' * ' xx{3} ';'];
				else
					cal_expr = [xx{1} '=' xx{2} ' * ' xx{3} ';'];
				end
				eval(cal_expr);

				% prevent intermediate variables from being saved
				if isempty(strfind(xx{1}, 'temp')) == 1
					% use the first index value to zero the calibrated data variable
					cal_expr = ['data_out.' xx{1} ' = ' xx{1} ' - ' xx{1} '(1);'];
					eval(cal_expr);
				end
			end
		end
		fclose (fid);
    
    % for fuselage the gages are calibrated for wing-fuse location
		if strfind(calibration_file, 'fuse') == 1;
			gauge_loc{1} = 'P0';
		end

		% write the output data
		eval (['kMessageTypeLoad.' gauge_loc{1} '.message = data_out;']);
		eval (['kMessageTypeLoad.' gauge_loc{1} '.capture_header = data_kMessageTypeStrain.' sensor_loc{1} '.capture_header;']);
		eval (['kMessageTypeLoad.' gauge_loc{1} '.aio_header = data_kMessageTypeStrain.' sensor_loc{1} '.aio_header;']);
		
		eval (['kMessageTypeLoad.' gauge_loc{1} '.aio_header.units = units;']);
		eval (['kMessageTypeLoad.' gauge_loc{1} '.aio_header.cal_data = cal_data;']);
	end
end
calibrated_data = kMessageTypeLoad;
