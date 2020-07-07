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

% script to process the airframe telemetry data acquired by structural loads monitoring system.
% this loads monitoring system is a standalone unit, installed in pylon 4 cavity.
%
clear all; close all; clc

%% declare all the paths here

base_path = pwd;

% the path to airframe data files
path_to_airframe_files = [base_path, '/airframe_data/'];

% the path to airframe data files
path_to_controls_files = [base_path, '/controls_data/'];

% the path to calibration files
path_to_cal_files = [base_path, '/cal_files/'];

% the path to configuration file
path_to_config_file = [base_path, '/'];

% the path to exported data
path_to_export_files = [base_path, '/export/'];

%% find flight id (e.g. rpx-## or hover)
[~,flight_id,~] = fileparts (base_path);

fprintf('*** Processing structural loads data for %s ***\n \n', upper(flight_id));
fprintf('==================================================\n \n');

%% initialize the data structure, message.kAioNode<comp_type>
% there is one Node for each kite component type
messages.kAioNodeWing  = [];
messages.kAioNodeFuse  = [];
messages.kAioNodeHtail = [];
messages.kAioNodeVtail = [];
messages.kAioNodeRotor = [];
messages.kAioNodePylon1 = [];
%messages.kAioNodePylon2 = [];
%messages.kAioNodePylon3 = [];
messages.kAioNodePylon4 = [];
messages.kAioNodeController = [];

fprintf('Reading controls telemetry data... \n \n');

%% compile all the h5 files to read
s = dir([path_to_controls_files '*.h5']);
for ii = 1:numel(s)
	file2read(ii,:) = getfield (s(ii),'name');
end
clear s ii;

% find the date from the controls telemetry file name
date_pattern = '\d{4,4}\d{2,2}\d{2,2}';
date_extracted = regexp(file2read(1,:),date_pattern,'match');
if isempty(date_extracted) ~= 1
  date_formatted = date_extracted{1,1};
else
  date_formatted = [];
end

% read the data from control telemetry files
messages.kAioNodeController = get_control_telemetry_data (file2read, path_to_controls_files);

fprintf('Reading structural loads data... \n \n');

fprintf('Syncing structural and airframe data... \n \n');
% timing parameters required to sync the airframe data with controller data
sync_parameters = get_sync_parameters(messages, path_to_airframe_files);
fprintf('Sync done, continue reading structural loads data... \n \n');

%% find all the csv files and load them one at a time
s = dir([path_to_airframe_files '*.csv']);

for ss = 1:numel(s)
  % get the filename to load
  file2read = getfield(s(ss),'name');
  fprintf([file2read '\n']);

  % read the instrument data file
  [data_read] = read_instrument_data(file2read, path_to_airframe_files, sync_parameters);

  % append this data to initialized data structure
  [messages] = append_to_data_structure (messages, data_read);
end
clear s ii;

% write the info structure
info.flight_id = flight_id;

info.date_utc = date_formatted;
info.start_timestamp_utc = sync_parameters(3);
info.end_timestamp_utc = sync_parameters(4);

% store das config file into info structure
s = dir([path_to_config_file, '*.cfg']);
info.das_cfg_name = s.name;
info.das_cfg = fileread(s.name);
clear s;


% apply some flight specific corrections
if flight_id == 'rpx-03'
  % corrections for the rotor strain gage labels
  BO_c = messages.kAioNodeRotor.kMessageTypeStrain.STO.message.BO;
  TI_c = messages.kAioNodeRotor.kMessageTypeStrain.STO.message.TO;
  TO_c = messages.kAioNodeRotor.kMessageTypeStrain.STO.message.BI;
  BI_c = messages.kAioNodeRotor.kMessageTypeStrain.STO.message.TI;

  messages.kAioNodeRotor.kMessageTypeStrain.STO.message.BO = BO_c;
  messages.kAioNodeRotor.kMessageTypeStrain.STO.message.TI = TI_c;
  messages.kAioNodeRotor.kMessageTypeStrain.STO.message.TO = TO_c;
  messages.kAioNodeRotor.kMessageTypeStrain.STO.message.BI = BI_c;
  clear BO_c BI_c TO_c TI_c;

  % correct Htail Z +800 IMU data
  % Original rotations cfg file in deg. [yaw, pitch, roll]
  rotation = [176.4, 8.6, 184.0];
  % Correct rotations in deg.
  rotation_c = [172.0, 4.6, -91.0];

  messages.kAioNodeHtail.kMessageTypeImu.P800.message = rotate_imu_data (messages.kAioNodeHtail.kMessageTypeImu.P800.message, rotation, 1);
  messages.kAioNodeHtail.kMessageTypeImu.P800.message = rotate_imu_data (messages.kAioNodeHtail.kMessageTypeImu.P800.message, rotation_c);

  % correct Vtail Z -3000 IMU data
  % Original rotations cfg file in deg. [yaw, pitch, roll]
  rotation = [172.0, 4.6, 91.0];
  % Correct rotations in deg.
  rotation_c = [176.4, 8.6, 184.0];

  messages.kAioNodeVtail.kMessageTypeImu.N3000.message = rotate_imu_data (messages.kAioNodeVtail.kMessageTypeImu.N3000.message, rotation, 1);
  messages.kAioNodeVtail.kMessageTypeImu.N3000.message = rotate_imu_data (messages.kAioNodeVtail.kMessageTypeImu.N3000.message, rotation_c);

elseif flight_id == 'rpx-04'
  % correct Htail Z +800 IMU data
  % Original rotations cfg file in deg. [yaw, pitch, roll]
  rotation = [172.0, 4.6, 91.0];
  % Correct rotations in deg.
  rotation_c = [172.0, 4.6, -91.0];

  messages.kAioNodeHtail.kMessageTypeImu.P800.message = rotate_imu_data (messages.kAioNodeHtail.kMessageTypeImu.P800.message, rotation, 1);
  messages.kAioNodeHtail.kMessageTypeImu.P800.message = rotate_imu_data (messages.kAioNodeHtail.kMessageTypeImu.P800.message, rotation_c);

end

fprintf('Done reading data! \n \n');
fprintf('Applying calibration to strain data... \n \n');

% Apply calibrations
% apply fuse calibrations
cal_file = 'fuse_calibration_N2000.cal';
calibrated_data.kAioNodeFuse.kMessageTypeLoad = apply_calibration (messages.kAioNodeFuse.kMessageTypeStrain, ...
																														       cal_file, path_to_cal_files);
[messages] = append_to_data_structure (messages, calibrated_data);
clear calibrated_data;

% apply wing calibrations
cal_file = 'wing_calibration_N600.cal';
calibrated_data.kAioNodeWing.kMessageTypeLoad = apply_calibration (messages.kAioNodeWing.kMessageTypeStrain, ...
																														       cal_file, path_to_cal_files);
[messages] = append_to_data_structure (messages, calibrated_data);
clear calibrated_data;

% apply wing calibrations
cal_file = 'wing_calibration_P600.cal';
calibrated_data.kAioNodeWing.kMessageTypeLoad = apply_calibration (messages.kAioNodeWing.kMessageTypeStrain, ...
																														       cal_file, path_to_cal_files);
[messages] = append_to_data_structure (messages, calibrated_data);
clear calibrated_data;

% apply wing calibrations
cal_file = 'wing_calibration_P5000.cal';
calibrated_data.kAioNodeWing.kMessageTypeLoad = apply_calibration (messages.kAioNodeWing.kMessageTypeStrain, ...
																														       cal_file, path_to_cal_files);
[messages] = append_to_data_structure (messages, calibrated_data);
clear calibrated_data;

% apply rotor calibrations
cal_file = 'rotor_calibration_STO.cal';
calibrated_data.kAioNodeRotor.kMessageTypeLoad = apply_calibration (messages.kAioNodeRotor.kMessageTypeStrain, ...
																														        cal_file, path_to_cal_files);
[messages] = append_to_data_structure (messages, calibrated_data);
clear calibrated_data;

% apply rotor calibrations
cal_file = 'rotor_calibration_SBO.cal';
calibrated_data.kAioNodeRotor.kMessageTypeLoad = apply_calibration (messages.kAioNodeRotor.kMessageTypeStrain, ...
																														        cal_file, path_to_cal_files);
[messages] = append_to_data_structure (messages, calibrated_data);
clear calibrated_data;

fprintf('Done processing all data! \n \n');

%% export the .mat and .h5 file
datestamp = messages.kAioNodeController.kMessageTypeImu.aio_header.date_utc;
file_sav = [path_to_export_files, datestamp, '-', flight_id, '-Airframe_Loads'];

fprintf('Saving mat file... \n \n');
save([file_sav, '.mat'], 'messages', 'info', '-v7.3');

% reassign names
messages_0 = messages;
info_0 = info;

% transpose the structure before saving the h5 file
messages = transpose_my_struct(messages);
info = transpose_my_struct (info);

fprintf('Saving h5 file... \n \n');
save([file_sav, '.h5'], 'messages', 'info', '-v7.3');

messages = messages_0;
info = info_0;

clearvars -except messages info

% plot and save the airframe loads data
airframe_loads_plotting;

fprintf('All done! \n \n');