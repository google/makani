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

function [data_controller] = get_control_telemetry_data (control_telemetry_file_array, path_to_file)
% get_control_telemetry_data -- reads the timestamped fcu data and flight mode from controller h5 files.
% [data_controller] = get_control_telemetry_data (control_telemetry_file_array, path_to_file)
%
% Arguments-
%
% control_telemetry_file_array: string array specifying all controls telemetry h5 files to be read
% path_to_file:                 optional, absolute path to the file. Default is working directory
%
% Return Values-
%
% data_read: structure with following
% data_read.kAioNode<kite_comp_type>.kMessageType<sensor_type>.<sensor_location>
% ... <sensor_location>.message
%                      .message.<data_channels> -- all the data channels with proper labels
%                      .aio_header
%                         .files_read -- filenames of h5 files read
%                         .flight_mode -- flight mode [int32]
%                         .tether_kN -- tether tension in kN
%                         .sample_rate -- the rate at which the data under message was recorded
%                         .units -- engineering units of the data under message
%                         .timestamp_utc -- utc clock of the sensor, sec from beginning of the day
%                         .date_utc -- utc date as specified in the data file, YYYYMMDD format
%                         .t0_index -- array index corresponding to flight mode change from 1 to 2
%                         .t0_value_utc -- utc timestamp corresponding to t0_index
%                      .capture_header
%                         .tv_sec -- utc time synced with controller time, sec
%                         .tv_usec -- utc time synced with controller time, micro sec
%                         .tdelta -- delta time (sec), t = 0 corresponds to flight mode switch from 1 to 2

% generate file_to_be_read
if nargin > 2 || nargin < 1
  disp ('Input error, check arguments.');
  return;
elseif nargin == 1
  path_to_file = [];
end

file_info = h5info([path_to_file, control_telemetry_file_array(1,:)], '/messages/kAioNodeControllerA/');
for ii = 1:length(file_info.Datasets);
  all_datasets{ii} = file_info.Datasets(ii).Name;
end
clear ii;

if sum(ismember(all_datasets, 'kMessageTypeControlDebug')) == 1
  cont_path = '/messages/kAioNodeControllerA/kMessageTypeControlDebug';
elseif sum(ismember(all_datasets, 'kMessageTypeControlTelemetry')) == 1
  cont_path = '/messages/kAioNodeControllerA/kMessageTypeControlTelemetry';
end

% initialize to allow concatenation of data from multiple h5 data files
fcu_time  = [];
t0_value  = [];
fcu_acc_x = [];
fcu_acc_y = [];
fcu_acc_z = [];
fcu_gyr_x = [];
fcu_gyr_y = [];
fcu_gyr_z = [];
oper_mode = [];
tether_kN = [];

% read and concatenate data from all h5 files, one at a time
for ii = 1:size(control_telemetry_file_array, 1)
  % read the relevant sub-tree from the h5 file
  data_read = h5read([path_to_file, control_telemetry_file_array(ii,:)], cont_path);

  % time_stamp from fcu (this is unix time)
  fcu_time = [fcu_time; get_node_time(data_read)];
  %fcu_time = [fcu_time; double(data_read.capture_header.tv_sec) + ...
  %                     double(data_read.capture_header.tv_usec)*10^-6];
  t0_value = min([t0_value, get_time_zero(data_read)]);
  % fcu accels, covert from m/s^2 to g
  fcu_acc_x = [fcu_acc_x; mean(data_read.message.control_input.imus.acc.x/9.81,1)'];
  fcu_acc_y = [fcu_acc_y; mean(data_read.message.control_input.imus.acc.y/9.81,1)'];
  fcu_acc_z = [fcu_acc_z; mean(data_read.message.control_input.imus.acc.z/9.81,1)'];

  % fcu gyro, covert from rad/s to deg/s
  fcu_gyr_x = [fcu_gyr_x; mean(data_read.message.control_input.imus.gyro.x*180/pi,1)'];
  fcu_gyr_y = [fcu_gyr_y; mean(data_read.message.control_input.imus.gyro.y*180/pi,1)'];
  fcu_gyr_z = [fcu_gyr_z; mean(data_read.message.control_input.imus.gyro.z*180/pi,1)'];

  % flight mode
  oper_mode = [oper_mode; data_read.message.flight_mode];

  % tether tension in kN
  tether_kN = [tether_kN; data_read.message.state_est.tether_force_b.sph.tension*0.001];

  clear data_read;

end

clear ii;

% find the date from the file name
date_pattern = '\d{4,4}\d{2,2}\d{2,2}';
date_extracted = regexp(control_telemetry_file_array(1,:),date_pattern,'match');
if isempty(date_extracted) ~= 1
  date_formatted = date_extracted;
else
  date_formatted = [];
end

% covert unix time to local time in sec since beginning of the day UTC
timestamp = fcu_time;

% find average sample rate
sample_rate = round(mean(1./diff(timestamp)));

% find t=0 from flight mode change from 1 to 2
t0_index = find(timestamp == t0_value);

%% write the output data structure
% fcu imu accelerations and gyro data
data_controller.kMessageTypeImu.message.AX = fcu_acc_x';
data_controller.kMessageTypeImu.message.AY = fcu_acc_y';
data_controller.kMessageTypeImu.message.AZ = fcu_acc_z';
data_controller.kMessageTypeImu.message.GX = fcu_gyr_x';
data_controller.kMessageTypeImu.message.GY = fcu_gyr_y';
data_controller.kMessageTypeImu.message.GZ = fcu_gyr_z';

% Write the aio header
data_controller.kMessageTypeImu.aio_header.files_read = control_telemetry_file_array;

data_controller.kMessageTypeImu.aio_header.flight_mode = oper_mode;

data_controller.kMessageTypeImu.aio_header.tether_kN = tether_kN;

data_controller.kMessageTypeImu.aio_header.sample_rate = sample_rate;
data_controller.kMessageTypeImu.aio_header.units{1} = 'g';
data_controller.kMessageTypeImu.aio_header.units{2} = 'deg/s';
data_controller.kMessageTypeImu.aio_header.timestamp_utc = timestamp;
data_controller.kMessageTypeImu.aio_header.date_utc = date_formatted{1,1};

data_controller.kMessageTypeImu.aio_header.t0_index = t0_index;
data_controller.kMessageTypeImu.aio_header.t0_value_utc = t0_value;

% write the capture header
data_controller.kMessageTypeImu.capture_header.tv_sec = floor(timestamp)';
data_controller.kMessageTypeImu.capture_header.tv_usec = (timestamp - floor(timestamp))'*10^6;
data_controller.kMessageTypeImu.capture_header.tdelta = timestamp - t0_value;
