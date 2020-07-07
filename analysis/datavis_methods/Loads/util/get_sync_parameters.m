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

function [sync_parameters] = get_sync_parameters(controller_messages, path_to_airframe_files)
% Generates the sync parameters from cross correlation between FCU and wing IMUs.
% [sync_parameters] = get_sync_parameters(file_to_sync, path_to_file, control_messages)
%
% Arguments-
%
% controller_messages  : struct, containing messages.kAioNodeController
% path_to_airframe_file: Optional string, absolute path to the file.
%                        Default is working directory
%
% Output Values-
%
% sync_parameters: [4 x 1 dbl] parameters for syncing airframe data.
%                  [zero timestamp, lag (sec), start timestamp, end timestamp]

% initialize parameters
messages = controller_messages;
sync_parameters(1) = messages.kAioNodeController.kMessageTypeImu.aio_header.t0_value_utc;
sync_parameters(2) = 0;
sync_parameters(3) = messages.kAioNodeController.kMessageTypeImu.aio_header.timestamp_utc(1);
sync_parameters(4) = messages.kAioNodeController.kMessageTypeImu.aio_header.timestamp_utc(end);

% Usually AX is used for syncing
wing_imu  = messages.kAioNodeController.kMessageTypeImu.message.AX;
wing_time = messages.kAioNodeController.kMessageTypeImu.capture_header.tdelta;

% get the filename for wing IMU at P2400
s = dir([path_to_airframe_files '*$INS Wing Y +2400*.csv']);
file2read = getfield(s,'name');
clear s;

% read the instrument data file
data_read = read_instrument_data(file2read, path_to_airframe_files, sync_parameters);
load_imu  = data_read.kAioNodeWing.kMessageTypeImu.P2400.message.AX;
load_time = data_read.kAioNodeWing.kMessageTypeImu.P2400.capture_header.tdelta;
sample_rate = data_read.kAioNodeWing.kMessageTypeImu.P2400.aio_header.sample_rate;

% make both datasets of same length, preferably higher bandwidth for more time
wing_imu2 = interp1(wing_time, wing_imu, load_time, 'pchip');

% make cross correlation calculation and get lag from it
[acor, lag] = xcorr(load_imu, wing_imu2);
[~,I] = max(abs(acor));
sync_parameters(2) = -lag(I)/sample_rate;