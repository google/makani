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

function [node_time] = get_node_time(node_dataset, info_dataset, varargin)
% get_node_time -- calculate node timestamp in seconds from beginning of day.
% [node_time] = get_node_time(node_dataset, info_dataset, varargin)
%
% Each node (kMessageType...) writes its own capture timestamp under
% capture_header field in UNIX/POSIX time format (number of seconds
% since 1-Jan-1970 00:00:00 UTC, not counting leap seconds). This function
% converts it into number of seconds from beginning of first day observed
% in the dataset, in Pacific timezone.
%
% Matlab function - datetime - is used for conversion, which accounts for
% timezone conversions and daylight savings as well.
%
% A general correction is now applied to sync node clock with GPS time.
% In case no GPS sync information is present in the dataset, it defaults to
% capture_header time.
%
% Arguments-
%
% node_dataset: [struct] dataset from controller h5 file
%               e.g. messages.kAioNodeControllerA.kMessageTypeControlDebug
%                    messages.kAioNodeMotorSbo.kMessageTypeMotorDebug
% info_dataset: [struct] dataset containing GPS sync '/info'
%               Optional if only capture time is desired
%
% Output Values-
%
% node_time: [dbl] number of seconds from beginning of day in Pacific timezone
%            calculated from capture_header under the dataset,
%            with GPS correction if info_dataset is provided
%
% Optional Flags-
% 'verbose': [char, 'yes'/'no'] print warning if no GPS sync parameters found
%

% set default
use_gps_correction = 0;

if nargin == 2
  if isstruct(info_dataset) & isfield(info_dataset, 'gps_time_coeff') & isfield(info_dataset, 'gps_time_bias');
    if ~isnan(info_dataset.gps_time_coeff) & ~isnan(info_dataset.gps_time_bias)
      gps_time_coeff = info_dataset.gps_time_coeff;
      gps_time_bias  = info_dataset.gps_time_bias;
      use_gps_correction = 1;
    end
  else
    if length(varargin) == 2 & strcmp(varargin{1}, 'verbose') & strcmp(varargin{2}, 'yes')
      warning('GPS sync info not found in dataset, no correction will be applied.');
    end
  end
end

if isstruct(node_dataset) & isfield(node_dataset, 'capture_header');
  data_read = node_dataset;
else
  error('get_node_time error, check node dataset.');
end

% read time arrays stored in two parts: integer seconds and integer microseconds
tv_sec  = data_read.capture_header.tv_sec;
tv_usec = data_read.capture_header.tv_usec;

% combine for unix timestamp
node_time_unix = double(tv_sec) + double(tv_usec)*10^-6;

% convert to local time, output is matlab datetime format
node_datetime = datetime(node_time_unix, 'ConvertFrom', 'posixtime', 'TimeZone', 'America/Los_Angeles');

% if info_dataset is present, use the gps_coefficients
if use_gps_correction == 1
  first_date  = node_datetime(1);
  day_of_week = weekday(first_date);
  sunday_date = first_date - days(day_of_week - 1);
  epoch_ref   = datestr(sunday_date, 1);

  % apply GPS correction to unix timestamp
  % this calibration converts the unix time to seconds from beginning of week
  node_time_gps = node_time_unix * gps_time_coeff + gps_time_bias;

  % convert to datetime with reference to beginning of day
  node_datetime = datetime(node_time_gps, 'ConvertFrom', 'epochtime', 'Epoch', epoch_ref);

  % adjust the hours based on timezone offset, this also accounts for dst
  node_datetime = node_datetime + tzoffset(first_date);
end

% combine to output seconds from beginning of the day
first_day = node_datetime.Day(1);
node_time = (node_datetime.Day - first_day)*86400 + ...
            node_datetime.Hour*3600 + ...
            node_datetime.Minute*60 + ...
            node_datetime.Second;

% TODO: Add selection of gps correction from kite station or ground station.
% E.g. controller node should use kite GPS correction coefficients
%      and tetherUp messages should use the ground GPS coefficients.