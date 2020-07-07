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


function [data_read] = read_instrument_data (file_to_be_read, path_to_file, sync_parameters)
% read_instrument_data -- reads the timestamped instrument data from the csv file saved by das. 
% [data_read] = function read_instrument_data (file_to_be_read, path_to_file)
%
% Arguments-
%
% file_to_be_read: string specifying the filename to be read. 
% path_to_file:    Optional, absolute path to the file. Default is working directory
% sync_parameters: Optional, [2x1] parameters to sync data with controls telemetry
%                  utc timestamp (sec) for t = 0; 
%                  manual adjustment (sec) to sync structural and controls data
%
% Return Values-
%
% data_read: structure with following 
% data_read.kAioNode<kite_comp_type>.kMessageType<sensor_type>.<sensor_location>
% ... <sensor_location>.message
%                      .message.<data_channels> -- all the data channels with proper labels
%                      .aio_header
%                      .aio_header.sample_rate -- the rate at which the data under message was recorded
%                      .aio_header.units -- engineering units of the data under message
%                      .aio_header.timestamp_utc -- utc clock of the sensor, sec from beginning of the day
%                      .aio_header.date_utc -- utc date as specified in the data file, YYYYMMDD format
%                      .capture_header
%                      .capture_header.tv_sec -- utc time synced with controller time, sec  
%                      .capture_header.tv_usec -- utc time synced with controller time, micro sec
%                      .capture_header.tdelta -- delta time (sec), t = 0 corresponds to flight mode switch from 1 to 2

%% generate file_to_be_read
if nargin > 3 || nargin < 1
  fprintf ('Input error, check arguments.');
  return;
elseif nargin == 1
  sync_parameters = [0,0];
  path_to_file = [];
elseif nargin == 2
  sync_parameters = [0,0];
end

file_to_be_read = [path_to_file, file_to_be_read];

% find the type of sensor being read from the filename
if isempty (strfind (file_to_be_read, 'Strains')) ~= 1
  sensor_type = 'Strain';
  sensor_unit = 'strain';
elseif isempty (strfind (file_to_be_read, 'INS')) ~= 1
  sensor_type = 'Imu';
  sensor_unit{1} = 'g';
  sensor_unit{2} = 'deg/s';
elseif isempty (strfind (file_to_be_read, 'ZOC')) ~= 1
  sensor_type = 'Pressure';
  sensor_unit = 'psi';
else
  sensor_type = [];
  sensor_unit = [];
end

% find the date from the file name by matching the pattern in which das writes the filename
date_pattern = '\d{4,4}\d{2,2}.\d{2,2}';
date_extracted = regexp(file_to_be_read,date_pattern,'match');

%if isempty(date_extracted) ~= 1
%  date_formatted = strrep(date_extracted,'.','');
%else
%  date_formatted = [];
%end

% Read the data file (can take time to read large files)
data_raw = dlmread(file_to_be_read,',',3,0);

% find the data encompassed by controller time limits
if numel (sync_parameters) == 4;
  tt = rem(data_raw(:,1),1)*24*60*60;
  ind_to_save = find (tt >= sync_parameters(3) & tt <= sync_parameters(4));
  data_raw = data_raw (ind_to_save,:);
end

% Read the first two header lines, will be used to assign channel names and write data
fid = fopen(file_to_be_read);
  line_read1 = fgetl(fid);
  line_read2 = fgetl(fid);
  % replace dashes with underscores 
  line_read1 = strrep(line_read1,'-','_');
  line_read1 = strrep(line_read1,'"','');
  % delete quotation marks
  line_read2 = strrep(line_read2,'-','_');
  line_read2 = strrep(line_read2,'"','');
fclose(fid);

% there is special handling for reading zocbox data
if strcmp (sensor_type, 'Pressure') == 1;
  comp_type = 'Wing';  % it is hardcoded for now
  sensor_loc =  'P5000';  % it is hardcoded for now
  
  % calls read_zocbox_data function to import scanivalve data
  [info_zocbox, data_zocbox] = read_zocbox_data (data_raw, line_read1, line_read2);
  
  % append the kind of zoc box to sensor type name
  sensor_type = [sensor_type info_zocbox.type];

  % generate a path for the variable
  var_path = ['messages.kAioNode' comp_type '.kMessageType' sensor_type '.' sensor_loc '.message'];
      
  % evaluate the command to read the value into the variable
  eval([var_path '= data_zocbox;']);

  % utc tiemstamp of pressure sampling
  timestamp = info_zocbox.timestamp;

  % temperature of ZOC module
  aio_header.tempC = info_zocbox.tempC;

elseif strcmp (sensor_type, 'Pressure') == 0;

  % break the first header line into labels of data channels read
  labels = textscan(line_read1,'%s','delimiter',',');

  % the first column is instrument specific timestamp
  labels{1,1}{1,1} = 'timestamp';

  % then loop through the columns, 
  for ii = 1:size(data_raw,2);
    vrnm = genvarname([labels{1,1}{ii, 1}]);
  
    % break variable name for identification
    ch_ids = strsplit(vrnm, '_');
  
    if ii == 1  % timestamp
      eval([vrnm '= data_raw(:,ii);']);
      
    elseif length(ch_ids{1}) <= 2 && length(ch_ids{3}) < 5;  % to prevent extra diagnostic channels to be written
      sensor_loc = ch_ids{2};

      % kite component
      switch ch_ids{1}
      case 'W'
        comp_type = 'Wing';
      case 'F'
        comp_type = 'Fuse';
      case 'T'
        if length(ch_ids{2}) == 2;
          comp_type = 'Fuse';
          sensor_loc = 'N6700';
        else
          switch ch_ids{2}(1)
          case 'P'
            comp_type = 'Htail';
          case 'N'
            comp_type = 'Vtail';
          otherwise
            comp_type = [];
          end
        end
      case 'R'
        comp_type = 'Rotor';
      case {'P', 'P4'}
        comp_type = 'Pylon4';
        %sensor_loc = ['Pylon4_' sensor_loc];
      case 'P1'
        comp_type = 'Pylon1';
        %sensor_loc = ['Pylon1_' sensor_loc];
      case 'P2'
        comp_type = 'Pylon2';
        %sensor_loc = ['Pylon2_' sensor_loc];
      case 'P3'
        comp_type = 'Pylon3';
        %sensor_loc = ['Pylon3_' sensor_loc];
      otherwise
        comp_type = [];
      end
  
      % reformat the variable name
      vrnm = strrep (vrnm, [ch_ids{1},'_', ch_ids{2},'_'], '');
  
      % generate a path for the variable
      var_path = ['messages.kAioNode' comp_type '.kMessageType' sensor_type '.' sensor_loc '.message.' vrnm ];
      
      % evaluate the command to read the value into the variable
      eval([var_path '= data_raw(:,ii);']);
    end
  end  
end

% rotor data is transmitted as microstrain
if strcmp(comp_type, 'Rotor') == 1
  sensor_unit = 'microstrain';
end

% convert the timestamp into seconds from begining of the day per UTC
timestamp = rem(timestamp,1)*24*60*60;

% find average sample rate
sample_rate = round(mean(1./diff(timestamp)));

% Write the aio header
aio_header.sample_rate = sample_rate;
aio_header.units = sensor_unit;
aio_header.timestamp_utc = timestamp;
%aio_header.date_utc = date_formatted{1,1};

% timestamp corresponding to flight mode shift from 1 to 2
t0_value_utc = sync_parameters(1);
manual_sync_adj = sync_parameters(2);

% adjust the instrument timestamp to sync with control telemetry stream
timestamp_utc_synced = aio_header.timestamp_utc + manual_sync_adj;

% find the delta time 
delta_time =  timestamp_utc_synced - t0_value_utc;

% Write the capture_header (create a space but it gets specified later)
capture_header.tv_sec = floor(timestamp_utc_synced);;
capture_header.tv_usec = (timestamp_utc_synced - floor(timestamp_utc_synced))*10^6;
capture_header.tdelta = delta_time;

% find the Node in the data file
node_types = fieldnames (messages);

% this nested loop cycles down to the sensor type to write the headers
for ii = 1:numel(node_types);
  if isempty (['messages.' node_types{ii,1}]) ~= 1
    msg_types = eval(['fieldnames(messages.' node_types{ii,1} ')']);
    for jj = 1:numel(msg_types);
      if isempty (['messages.' node_types{ii,1} '.' msg_types{jj,1}]) ~= 1
        sen_type = eval(['fieldnames(messages.' node_types{ii,1} '.' msg_types{jj,1} ')']);
        for kk = 1:numel(sen_type);
          eval (['messages.' node_types{ii,1} '.' msg_types{jj,1} '.' sen_type{kk,1} '.aio_header = aio_header;']);
          eval (['messages.' node_types{ii,1} '.' msg_types{jj,1} '.' sen_type{kk,1} '.capture_header = capture_header;']);
        end
      end
    end
  end
end
clear ii jj kk;

% 
data_read = messages;
