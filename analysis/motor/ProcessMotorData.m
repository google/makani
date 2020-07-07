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

function [motor_data] = ProcessMotorData(h5files, varargin)
% ProcessMotoeData -- Parse the motor data in h5 logs and save as a mat file.
% motor_data = ProcessMotorData(h5files, varargin)
% This function is suitable for both wing as well as command center logs.
%
% Example 1: motor_data = ProcessMotorData('20171116-rpx07_Wing.h5');
% Example 2: motor_data = ProcessMotorData('20171116-rpx07_Wing.h5', ...
%                                          'tables', 'rotor_rev4.json');
% This will use the tables specified to report each rotor's thrust and torque.
%
% Arguments-
%
% h5files: [char or cell] h5 log file name.
%
% Output Values-
%
% motor_data: [struct] data structure with important motors related parameters
%             .info [struct]  - general information
%                  .filesread - list of all h5 logs processed
%                  .filespath - path for h5 logs
%                  .motors    - labels for all the motors processed
%                  .datarate  - 'slow' for command center, 'fast' for wing logs
%                  .params    - parameters used for motor torque calculations
%                  .radius    - rotor radius [m] used for coefficient calcs
%                  .sync      - 'yes' for sync with flight time
%                  .ref_time  - ref_time as specified by user
%                               field not displayed if info.sync is 'yes'
%                  .rotortable - rotor table, id specified as a [struct].
%             .<Abc> [struct] - contains parameters for <Abc> motor e.g. Sbo, ..
%                  .cap_time    [dbl] - capture time from the node
%                  .omega       [dbl] - motor speed [RPM]
%                  .adv_ratio   [dbl] - propeller advance ratio (J)
%                  .bus_current [dbl]
%                  .bus_voltage [dbl]
%                  .current_iq  [dbl]
%                  .current_id  [dbl]
%                  .torque      [dbl] - mech torque [Nm], positive is generation
%                  .torque_coeff[dbl] - in prop notation
%                  .bus_power   [dbl] - electrical power [W]
%                  .mech_power  [dbl] - mechanical power [W]
%
%             .total_bus_power  [dbl] - total electrical power [W]
%             .total_mech_power [dbl] - total mechanical power [W]
%             .flight_time      [dbl] - common time for all parameters
%                                       same as ref_time it its user specified
%                                       advice to check sync under info field
%
% If rotor tables are specified, additional parameters are included:
% motor_data.total_mech_power_tables    [dbl] - total power calculated [W]
% motor_data. <Abc>.thrust_coeff_tables [dbl]
%                  .torque_coeff_tables [dbl]
%                  .thrust_tables       [dbl]
%                  .torque_tables       [dbl]
%                  .mech_power_tables   [dbl]
%
% Optional flags-
% usage: motor_data = ProcessMotorData(h5files, flag1, flag1_value, .....)
% 'filepath': [char, default []   ] path to h5 files
% 'motors'  : [char, default 'all'] motors to process e.g. 'sbo, pbo'
% 'save_mat': [char, default 'yes'] save processed mat file (yes/no)
% 'mat_path': [char, default './export'] path for saving mat file
% 'mat_name': [char, default 'Motors'] filename for mat file
% 'ref_time': [dbl] user specified time array to sync all time series,
%                    default is sync with controller time
% 'params'  : [struct] parameters for torque calculations
%                    default is Yasa23Params()
% 'radius'  : [dbl] rotor radius [m], default is 1.15 m
% 'tables'  : [char] filename for rotor tables, expected to be '<filename>.json'
%                    and located in 'makani/database/m600'.
%

% Parse the input fields.
if nargin < 1
  error('Must specify the h5 file.')
elseif nargin >= 1
  % Set the defaults here.
  path_to_files  = [];
  process_motors = 'all';
  save_mat_file  = 'yes';
  save_mat_path  = './export';
  save_mat_name  = 'Motors';
  sync_with_cont = 'yes';
  params         = Yasa23Params();
  radius         = 1.15;  % rotor radius [m]
end

% Parse the specified optional input arguments.
if nargin > 1
  if rem(length(varargin), 2) ~= 0
    error('Input error, check arguments in.')
  else
    for ii = 1:length(varargin)/2
      input_field = varargin{2*ii - 1};
      input_value = varargin{2*ii};
      if strcmp (input_field, 'filepath')
        path_to_files = input_value;
      elseif strcmp (input_field, 'motors')
        process_motors = input_value;
      elseif strcmp (input_field, 'save_mat')
        save_mat_file = input_value;
      elseif strcmp (input_field, 'mat_path')
        save_mat_path = input_value;
      elseif strcmp (input_field, 'mat_name')
        save_mat_name = input_value;
      elseif strcmp (input_field, 'ref_time')
        ref_time = input_value;
        sync_with_cont = 'no';
      elseif strcmp (input_field, 'params')
        params = input_value;
      elseif strcmp (input_field, 'radius')
        radius = input_value;
      elseif strcmp (input_field, 'tables')
        rot_table = input_value;
      else
        error('Unrecognized input, please verify!')
      end
    end
  end
end

% Load the rotor table, if specified.
if exist('rot_table', 'var')
  % Check if Makani repo is added to the env path.
  MAKANI_HOME = getenv('MAKANI_HOME');
  if isempty(MAKANI_HOME)
    fprintf('Set MAKANI_HOME, to access rotor database table. \n')
    fprintf('Use setenv(''MAKANI_HOME'', ''<local path to makani git repo>'').\n')
    fprintf('Test using getenv(''MAKANI_HOME'').\n')
    error  ('Please set MAKANI_HOME.')
  else
    rotor_table_path = fullfile(MAKANI_HOME, 'database', 'm600', rot_table);
    if exist(rotor_table_path, 'file') == 2
      rotortable = loadjson(rotor_table_path);
      if (rotortable.diameter < 2 * radius * 0.98) || (rotortable.diameter > 2 * radius * 1.02)
        warning('Rotor radius in tables is not within 2% of value used in this script.')
      end
    else
      warning('Rotor table not found, will not be used.')
    end
  end
  clear MAKANI_HOME rotor_table_path rot_table;
end

% Convert h5 files and motors array to cell array if it is a char.
if ischar(h5files)
  h5files = cellstr(h5files);
end

% Check which motors to process.
all_motors = {'Sbo'; 'Sbi'; 'Pbi'; 'Pbo'; 'Pto'; 'Pti'; 'Sti'; 'Sto'};

if strcmp(process_motors, 'all')
  motor_id = all_motors;
else
  motor_id = strtrim(strsplit(process_motors{1}, ','))';
  for ii = 1:length(motor_id)
    motor_id{ii} = regexprep(motor_id{ii},'(\<[a-z])','${upper($1)}');
    motor_exists = strcmp(all_motors, motor_id{ii});
    if sum(motor_exists) ~= 1
      disp(motor_id{ii});
      error ('Please check motor name')
    end
  end
  clear ii;
end

% Check if wing or command center file is specified.
file_info = h5info(h5files{1}, '/messages/kAioNodeControllerA/');
for ii = 1:length(file_info.Datasets);
  all_datasets{ii} = file_info.Datasets(ii).Name;
end
clear ii;

if sum(ismember(all_datasets, 'kMessageTypeControlDebug')) == 1
  datarate = 'fast';
  cont_msg_id  = 'kMessageTypeControlDebug';
  motor_msg_id = 'kMessageTypeMotorDebug';
elseif sum(ismember(all_datasets, 'kMessageTypeControlTelemetry')) == 1
  datarate = 'slow';
  cont_msg_id  = 'kMessageTypeControlTelemetry';
  motor_msg_id = 'kMessageTypeMotorStatus';
end

% Initialize variables.
ctime     = [];
time_zero = [];
rhotime   = [];
rho       = [];
v_app_locals = [];
for ii = 1:length(motor_id)
  motor_data.(motor_id{ii}).cap_time    = [];
  motor_data.(motor_id{ii}).omega       = [];
  motor_data.(motor_id{ii}).adv_ratio   = [];
  motor_data.(motor_id{ii}).bus_current = [];
  motor_data.(motor_id{ii}).bus_voltage = [];
  motor_data.(motor_id{ii}).current_iq  = [];
  motor_data.(motor_id{ii}).current_id  = [];
end
clear ii;

% Read the data from h5 files.
for ii = 1:length(h5files)
  % Read controller data.
  if strcmp(sync_with_cont, 'yes')
    cont_path = ['/messages/kAioNodeControllerA/', cont_msg_id];
    control_data = h5read([path_to_files, h5files{ii}], cont_path);
    control_info  = h5read([path_to_files, h5files{ii}], '/info');
    ctime_temp = get_node_time(control_data, control_info);
    time_zero_temp = get_time_zero(control_data, control_info);
    ctime = [ctime, ctime_temp];
    time_zero = min([time_zero, time_zero_temp]);
    v_app_locals_temp = control_data.message.v_app_locals;
    v_app_locals = [v_app_locals, v_app_locals_temp];
    clear cont_path control_data ctime_temp time_zero_temp v_app_locals_temp;
  elseif strcmp(sync_with_cont, 'no')
    control_info = struct();
    time_zero = 0;
  end

  % Read weather conditions, stored in TetherUp node, for density calculations.
  tether_up = h5read([path_to_files, h5files{ii}], '/messages/kAioNodeCsGsA/kMessageTypeTetherUp');
  rhotime_temp = get_node_time(tether_up, control_info);
  pressure     = tether_up.message.weather.pressure_pa;
  temperature  = tether_up.message.weather.temperature;
  humidity     = tether_up.message.weather.humidity;
  rho_temp = calculate_density(pressure, temperature, humidity);
  rhotime  = [rhotime, rhotime_temp];
  rho = [rho, rho_temp];
  clear tether_up rhotime_temp pressure temperature humidity rho_temp;

  % Read motor data.
  for jj = 1:length(motor_id)
    motor_path = ['/messages/kAioNodeMotor', (motor_id{jj}), '/', motor_msg_id];
    motor_dataset = h5read([path_to_files, h5files{ii}], motor_path);
    mtime_temp = get_node_time(motor_dataset, control_info);
    omega_temp = double(motor_dataset.message.omega);
    bus_current_temp = double(motor_dataset.message.bus_current);
    bus_voltage_temp = double(motor_dataset.message.bus_voltage);
    current_id_temp  = double(motor_dataset.message.id);
    current_iq_temp  = double(motor_dataset.message.iq);
    motor_data.(motor_id{jj}).cap_time = [motor_data.(motor_id{jj}).cap_time, mtime_temp];
    motor_data.(motor_id{jj}).omega = [motor_data.(motor_id{jj}).omega, omega_temp];
    motor_data.(motor_id{jj}).bus_current = [motor_data.(motor_id{jj}).bus_current, bus_current_temp];
    motor_data.(motor_id{jj}).bus_voltage = [motor_data.(motor_id{jj}).bus_voltage, bus_voltage_temp];
    motor_data.(motor_id{jj}).current_id = [motor_data.(motor_id{jj}).current_id, current_id_temp];
    motor_data.(motor_id{jj}).current_iq = [motor_data.(motor_id{jj}).current_iq, current_iq_temp];
    clear mtime_temp omega_temp bus_current_temp bus_voltage_temp current_iq_temp current_id_temp;
  end
  clear control_info jj;
end
clear ii;

if strcmp(sync_with_cont, 'yes')
  flight_time = ctime - time_zero;
elseif strcmp(sync_with_cont, 'no')
  flight_time = ref_time;
end

rho = interp1(rhotime - time_zero, rho, flight_time, 'linear', 'extrap');

% Round local apparent speed below 0.1 to 0.1. This is needed for table lookup.
v_app_locals(v_app_locals < 0.1) = 0.1;

% Process motor data.
for ii = 1:length(motor_id)
  motor_data.(motor_id{ii}).cap_time = motor_data.(motor_id{ii}).cap_time - time_zero;
  motor_data.(motor_id{ii}).omega       = interp1(motor_data.(motor_id{ii}).cap_time, ...
                                                  motor_data.(motor_id{ii}).omega, flight_time,     'linear', 'extrap');
  motor_data.(motor_id{ii}).adv_ratio   = v_app_locals(ii,:)' * pi ./ (abs(motor_data.(motor_id{ii}).omega) * radius);
  motor_data.(motor_id{ii}).bus_voltage = interp1(motor_data.(motor_id{ii}).cap_time, ...
                                                motor_data.(motor_id{ii}).bus_voltage, flight_time, 'linear', 'extrap');
  motor_data.(motor_id{ii}).bus_current = interp1(motor_data.(motor_id{ii}).cap_time, ...
                                                motor_data.(motor_id{ii}).bus_current, flight_time, 'linear', 'extrap');
  motor_data.(motor_id{ii}).current_iq  = interp1(motor_data.(motor_id{ii}).cap_time, ...
                                                motor_data.(motor_id{ii}).current_iq, flight_time,  'linear', 'extrap');
  motor_data.(motor_id{ii}).current_id  = interp1(motor_data.(motor_id{ii}).cap_time, ...
                                                motor_data.(motor_id{ii}).current_id, flight_time,  'linear', 'extrap');
  motor_data.(motor_id{ii}).torque      = -MotorTorque(motor_data.(motor_id{ii}).current_id, ...
                                                   motor_data.(motor_id{ii}).current_iq, params);
  motor_data.(motor_id{ii}).torque_coeff = motor_data.(motor_id{ii}).torque ./ ...
                                                   (rho .* (motor_data.(motor_id{ii}).omega/(2*pi)).^2 * (2*radius)^5);
  motor_data.(motor_id{ii}).mech_power   = motor_data.(motor_id{ii}).torque .* motor_data.(motor_id{ii}).omega;
  motor_data.(motor_id{ii}).bus_power = -motor_data.(motor_id{ii}).bus_voltage .* motor_data.(motor_id{ii}).bus_current;

  % Add rotor table lookup quantities.
  % Reference: http://web.mit.edu/16.unified/www/FALL/thermodynamics/notes/node86.html
  if exist('rotortable', 'var')
    motor_data.(motor_id{ii}).thrust_coeff_tables = ...
                                     interp2(rotortable.omegas, rotortable.v_freestreams, rotortable.thrust_coeffs', ...
                                         abs(motor_data.(motor_id{ii}).omega), abs(v_app_locals(ii,:)')) .* rho / 1.225;
    motor_data.(motor_id{ii}).power_coeff_tables  = ...
                                     interp2(rotortable.omegas, rotortable.v_freestreams, rotortable.power_coeffs',  ...
                                         abs(motor_data.(motor_id{ii}).omega), abs(v_app_locals(ii,:)')) .* rho / 1.225;
    motor_data.(motor_id{ii}).thrust_tables     = motor_data.(motor_id{ii}).thrust_coeff_tables .* ...
                                                   rho.*(abs(motor_data.(motor_id{ii}).omega)/(2*pi)).^2 * (2*radius)^4;
    motor_data.(motor_id{ii}).mech_power_tables = motor_data.(motor_id{ii}).power_coeff_tables .* ...
                                                   rho.*(abs(motor_data.(motor_id{ii}).omega)/(2*pi)).^3 * (2*radius)^5;
    motor_data.(motor_id{ii}).torque_tables     = motor_data.(motor_id{ii}).mech_power_tables ./ ...
                                                   motor_data.(motor_id{ii}).omega;
  end
end
clear ii;

% Find total power.
if length(motor_id) == 8
  total_bus_power    = 0;
  total_mech_power   = 0;
  total_mech_power_tables = 0;
  for ii = 1:length(motor_id)
    total_bus_power  = total_bus_power  + motor_data.(motor_id{ii}).bus_power;
    total_mech_power = total_mech_power + motor_data.(motor_id{ii}).mech_power;
    if exist('rotortable', 'var')
      total_mech_power_tables = total_mech_power_tables + motor_data.(motor_id{ii}).mech_power_tables;
    end
  end
  clear ii;
  motor_data.total_bus_power  = total_bus_power;
  motor_data.total_mech_power = total_mech_power;
  if exist('rotortable', 'var')
    motor_data.total_mech_power_tables = total_mech_power_tables;
  end
end

% add flight time to the output structure if synced with controller
motor_data.flight_time   = flight_time;

% build the info output
motor_data.info.filesread = h5files;
motor_data.info.filespath = path_to_files;
motor_data.info.motors    = motor_id;
motor_data.info.datarate  = datarate;
motor_data.info.params    = params;
motor_data.info.radius    = radius;
motor_data.info.sync      = sync_with_cont;
if exist('rotortable', 'var')
  motor_data.info.rotortable = rotortable;
end

% add ref_time under info structure
if strcmp(sync_with_cont, 'no')
  motor_data.info.ref_time = ref_time;
end

% save the mat file
if save_mat_file == 'yes'
  save_filename = [save_mat_name, '_', datarate];
  motor_data.info.save_filename = save_filename;
  motor_data.info.save_filepath = save_mat_path;
  if exist('./export') == 0
    mkdir('./export');
  end
  save([save_mat_path, '/', save_filename, '.mat'], 'motor_data', '-v7.3');
end