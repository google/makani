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

plotting_defaults;
% load('./export/20170627-rpx-05-airframe_loads.mat')
% h5_file = './controls_data/20170627-rpx05_wing.h5';
% control_dataset = h5read(h5_file, '/messages/kAioNodeControllerA/kMessageTypeControlDebug');
load ('./motor_data/rpx05_motor.mat');
% load('./export/20170525-rpx-04-airframe_loads.mat')
% h5_file = './controls_data/rpx-04-flt-wing-log-full.h5';
% control_dataset = h5read(h5_file, '/messages/kAioNodeControllerA/kMessageTypeControlDebug');
% load ('./motor_data/rpx04_motor.mat');
close all; clc;
clearvars -except control_dataset messages info motor;

% get the flight mode and crosswind configuration information
[flight_modes, crosswind_cases] = get_modes_and_cases(control_dataset);
all_modes = flight_modes.all_modes;

% find unique cases
case_array = [];
for ii = 1:numel(all_modes);
  case_tt = getfield(flight_modes.time_in_mode, all_modes{ii});
  case_id = str2num(all_modes{ii}(regexp(all_modes{ii}, '\d')))*ones(size(case_tt));
  case_array = [case_array; case_tt, case_id];
end
clear ii flight_modes crosswind_cases all_cases case_id case_tt;

% identifier key-value map
[mode_id, mode_label] = get_flight_mode_labels;
for ii = 1:size(mode_id, 1)
  id_names{ii, 1} = [num2str(mode_id(ii)), ' - ' mode_label{ii}];
end
id_value = mode_id;
clear mode_id mode_label;

id_map = containers.Map(id_value, id_names);

case_array = unique(case_array,'rows');
case_array = sortrows(case_array);

% kite aoa and power
yy = messages.kAioNodePylon1.kMessageTypeImu.P1700.message.AY;
tt = messages.kAioNodePylon1.kMessageTypeImu.P1700.capture_header.tdelta;
xx = interp1(motor.time, motor.omega{4}, tt, 'linear', 'extrap');

% apply any calibrations or unit conversions
xx = abs(double(xx));
yy = yy*9.81;
[b,a] = butter(4, 300/500);
yy = filtfilt(b,a,yy);

% find the data indices corresponding to crosswind flight mode in the data
idx = 1:length(xx);

% downsample the data to simulate realtime acquisition.
% to avoid downsample set this factor to 1.
downsample_factor = 4;

x_ds = downsample(xx(idx), downsample_factor);
y_ds = downsample(yy(idx), downsample_factor);
t_ds = downsample(tt(idx), downsample_factor);

% Specify the plotting info
plot_info.samples_per_plot = 3000;
plot_info.samples_overlap =  2000;

plot_info.time   = t_ds;
plot_info.fig_no = 2;
plot_info.xlabel = {'Motor Speed [RPM]'};
plot_info.ylabel = {'Lateral Vibe [m/s^2]'};
plot_info.title  = {'RPX-05: PBO Lateral Vibration - Flight Envelope'};
plot_info.axis_style = 'normal';

plot_info.identifiers       = interp1(case_array(:,1), case_array(:,2), t_ds, 'nearest', 'extrap');
plot_info.identifiers_label = 'mode';
plot_info.identifiers_map   = id_map;

% Specify envelopes
envelopes.a.data = [240 70; 240 -70; 0 -70; 0 70; 240 70];
envelopes.a.color = [1 0.5 0];
envelopes.a.label = 'Caution';

envelopes.b.data = [255 100; 255 -100; 0 -100; 0 100; 255 100];
envelopes.b.color = 'red';
envelopes.b.label = 'DNE';

%envelopes.c.data = [0.001032       1.6491
%   0.00045256       1.4642
%   0.00021287       1.3635
%   7.1197e-05       1.2898
%   2.1332e-06       1.2068
%   5.3598e-07      0.89942
%    3.227e-07      0.81685
%   3.9884e-07      0.60434
%   1.2973e-06       0.5489
%   8.2254e-06      0.46928
%   6.9926e-05      0.22244
%   0.00096323      0.17052
%    0.0065323     -0.11993
%       2.5721      -2.5617
%       103.75       -91.94
%       166.14      -132.73
%       206.19      -59.607
%       207.67      -55.504
%       209.34      -50.096
%       209.72      -30.617
%       209.74      -23.204
%       209.73      -12.718
%       209.73        -10.1
%       209.73      -7.6542
%       209.72        -2.88
%       206.45       6.3409
%       165.88       107.25
%       111.69       93.962
%        3.229       5.5323
%     0.001032       1.6491];
%envelopes.c.color = 'blue';
%envelopes.c.label = 'RPX-05';

export_info.save_plot  = 1;
export_info.save_movie = 0;
export_info.view_movie = 0;
export_info.file_name  = 'pbo_vibe_flight_envelope';

% Make envelope plots
envelopes_out = envelope_plot(x_ds, y_ds, envelopes, plot_info, export_info);