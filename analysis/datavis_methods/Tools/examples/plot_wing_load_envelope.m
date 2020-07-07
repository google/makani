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
% load('./export/20170627-rpx-05-airframe_loads.mat');
% h5_file = './controls_data/20170627-rpx05_wing.h5';
% control_dataset = h5read(h5_file, '/messages/kAioNodeControllerA/kMessageTypeControlDebug');

close all; clc;
clearvars -except control_dataset messages info;

% get the flight mode and crosswind configuration information
[flight_modes, crosswind_cases] = get_modes_and_cases(control_dataset);
all_cases = crosswind_cases.all_cases;
time_in_crosswind = flight_modes.time_in_mode.mode7;

% find unique cases
case_array = [];
for ii = 1:numel(all_cases);
  case_tt = getfield(crosswind_cases.time_in_case, all_cases{ii});
  case_id = str2num(all_cases{ii}(regexp(all_cases{ii}, '\d')))*ones(size(case_tt));
  case_array = [case_array; case_tt, case_id];
end
clear ii flight_modes crosswind_cases all_cases case_id case_tt;

% identifier key-value map
id_names = {'# 0 - 40 m/s, \alpha = 2^\circ, OFF', '# 1 - 40 m/s, \alpha = 4^\circ, OFF',...
            '# 2 - 40 m/s, \alpha = 4^\circ, ON ', '# 3 - 40 m/s, \alpha = 4^\circ, OFF',...
            '# 4 - 40 m/s, \alpha = 4^\circ, ON ', '# 5 - 40 m/s, \alpha = 4^\circ, OFF',...
            '# 6 - 40 m/s, \alpha = 4^\circ, ON ', '# 7 - 38 m/s, \alpha = 5^\circ, OFF',...
            '# 8 - 38 m/s, \alpha = 5^\circ, ON ', '# 9 - 44 m/s, \alpha = 5^\circ, ON ',...
            '#10 - 36 m/s, \alpha = 5^\circ, ON ', '#11 - 40 m/s, \alpha = 6^\circ, ON ',...
            '#12 - 36 m/s, \alpha = 6^\circ, OFF', '#13 - 44 m/s, \alpha = 4^\circ, ON ',...
            '#14 - 48 m/s, \alpha = 6^\circ, ON ', '#15 - 33 m/s, \alpha = 6^\circ, ON '};
id_value = [0:1:15];

id_map = containers.Map(id_value, id_names);

case_array = unique(case_array,'rows');
case_array = sortrows(case_array);

% time and x,y data to be plotted
tt = messages.kAioNodeWing.kMessageTypeLoad.P5000.capture_header.tdelta;
xx = messages.kAioNodeWing.kMessageTypeLoad.P5000.message.MX;
% tension calculated from difference in strain measured on either side of BHP
delta_strain = messages.kAioNodeWing.kMessageTypeStrain.P7000.message.S_FR_F ...
             - messages.kAioNodeWing.kMessageTypeStrain.P5000.message.S_FR_F;
tether_tension = (0.090138*delta_strain*10^6);
yy = tether_tension;

% find the data indices corresponding to crosswind flight mode in the data
idx = find(tt >= time_in_crosswind(1) & tt <= time_in_crosswind(2));

% downsample the data to simulate realtime acquisition.
% to avoid downsample set this factor to 1.
downsample_factor = 3;

x_ds = downsample(xx(idx), downsample_factor);
y_ds = downsample(yy(idx), downsample_factor);
t_ds = downsample(tt(idx), downsample_factor);

% Specify the plotting info
plot_info.samples_per_plot = 1000;
plot_info.samples_overlap  =  600;

plot_info.time   = t_ds;
plot_info.fig_no = 1;
plot_info.xlabel = {'Wing Mx [kNm] at 5 m'};
plot_info.ylabel = {'Tether Tension, Ft [kN]'};
plot_info.title  = {'RPX-05: Wing Loads Envelope'};

plot_info.identifiers       = interp1(case_array(:,1), case_array(:,2), t_ds, 'nearest');
plot_info.identifiers_label = 'case';
plot_info.identifiers_map   = id_map;

% Specify envelopes
envelopes.a.data = [68 275; 68 0; -100 0; -100 275; 68 275];
envelopes.a.color = 'red';
envelopes.a.label = 'Zero Margin';

envelopes.b.data = [50 260; 50 0; -71.3 0; -71.3 260; 50 260];
envelopes.b.color = [1 0.5 0]; %orange
envelopes.b.label = 'Proof Load';

export_info.save_plot  = 1;
export_info.save_movie = 0;
export_info.view_movie = 0;
export_info.file_name  = 'wing_envelope';

% Make envelope plots
envelopes_out = envelope_plot(x_ds, y_ds, envelopes, plot_info, export_info);