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
%load('20170627-rpx-05-airframe_loads.mat')
% h5_file = '20170627-rpx05_wing.h5';
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

% fuse loads
xx = messages.kAioNodeFuse.kMessageTypeLoad.P0.message.MZ;
yy = messages.kAioNodeFuse.kMessageTypeLoad.P0.message.MY + 20;
tt = messages.kAioNodeFuse.kMessageTypeLoad.P0.capture_header.tdelta;

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
plot_info.samples_overlap =   600;

plot_info.time   = t_ds;
plot_info.fig_no = 2;
plot_info.xlabel = {'Fuse Mz [kNm]'};
plot_info.ylabel = {'Fuse My [kNm]'};
plot_info.title  = {'RPX-05: Wing-Fuse Loads Envelope'};

plot_info.identifiers       = interp1(case_array(:,1), case_array(:,2), t_ds, 'nearest', 'extrap');
plot_info.identifiers_label = 'case';
plot_info.identifiers_map   = id_map;

% Specify envelopes
envelopes.a.data = [0 -40.5; 39.2 -39.2; 152 -35.5; 150 0; 152 73.8; 75.6 75.6; ...
                    0 77.4; -75.5 75.6; -152 73.8; -152 0; -152 -35.5; -39.2 -39.2; 0 -40.5];
envelopes.a.color = 'red';
envelopes.a.label = 'Zero Margin';

envelopes.b.data = [0, -26.5; 81.7, -26.5; 81.7, 0; 68, 55.5; 0, 65.1; ...
                   -68, 55.5;-81.7, 0; -81.7, -26.5; 0, -26.5];
envelopes.b.color = [1 0.5 0]; % orange
envelopes.b.label = 'Proof Load';

export_info.save_plot  = 1;
export_info.save_movie = 0;
export_info.view_movie = 0;
export_info.file_name = 'wing_fuse_envelope';

% Make envelope plots
envelopes_out = envelope_plot(x_ds, y_ds, envelopes, plot_info, export_info);