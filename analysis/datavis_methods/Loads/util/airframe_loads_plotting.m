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

% Generating plots from rpx data file
%
plotting_defaults;

fprintf('Plotting airframe telemetry data... \n \n');

% Load the mat file
%load ('20161214-airframe_loads.mat');
close all;

% the path to save figures
if ~exist ('export/figures/');
  mkdir('export/figures/');
end

cd('export/figures/');

% Flight mode labels.
[~, YTickLabel] = get_flight_mode_labels;

% setting variables for plotting parameters
wing_accel = messages.kAioNodeWing.kMessageTypeImu;
fcu_accel  = messages.kAioNodeController.kMessageTypeImu;
fuse_accel = messages.kAioNodeFuse.kMessageTypeImu;
pylon1_accel = messages.kAioNodePylon1.kMessageTypeImu;
pylon4_accel = messages.kAioNodePylon4.kMessageTypeImu;
htail_accel = messages.kAioNodeHtail.kMessageTypeImu;
vtail_accel = messages.kAioNodeVtail.kMessageTypeImu;

wing_load = messages.kAioNodeWing.kMessageTypeLoad;
fuse_load = messages.kAioNodeFuse.kMessageTypeLoad;
rotor_load = messages.kAioNodeRotor.kMessageTypeLoad;

wing_strain = messages.kAioNodeWing.kMessageTypeStrain;
pylon4_strain = messages.kAioNodePylon4.kMessageTypeStrain;
rotor_strain = messages.kAioNodeRotor.kMessageTypeStrain;

tether_tension = messages.kAioNodeController.kMessageTypeImu.aio_header.tether_kN;

% calculate time axis limmits for plots display
time_limit = [fcu_accel.capture_header.tdelta(1), fcu_accel.capture_header.tdelta(end)];

% flight id
flight_id = upper(info.flight_id);

%% wing accels

[b,a] = butter(4, 30/(wing_accel.P2400.aio_header.sample_rate));

better_figure (1, 1100, 750)
sp1 = subplot (4,1,1);
plot(wing_accel.P12000.capture_header.tdelta, filtfilt(b,a,wing_accel.P12000.message.AX))
grid on
hold on
plot(wing_accel.P7000.capture_header.tdelta, filtfilt(b,a,wing_accel.P7000.message.AX))
plot(wing_accel.P2400.capture_header.tdelta, filtfilt(b,a,wing_accel.P2400.message.AX))
ylabel ('\bf A_x [g]')
legend ('show', '\bf 12 m', '\bf 7.0 m', '\bf 2.4 m', 'location', 'best')
xlim (time_limit)
title ([flight_id, ': ', '\bf Wing Vibrations (LPF: 30 Hz)'])

sp2 = subplot (4,1,2);
plot(wing_accel.P12000.capture_header.tdelta, filtfilt(b,a,wing_accel.P12000.message.AY))
grid on
hold on
plot(wing_accel.P7000.capture_header.tdelta, filtfilt(b,a,wing_accel.P7000.message.AY))
plot(wing_accel.P2400.capture_header.tdelta, filtfilt(b,a,wing_accel.P2400.message.AY))
ylabel ('\bf A_y [g]')
legend ('hide', '\bf 12 m', '\bf 7.0 m', '\bf 2.4 m', 'location', 'best')
xlim (time_limit)

sp3 = subplot (4,1,3);
plot(wing_accel.P12000.capture_header.tdelta, filtfilt(b,a,wing_accel.P12000.message.AZ))
grid on
hold on
plot(wing_accel.P7000.capture_header.tdelta, filtfilt(b,a,wing_accel.P7000.message.AZ))
plot(wing_accel.P2400.capture_header.tdelta, filtfilt(b,a,wing_accel.P2400.message.AZ))
ylabel ('\bf A_z [g]')
legend ('hide', '\bf 12 m', '\bf 7.0 m', '\bf 2.4 m', 'location', 'best')
xlim (time_limit)

sp4 = subplot (4,1,4);
plot(fcu_accel.capture_header.tdelta, fcu_accel.aio_header.flight_mode,'k')
grid on
%ylabel ('\bf Flight Mode')
xlabel ('\bf time [sec]')
xlim (time_limit)
ylim ([-0.5 12.5])
ax = gca;
ax.YTick = [1:2:12]; ax.YTickLabel = YTickLabel (2:2:end);
linkaxes([sp1,sp2,sp3,sp4],'x')
clear ax sp1 sp2 sp3 sp4;
savefig (gcf, [flight_id, '-wing_accels.fig'],'compact');
saveas (gcf, [flight_id, '-wing_accels.png']);

better_figure (2, 1100, 750)
sp1 = subplot (4,1,1);
plot(wing_accel.P12000.capture_header.tdelta, filtfilt(b,a,wing_accel.P12000.message.GX))
grid on
hold on
plot(wing_accel.P7000.capture_header.tdelta, filtfilt(b,a,wing_accel.P7000.message.GX))
plot(wing_accel.P2400.capture_header.tdelta, filtfilt(b,a,wing_accel.P2400.message.GX))
ylabel ('\bf G_x [deg/s]')
legend ('show', '\bf 12 m', '\bf 7.0 m', '\bf 2.4 m', 'location', 'best')
xlim (time_limit)
title ([flight_id, ': ', '\bf Wing Angular Rate (LPF: 30 Hz)'])

sp2 = subplot (4,1,2);
plot(wing_accel.P12000.capture_header.tdelta, filtfilt(b,a,wing_accel.P12000.message.GY))
grid on
hold on
plot(wing_accel.P7000.capture_header.tdelta, filtfilt(b,a,wing_accel.P7000.message.GY))
plot(wing_accel.P2400.capture_header.tdelta, filtfilt(b,a,wing_accel.P2400.message.GY))
ylabel ('\bf G_y [deg/s]')
legend ('hide', '\bf 12 m', '\bf 7.0 m', '\bf 2.4 m', 'location', 'best')
xlim (time_limit)

sp3 = subplot (4,1,3);
plot(wing_accel.P12000.capture_header.tdelta, filtfilt(b,a,wing_accel.P12000.message.GZ))
grid on
hold on
plot(wing_accel.P7000.capture_header.tdelta, filtfilt(b,a,wing_accel.P7000.message.GZ))
plot(wing_accel.P2400.capture_header.tdelta, filtfilt(b,a,wing_accel.P2400.message.GZ))
ylabel ('\bf G_z [deg/s]')
legend ('hide', '\bf 12 m', '\bf 7.0 m', '\bf 2.4 m', 'location', 'best')
xlim (time_limit)

sp4 = subplot (4,1,4);
plot(fcu_accel.capture_header.tdelta, fcu_accel.aio_header.flight_mode,'k')
grid on
%ylabel ('\bf Flight Mode')
xlabel ('\bf time [sec]')
xlim (time_limit)
ylim ([-0.5 12.5])
ax = gca;
ax.YTick = [1:2:12]; ax.YTickLabel = YTickLabel (2:2:end);
linkaxes([sp1,sp2,sp3,sp4],'x')
clear ax sp1 sp2 sp3 sp4;
savefig (gcf, [flight_id, '-wing_rates.fig'],'compact');
saveas (gcf, [flight_id, '-wing_rates.png']);


%% Pylon Vibrations

[b,a] = butter(4, 200/(pylon4_accel.P1700.aio_header.sample_rate));

better_figure (3, 1100, 750)
sp1 = subplot (4,1,1);
plot(pylon4_accel.N1300.capture_header.tdelta, filtfilt(b,a,pylon4_accel.N1300.message.AX))
grid on
hold on
plot(pylon4_accel.P1700.capture_header.tdelta, filtfilt(b,a,pylon4_accel.P1700.message.AX))
plot(pylon1_accel.P1700.capture_header.tdelta, filtfilt(b,a,pylon1_accel.P1700.message.AX))
ylabel ('\bf A_x [g]')
legend ('show', '\bf sto', '\bf sbo', '\bf pbo', 'location', 'best')
xlim (time_limit)
title ([flight_id, ': ', '\bf Pylon Vibrations (LPF: 200 Hz)'])

sp2 = subplot (4,1,2);
plot(pylon4_accel.N1300.capture_header.tdelta, filtfilt(b,a,pylon4_accel.N1300.message.AY))
grid on
hold on
plot(pylon4_accel.P1700.capture_header.tdelta, filtfilt(b,a,pylon4_accel.P1700.message.AY))
plot(pylon1_accel.P1700.capture_header.tdelta, filtfilt(b,a,pylon1_accel.P1700.message.AY))
ylabel ('\bf A_y [g]')
legend ('hide', '\bf sto', '\bf sbo', '\bf pbo', 'location', 'best')
xlim (time_limit)

sp3 = subplot (4,1,3);
plot(pylon4_accel.N1300.capture_header.tdelta, filtfilt(b,a,pylon4_accel.N1300.message.AZ))
grid on
hold on
plot(pylon4_accel.P1700.capture_header.tdelta, filtfilt(b,a,pylon4_accel.P1700.message.AZ))
plot(pylon1_accel.P1700.capture_header.tdelta, filtfilt(b,a,pylon1_accel.P1700.message.AZ))
ylabel ('\bf A_z [g]')
legend ('hide', '\bf sto', '\bf sbo', '\bf pbo', 'location', 'best')
xlim (time_limit)

sp4 = subplot (4,1,4);
plot(fcu_accel.capture_header.tdelta, fcu_accel.aio_header.flight_mode,'k')
grid on
%ylabel ('\bf Flight Mode')
xlabel ('\bf time [sec]')
xlim (time_limit)
ylim ([-0.5 12.5])
ax = gca;
ax.YTick = [1:2:12]; ax.YTickLabel = YTickLabel (2:2:end);
linkaxes([sp1,sp2,sp3,sp4],'x')
clear ax sp1 sp2 sp3 sp4;

savefig (gcf, [flight_id, '-pylon_accels.fig'],'compact');
saveas (gcf, [flight_id, '-pylon_accels.png']);


better_figure (4, 1100, 750)
sp1 = subplot (4,1,1);
plot(pylon4_accel.N1300.capture_header.tdelta, filtfilt(b,a,pylon4_accel.N1300.message.GX))
grid on
hold on
plot(pylon4_accel.P1700.capture_header.tdelta, filtfilt(b,a,pylon4_accel.P1700.message.GX))
plot(pylon1_accel.P1700.capture_header.tdelta, filtfilt(b,a,pylon1_accel.P1700.message.GX))
ylabel ('\bf G_x [deg/s]')
legend ('show', '\bf sto', '\bf sbo', '\bf pbo', 'location', 'best')
xlim (time_limit)
title ([flight_id, ': ', '\bf Pylon Angular Rate (LPF: 200 Hz)'])

sp2 = subplot (4,1,2);
plot(pylon4_accel.N1300.capture_header.tdelta, filtfilt(b,a,pylon4_accel.N1300.message.GY))
grid on
hold on
plot(pylon4_accel.P1700.capture_header.tdelta, filtfilt(b,a,pylon4_accel.P1700.message.GY))
plot(pylon1_accel.P1700.capture_header.tdelta, filtfilt(b,a,pylon1_accel.P1700.message.GY))
ylabel ('\bf G_y [deg/s]')
legend ('hide', '\bf sto', '\bf sbo', '\bf pbo', 'location', 'best')
xlim (time_limit)

sp3 = subplot (4,1,3);
plot(pylon4_accel.N1300.capture_header.tdelta, filtfilt(b,a,pylon4_accel.N1300.message.GZ))
grid on
hold on
plot(pylon4_accel.P1700.capture_header.tdelta, filtfilt(b,a,pylon4_accel.P1700.message.GZ))
plot(pylon1_accel.P1700.capture_header.tdelta, filtfilt(b,a,pylon1_accel.P1700.message.GZ))
ylabel ('\bf G_z [deg/s]')
legend ('hide', '\bf sto', '\bf sbo', '\bf pbo', 'location', 'best')
xlim (time_limit)

sp4 = subplot (4,1,4);
plot(fcu_accel.capture_header.tdelta, fcu_accel.aio_header.flight_mode,'k')
grid on
%ylabel ('\bf Flight Mode')
xlabel ('\bf time [sec]')
xlim (time_limit)
ylim ([-0.5 12.5])
ax = gca;
ax.YTick = [1:2:12]; ax.YTickLabel = YTickLabel (2:2:end);
linkaxes([sp1,sp2,sp3,sp4],'x')
clear ax sp1 sp2 sp3 sp4;

savefig (gcf, [flight_id, '-pylon_rates.fig'],'compact');
saveas (gcf, [flight_id, '-pylon_rates.png']);

%% Fuselage Vibrations

clear b a;
[b1,a1] = butter(4, 30/(fcu_accel.aio_header.sample_rate));
%[b1,a1] = butter(4, 0.999);
[b2,a2] = butter(4, 30/(fuse_accel.N6700.aio_header.sample_rate));

better_figure (5, 1100, 750)
sp1 = subplot (4,1,1);
plot(fcu_accel.capture_header.tdelta, filtfilt(b1,a1,fcu_accel.message.AX))
grid on
hold on
plot(fuse_accel.N6700.capture_header.tdelta, filtfilt(b2,a2,fuse_accel.N6700.message.AX))
ylabel ('\bf A_x [g]')
legend ('show', '\bf FCU', '\bf tail-end',  'location', 'best')
xlim (time_limit)
title ([flight_id, ': ', '\bf Fuselage Vibrations (LPF: 30 Hz)'])

sp2 = subplot (4,1,2);
plot(fcu_accel.capture_header.tdelta, filtfilt(b1,a1,fcu_accel.message.AY))
grid on
hold on
plot(fuse_accel.N6700.capture_header.tdelta, filtfilt(b2,a2,fuse_accel.N6700.message.AY))
ylabel ('\bf A_y [g]')
legend ('hide', '\bf FCU', '\bf tail-end',  'location', 'best')
xlim (time_limit)

sp3 = subplot (4,1,3);
plot(fcu_accel.capture_header.tdelta, filtfilt(b1,a1,fcu_accel.message.AZ))
grid on
hold on
plot(fuse_accel.N6700.capture_header.tdelta, filtfilt(b2,a2,fuse_accel.N6700.message.AZ))
ylabel ('\bf A_z [g]')
legend ('hide', '\bf FCU', '\bf tail-end',  'location', 'best')
xlim (time_limit)

sp4 = subplot (4,1,4);
plot(fcu_accel.capture_header.tdelta, fcu_accel.aio_header.flight_mode,'k')
grid on
%ylabel ('\bf Flight Mode')
xlabel ('\bf time [sec]')
xlim (time_limit)
ylim ([-0.5 12.5])
ax = gca;
ax.YTick = [1:2:12]; ax.YTickLabel = YTickLabel (2:2:end);
linkaxes([sp1,sp2,sp3,sp4],'x')
clear ax sp1 sp2 sp3 sp4;
savefig (gcf, [flight_id, '-fuse_accels.fig'],'compact');
saveas (gcf, [flight_id, '-fuse_accels.png']);


better_figure (6, 1100, 750)
sp1 = subplot (4,1,1);
plot(fcu_accel.capture_header.tdelta, filtfilt(b1,a1,fcu_accel.message.GX))
grid on
hold on
plot(fuse_accel.N6700.capture_header.tdelta, filtfilt(b2,a2,fuse_accel.N6700.message.GX))
ylabel ('\bf G_x [deg/s]')
legend ('show', '\bf FCU', '\bf tail-end',  'location', 'best')
xlim (time_limit)
title ([flight_id, ': ', '\bf Fuselage Angular Rate (LPF: 30 Hz)'])

sp2 = subplot (4,1,2);
plot(fcu_accel.capture_header.tdelta, filtfilt(b1,a1,fcu_accel.message.GY))
grid on
hold on
plot(fuse_accel.N6700.capture_header.tdelta, filtfilt(b2,a2,fuse_accel.N6700.message.GY))
ylabel ('\bf G_y [deg/s]')
legend ('hide', '\bf FCU', '\bf tail-end',  'location', 'best')
xlim (time_limit)

sp3 = subplot (4,1,3);
plot(fcu_accel.capture_header.tdelta, filtfilt(b1,a1,fcu_accel.message.GZ))
grid on
hold on
plot(fuse_accel.N6700.capture_header.tdelta, filtfilt(b2,a2,fuse_accel.N6700.message.GZ))
ylabel ('\bf G_z [deg/s]')
legend ('hide', '\bf FCU', '\bf tail-end',  'location', 'best')
xlim (time_limit)

sp4 = subplot (4,1,4);
plot(fcu_accel.capture_header.tdelta, fcu_accel.aio_header.flight_mode,'k')
grid on
%ylabel ('\bf Flight Mode')
xlabel ('\bf time [sec]')
xlim (time_limit)
ylim ([-0.5 12.5])
ax = gca;
ax.YTick = [1:2:12]; ax.YTickLabel = YTickLabel (2:2:end);
linkaxes([sp1,sp2,sp3,sp4],'x')
clear ax sp1 sp2 sp3 sp4;
savefig (gcf, [flight_id, '-fuse_rates.fig'],'compact');
saveas (gcf, [flight_id, '-fuse_rates.png']);


%% Tail Vibrations

clear a1 b1 a2 b2;
[b,a] = butter(4, 30/(htail_accel.P800.aio_header.sample_rate));

better_figure (7, 1100, 750)
sp1 = subplot (4,1,1);
plot(htail_accel.P800.capture_header.tdelta, filtfilt(b,a,htail_accel.P800.message.AX))
grid on
hold on
plot(vtail_accel.N3000.capture_header.tdelta, filtfilt(b,a,vtail_accel.N3000.message.AX))
ylabel ('\bf A_x [g]')
legend ('show', '\bf elev', '\bf vtail',  'location', 'best')
xlim (time_limit)
title ([flight_id, ': ', '\bf Tail Vibrations (LPF: 30 Hz)'])

sp2 = subplot (4,1,2);
plot(htail_accel.P800.capture_header.tdelta, filtfilt(b,a,htail_accel.P800.message.AZ))
grid on
hold on
plot(vtail_accel.N3000.capture_header.tdelta, filtfilt(b,a,vtail_accel.N3000.message.AZ))
ylabel ('\bf A_y [g]')
legend ('hide', '\bf elev', '\bf vtail',  'location', 'best')
xlim (time_limit)

sp3 = subplot (4,1,3);
plot(htail_accel.P800.capture_header.tdelta, filtfilt(b,a,htail_accel.P800.message.AY))
grid on
hold on
plot(vtail_accel.N3000.capture_header.tdelta, filtfilt(b,a,vtail_accel.N3000.message.AY))
ylabel ('\bf A_z [g]')
legend ('hide', '\bf elev', '\bf vtail',  'location', 'best')
xlim (time_limit)

sp4 = subplot (4,1,4);
plot(fcu_accel.capture_header.tdelta, fcu_accel.aio_header.flight_mode,'k')
grid on
%ylabel ('\bf Flight Mode')
xlabel ('\bf time [sec]')
xlim (time_limit)
ylim ([-0.5 12.5])
ax = gca;
ax.YTick = [1:2:12]; ax.YTickLabel = YTickLabel (2:2:end);
linkaxes([sp1,sp2,sp3,sp4],'x')
clear ax sp1 sp2 sp3 sp4;
savefig (gcf, [flight_id, '-tail_accels.fig'],'compact');
saveas (gcf, [flight_id, '-tail_accels.png']);


better_figure (8, 1100, 750)
sp1 = subplot (4,1,1);
plot(htail_accel.P800.capture_header.tdelta, filtfilt(b,a,htail_accel.P800.message.GX))
grid on
hold on
plot(vtail_accel.N3000.capture_header.tdelta, filtfilt(b,a,vtail_accel.N3000.message.GX))
ylabel ('\bf G_x [deg/s]')
legend ('show', '\bf elev', '\bf vtail',  'location', 'best')
xlim (time_limit)
title ([flight_id, ': ', '\bf Tail Angular Rate (LPF: 30 Hz)'])

sp2 = subplot (4,1,2);
plot(htail_accel.P800.capture_header.tdelta, filtfilt(b,a,htail_accel.P800.message.GY))
grid on
hold on
plot(vtail_accel.N3000.capture_header.tdelta, filtfilt(b,a,vtail_accel.N3000.message.GY))
ylabel ('\bf G_y [deg/s]')
legend ('hide', '\bf elev', '\bf vtail',  'location', 'best')
xlim (time_limit)

sp3 = subplot (4,1,3);
plot(htail_accel.P800.capture_header.tdelta, filtfilt(b,a,htail_accel.P800.message.GZ))
grid on
hold on
plot(vtail_accel.N3000.capture_header.tdelta, filtfilt(b,a,vtail_accel.N3000.message.GZ))
ylabel ('\bf G_z [deg/s]')
legend ('hide', '\bf elev', '\bf vtail',  'location', 'best')
xlim (time_limit)

sp4 = subplot (4,1,4);
plot(fcu_accel.capture_header.tdelta, fcu_accel.aio_header.flight_mode,'k')
grid on
%ylabel ('\bf Flight Mode')
xlabel ('\bf time [sec]')
xlim (time_limit)
ylim ([-0.5 12.5])
ax = gca;
ax.YTick = [1:2:12]; ax.YTickLabel = YTickLabel (2:2:end);
linkaxes([sp1,sp2,sp3,sp4],'x')
clear ax sp1 sp2 sp3 sp4;
savefig (gcf, [flight_id, '-tail_rates.fig'],'compact');
saveas (gcf, [flight_id, '-tail_rates.png']);

%% Strain gage data - wing at P600
clear b a;
[b,a] = butter (4, 30/(wing_load.P600.aio_header.sample_rate));

better_figure (9, 1100, 750)
sp1 = subplot (5,1,1);
plot(wing_load.P600.capture_header.tdelta, filtfilt(b,a,wing_load.P600.message.FZ))
grid on
hold on
plot(wing_load.P600.capture_header.tdelta([1, end]), +6*[1, 1],'r')
plot(wing_load.P600.capture_header.tdelta([1, end]), -0*[1, 1],'--r')
ylabel ('\bf F_z [kN]')
xlim (time_limit)
legend ('show', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')
title ([flight_id, ': ', '\bf Wing Loads at 600 mm stbd (LPF: 30 Hz)'])

sp2 = subplot (5,1,2);
plot(wing_load.P600.capture_header.tdelta, filtfilt(b,a,wing_load.P600.message.MX))
grid on
hold on
plot(wing_load.P600.capture_header.tdelta([1, end]), +133.6*[1, 1],'r')
plot(wing_load.P600.capture_header.tdelta([1, end]), - 46.7*[1, 1],'--r')
ylabel ('\bf M_x [kNm]')
xlim (time_limit)
legend ('hide', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')

sp3 = subplot (5,1,3);
plot(wing_load.P600.capture_header.tdelta, filtfilt(b,a,wing_load.P600.message.MY))
grid on
hold on
plot(wing_load.P600.capture_header.tdelta([1, end]), +   0*[1, 1],'r')
plot(wing_load.P600.capture_header.tdelta([1, end]), -55.1*[1, 1],'--r')
ylabel ('\bf M_y [kNm]')
xlim (time_limit)
legend ('hide', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')

sp4 = subplot (5,1,4);
plot(wing_load.P600.capture_header.tdelta, filtfilt(b,a,wing_load.P600.message.MZ))
grid on
hold on
plot(wing_load.P600.capture_header.tdelta([1, end]), +28.3*[1, 1],'r')
plot(wing_load.P600.capture_header.tdelta([1, end]), - 3.7*[1, 1],'--r')
ylabel ('\bf M_z [kNm]')
xlim (time_limit)
legend ('hide', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')

sp5 = subplot (5,1,5);
plot(fcu_accel.capture_header.tdelta, fcu_accel.aio_header.flight_mode,'k')
grid on
%ylabel ('\bf Flight Mode')
xlabel ('\bf time [sec]')
ylim ([-0.5 12.5])
xlim (time_limit)
ax = gca;
ax.YTick = [1:2:12]; ax.YTickLabel = YTickLabel (2:2:end);
linkaxes([sp1,sp2,sp3,sp4,sp5],'x')
clear ax sp1 sp2 sp3 sp4 sp5;
savefig (gcf, [flight_id, '-wing_loads_p600.fig'],'compact');
saveas (gcf, [flight_id, '-wing_loads_p600.png']);


%% Strain gage data - wing at P5000
better_figure (10, 1100, 750)
sp1 = subplot (5,1,1);
plot(wing_load.P5000.capture_header.tdelta, filtfilt(b,a,wing_load.P5000.message.FZ))
grid on
hold on
plot(wing_load.P5000.capture_header.tdelta([1, end]), +50.2*[1, 1],'r')
plot(wing_load.P5000.capture_header.tdelta([1, end]), -20.0*[1, 1],'--r')
ylabel ('\bf F_z [kN]')
xlim (time_limit)
legend ('show', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')
title ([flight_id, ': ', '\bf Wing Loads at 5000 mm stbd (LPF: 30 Hz)'])

sp2 = subplot (5,1,2);
plot(wing_load.P5000.capture_header.tdelta, filtfilt(b,a,wing_load.P5000.message.MX))
grid on
hold on
plot(wing_load.P5000.capture_header.tdelta([1, end]), + 0*[1, 1],'r')
plot(wing_load.P5000.capture_header.tdelta([1, end]), -58.9*[1, 1],'--r')
ylabel ('\bf M_x [kNm]')
xlim (time_limit)
legend ('hide', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')

sp3 = subplot (5,1,3);
plot(wing_load.P5000.capture_header.tdelta, filtfilt(b,a,wing_load.P5000.message.MY))
grid on
hold on
plot(wing_load.P5000.capture_header.tdelta([1, end]), +5.4*[1, 1],'r')
plot(wing_load.P5000.capture_header.tdelta([1, end]), -2.5*[1, 1],'--r')
ylabel ('\bf M_y [kNm]')
xlim (time_limit)
legend ('hide', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')

sp4 = subplot (5,1,4);
plot(wing_load.P5000.capture_header.tdelta, filtfilt(b,a,wing_load.P5000.message.MZ))
grid on
hold on
plot(wing_load.P5000.capture_header.tdelta([1, end]), + 8.1*[1, 1],'r')
plot(wing_load.P5000.capture_header.tdelta([1, end]), -10.1*[1, 1],'--r')
ylabel ('\bf M_z [kNm]')
xlim (time_limit)
legend ('hide', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')

sp5 = subplot (5,1,5);
plot(fcu_accel.capture_header.tdelta, fcu_accel.aio_header.flight_mode,'k')
grid on
xlabel ('\bf time [sec]')
%ylabel ('\bf Flight Mode')
ylim ([-0.5 12.5])
xlim (time_limit)
ax = gca;
ax.YTick = [1:2:12]; ax.YTickLabel = YTickLabel (2:2:end);
linkaxes([sp1,sp2,sp3,sp4,sp5],'x')
clear ax sp1 sp2 sp3 sp4 sp5;
savefig (gcf, [flight_id, '-wing_loads_p5000.fig'],'compact');
saveas (gcf, [flight_id, '-wing_loads_p5000.png']);

% calculate the vertical shear at the stbd BHP
FZ_BHP = 0.03033 * (wing_strain.P7000.message.S_FR_F - wing_strain.P5000.message.S_FR_F) * 10^6;

%% BHP reaction P 6000 and tether tension
better_figure (11, 1100, 750)
sp1 = subplot (3,1,1);
plot(wing_strain.P5000.capture_header.tdelta, filtfilt(b,a,FZ_BHP))
grid on
hold on
plot(wing_strain.P5000.capture_header.tdelta([1, end]), +44.3*[1, 1],'r')
plot(wing_strain.P5000.capture_header.tdelta([1, end]),     0*[1, 1],'--r')
ylabel ('\bf F_z [kN]')
xlim (time_limit)
legend ('show', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')
title ([flight_id, ': ', '\bf BHP load at 6000 mm stbd (LPF: 30 Hz)'])

sp2 = subplot (3,1,2);
plot(fcu_accel.capture_header.tdelta, tether_tension)
grid on
hold on
plot(fcu_accel.capture_header.tdelta([1, end]), +165*[1, 1],'r')
plot(fcu_accel.capture_header.tdelta([1, end]),    0*[1, 1],'--r')
ylabel ('\bf F_t [kN]')
xlim (time_limit)
legend ('hide', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')
title ([flight_id, ': ', '\bf Tether tension'])

sp3 = subplot (3,1,3);
plot(fcu_accel.capture_header.tdelta, fcu_accel.aio_header.flight_mode,'k')
grid on
xlabel ('\bf time [sec]')
%ylabel ('\bf Flight Mode')
ylim ([-0.5 12.5])
xlim (time_limit)
ax = gca;
ax.YTick = [0:4:12];
linkaxes([sp1,sp2 sp3],'x')
clear ax sp1 sp2 sp3 ;
savefig (gcf, [flight_id, '-wing_loads_bhp.fig'],'compact');
saveas (gcf, [flight_id, '-wing_loads_bhp.png']);

%% Strain gage data - Fuse
% find zero index
idx = find (fuse_load.P0.capture_header.tdelta > 60);
idx_0 = idx(1);
%idx_0 = find (abs(fuse_load.P0.capture_header.tdelta) == min(abs(fuse_load.P0.capture_header.tdelta)));

better_figure (12, 1100, 750)
sp1 = subplot (6,1,1);
plot(fuse_load.P0.capture_header.tdelta, filtfilt(b,a,(fuse_load.P0.message.FY - fuse_load.P0.message.FY(idx_0))))
grid on
hold on
plot(fuse_load.P0.capture_header.tdelta([1, end]), +8.7*[1, 1],'r')
plot(fuse_load.P0.capture_header.tdelta([1, end]), -8.7*[1, 1],'--r')
ylabel ('\bf F_y [kN]')
xlim (time_limit)
legend ('show', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')
title ([flight_id, ': ', '\bf Fuselage and Tail Loads (LPF: 30 Hz)'])

sp2 = subplot (6,1,2);
plot(fuse_load.P0.capture_header.tdelta, filtfilt(b,a,(fuse_load.P0.message.FZ - fuse_load.P0.message.FZ(idx_0))))
grid on
hold on
plot(fuse_load.P0.capture_header.tdelta([1, end]), +9.2*[1, 1],'r')
plot(fuse_load.P0.capture_header.tdelta([1, end]), -7.0*[1, 1],'--r')
%plot(fuse_load.P0.capture_header.tdelta, -2.5*ones(size(fuse_load.P0.capture_header.tdelta)),'--r')
ylabel ('\bf F_z [kN]')
xlim (time_limit)
legend ('hide', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')

sp3 = subplot (6,1,3);
plot(fuse_load.P0.capture_header.tdelta, filtfilt(b,a,(fuse_load.P0.message.MX_R - fuse_load.P0.message.MX_R(idx_0))))
grid on
hold on
plot(fuse_load.P0.capture_header.tdelta([1, end]), +10.3*[1, 1],'r')
plot(fuse_load.P0.capture_header.tdelta([1, end]), -14.3*[1, 1],'--r')
ylabel ('\bf M_x [kNm]')
xlim (time_limit)
legend ('hide', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')

sp4 = subplot (6,1,4);
plot(fuse_load.P0.capture_header.tdelta, filtfilt(b,a,(fuse_load.P0.message.MY - fuse_load.P0.message.MY(idx_0))))
grid on
hold on
plot(fuse_load.P0.capture_header.tdelta([1, end]), +53.8*[1, 1],'r')
plot(fuse_load.P0.capture_header.tdelta([1, end]), -40.0*[1, 1],'--r')
%plot(fuse_load.P0.capture_header.tdelta, -14.2*ones(size(fuse_load.P0.capture_header.tdelta)),'.')
ylabel ('\bf M_y [kNm]')
xlim (time_limit)
legend ('hide', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')

sp5 = subplot (6,1,5);
plot(fuse_load.P0.capture_header.tdelta, filtfilt(b,a,(fuse_load.P0.message.MZ -  - fuse_load.P0.message.MZ(idx_0))))
grid on
hold on
plot(fuse_load.P0.capture_header.tdelta([1, end]), +67.5*[1, 1],'r')
plot(fuse_load.P0.capture_header.tdelta([1, end]), -67.5*[1, 1],'--r')
ylabel ('\bf M_z [kNm]')
xlim (time_limit)
legend ('hide', '\bf RPX', '\bf \uparrow Limit', '\bf \downarrow Limit' ,'location', 'best')

sp6 = subplot (6,1,6);
plot(fcu_accel.capture_header.tdelta, fcu_accel.aio_header.flight_mode,'k')
grid on
xlabel ('\bf time [sec]')
%ylabel ('\bf Flight Mode')
ylim ([-0.5 12.5])
xlim (time_limit)
ax = gca;
ax.YTick = [1:2:12]; ax.YTickLabel = YTickLabel (2:2:end);
linkaxes([sp1,sp2,sp3,sp4,sp5,sp6],'x')
clear ax sp1 sp2 sp3 sp4 sp5 sp6;
savefig (gcf, [flight_id, '-tail_loads.fig'],'compact');
saveas (gcf, [flight_id, '-tail_loads.png']);

%% Strain gage data - rotor
better_figure (13, 1100, 750)
sp1 = subplot (3,1,1);
plot(rotor_load.SBO.capture_header.tdelta, rotor_load.SBO.message.T)
hold on
plot(rotor_load.SBO.capture_header.tdelta, movmean(rotor_load.SBO.message.T, 101))
grid on
ylabel ('\bf F [N]')
xlim (time_limit)
legend ('show', '\bf sbo' , '\bf T_{avg}' ,'location', 'best')
title ([flight_id, ': ', '\bf Rotor Thrust'])

sp2 = subplot (3,1,2);
plot(rotor_load.STO.capture_header.tdelta, rotor_load.STO.message.T)
hold on
plot(rotor_load.STO.capture_header.tdelta, movmean(rotor_load.STO.message.T, 101))
grid on
ylabel ('\bf F [N]')
xlim (time_limit)
legend ('show', '\bf sto' , '\bf T_{avg}' ,'location', 'best')

sp3 = subplot (3,1,3);
plot(fcu_accel.capture_header.tdelta, fcu_accel.aio_header.flight_mode,'k')
grid on
xlabel ('\bf time [sec]')
%ylabel ('\bf Flight Mode')
ylim ([-0.5 12.5])
xlim (time_limit)
ax = gca;
ax.YTick = [1:2:12]; ax.YTickLabel = YTickLabel (2:2:end);
linkaxes([sp1,sp2,sp3],'x')
clear ax sp1 sp2 sp3;
savefig (gcf, [flight_id, '-rotor_thrust.fig'],'compact');
saveas (gcf, [flight_id, '-rotor_thrust.png']);


better_figure (14, 1100, 750)
clear b a;
sp1 = subplot (4,1,1);
plot(pylon4_strain.P1700.capture_header.tdelta, pylon4_strain.P1700.message.A_MMOF_F*10^6)
grid on
hold on
plot(pylon4_strain.P1700.capture_header.tdelta, pylon4_strain.P1700.message.A_MMOA_F*10^6)
plot(pylon4_strain.P1700.capture_header.tdelta, pylon4_strain.P1700.message.A_MMIF_F*10^6)
plot(pylon4_strain.P1700.capture_header.tdelta, pylon4_strain.P1700.message.A_MMIA_F*10^6)
ylabel ('\bf strain [\mu\epsilon]')
title ([flight_id, ': ', '\bf SBO Motor Mount'])
legend ('show', '\bf OF', '\bf OA', '\bf IF', '\bf IA',  'location', 'best')
xlim (time_limit)

sp2 = subplot (4,1,2);
plot(pylon4_strain.P0.capture_header.tdelta, pylon4_strain.P0.message.A_LH_F*10^6)
grid on
hold on
plot(pylon4_strain.P0.capture_header.tdelta, pylon4_strain.P0.message.A_RH_F*10^6)
plot(pylon4_strain.P0.capture_header.tdelta, pylon4_strain.P0.message.A_RLH_F*10^6)
plot(pylon4_strain.P0.capture_header.tdelta, pylon4_strain.P0.message.A_RRH_F*10^6)
ylabel ('\bf strain [\mu\epsilon]')
title ([flight_id, ': ', '\bf Pylon and Pylon-Wing (PW) Junction'])
legend ('show', '\bf Spar LH', '\bf Spar RH', '\bf PW - LH', '\bf PW - RH',  'location', 'best')
xlim (time_limit)

sp3 = subplot (4,1,3);
plot(pylon4_strain.N1300.capture_header.tdelta, pylon4_strain.N1300.message.A_MMOF_F*10^6)
grid on
hold on
plot(pylon4_strain.N1300.capture_header.tdelta, pylon4_strain.N1300.message.A_MMOA_F*10^6)
plot(pylon4_strain.N1300.capture_header.tdelta, pylon4_strain.N1300.message.A_MMIF_F*10^6)
plot(pylon4_strain.N1300.capture_header.tdelta, pylon4_strain.N1300.message.A_MMIA_F*10^6)
ylabel ('\bf strain [\mu\epsilon]')
title ([flight_id, ': ', '\bf STO Motor Mount'])
legend ('show', '\bf OF', '\bf OA', '\bf IF', '\bf IA',  'location', 'best')
xlim (time_limit)

sp4 = subplot (4,1,4);
plot(fcu_accel.capture_header.tdelta, fcu_accel.aio_header.flight_mode,'k')
grid on
%ylabel ('\bf Flight Mode')
xlabel ('\bf time [sec]')
xlim (time_limit)
ylim ([-0.5 12.5])
ax = gca;
ax.YTick = [1:2:12]; ax.YTickLabel = YTickLabel (2:2:end);
linkaxes([sp1,sp2,sp3,sp4],'x')
clear ax sp1 sp2 sp3 sp4;
savefig (gcf, [flight_id, '-pylon_strains.fig'],'compact');
saveas (gcf, [flight_id, '-pylon_strains.png']);

% wing strain gages
loc_id = fieldnames(wing_strain);
for ii = 1:numel (loc_id);
  strain_time = getfield(getfield(getfield(wing_strain, loc_id{ii,1}), 'capture_header'), 'tdelta');
  strain_gages = getfield(getfield(wing_strain, loc_id{ii,1}), 'message');
  gage_id = fieldnames(strain_gages);
  for jj = 1:numel(gage_id);
    better_figure (14+ii, 1100, 750);
    hold on
    plot (strain_time, getfield(strain_gages, gage_id{jj,1}))
    hold off
  end
  grid on
  xlabel ('\bf time [sec]')
  ylabel ('\bf strain [\epsilon]')
  xlim (time_limit)
  legend (strrep(gage_id, '_', '-'), 'location', 'best', 'FontWeight', 'bold')

  lbl = strrep(loc_id{ii,1}, '_', '-');
  if ~isempty(strfind(lbl, 'N'));
    lbl = strrep(lbl, 'N', 'port ');
  elseif ~isempty(strfind(lbl, 'P'));
    lbl = strrep(lbl, 'P', 'stbd ');
  end

  title ([flight_id, ': ', '\bf Wing Strain at ', lbl, ' mm'])
  savefig (gcf, [flight_id, '-wing_strains_', loc_id{ii,1} '.fig'],'compact');
  saveas (gcf, [flight_id, '-wing_strains_', loc_id{ii,1} '.png']);
  clear lbl;
end

cd('../../');

clearvars -except messages info
