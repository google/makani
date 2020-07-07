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

% Example file to process main pitot data from RPX telemetry
% and compare pitot analytical model with the calibration data.

% User inputs
% ==========
% specify the h5 file, with local path, not on makani repo
h5file =  './controls_data/20170807-rpx06_CommandCenter.h5';

% specify the pitot serial number
% please refer to README.txt in makani/database/AeroProbe
% to find the serial number for the pitot on the kite
pitot_sno = '15239-3';
% shouldn't need to change any inputs below this block
% ==========
% User inputs End

% Get all calibrations for specified pitot.
pitot_cal = CollectAllPitotCalibrations(pitot_sno);

% find if log is from wing recorder or command center
file_info = h5info(h5file, '/messages/kAioNodeControllerA/');
for ii = 1:length(file_info.Datasets);
  all_datasets{ii} = file_info.Datasets(ii).Name;
end
clear ii;
if sum(ismember(all_datasets, 'kMessageTypeControlDebug')) == 1
  cont_path = '/messages/kAioNodeControllerA/kMessageTypeControlDebug';
elseif sum(ismember(all_datasets, 'kMessageTypeControlTelemetry')) == 1
  cont_path = '/messages/kAioNodeControllerA/kMessageTypeControlTelemetry';
end
param_path = '/parameters/system_params';

% read control and system parameters dataset
control_dataset = h5read(h5file, cont_path);
system_params   = h5read(h5file, param_path);

% calculate density
weather = control_dataset.message.control_input.weather;
rho = calculate_density(weather.pressure, weather.temperature, weather.humidity).';

port_angle = system_params.pitot.port_angle;

% Fade between low-speed and high-speed Pitot sensors.
[dyn_press, alpha_press, beta_press, stat_press] = ...
    CrossfadePitotPressures(control_dataset.message.control_input.pitots, ...
                            system_params.pitot);

% get the apparent wind from pressure data in pitot frame,
% using spherical flow model
apparent_wind_model_p = PitotPressuresToApparentWindUsingModel(alpha_press, ...
                                       beta_press, dyn_press, rho, port_angle);

% convert apparent wind to speed and aero angles
[model_v, model_aoa, model_aos] = ApparentWindCartToSph(apparent_wind_model_p);

model_aoa = model_aoa * 180/pi;  % [deg]
model_aos = model_aos * 180/pi;  % [deg]

% get the apparent wind from pressure data in pitot frame,
% using pitot calibrations
apparent_wind_cal_p = PitotPressuresToApparentWindUsingCalibrations(alpha_press, ...
                             beta_press, dyn_press, stat_press, rho, pitot_cal);

% convert apparent wind to speed and aero angles
[cal_v, cal_aoa, cal_aos] = ApparentWindCartToSph(apparent_wind_cal_p);

cal_aoa = cal_aoa * 180/pi;  % [deg]
cal_aos = cal_aos * 180/pi;  % [deg]

% Get calibrated 5-port pitot differential pressures by converting the
% calibrated apparent wind speed back into pressures
p_dynamic = SphereDynamicPressure(rho, apparent_wind_cal_p, [1; 0; 0]);
p_alpha = SphereDynamicPressure(rho, apparent_wind_cal_p, ...
                                [cos(port_angle); 0; sin(port_angle)]) - ...
          SphereDynamicPressure(rho, apparent_wind_cal_p, ...
                                [cos(port_angle); 0; -sin(port_angle)]);
p_beta = SphereDynamicPressure(rho, apparent_wind_cal_p, ...
                               [cos(port_angle); sin(port_angle); 0]) - ...
         SphereDynamicPressure(rho, apparent_wind_cal_p, ...
                               [cos(port_angle); -sin(port_angle); 0]);

% get flight time for reference
flight_time = get_node_time(control_dataset) - get_time_zero(control_dataset);

% calibration speeds
all_cals = fieldnames(pitot_cal);
for ii = 1:length(all_cals)
  cal_speed(ii) = pitot_cal.(all_cals{ii}).speed;
end
clear ii;

% find modes and cases in the flight
[flight_modes, crosswind_cases] = get_modes_and_cases(control_dataset);
% the cw mode index
idx = [flight_modes.indices.mode7(1):flight_modes.indices.mode7(2)];
% case 9 for rpx-06
idx_9 = [crosswind_cases.indices.case9(1):crosswind_cases.indices.case9(2)];
% loop angle
loop_angle = control_dataset.message.crosswind.loop_angle;
% loop averaged data
time_data = [model_v; model_aoa; model_aos; cal_v; cal_aoa; cal_aos];
loop_data = LoopAverager(loop_angle(idx_9), time_data(:,idx_9), 360);

% plot and compare
figure(1)
sp1 = subplot(4,1,1);
plot(flight_time, model_aoa)
hold on
plot(flight_time, cal_aoa)
legend ('\bf \alpha_{model}', '\bf \alpha_{cal}', 'orientation', 'horizontal')
ylabel ('\bf \alpha [deg]')
grid on
hold off

sp2 = subplot(4,1,2);
plot(flight_time, model_aos)
hold on
plot(flight_time, cal_aos)
legend ('\bf \beta_{model}', '\bf \beta_{cal}', 'orientation', 'horizontal')
ylabel ('\bf \beta [deg]')
grid on
hold off

sp3 = subplot(4,1,3);
plot(flight_time, model_v)
hold on
plot(flight_time, cal_v)
legend ('\bf V_{model}', '\bf V_{cal}', 'orientation', 'horizontal')
ylabel ('\bf V [m/s]')
grid on
hold off

sp4 = subplot(4,1,4);
plot(flight_time, abs(cal_aoa - model_aoa))
hold on
plot(flight_time, abs(cal_aos - model_aos))
plot(flight_time, abs(cal_v - model_v))
legend ('\bf \alpha', '\bf \beta', '\bf V', 'orientation', 'horizontal')
ylabel ('\bf |\Delta|')
xlabel ('\bf flight time [sec]')
grid on
hold off

linkaxes([sp1, sp2, sp3, sp4], 'x');
xlim ([300 2500])
set(findall(gcf,'-property','FontSize'),'FontSize',20);

figure(2)
set (0, 'DefaultLineLineWidth',2);
sp1 = subplot(2,2,1);
plot(cal_aoa(idx), abs(cal_aoa(idx) - model_aoa(idx)), '.', 'Color', [0.5 0.5 0.5])
ylabel ('\bf |\Delta\alpha| [deg]')
xlabel ('\bf \alpha_{cal}')
grid on

sp2 = subplot(2,2,2);
plot(cal_v(idx), abs(cal_aoa(idx) - model_aoa(idx)), '.', 'Color', [0.5 0.5 0.5])
ylabel ('\bf |\Delta\alpha| [deg]')
xlabel ('\bf V_{cal}')
grid on
hold on
for ii = 1:length(all_cals)
  plot(round(cal_speed(ii))*[1,1], [0,7], '--k')
end
clear ii;
hold off

sp3 = subplot(2,2,3);
plot(cal_aos(idx), abs(cal_aos(idx) - model_aos(idx)), '.', 'Color', [0.5 0.5 0.5])
ylabel ('\bf |\Delta\beta| [deg]')
xlabel ('\bf \beta_{cal}')
grid on

sp4 = subplot(2,2,4);
plot(cal_v(idx), abs(cal_aos(idx) - model_aos(idx)), '.', 'Color', [0.5 0.5 0.5])
ylabel ('\bf |\Delta\beta| [deg]')
xlabel ('\bf V_{cal}')
grid on
hold on
for ii = 1:length(all_cals)
  plot(round(cal_speed(ii))*[1,1], [0,7], '--k')
end
clear ii;
hold off

linkaxes([sp2, sp4], 'x'); xlim ([15 65]);
linkaxes([sp1, sp3], 'x');
set(findall(gcf,'-property','FontSize'),'FontSize',20);

figure(3)
sp1 = subplot(4,1,1);
plot(loop_data.common_angles*180/pi, loop_data.average(2,:), 'b')
hold on
plot(loop_data.common_angles*180/pi, loop_data.average(5,:), 'r')
legend ('\bf \alpha_{model}', '\bf \alpha_{cal}', 'orientation', 'horizontal')
ylabel ('\bf \alpha [deg]')
grid on
hold off
set(gca, 'xtick', 0:45:360)
title ('\bf RPX - Case 9')

sp2 = subplot(4,1,2);
plot(loop_data.common_angles*180/pi, loop_data.average(3,:), 'b')
hold on
plot(loop_data.common_angles*180/pi, loop_data.average(6,:), 'r')
legend ('\bf \beta_{model}', '\bf \beta_{cal}', 'orientation', 'horizontal')
ylabel ('\bf \beta [deg]')
grid on
hold off
set(gca, 'xtick', 0:45:360)

sp3 = subplot(4,1,3);
plot(loop_data.common_angles*180/pi, loop_data.average(1,:), 'b')
hold on
plot(loop_data.common_angles*180/pi, loop_data.average(4,:), 'r')
legend ('\bf V_{model}', '\bf V_{cal}', 'orientation', 'horizontal')
ylabel ('\bf V [m/s]')
grid on
hold off
set(gca, 'xtick', 0:45:360)

sp4 = subplot(4,1,4);
plot(loop_data.common_angles*180/pi, abs(loop_data.average(2,:) - loop_data.average(5,:)))
hold on
plot(loop_data.common_angles*180/pi, abs(loop_data.average(3,:) - loop_data.average(6,:)))
plot(loop_data.common_angles*180/pi, abs(loop_data.average(1,:) - loop_data.average(4,:)))
legend ('\bf \alpha', '\bf \beta', '\bf V', 'orientation', 'horizontal')
ylabel ('\bf |\Delta|')
xlabel ('\bf kite position [deg]')
grid on
hold off
set(gca, 'xtick', 0:45:360)

linkaxes([sp1, sp2, sp3, sp4], 'x');
xlim([0 360])
set(findall(gcf,'-property','FontSize'),'FontSize',20);
