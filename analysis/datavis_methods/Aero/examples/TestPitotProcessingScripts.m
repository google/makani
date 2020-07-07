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
h5file =  [getenv('MAKANI_HOME'), 'logs/20170807-rpx06_CommandCenter.h5'];
% specify if pitot calibrations are to be used
use_calibrations = 1;
pitot_sno = '15239-3';
% shouldn't need to change any inputs below this block
% ==========
% User inputs End

% Find if log is from wing recorder or command center.
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
system_params_path  = '/parameters/system_params';
control_params_path = '/parameters/control_params';

% Read control and system parameters dataset.
control_dataset = h5read(h5file, cont_path);
system_params   = h5read(h5file, system_params_path);
control_params  = h5read(h5file, control_params_path);

% Get the air density used by controller.
rho = system_params.phys.rho;

% Fade between low-speed and high-speed Pitot sensors.
[dyn_press, alpha_press, beta_press, stat_press] = ...
    CrossfadePitotPressures(control_dataset.message.control_input.pitots, ...
                            system_params.pitot);
port_angle = system_params.pitot.port_angle;

% Step 1: get the apparent wind from pressure data.
if use_calibrations == 1
  % Get all calibrations for specified pitot.
  pitot_cal = CollectAllPitotCalibrations(pitot_sno);
  % Use the factory calibrations
  % to get apparent wind from measured pitot pressures.
  apparent_wind = PitotPressuresToApparentWindUsingCalibrations(alpha_press, ...
                             beta_press, dyn_press, stat_press, rho', pitot_cal);
else
  % Use the physics based pressure distribution on a sphere modeling
  % to get apparent wind from measured pitot pressures.
  apparent_wind = PitotPressuresToApparentWindUsingModel(alpha_press, ...
                                       beta_press, dyn_press, rho', port_angle);
end

% Step 2: transform from pitot to body coordinates.
% Note: Transpose the matrices when imported from the h5 logs into matlab.
dcm_b2p = system_params.pitot.dcm_b2p.d';
apparent_wind_body = TransformPitotToBody(apparent_wind, dcm_b2p);

% Step 3: apply correction for body rates

pitot_position = s2m(system_params.pitot.pos);
p = control_dataset.message.state_est.pqr.x';
q = control_dataset.message.state_est.pqr.y';
r = control_dataset.message.state_est.pqr.z';
apparent_wind_rate_corrected = ...
    CorrectApparentWindForBodyRates(apparent_wind_body, pitot_position, p, q, r);

% Step 4: convert apparent wind to aero parameters
[v, aoa, aos] = ApparentWindCartToSph(apparent_wind_rate_corrected);

% Step 5: apply upwash correction to alpha
alpha_scale = control_params.estimator.apparent_wind.pitot_upwash_alpha_scale;
alpha_bias  = control_params.estimator.apparent_wind.pitot_upwash_alpha_bias;
aoa = CorrectAlphaForUpwash(aoa, alpha_scale, alpha_bias);

% Read speed, aoa and aos from the h5 log.
h5_v   = control_dataset.message.state_est.apparent_wind.sph.v';
h5_aoa = control_dataset.message.state_est.apparent_wind.sph.alpha';
h5_aos = control_dataset.message.state_est.apparent_wind.sph.beta';

% Get flight time for reference.
flight_time = get_node_time(control_dataset) - get_time_zero(control_dataset);

% plot and compare
figure(1)
sp1 = subplot(4,1,1);
plot(flight_time, aoa*180/pi)
hold on
plot(flight_time, h5_aoa*180/pi)
legend ('\bf \alpha_{test}', '\bf \alpha_{h5}', 'orientation', 'horizontal')
ylabel ('\bf \alpha [deg]')
grid on
hold off

sp2 = subplot(4,1,2);
plot(flight_time, aos*180/pi)
hold on
plot(flight_time, h5_aos*180/pi)
legend ('\bf \beta_{test}', '\bf \beta_{h5}', 'orientation', 'horizontal')
ylabel ('\bf \beta [deg]')
grid on
hold off

sp3 = subplot(4,1,3);
plot(flight_time, v)
hold on
plot(flight_time, h5_v)
legend ('\bf V_{test}', '\bf V_{h5}', 'orientation', 'horizontal')
ylabel ('\bf V [m/s]')
grid on
hold off

sp4 = subplot(4,1,4);
plot(flight_time, (aoa - h5_aoa)*180/pi)
hold on
plot(flight_time, (aos - h5_aos)*180/pi)
plot(flight_time, (v - h5_v))
legend ('\bf \alpha', '\bf \beta', '\bf V', 'orientation', 'horizontal')
ylabel ('\bf |\Delta|')
xlabel ('\bf flight time [sec]')
grid on
hold off

linkaxes([sp1, sp2, sp3, sp4], 'x');
xlim([300 2400])
set(findall(gcf,'-property','FontSize'),'FontSize',20);