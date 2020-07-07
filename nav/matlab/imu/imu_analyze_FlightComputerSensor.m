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

function [fig_i] = imu_analyze_FlightComputerSensor(fig_i, msg, t0, tf)
if ~exist('t0', 'var') || isempty(t0)
  t0 = 0;
end
if ~exist('tf', 'var') || isempty(tf)
  tf = Inf;
end

% Select time interval.
time = msg.t - msg.t(1);
ii = find(time >= t0);
ii_imu = ii(time(ii) <= tf);
ii_mag = ii_imu(bitget(uint32(msg.imu.status(ii_imu)), 9) ~= 0);

% Compare to ideal model.
model = adis16488_spec(1 / 2400);

% Process.
fig_i = imu_plot_accel(fig_i, time(ii_imu), msg.imu.raw.acc(:, ii_imu)', model.accel);
fig_i = imu_plot_gyro(fig_i, time(ii_imu), msg.imu.raw.gyro(:, ii_imu)', model.gyro);
fig_i = imu_plot_mag(fig_i, time(ii_mag), msg.imu.raw.mag(:, ii_mag)', model.mag);
