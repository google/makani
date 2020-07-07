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

function [fig_i] = plot_ins_inputs(data, t0, t1, fig_i)
if ~exist('t0', 'var') || isempty(t0)
  t0 = 0;
end
if ~exist('t1', 'var') || isempty(t1)
  t1 = Inf;
end
if ~exist('fig_i', 'var') || isempty(fig_i)
  fig_i = 0;
end


fig_i = plot_imu(data.imu, t0, t1, fig_i);
fig_i = plot_obs(data.obs, 'pr', 'Pseudo-range', 'm', t0, t1, fig_i);
fig_i = plot_obs(data.obs, 'do', 'Doppler', 'Hz', t0, t1, fig_i);
fig_i = plot_obs(data.obs, 'cp', 'Carrier phase', 'm', t0, t1, fig_i);
fig_i = plot_obs(data.obs, 'cn0', 'C/N_0', 'm', t0, t1, fig_i);
fig_i = plot_obs(data.obs, 'lock_time', 'Lock time', 's', t0, t1, fig_i);
fig_i = plot_obs_latency(data.obs, t0, t1, fig_i);
fig_i = plot_tow(data, 't_oc', 'Time of clock', t0, t1, fig_i);
fig_i = plot_tow(data, 't_oe', 'Time of ephemeris', t0, t1, fig_i);

if isfield(data, 'wheel')
  fig_i = plot_odometry(data, t0, t1, fig_i);
end


function [fig_i] = plot_imu(data, t0, t1, fig_i)
ii = get_interval(data.t, t0, t1);
fig_i = inc_figure(fig_i, 'IMU data');
h = zeros(2, 1);
h(1) = subplot(2, 1, 1);
plot(data.t(ii), data.dvsf(ii, :));
ylabel('DvSF (m/s)');
h(2) = subplot(2, 1, 2);
plot(data.t(ii), data.phi(ii, :) * 180/pi);
ylabel('Phi (deg)');
xlabel('Time (s)');
linkaxes(h, 'x');
axis tight;


function [fig_i] = plot_odometry(data, t0, t1, fig_i)
ii_resolver = get_interval(data.resolver.t, t0, t1);
ii_wheel = get_interval(data.wheel.t, t0, t1);
fig_i = inc_figure(fig_i, 'Odometry data');
plot(data.resolver.t(ii_resolver), data.resolver.ll_vel(ii_resolver, :), ...
     data.wheel.t(ii_wheel), data.wheel.speed(ii_wheel, :));
legend('Resolver', 'Wheel FL', 'Wheel FR', 'Wheel RL', 'Wheel RR');
ylabel('Angle (counts)');
xlabel('Time (s)');
axis tight;


function [fig_i] = plot_obs_latency(data, t0, t1, fig_i)
fig_i = inc_figure(fig_i, 'Observable latency');
ii = get_interval(data.t, t0, t1);
plot(data.t(ii), data.t(ii) - data.t_obs(ii));
ylabel('Latency (s)');
xlabel('Time (s)');
axis tight;


function [fig_i] = plot_obs(data, field, name, units, t0, t1, fig_i)
fig_i = inc_figure(fig_i, name);
hold on;
prns = unique(data.prn);
for p = 1:length(prns)
  prn = prns(p);
  ii = get_interval(data.t, t0, t1);
  ii = ii(data.prn(ii) == prn);
  plot(data.t(ii), data.(field)(ii, :));
end
hold off;
ylabel(sprintf('%s (%s)', name, units));
xlabel('Time (s)');
axis tight;


function [fig_i] = plot_tow(data, field, name, t0, t1, fig_i)
fig_i = inc_figure(fig_i, name);
hold on;
ii = get_interval(data.obs.t, t0, t1);
scale = 1 / (3600 * 24);
plot(data.obs.t(ii), data.obs.tow(ii, :) * scale);
prns = unique(data.eph.prn);
for p = 1:length(prns)
  prn = prns(p);
  ii = get_interval(data.eph.t, t0, t1);
  ii = ii(data.eph.prn(ii) == prn);
  plot(data.eph.t(ii), data.eph.(field)(ii, :) * scale);
end
hold off;
ylabel(sprintf('%s (dow)', name));
xlabel('Time (s)');
axis tight;
