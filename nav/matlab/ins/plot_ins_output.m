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

function [fig_i] = plot_ins_output(out, t0, t1, fig_i)
if ~exist('t0','var') || isempty(t0)
  t0 = 0;
end
if ~exist('t1','var') || isempty(t1)
  t1 = inf;
end
if ~exist('fig_i','var') || isempty(fig_i)
  fig_i = 0;
end
r2d = 180 / pi;

ii = get_interval(out.t, t0, t1);
ii = ii(out.init(ii) > 0);
init_time = out.t(out.init(ii(1)));

ii_gps = get_interval(out.gps.t, t0, t1);
ii_gps = ii_gps(out.gps.t(ii_gps) >= init_time);
ii_gps = ii_gps(ii_gps <= out.gps.count);

ii_pose = get_interval(out.pose.t, t0, t1);
ii_pose = ii_pose(out.pose.t(ii_pose) >= init_time);
ii_pose = ii_pose(out.pose.align(ii_pose) > 1);
ii_pose = ii_pose(out.pose.lat(ii_pose) ~= 0);
ii_pose = ii_pose(out.pose.lon(ii_pose) ~= 0);

ii_posllh = get_interval(out.posllh.t, t0, t1);
ii_posllh = ii_posllh(out.posllh.t(ii_posllh) >= init_time);

ii_velned = get_interval(out.velned.t, t0, t1);
ii_velned = ii_velned(out.velned.t(ii_velned) >= init_time);

ii_clock = get_interval(out.clock.t, t0, t1);
ii_clock = ii_clock(out.clock.t(ii_clock) >= init_time);

ii_resolver = get_interval(out.resolver.t, t0, t1);
ii_resolver = ii_resolver(out.resolver.t(ii_resolver) >= init_time);

ii_wheel = get_interval(out.wheel.t, t0, t1);
ii_wheel = ii_wheel(out.wheel.t(ii_wheel) >= init_time);

% GPS antenna trajectory.
fig_i = inc_figure(fig_i, 'GPS antenna trajectory');
hold on;
plot(out.posllh.lon(ii_posllh) * r2d, out.posllh.lat(ii_posllh) * r2d, 'g', ...
     out.pose.lon(ii_pose) * r2d, out.pose.lat(ii_pose) * r2d, 'r', ...
     out.gps.pt_lon(ii_gps) * r2d, out.gps.pt_lat(ii_gps) * r2d, 'm', ...
     out.antenna_lon(ii) * r2d, out.antenna_lat(ii) * r2d, 'b');
plot(out.posllh.lon(ii_posllh(1)) * r2d, ...
     out.posllh.lat(ii_posllh(1)) * r2d, 'go', ...
     out.posllh.lon(ii_posllh(end)) * r2d, ...
     out.posllh.lat(ii_posllh(end)) * r2d, 'rx', 'MarkerSize', 10);
plot(out.antenna_lon(ii(1)) * r2d, ...
     out.antenna_lat(ii(1)) * r2d, 'go', ...
     out.antenna_lon(ii(end)) * r2d, ...
     out.antenna_lat(ii(end)) * r2d, 'rx', 'MarkerSize', 10);
axis tight;
hold off;
xlabel('Longitude (deg)');
ylabel('Latitude (deg)');
legend('POSLLH', 'KEA', 'POINT', 'INS');
grid on;
axis square;

% GPS antenna position.
fig_i = inc_figure(fig_i, 'GPS antenna position');
h = zeros(3, 1);
h(1) = subplot(3, 1, 1);
plot(out.posllh.t(ii_posllh), out.posllh.lat(ii_posllh) * r2d, 'g', ...
     out.gps.t(ii_gps), out.gps.pt_lat(ii_gps) * r2d, 'm', ...
     out.t(ii), out.antenna_lat(ii) * r2d, 'b');
axis tight;
ylabel('Latitude (deg)');
legend('POSLLH', 'POINT', 'INS');
h(2) = subplot(3, 1, 2);
plot(out.posllh.t(ii_posllh), out.posllh.lon(ii_posllh) * r2d, 'g', ...
     out.gps.t(ii_gps), out.gps.pt_lon(ii_gps) * r2d, 'm', ...
     out.t(ii), out.antenna_lon(ii) * r2d, 'b');
ylabel('Longitude (deg)');
h(3) = subplot(3, 1, 3);
plot(out.posllh.t(ii_posllh), out.posllh.height(ii_posllh), 'g', ...
     out.gps.t(ii_gps), out.gps.pt_height(ii_gps), 'm', ...
     out.t(ii), out.antenna_height(ii), 'b');
axis tight;
ylabel('Height (m)');
xlabel('Time (sec)');
linkaxes(h, 'x');

% GPS receiver clock.
fig_i = inc_figure(fig_i, 'GPS receiver clock');
h = zeros(2, 1);
h(1) = subplot(2, 1, 1);
plot(out.clock.t(ii_clock), out.clock.bias(ii_clock), 'g', ...
     out.gps.t(ii_gps), out.gps.pt_b_u(ii_gps), 'm', ...
     out.t(ii), out.cb_phase(ii), 'b');
axis tight;
ylabel('Bias (m)');
legend('CLOCK', 'POINT', 'INS');
h(2) = subplot(2, 1, 2);
plot(out.clock.t(ii_clock), out.clock.drift(ii_clock), 'g', ...
     out.gps.t(ii_gps), out.gps.pt_f_u(ii_gps), 'm', ...
     out.t(ii), out.cf_bias(ii) + out.cf_walk(ii), 'b');
axis tight;
ylabel('Drift (m/s)');
xlabel('Time (sec)');
linkaxes(h, 'x');

% GPS satellite angles.
fig_i = inc_figure(fig_i, 'GPS satellite angles');
h = zeros(2, 1);
h(1) = subplot(2, 1, 1);
hold on;
for i = 1:length(out.gps.prns)
  prn = out.gps.prns(i);
  jj = ii_gps(out.gps.pt_azimuth(ii_gps, prn) ~= 0);
  plot(out.gps.t(jj), out.gps.pt_azimuth(jj, prn) * r2d);
end
hold off;
ylabel('Azimuth (deg)');
h(2) = subplot(2, 1, 2);
hold on;
for i = 1:length(out.gps.prns)
  prn = out.gps.prns(i);
  jj = ii_gps(out.gps.pt_elev(ii_gps, prn) ~= 0);
  plot(out.gps.t(jj), out.gps.pt_elev(jj, prn) * r2d);
end
hold off;
ylabel('Elevation (deg)');
xlabel('Time (sec)');
linkaxes(h, 'x');

% GPS satellite path.
fig_i = inc_figure(fig_i, 'GPS satellite path');
hold on;
for i = 1:length(out.gps.prns)
  prn = out.gps.prns(i);
  jj = ii_gps(out.gps.pt_dt_path(ii_gps, prn) ~= 0);
  plot(out.gps.t(jj), out.gps.pt_dt_path(jj, prn));
end
hold off;
ylabel('Path (s)');
xlabel('Time (sec)');

% Specific force.
fig_i = inc_figure(fig_i, 'Specific force');
plot(out.t(ii), out.f_ib_b(ii, :));
axis tight;
ylabel('Specific force (m/s^2)');
xlabel('Time (sec)');
legend('x', 'y', 'z');

% Acceleration.
fig_i = inc_figure(fig_i, 'Body acceleration');
plot(out.t(ii), out.a_ib_b(ii, :));
axis tight;
ylabel('Acceleration (m/s^2)');
xlabel('Time (sec)');
legend('x', 'y', 'z');

% Body velocity.
fig_i = inc_figure(fig_i, 'Body velocity');
plot(out.t(ii), out.v_ib_b(ii, :), ...
     out.resolver.t(ii_resolver), out.resolver.ll_vel(ii_resolver, :), ...
     out.wheel.t(ii_wheel), out.wheel.speed(ii_wheel, :));
axis tight;
ylabel('Velocity (m/s)');
xlabel('Time (sec)');
legend('x', 'y', 'z', 'resolver', ...
       'wheel FL', 'wheel FR', 'wheel RL', 'wheel RR');

% NED velocity.
fig_i = inc_figure(fig_i, 'NED velocity');
plot(out.t(ii), out.v_ib_l(ii, :), ...
     out.velned.t_obs(ii_velned), out.velned.vel(ii_velned, :));
axis tight;
ylabel('Velocity (m/s)');
xlabel('Time (sec)');
legend('x', 'y', 'z', 'GPS x', 'GPS y', 'GPS z');

% ECEF velocity.
fig_i = inc_figure(fig_i, 'ECEF velocity');
plot(out.t(ii), out.v_ib_e(ii, :), ...
     out.gps.t(ii_gps), out.gps.pt_v_u_ecef(ii_gps, :));
axis tight;
ylabel('Velocity (m/s)');
xlabel('Time (sec)');
legend('INS x', 'INS y', 'INS z', 'POINT x', 'POINT y', 'POINT z');

% Quaternion attitude.
fig_i = inc_figure(fig_i, 'Quaternion attitude');
plot(out.t(ii), out.q_eb(ii, :));
axis tight;
ylabel('q\_eb');
xlabel('Time (sec)');

% Euler attitude.
fig_i = inc_figure(fig_i, 'Euler attitude');
plot(out.t(ii), out.euler(ii, :) * r2d);
axis tight;
ylabel('Angle (deg)');
xlabel('Time (sec)');
legend('Roll', 'Pitch', 'Yaw');
