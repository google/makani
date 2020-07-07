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

function [] = draw_crosswind_trajectory(traj)
% draw_crosswind_trajectory -- Create a 3D plot of the kinematic components.
%
% draw_crosswind_trajectory(traj)

hold on;
draw_tether(traj);
draw_body_position(traj);
draw_body_velocity(traj);
draw_sensor_accel(traj);
draw_specific_force(traj);
hold off;
grid on;
title('Crosswind trajectory');
xlabel('Wind X position (m)');
ylabel('Wind Y position (m)');
zlabel('Wind Z position (m)');
end


function [] = draw_tether(traj)
r_wb_w = traj.r_wb_w;
plot3([0; r_wb_w(1)], [0; r_wb_w(2)], [0; r_wb_w(3)], 'Color', [0.5, 0.5, 0.5]);
plot3([0; r_wb_w(1)], [0; r_wb_w(2)], [0; r_wb_w(3)], 'ko', 'MarkerSize', 10);
end


function [] = draw_body_position(traj)
r_bx_b = [10; 0; 0];
r_by_b = [0; 10; 0];
r_bz_b = [0; 0; 10];

r_wb_w = traj.r_wb_w;
dcm_wb = traj.dcm_wb;

r_wx_w = r_wb_w + dcm_wb * r_bx_b;
r_wy_w = r_wb_w + dcm_wb * r_by_b;
r_wz_w = r_wb_w + dcm_wb * r_bz_b;

plot3([r_wb_w(1); r_wx_w(1)], ...
      [r_wb_w(2); r_wx_w(2)], ...
      [r_wb_w(3); r_wx_w(3)], 'Color', 'b', 'LineWidth', 2);
plot3([r_wb_w(1); r_wy_w(1)], ...
      [r_wb_w(2); r_wy_w(2)], ...
      [r_wb_w(3); r_wy_w(3)], 'Color', 'g', 'LineWidth', 2);
plot3([r_wb_w(1); r_wz_w(1)], ...
      [r_wb_w(2); r_wz_w(2)], ...
      [r_wb_w(3); r_wz_w(3)], 'Color', 'r', 'LineWidth', 2);
end


function [] = draw_body_velocity(traj)
r_wb_w = traj.r_wb_w;
v_wb_w = traj.v_wb_w;
plot3([r_wb_w(1); r_wb_w(1) + v_wb_w(1)], ...
      [r_wb_w(2); r_wb_w(2) + v_wb_w(2)], ...
      [r_wb_w(3); r_wb_w(3) + v_wb_w(3)], 'Color', 'k');
end


function [] = draw_sensor_accel(traj)
r_ws_w = traj.r_ws_w;
a_ws_w = traj.a_ws_w;
plot3([r_ws_w(1); r_ws_w(1) + a_ws_w(1)], ...
      [r_ws_w(2); r_ws_w(2) + a_ws_w(2)], ...
      [r_ws_w(3); r_ws_w(3) + a_ws_w(3)], 'Color', 'r');
end


function [] = draw_specific_force(traj)
r_ws_w = traj.r_ws_w;
f_ws_w = traj.dcm_ws * traj.f_ns_s;
plot3([r_ws_w(1); r_ws_w(1) + f_ws_w(1)], ...
      [r_ws_w(2); r_ws_w(2) + f_ws_w(2)], ...
      [r_ws_w(3); r_ws_w(3) + f_ws_w(3)], 'Color', 'm');
end
