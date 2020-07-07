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

function [x] = compute_crosswind_trajectory(p, t)
% compute_crosswind_trajectory -- Compute the kite's crosswind kinematic state.
%
% x = compute_crosswind_trajectory(p, t)
%
% This module defines the following frames:
% s: Sensor frame; the coordinate system fixed to the sensor. We assume a
%    fixed relationship between the sensor frame and body frame.
% b: Body frame; the coordinate system fixed to the kite origin. The x-axis
%    points forward, the z-axis points down.
% l: Loop frame; the coordinate system rotating about the crosswind virtual
%    hub location. In this coordinate system, the kite has zero velocity and
%    zero acceleration.
% c: Crosswind frame; the coordinate system fixed relative to the wind frame.
% w: Wind frame; the coordinate system level to Earth and aligned to the wind
%    direction.
% n: North-east-down frame; the coordinate system level to Earth with its x-axis
%    aligned with north, and z-axis down.
%
% Arguments:
% p: A structure containing the crosswind trajectory parameters. See
%    crosswind_trajectory_params().
% t: A scalar time [s].
%
% Return values:
% x: A structure of kinematic quantities that describe the kite state.

x = struct();

% Define body to sensor relationships.
x.r_bs_b = p.sensor_offset;
x.dcm_bs = p.dcm_sensor_to_body;
x.dcm_sb = x.dcm_bs';

% Compute loop angle and rotations between each frame.
[theta, omega, alpha] = loop_trajectory(p, t);
dcm_lb = dcm_body_to_loop(p);
dcm_cl = dcm_loop_to_crosswind(theta);
dcm_cb = dcm_cl * dcm_lb;
dcm_wc = dcm_crosswind_to_wind(p);
dcm_wb = dcm_wc * dcm_cb;

% Compute crosswind coordinates to sensor relationships.
x.dcm_cs = dcm_cb * x.dcm_bs;
x.dcm_sc = x.dcm_cs';
x.omega_cs_c = sensor_angular_rate_in_crosswind(p, theta, omega);
x.r_cs_c = sensor_position_in_crosswind(p, x.r_bs_b, theta);
x.v_cs_c = sensor_velocity_in_crosswind(p, x.r_bs_b, theta, omega);
x.a_cs_c = sensor_accel_in_crosswind(p, x.r_bs_b, theta, omega, alpha);

% Compute wind coordinates to sensor relationships.
x.dcm_ws = dcm_wb * x.dcm_bs;
x.dcm_sw = x.dcm_ws';
x.omega_ws_w = sensor_angular_rate_in_wind(p, theta, omega);
x.r_ws_w = sensor_position_in_wind(p, x.r_bs_b, theta);
x.v_ws_w = sensor_velocity_in_wind(p, x.r_bs_b, theta, omega);
x.a_ws_w = sensor_accel_in_wind(p, x.r_bs_b, theta, omega, alpha);

% Compute north-east-down coordinates to sensor relationships.
x.dcm_wn = dcm_ned_to_wind(p);
x.dcm_nw = x.dcm_wn';
x.dcm_ns = x.dcm_nw * x.dcm_ws;
x.dcm_sn = x.dcm_ns';
x.omega_ns_n = x.dcm_nw * x.omega_ws_w;
x.r_ns_n = x.dcm_nw * x.r_ws_w;
x.v_ns_n = x.dcm_nw * x.v_ws_w;
x.a_ns_n = x.dcm_nw * x.a_ws_w;
x.f_ns_s = x.dcm_sn * (x.a_ns_n + p.g_n);
x.field_s = x.dcm_sn * p.field_n;

% Compute crosswind coordinates to body relationships.
r_zero = [0; 0; 0];
x.dcm_cb = dcm_cb;
x.dcm_bc = dcm_cb';
x.omega_cb_b = x.dcm_bc * body_angular_rate_in_crosswind(p, theta, omega);
x.r_cb_c = sensor_position_in_crosswind(p, r_zero, theta);
x.v_cb_c = sensor_velocity_in_crosswind(p, r_zero, theta, omega);
x.a_cb_c = sensor_accel_in_crosswind(p, r_zero, theta, omega, alpha);

% Compute wind coordinates to body relationships.
x.dcm_wb = dcm_wb;
x.dcm_bw = dcm_wb';
x.omega_wb_w = sensor_angular_rate_in_wind(p, theta, omega);
x.r_wb_w = sensor_position_in_wind(p, r_zero, theta);
x.v_wb_w = sensor_velocity_in_wind(p, r_zero, theta, omega);
x.a_wb_w = sensor_accel_in_wind(p, r_zero, theta, omega, alpha);

% Compute north-east-down coordinates to body relationships.
x.dcm_nb = x.dcm_nw * x.dcm_wb;
x.dcm_bn = x.dcm_nb';
x.omega_nb_n = x.dcm_nw * x.omega_wb_w;
x.r_nb_n = x.dcm_nw * x.r_wb_w;
x.v_nb_n = x.dcm_nw * x.v_wb_w;
x.a_nb_n = x.dcm_nw * x.a_wb_w;
end


function [theta, omega, alpha] = loop_trajectory(p, t)
% Compute the loop angle, rate, and acceleration from a quintic trajectory.
t0 = 0;
tf = p.loop_period;
t = mod(t, tf);
% Define the angle at the top of the loop.
theta0 = pi;
% Define the angular rate at the top of the loop (at time t0 and tf). For
% constant angular rate, this parameter must be 2 * pi / period.
omega0 = 2 * pi * p.loop_velocity_scale / tf;

A = [1, t0,   t0^2,     t0^3,      t0^4,      t0^5;
     0,  1, 2 * t0, 3 * t0^2,  4 * t0^3,  5 * t0^4;
     0,  0,      2,   6 * t0, 12 * t0^2, 20 * t0^3;
     1, tf,   tf^2,     tf^3,      tf^4,      tf^5;
     0,  1, 2 * tf, 3 * tf^2,  4 * tf^3,  5 * tf^4;
     0,  0,      2,   6 * tf, 12 * tf^2, 20 * tf^3];
a = flipud(inv(A) * [theta0; omega0; 0; theta0 + 2 * pi; omega0; 0]);
theta = polyval(a, t);

da = polyder(a);
omega = polyval(da, t);

dda = polyder(da);
alpha = polyval(dda, t);
end


function [r_ls_l] = sensor_position_in_loop(p, r_bs_b)
% r_ls_l = r_lb_l + dcm_lb * r_bs_b
r_lb_l = body_position_in_loop(p);
dcm_lb = dcm_body_to_loop(p);
r_ls_l = r_lb_l + dcm_lb * r_bs_b;
end


function [v_ls_l] = sensor_velocity_in_loop()
% v_ls_l = v_lb_l + dcm_dot_lb * r_bs_b + dcm_lb * v_bs_b
v_lb_l = body_velocity_in_loop();
v_ls_l = v_lb_l;
end


function [a_ls_l] = sensor_accel_in_loop()
% a_ls_l = a_lb_l + dcm_ddot_lb * r_bs_b + 2 * dcm_dot_lb * v_bs_b
%          + dcm_lb * a_bs_b
a_lb_l = body_accel_in_loop();
a_ls_l = a_lb_l;
end


function [r_cs_c] = sensor_position_in_crosswind(p, r_bs_b, theta)
% r_cs_c = r_cl_c + dcm_cl * r_ls_l
dcm_cl = dcm_loop_to_crosswind(theta);
r_ls_l = sensor_position_in_loop(p, r_bs_b);
r_cs_c = dcm_cl * r_ls_l;
end


function [v_cs_c] = sensor_velocity_in_crosswind(p, r_bs_b, theta, omega)
% v_cs_c = v_cl_l + dcm_dot_cl * r_ls_l + dcm_cl * v_ls_l
dcm_cl = dcm_loop_to_crosswind(theta);
dcm_dot_cl = dcm_dot_loop_to_crosswind(theta, omega);
r_ls_l = sensor_position_in_loop(p, r_bs_b);
v_ls_l = sensor_velocity_in_loop();
v_cs_c = dcm_dot_cl * r_ls_l + dcm_cl * v_ls_l;
end


function [a_cs_c] = sensor_accel_in_crosswind(p, r_bs_b, theta, omega, alpha)
% a_cs_c = a_cl_l + dcm_ddot_cl * r_ls_l + 2 * dcm_dot_cl * v_ls_l
%          + dcm_cl * a_ls_l
dcm_cl = dcm_loop_to_crosswind(theta);
dcm_dot_cl = dcm_dot_loop_to_crosswind(theta, omega);
dcm_ddot_cl = dcm_ddot_loop_to_crosswind(theta, omega, alpha);
r_ls_l = sensor_position_in_loop(p, r_bs_b);
v_ls_l = sensor_velocity_in_loop();
a_ls_l = sensor_accel_in_loop();
a_cs_c = dcm_ddot_cl * r_ls_l + 2 * dcm_dot_cl * v_ls_l + dcm_cl * a_ls_l;
end


function [r_ws_w] = sensor_position_in_wind(p, r_bs_b, theta)
% r_ws_w = r_wc_w + dcm_wc * r_cs_c
r_wc_w = crosswind_position_in_wind(p);
r_cs_c = sensor_position_in_crosswind(p, r_bs_b, theta);
dcm_wc = dcm_crosswind_to_wind(p);
r_ws_w = r_wc_w + dcm_wc * r_cs_c;
end


function [v_ws_w] = sensor_velocity_in_wind(p, r_bs_b, theta, omega)
% v_ws_w = v_wc_w + dcm_dot_wc * r_cs_c + dcm_wc * v_cs_c
v_cs_c = sensor_velocity_in_crosswind(p, r_bs_b, theta, omega);
dcm_wc = dcm_crosswind_to_wind(p);
v_ws_w = dcm_wc * v_cs_c;
end


function [a_ws_w] = sensor_accel_in_wind(p, r_bs_b, theta, omega, alpha)
% a_ws_w = a_wc_w + dcm_ddot_wc * r_cs_c + 2 * dcm_dot_wc * v_cs_c
%          + dcm_wc * a_cs_c
a_cs_c = sensor_accel_in_crosswind(p, r_bs_b, theta, omega, alpha);
dcm_wc = dcm_crosswind_to_wind(p);
a_ws_w = dcm_wc * a_cs_c;
end


function [r_lb_l] = body_position_in_loop(p)
r_lb_l = [0; 0; p.loop_radius];
end


function [v_lb_l] = body_velocity_in_loop()
v_lb_l = zeros(3, 1);
end


function [a_lb_l] = body_accel_in_loop()
a_lb_l = zeros(3, 1);
end


function [r_wc_w] = crosswind_position_in_wind(p)
dcm_wc = dcm_crosswind_to_wind(p);
r_wc_c = -[p.tether_length; 0; 0];
r_wc_w = dcm_wc * r_wc_c;
end


function [omega_ws_w] = sensor_angular_rate_in_wind(p, theta, omega)
dcm_wc = dcm_crosswind_to_wind(p);
omega_cs_c = sensor_angular_rate_in_crosswind(p, theta, omega);
omega_ws_w = dcm_wc * omega_cs_c;
end


function [omega_cs_c] = sensor_angular_rate_in_crosswind(p, theta, omega)
omega_cb_c = body_angular_rate_in_crosswind(p, theta, omega);
omega_cs_c = omega_cb_c;
end


function [omega_cb_c] = body_angular_rate_in_crosswind(p, theta, omega)
dcm_cl = dcm_loop_to_crosswind(theta);
dcm_lb = dcm_body_to_loop(p);
omega_cl_l = loop_angular_rate_in_loop(theta, omega);
omega_lb_b = body_angular_rate_in_body();
omega_cb_c = dcm_cl * (omega_cl_l + dcm_lb * omega_lb_b);
end


function [omega_cl_l] = loop_angular_rate_in_loop(theta, omega)
dcm = dcm_loop_to_crosswind(theta);
dcm_dot = dcm_dot_loop_to_crosswind(theta, omega);
omega_x = dcm' * dcm_dot;
omega_cl_l = [(omega_x(3, 2) - omega_x(2, 3)) / 2,
              (omega_x(1, 3) - omega_x(3, 1)) / 2,
              (omega_x(2, 1) - omega_x(1, 2)) / 2];
end


function [omega_lb_b] = body_angular_rate_in_body()
omega_lb_b = zeros(3, 1);
end


function [dcm_lb] = dcm_body_to_loop(p)
dcm_lb = dcm_loop_to_body(p)';
end


function [dcm_wc] = dcm_crosswind_to_wind(p)
dcm_wc = dcm_wind_to_crosswind(p)';
end


function [dcm_cl] = dcm_loop_to_crosswind(theta)
dcm_cl = dcm_crosswind_to_loop(theta)';
end


function [dcm_dot_cl] = dcm_dot_loop_to_crosswind(theta, omega)
dcm_dot_cl = dcm_dot_crosswind_to_loop(theta, omega)';
end


function [dcm_ddot_cl] = dcm_ddot_loop_to_crosswind(theta, omega, alpha)
dcm_ddot_cl = dcm_ddot_crosswind_to_loop(theta, omega, alpha)';
end


function [dcm_wn] = dcm_ned_to_wind(p)
dcm_wn = dcm_about_z(p.wind_direction) * dcm_about_x(pi);
end


function [dcm_cw] = dcm_wind_to_crosswind(p)
tether_inc = asin(p.hub_height / p.tether_length);
dcm_cw = dcm_about_x(pi) * dcm_about_z(pi) * dcm_about_y(-tether_inc);
end


function [dcm_lc] = dcm_crosswind_to_loop(theta)
dcm_lc = dcm_about_x(-theta);
end


function [dcm_dot_lc] = dcm_dot_crosswind_to_loop(theta, omega)
dcm_dot_lc = dcm_dot_about_x(-theta, -omega);
end


function [dcm_ddot_lc] = dcm_ddot_crosswind_to_loop(theta, omega, alpha)
dcm_ddot_lc = dcm_ddot_about_x(-theta, -omega, -alpha);
end


function [dcm_bl] = dcm_loop_to_body(p)
dcm_bl = dcm_about_y(-p.kite_alpha) * dcm_about_x(pi/2) * dcm_about_z(pi/2);
end
