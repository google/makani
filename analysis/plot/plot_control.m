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

function ax = plot_control(plot_names, data, t)
% plot_control -- Makes a standard set of plots from the controller data.
%
% ax = plot_control(plot_names, data, t)
% Makes predefined plots labeled by various keywords (e.g. 'crit', 'gps',
% etc...).  This is intended to be called by plot_telem, which is the general
% plotting routine.  Also, links axes of plots.
%
% plot_names: Cell array of strings describing what to plot.
%
% Example:
%   data = load_h5log('20130307-143445.log');
%   plot_control({'fm', 'pos', 'tension'}, data);

make_naming_shortcuts(data);
lasterror('reset');
ax = [];
if isempty(C)
  return
end
if nargin < 3
  t = C.message.time;
end

% Some handy helper functions for plotting.
hline = @(y) plot(xlim, [y, y], '--', 'Color', [0.7, 0.7, 0.7]);

try
  if contains({'fm', 'crit', 'faults', 'control', 'all'}, plot_names)
    mfig('c: Flight mode', 'Time [s]', 'Flight Mode [#]');
    plot(t, c.flight_mode);
    set(gca, 'YTick', 0:14, ...
        'YTickLabel', {'Pilot hover', 'Perched', 'Hover ascend', ...
                       'Hover pay-out', 'Hover full length', ...
                       'Hover transform GS up', 'Hover accel', ...
                       'Hover trans-in', 'Crosswind normal', ...
                       'Crosswind prep-trans-out', 'Hover trans-out', ...
                       'Hover transform GS down', 'Hover reel-in', ...
                       'Hover perch descend', 'Manual on tether', ...
                       'Off tether'});

    % Escape the underscores so that they don't get TeXified.
    set(gca, 'YTickLabel', ...
        cellfun(@(s) strrep(s, '_', '\_'), get(gca, 'YTickLabel'), ...
                'UniformOutput', false));
  end
end

try
  if contains({'attitude', 'imu', 'redundant_imu', 'all'}, plot_names)
    mfig('c: Attitude Estimate Differences', 'Time [s]', ...
         'Rotation Angle [deg]');
    q = s2m(c.estimator.q);
    q_hat = @(i) [c.estimator.q_estimates.q0(i, :)
                  c.estimator.q_estimates.q1(i, :)
                  c.estimator.q_estimates.q2(i, :)
                  c.estimator.q_estimates.q3(i, :)];
    angle_diff = @(q_a, q_b) (180/pi)*acos(min(2*dot(q_a, q_b).^2 - 1, 1.0));
    plot(t, angle_diff(q, q_hat(1)), ...
         t, angle_diff(q, q_hat(2)), ...
         t, angle_diff(q, q_hat(3)));
    legend('q_a', 'q_b', 'q_c');
  end
end

try
  if contains({'imu_acc', 'imu', 'redundant_imu', 'all'}, plot_names)
    mfig('c: Acc_b', 'Time [s]', 'Acc. [m/s^2]');
    plot_vec_of_signals(t, c.estimator.acc_b_estimates, 'a', true);
  end
end

try
  if contains({'pos', 'crit', 'faults', 'control', 'all'}, plot_names)
    mfig('c: Position comparison', 'Time [s]', 'Position [m]');
    sigma_glas = s2m(c.estimator.sigma_Xg_glas);
    plot(t, s2m(c.estimator.Xg_gps));
    plot(t, s2m(c.estimator.Xg_glas), '-.');
    plot(t, s2m(c.state_est.Xg), '--');
    plot(t, c.estimator.Xg_press.z, 'r:');
    plot(t, c.state_est.tether_force_b.sph.tension/100, 'k');
    plot(t, s2m(c.estimator.Xg_glas) - sigma_glas, ':');
    plot(t, s2m(c.estimator.Xg_glas) + sigma_glas, ':');
    legend('GPS x', 'GPS y', 'GPS z', 'GLAS x', 'GLAS y', 'GLAS z', ...
           'EST x', 'EST y', 'EST z', 'PRESS z', 'tension/100');
    ylim(1.5 * system_params.tether.length * [-1, 1]);
  end
end

try
  if contains({'pos', 'control', 'all'}, plot_names)
    mfig('c: GLAS position uncertainty', 'Time [s]', 'Position [m]');
    plot(t, s2m(c.estimator.delta_Xg_sensors), ':');
    plot(t, s2m(c.estimator.delta_Xg_perch_dyn), '--');
    plot(t, s2m(c.estimator.delta_Xg_tether_dyn), '-.');
    plot(t, s2m(c.estimator.sigma_Xg_glas));
    legend('delta Xg sensors', '', '', ...
           'delta Xg perch dynamics', '', '', ...
           'delta Xg tether dynamics', '', '', ...
           'sigma Xg GLAS', '', '');
  end
end

try
  if contains({'levelwind_azi', 'control', 'all'}, plot_names)
    mfig('c: levelwind azimuth error', 'Time [s]', 'Azimuth [rad]', '');
    azi_gps = atan2(-c.estimator.Xg_gps.y, -c.estimator.Xg_gps.x);
    azi_glas = atan2(-c.estimator.Xg_glas.y, -c.estimator.Xg_glas.x);
    plot(t, azi_gps, t, azi_glas, t, azi_gps - azi_glas, ...
         t, double(c.flight_mode) / 10);
    legend('GPS', 'GLAS', 'GPS - GLAS');
  end
end

try
  if contains({'gsg', 'control', 'all'}, plot_names)
    mfig('c: GSG angles', 'Time [s]', 'Angles [rad]');
    plot(t, s2m(c.control_input.gsg));
    plot(t, s2m(c.state_est.gsg_bias), '--');
    plot(t, c.control_input.perch.perch_azi, 'c');
    plot(t, c.control_input.gsg.azi + c.control_input.perch.perch_azi, 'k');
    legend('GSG azi', 'GSG ele', 'GSG twist', ...
           'GSG bias azi', 'GSG bias ele', 'GSG bias twist', ...
           'perch azi', 'perch+GSG azi');
  end
end

try
  if contains({'vel', 'crit', 'faults', 'control', 'all'}, plot_names)
    mfig('c: Vg comparison', 'Time [s]', 'Velocity [m/s]');
    plot(t, s2m(c.estimator.Vg_gps));
    plot(t, s2m(c.estimator.Vg_glas), '-.');
    plot(t, s2m(c.state_est.Vg), '--');
    plot(t, sqrt(c.estimator.Vg_gps.x.^2 ...
                 + c.estimator.Vg_gps.y.^2 ...
                 + c.estimator.Vg_gps.z.^2), 'k');
    plot(t, sqrt(c.estimator.Vg_glas.x.^2 ...
                 + c.estimator.Vg_glas.y.^2 ...
                 + c.estimator.Vg_glas.z.^2), 'k:');
    plot(t, sqrt(c.state_est.Vg.x.^2 ...
                 + c.state_est.Vg.y.^2 ...
                 + c.state_est.Vg.z.^2), 'k--');
    legend('GPS v_x', 'GPS v_y', 'GPS v_z', ...
           'GLAS v_x', 'GLAS v_y', 'GLAS v_z', ...
           'EST v_x', 'EST v_y', 'EST v_z', ...
           'GPS |v|', 'GLAS |v|', 'EST |v|');
    ylim([-100, 100]);
  end
end

try
  if contains({'vel', 'gps', 'control', 'all'}, plot_names)
    mfig('c: Vg simple', 'Time [s]', 'Velocity [m/s]');
    plot(t, s2m(c.estimator.Vg_gps));
    plot(t, s2m(c.state_est.Vg), '--');
    legend('GPS v_x', 'GPS v_y', 'GPS v_z', 'EST v_x', 'EST v_y', 'EST v_z');
  end
end

try
  if contains({'gpsinfo', 'gps', 'crit', 'faults', 'control', 'all'}, ...
              plot_names)
    mfig('c: GPS info', 'Time [s]', 'GPS status/sigmas');
    plot(t, c.control_input.gps.pos_sol_stat, 'k');
    plot(t, c.control_input.gps.vel_sol_stat, 'k');
    ylim([-1, 8]);
    legend('pos status', 'vel status');
  end
end

try
  if contains({'gpssigmas', 'control', 'all'}, plot_names)
    mfig('c: GPS sigmas', 'Time [s]', 'GPS Xg/Vg sigmas');
    plot(t, s2m(c.control_input.gps.pos_sigma));
    plot(t, s2m(c.control_input.gps.vel_sigma), '--');
    legend('\sigma_x', '\sigma_y', '\sigma_z', ...
           '\sigma_v_x', '\sigma_v_y', '\sigma_v_z');
  end
end

try
  if contains({'cutoff', 'poscutoff', 'control', 'all'}, plot_names)
    mfig('c: Estimator position cutoff frequencies');
    plot(t, s2m(c.estimator.glas_position_fcs));
    plot(t, s2m(c.estimator.gps_position_fcs), '--');
    ylim([-5, 5]);
    legend('fx_{glas}', '', '', 'fx_{gps}', '', '');
  end
end

try
  if contains({'cutoff', 'velcutoff', 'control', 'all'}, plot_names)
    mfig('c: Estimator velocity cutoff frequencies');
    plot(t, s2m(c.estimator.glas_position_fcs));
    plot(t, s2m(c.estimator.gps_position_fcs), '--');
    ylim([-5, 5]);
    legend('fv_{glas}', '', '', 'fv_{gps}', '', '');
  end
end

try
  if contains({'pos', 'bias', 'control', 'all'}, plot_names)
    mfig('c: Position bias', 'Time [s]', '[m]');
    plot(t, s2m(c.estimator.pos_vel_bias.Xg_gps));
    plot(t, sqrt(c.estimator.pos_vel_bias.P_zg_gps_lw), 'r--');
    plot(t, c.estimator.pos_vel_bias.press, 'c');
    plot(t, sqrt(c.estimator.pos_vel_bias.P_press), 'c--');
    legend('bias.Xg_gps', '', '', 'bias.sig\_zg\_gps\_lw', ...
           'bias.press', 'bias.sig\_press')
  end
end

try
  if contains({'pos', 'bias', 'control', 'all'}, plot_names)
    mfig('c: GLAS bias', 'Time [s]', '[rad]');
    plot(t, c.estimator.pos_vel_bias.gsg.azi, ...
         t, c.estimator.pos_vel_bias.gsg.ele);
    legend('GSG azimuth', 'GSG elevation')
  end
end

try
  if contains({'anglesspeed', 'crit', 'faults', 'control', 'all'}, plot_names)
    mfig('c: Angles/Airspeed', ...
         'Time [s]', 'Speed [m/s], Angles [deg]');
    plot(t, s2m(c.estimator.apparent_wind_pitot) .* ...
         repmat([1; 180/pi; 180/pi], 1, length(t)));
    plot(t, s2m(c.estimator.apparent_wind_est) .*  ...
         repmat([1; 180/pi; 180/pi], 1, length(t)), '--');
    ylim([-30, 60]);
    legend('vel (pitot)', 'alpha (pitot)', 'beta (pitot)', ...
           'vel (est)', 'alpha (est)', 'beta (est)');
  end
end

try
  if contains({'wind', 'crit', 'control', 'all'}, plot_names)
    mfig('c: Wind speed', 'Time [s]', 'Wind speed [m/s]');
    plot(t, s2m(c.state_est.wind_g.vector));
    plot(t, struct_norm(c.state_est.wind_g.vector), 'k');
  end
end

try
  if contains({'loadcells', 'crit', 'control', 'all'}, plot_names)
    mfig('c: Loadcells', 'Time [s]', 'Tension [N]');
    plot(t, c.control_input.loadcells);
    plot(t, c.state_est.tether_force_b.sph.tension, 'k');
    legend('Port', 'Center (aft.)', 'Center (fore.)', 'Star', 'Tension');
  end
end

try
  if contains({'tension', 'crit', 'trans_in', 'trans_out', ...
               'control', 'all'}, plot_names)
    mfig('c: Tension', 'Time [s]', 'Tension [N]');
    plot(t, c.state_est.tether_force_b.sph.tension);
    plot(t, c.state_est.tension_f, ':');
    plot(t, c.hover.T0_ff, '--');
    legend('Unfiltered', 'Filtered', 'hover T0\_ff');
  end
end

try
  if contains({'flaps', 'crit', 'control', 'hover', 'all'}, plot_names)
    mfig('c: Flaps', 'Time [s]', 'Deflection [rad]');
    plot(t, c.control_output.flaps([1, 3, 5, 7, 8], :));
    if contains({'hover'}, plot_names);
      plot(t, c.cont_h.flaps(1:5, :), '-.');
    else
      ylim([-0.6, 0.6]);
    end
    legend('Port Ail', 'Center Flap', 'Star Ail', 'Ele', 'Rud', ...
           'location', 'northwest');
  end
end

try
  if contains({'motors', 'rotors', 'crit', 'control', 'hover', 'all'}, ...
              plot_names)
    mfig('c: Rotors', 'Time [s]', 'Rotor ang. rate [rad/s]');
    plot(t, c.control_output.rotors);
    plot(t, c.control_input.rotors, '--');
    legend('SBO cmd', 'SBI cmd', 'PBI cmd', 'PBO cmd', ...
           'PTO cmd', 'PTI cmd', 'STI cmd', 'STO cmd', ...
           'SBO', 'SBI', 'PBI', 'PBO', 'PTO', 'PTI', 'STI', 'STO', ...
           'location', 'northwest');
  end
end

try
  if contains({'joystick', 'crit', 'control', 'all'}, plot_names)
    mfig('c: Pilot', 'Time [s]', 'Stick position [#]');
    plot(t, [c.control_input.joystick.throttle, ...
             c.control_input.joystick.roll, ...
             c.control_input.joystick.pitch, ...
             c.control_input.joystick.yaw, ...
             double(c.control_input.joystick.switches)/16]);
    legend('Thrust', 'Roll', 'Pitch', 'Yaw', 'Switches/16', ...
           'location', 'northwest');
  end
end

try
  if contains({'imu_acc', 'imu', 'crit', 'control', 'all'}, plot_names)
    mfig('c: Acc', 'Time [s]', 'Acc. [m/s^2]');
    plot_vec_of_signals(t, c.control_input.imus.acc, 'a', true);
  end
end

try
  if contains({'imu_mag', 'imu', 'crit', 'control', 'all'}, plot_names)
    mfig('c: Mag', 'Time [s]', 'Magnetometer [G]');
    plot_vec_of_signals(t, c.control_input.imus.mag, 'm');
  end
end

try
  if contains({'imu_gyro', 'imu', 'crit', 'control', 'all'}, plot_names)
    mfig('c: Gyro', 'Time [s]', 'Ang. rate [rad/s]');
    plot_vec_of_signals(t, c.control_input.imus.gyro, '\omega');
  end
end

try
  if contains({'loop_angle', 'control', 'all'}, plot_names)
    mfig('c: Loop angle', 'Time [s]', 'Loop angle [rad]');
    plot(t, c.crosswind.loop_ang);
  end
end

try
  if contains({'Jall', 'J', 'control', 'all'}, plot_names)
    mfig('c: Advance ratio (comm + diff)', 'Time [s]', 'J [#]');
    n = length(t);
    rotor_pos = s2m(system_params.rotors.pos);

    n = length(c.time);
    dV_sb = cross(s2m(c.state_est.pqr), repmat(rotor_pos(:, 1), 1, n))';
    dV_pb = cross(s2m(c.state_est.pqr), repmat(rotor_pos(:, 2), 1, n))';
    dV_pt = cross(s2m(c.state_est.pqr), repmat(rotor_pos(:, 3), 1, n))';
    dV_st = cross(s2m(c.state_est.pqr), repmat(rotor_pos(:, 4), 1, n))';

    vx = -c.estimator.apparent_wind_vector.x;

    V_sb = (vx + dV_sb(:, 1)) * sqrt(0.92);
    V_pb = (vx + dV_pb(:, 1)) * sqrt(0.92);
    V_pt = (vx + dV_pt(:, 1)) * sqrt(1.08);
    V_st = (vx + dV_st(:, 1)) * sqrt(1.08);

    J = [V_sb, V_pb, V_pt, V_st] ...
        ./ (system_params.rotors.D(1) * c.control_output.rotors / (2*pi));
    plot(t, control_params.rotor_control.simple_models.J_neutral(1) ...
         * ones(size(t)), 'k--');
    plot(t, J);
    legend('J_{neutral}', 'J (SB)', 'J (PB)', 'J (PT)', 'J (ST)');
  end
end

try
  if contains({'crosswind', 'curvature', 'control', 'all'}, plot_names)
    mfig('c: Curvatures', 'Time [s]', 'Curvatures [1/m]');
    plot(t, [c.crosswind.k_geom_curr, c.crosswind.k_aero_curr, ...
             c.crosswind.k_geom_cmd, c.crosswind.k_aero_cmd]);
    ylim([-0.1, 0.1]);
    legend('k_{geom}_{curr}', 'k_{aero}_{curr}', ...
           'k_{geom}_{cmd}', 'k_{aero}_{cmd}', ...
           'location', 'northwest');
  end
end

try
  if contains({'tether_roll', 'crit', 'control', 'all'}, plot_names)
    mfig('c: Tether roll', 'Time [s]', 'Tether roll angle [rad]');
    plot(t, c.crosswind.tether_roll_cmd, '-');
    plot(t, c.state_est.tether_force_b.sph.roll, ':');
    plot(t, c.crosswind.int_aileron, 'g-');
    plot(t, c.state_est.tether_force_b.sph.pitch, 'r');
    legend('cmd', 'meas', 'int. err', 'tether pitch', ...
           'location', 'northwest');
    ylim([-1, 1]);
  end
end

try
  if contains({'fp', 'control', 'all'}, plot_names)
    mfig('c: Flight path', '', 'y [m]', 'z [m]');
    plot_fp(t, -c.crosswind.X_fp_curr.x, -c.crosswind.X_fp_curr.y, 'b', ...
            control_params.crosswind.path.r0);
  end
end

try
  if contains({'fp_tension', 'control', 'all'}, plot_names)
    mfig('c: Flight path (tension)', '', 'y [m]', 'z [m]');
    plot_fp(t, -c.crosswind.X_fp_curr.x, -c.crosswind.X_fp_curr.y, ...
            c.state_est.tether_force_b.sph.tension);
  end
end

try
  if contains({'fp_Xg', 'control', 'all'}, plot_names)
    mfig('c: Flight path (Xg comp)', '', 'y [m]', 'z [m]');
    Xg_0 = s2m(c.crosswind.Xg_path_center);
    Xg_glas = s2m(c.estimator.Xg_glas);
    Xg_gps = s2m(c.estimator.Xg_gps);
    Xg = s2m(c.state_est.Xg);
    Xg_0_norm = sqrt(Xg_0(1, :).^2 + Xg_0(2, :).^2 + Xg_0(3, :).^2);
    X_fp_glas = zeros(3, length(t));
    X_fp_gps = zeros(3, length(t));
    X_fp = zeros(3, length(t));
    for k = 1:length(t)
      ex = -Xg_0(:, k) / Xg_0_norm(k);
      ey = cross([0, 0, 1]', ex) / norm(cross([0, 0, 1]', ex));
      ez = cross(ex, ey);
      dcm_g2fp = [ex, ey, ez]';
      X_fp_glas(:, k) = dcm_g2fp * Xg_glas(:, k);
      X_fp_gps(:, k) = dcm_g2fp * Xg_gps(:, k);
      X_fp(:, k) = dcm_g2fp * Xg(:, k);
    end
    plot_fp(t, -X_fp(2, :), -X_fp(3, :), 'b');
    plot_fp(t, -X_fp_gps(2, :), -X_fp_gps(3, :), 'g');
    plot_fp(t, -X_fp_glas(2, :), -X_fp_glas(3, :), 'r');
    legend('est', '', '', 'gps', '', '', 'glas', '', '');
  end
end

try
  if contains({'fp_Vg', 'control', 'all'}, plot_names)
    mfig('c: Flight path (Vg comp)', '', 'y [m]', 'z [m]');
    Xg_0 = s2m(c.crosswind.Xg_path_center);
    Vg_glas = s2m(c.estimator.Vg_glas);
    Vg_gps = s2m(c.estimator.Vg_gps);
    Vg = s2m(c.state_est.Vg);
    Xg_0_norm = sqrt(Xg_0(1, :).^2 + Xg_0(2, :).^2 + Xg_0(3, :).^2);
    V_fp_glas = zeros(3, length(t));
    V_fp_gps = zeros(3, length(t));
    V_fp = zeros(3, length(t));
    for k = 1:length(t)
      ex = -Xg_0(:, k) / Xg_0_norm(k);
      ey = cross([0, 0, 1]', ex) / norm(cross([0, 0, 1]', ex));
      ez = cross(ex, ey);
      dcm_g2fp = [ex, ey, ez]';
      V_fp_glas(:, k) = dcm_g2fp * Vg_glas(:, k);
      V_fp_gps(:, k) = dcm_g2fp * Vg_gps(:, k);
      V_fp(:, k) = dcm_g2fp * Vg(:, k);
    end
    plot_fp(t, -V_fp(2, :), -V_fp(3, :), 'b');
    plot_fp(t, -V_fp_gps(2, :), -V_fp_gps(3, :), 'g');
    plot_fp(t, -V_fp_glas(2, :), -V_fp_glas(3, :), 'r');
    legend('est', '', '', 'gps', '', '', 'glas', '', '');
  end
end

try
  if contains({'accel_start', 'hover', 'control', 'all'}, plot_names)
    mfig('c: Accel. start position', 'Time [s]', 'Position [m]');
    plot(t, s2m(c.state_est.Xg));
    plot(t, s2m(c.hover.accel_start_g), '--');
    legend('x_g', 'y_g', 'z_g', ...
           'x_g (accel start)', 'y_g (accel start)', 'z_g (accel start)');
  end
end

try
  if contains({'pqr', 'hover_ang', 'control', 'all'}, plot_names)
    mfig('c: pqr', [], '[rad/s]');
    plot(t, s2m(c.state_est.pqr));
    plot(t, double(c.flight_mode)/10);
    legend('p', 'q', 'r', 'flight\_mode/10');
  end
end

try
  if contains({'hover', 'control', 'all'}, plot_names)
    mfig('c: cx_cy_cz', [], '[#]');
    plot(t, s2m(c.hover.thrust_moment_scale.moment));
    legend('c\_x', 'c\_y', 'c\_z');
  end
end

try
  if contains({'hover_ang', 'hover', 'control', 'all'}, plot_names)
    mfig('c: hover pitch', [], '[rad]');

    hover_params = control_params.hover;
    nXg = struct_norm(c.state_est.Xg);
    pit_min = crossfade(hover_params.feed_forward.pit_min_short_tether, ...
                        hover_params.feed_forward.pit_min_long_tether, ...
                        nXg, ...
                        hover_params.feed_forward.short_tether, ...
                        hover_params.feed_forward.long_tether);
    pit_max = crossfade(hover_params.feed_forward.pit_max_short_tether, ...
                        hover_params.feed_forward.pit_max_long_tether, ...
                        nXg, ...
                        hover_params.feed_forward.short_tether, ...
                        hover_params.feed_forward.long_tether);

    plot(c.time, c.hover.pit_ff, ...
         c.time, c.hover.pit_int, ...
         c.time, c.hover.hover_ang_ff.y, ...
         c.time, c.hover.hover_ang_cmd.y, ...
         c.time, c.hover.hover_ang.y);
    plot(c.time, [pit_min, pit_max], 'r:');
    legend('pit ff', 'pit int', 'hover ang ff y', 'hover ang req y', ...
           'hover ang y', 'pit min', 'pit max');
  end
end

try
  if contains({'hover', 'hover_ang', 'control', 'all'}, plot_names)
    mfig('c: Hover angles', [], '[rad]');
    plot(t, s2m(c.hover.hover_ang));
    plot(t, s2m(c.hover.hover_ang_ff), '-.');
    plot(t, s2m(c.hover.hover_ang_cmd), '--');
    plot(t, c.hover.pit_int, 'g--');
    legend('rol', 'pit', 'yaw', 'rol ff', 'pit ff', 'yaw ff', ...
           'rol req', 'pit req', 'yaw req', 'pit\_int');
  end
end

try
  if contains({'hover', 'hover_ang', 'control', 'all'}, plot_names)
    mfig('c: Hover U', [], '[N-m]');
    plot(t, s2m(c.hover.U.torq));
    plot(t, s2m(c.hover.int_ang_err), '--');
    plot(t, c.hover.U.thrust, 'k');

    legend('U.torq', '', '', 'tau\_int', '', '', 'U.thrust');
  end
end

try
  if contains({'hover', 'control', 'all'}, plot_names)
    mfig('c: thrust', [], '[N]');
    plot(t, c.hover.U.thrust);
    plot(t, sum(c.hover.thrusts), 'Linewidth', 2);
    ylabel('thrust');
  end
end

try
  if contains({'hover', 'hover_pos', 'perch', 'control', 'all'}, plot_names)
    mfig('c: Hover Pos');
    plot(t, s2m(c.state_est.Xg));
    plot(t, s2m(c.hover.wing_pos_g_cmd), '--');
    plot(t, struct_norm(c.state_est.Xg), 'k');
    plot(t, c.state_est.winch.payout ...
            + struct_norm(control_params.hover.path.perched_wing_pos_p), ...
         'k');
    plot(c.time, -struct_norm(c.state_est.Xg).*sin(c.hover.ele_ff), 'r:');

    legend('Xg', '', '', 'Xg\_cmd', '', '', '|Xg|', ...
           'payout + |perched_wing_pos_p|', '|Xg|*sin(ele\_ff)');
  end
end

try
  if contains({'hover', 'perch', 'altitude', 'proximity', ...
               'control', 'all'}, plot_names)
    mfig('c: Perch Zg');

    % 11=reel_in, 3=pay_out.
    I = (c.flight_mode == 11 ...
         | c.flight_mode == 12 ...
         | c.flight_mode == 3 ...
         | c.flight_mode == 2);

    xy_gps = sqrt(c.estimator.Xg_gps.x(I).^2 + c.estimator.Xg_gps.y(I).^2);
    z_gps_levelwind = -xy_gps.*tan(c.control_input.perch.levelwind_ele(I));
    h_gps = c.estimator.Xg_gps.z(I);

    msub(2, 1, 1, []);
    plot(t, double(c.flight_mode)/10);
    plot(t, bitget_array(c.control_input.perch.proximity_flag, [1, 2]));
    legend('flight\_mode/10', 'early prox', 'final prox');
    ylim([-0.1, 2.0]);

    msub(2, 1, 2, []);
    plot(t, c.estimator.Xg_press.z, 'Color', [0.8, 1, 1]);
    plot(t, c.estimator.Xg_glas.z, 'k');
    plot(t, c.estimator.Xg_gps.z, 'b');
    plot(t, c.state_est.Xg.z, 'b');
    plot(t, c.hover.wing_pos_g_cmd.z, 'm--');

    leg_strs = {'zg\_press', 'zg\_levelwind', 'zg\_gps', 'zg', ...
                'h\_zg'};
    if any(I)
      leg_strs{end + 1} = 'h\_zg\_cmd';
      plot(t(I), z_gps_levelwind, 'r');
    end
    legend(leg_strs);
    ylabel('zg [m]');
  end
end

try
  if contains({'hover', 'perch', 'levelwind', 'altitude', ...
               'control', 'all'}, plot_names)
    mfig('c: Perch Altitude', 'sqrt(x^2+y^2)', 'alt', '', false);

    % 11=reel_in, 3=pay_out.
    I = (c.flight_mode == 11 ...
         | c.flight_mode == 12 ...
         | c.flight_mode == 3 | c.flight_mode == 3 ...
         | c.flight_mode == 4 | c.flight_mode == 5 ...
         | c.flight_mode == 6 | c.flight_mode == 7);

    vnormxy = @(X, I) sqrt(X.x(I).^2 + X.y(I).^2);
    xy_gps = vnormxy(c.estimator.Xg_gps, I);
    xy_glas = vnormxy(c.estimator.Xg_glas, I);
    xy_Xg = vnormxy(c.state_est.Xg, I);

    h_gps_levelwind = xy_gps .* tan(c.control_input.perch.levelwind_ele(I));
    h_gps = -c.estimator.Xg_gps.z(I);

    if contains('perch', plot_names)
      axis([0, 10, -1, 4]);
    end

    plot(xy_gps, h_gps, ...
         xy_glas, -c.estimator.Xg_glas.z(I), ...
         xy_Xg, -c.state_est.Xg.z(I), ...
         xy_gps, h_gps_levelwind);
    legend('Xg\_gps', 'Xg\_glas', 'Xg', 'r gps, h lw');

    alt_off = -control_params.hover.reel.zg_offset;
    tol = 0.3;  % [m]
    r_max = 10.0;  % [m]
    alt_max = r_max * tan(control_params.hover.reel.launch_perch_ele);

    plot([0, r_max], [alt_off, alt_max], 'Color', [0.5, 0.5, 0.5]);
    plot([0, r_max], [alt_off, alt_max] - tol, 'Color', [0.5, 0.5, 0.5]);
    plot([0, r_max], [alt_off, alt_max] + tol, 'Color', [0.5, 0.5, 0.5]);

    zg_lw = system_params.perch.levelwind_origin_p_0.z ...
            + system_params.levelwind.drum_angle_to_vertical_travel ...
              * -c.control_input.perch.winch_pos(1) ...
              / system_params.winch.r_drum;
    cent = [sqrt(system_params.perch.levelwind_origin_p_0.x^2 ...
                 + system_params.perch.levelwind_origin_p_0.y^2), ...
            -zg_lw];
    radius = system_params.wing.bridle_rad ...
             + system_params.wing.bridle_pos.z(1) ...
             + system_params.levelwind.pivot_axis_to_bridle_point;

    x = -radius:0.001:radius;
    y = sqrt(radius.^2 - x.^2);
    x = [x, fliplr(x)];
    y = [y, -fliplr(y)];
    plot(x + cent(1), y + cent(2), 'Color', [0.5, 0.5, 0.5]);
    plot(cent(1), cent(2), 'x', 'Color', [0.5, 0.5, 0.5]);
  end
end

try
  if contains({'levelwind', 'control', 'all'}, plot_names)
    lw_engage = system_params.levelwind;
    calc_ele = @(X) atan2(-X.z, hypot(X.x, X.y));

    mfig('c: levelwind', '', '[rad]');
    plot(t, c.control_input.gsg.ele, 'b', ...
         t, c.control_input.perch.levelwind_ele, 'c', ...
         t, c.hover.integrated_levelwind_error, 'r--', ...
         t, calc_ele(c.hover.wing_pos_g_cmd), 'm--', ...
         t, calc_ele(c.state_est.Xg), 'r');
    hline(lw_engage.elevation_nominal - lw_engage.elevation_tolerance);
    hline(lw_engage.elevation_nominal);
    hline(lw_engage.elevation_nominal + lw_engage.elevation_tolerance);
    legend('gsg.ele', 'lw.ele', 'hover.ele_r', 'ele from Xg\_cmd', ...
           'ele from Xg');
  end
end

try
  if contains({'trans_out', 'control', 'all'}, plot_names)
    mfig('c: trans_out', '', '', '', false);
    delete(gca);  % Required b/c this is a polar plot.
    rfm = control_params.crosswind.ready_for_mode;

    % First plot made polar sets radius limit (blarg).
    polar(0, 30, 'w'); hold on;

    I_prep = c.flight_mode == 9;
    I_hover = c.flight_mode == 10;
    nVg = struct_norm(c.state_est.Vg);
    polar(c.crosswind.loop_ang(I_prep), nVg(I_prep), 'b');
    polar(c.crosswind.loop_ang(I_hover), nVg(I_hover), 'g');
    set(gca, 'XDir', 'reverse', 'YDir', 'reverse');

    angs = 0:0.01:2*pi;
    Vg_thresh = interp1(rfm.loop_angle_circ_tab, ...
                        rfm.Vg_norm_thresh_circ_tab, angs);
    h = polar(angs, Vg_thresh);
    set(h, 'Color', [0.5, 0.5, 0.5]);
  end
end

try
  if contains({'trans_out', 'control', 'all'}, plot_names)
    calc_ele = @(X) atan2(-X.z, hypot(X.x, X.y));
    mfig('c: elevation', 'Time [s]', 'Elevation [rad]');
    plot(t, calc_ele(c.state_est.Xg));
  end
end

try
  if contains({'trans_out', 'control', 'all'}, plot_names)
    mfig('c: hover elevator', 'Time [s]', 'elevator [rad]');
    plot(c.time, c.hover.delta_ele_ff, c.time, c.hover.delta_ele_fb, ...
         c.time, c.hover.delta_ele_ff + c.hover.delta_ele_fb, ...
         c.time, c.control_output.flaps(7, :));
    legend('de\_ff', 'de\_fb', 'de\_ff + de\_fb', 'de final');
  end
end

try
  if contains({'trans_out', 'control', 'all'}, plot_names)
    mfig('c: Vapp to Vg', 'Time [s]', 'angle [rad]');
    plot(c.time, -c.hover.delta_ele_ff, c.time, ...
         c.state_est.apparent_wind.sph.alpha);
    legend('angle Vapp to Vg', 'alpha');
  end
end

try
  if contains({'perch', 'control', 'all'}, plot_names)
    Vec3Azi = @(X) atan2(X.y, X.x);
    mfig('c: perch azimuth', 'Time [s]', 'Azimuth [rad]');
    plot(t, unwrap(c.control_input.perch.perch_azi) - pi);
    plot(t, unwrap(Vec3Azi(c.state_est.Xg)) + pi);
    plot(t, -c.hover.hover_ang.x);
    % plot(t, unwrap(Vec3Azi(c.state_est.wind_g.vector)) - pi);
    plot(t, c.state_est.wind_dir_f);
    legend('unwrap(perch\_azi)-pi', 'unwrap(azi(Xg))-pi', ...
           '-hover\_roll', 'wind dir');
  end
end

try
  if contains({'perch', 'winch', 'control', 'all'}, plot_names)
    mfig('c: Payout', 'Time [s]', 'Payout [m]');
    plot(t, [c.state_est.winch.payout, struct_norm(c.state_est.Xg), ...
             c.control_input.perch.winch_pos]);
    legend('payout', '|Xg|', 'winch pos');
  end
end

try
  if contains({'wind', 'control', 'all'}, plot_names)
    mfig('c: wind parallel', 'Time [s]', 'wind [m/s]');
    data_dot = @(x, y) dot(s2m(x)', s2m(y)', 2);
    Xg = c.state_est.Xg;

    wind_g_f = structfun(@(x) filter_LPF(x, 0.05, system_params.ts), ...
                         c.state_est.wind_g.vector, 'UniformOutput', false);
    Vg_app = struct_op(@minus, wind_g_f, c.state_est.Vg);
    Vw = data_dot(Xg, Vg_app)./struct_norm(Xg);

    plot(t, struct_norm(c.state_est.wind_g.vector), t, Vw);
    legend('|wind\_g|', '|Vapp| || to tether');
  end
end

try
  if contains({'crit', 'trans_in', 'control', 'all'}, plot_names)
    mfig('c: Transition-in sensors and actuators', 'Time [s]', '');

    hold on;
    plot(c.time, [c.state_est.apparent_wind.sph.v / 10, ...
                  c.state_est.apparent_wind.sph.alpha, ...
                  c.state_est.apparent_wind.sph.beta, ...
                  s2m(c.state_est.pqr).', ...
                  c.state_est.tension_f / 1e5]);
    plot(c.time, double(c.flight_mode) / 10, 'k');
    plot(c.time, c.control_output.rotors([1, 4, 5, 8], :) / 100, '--');
    plot(c.time, c.control_output.flaps([6, 7, 8], :), '-.');
    legend('v/10', 'alpha', 'beta', ...
           'p', 'q', 'r', ...
           'tension/1e5', ...
           'flight mode', ...
           'M(sbo)/100', 'M(pbo)/100', 'M(pto)/100', 'M(sto)/100', ...
           'aileron (starboard)', 'elevator', 'rudder', ...
           'location', 'northwest');
    hold off;
  end
end

try
  if contains({'3d', 'trans_in', 'trans_out', 'control', 'all'}, plot_names)
    mfig('c: 3D', 'x', 'y', 'z', false);

    if contains('trans_in', plot_names)
      I = c.flight_mode <= 6;
    elseif contains('trans_out', plot_names)
      I = c.flight_mode >= 9 | c.flight_mode == 0;
    else
      I = true(size(c.flight_mode));
    end

    Xg = structfun(@(x) x(I), c.state_est.Xg, 'UniformOutput', false);
    dcm_g2b = c.state_est.dcm_g2b.d(:, :, I);

    dsn = round(length(t)/1000);
    td = downsample(t, dsn);
    I_dcm = 1:dsn:size(dcm_g2b, 3);
    DCM = dcm_g2b(:, :, I_dcm);
    X = structfun(@(x) downsample(x, dsn), Xg, 'UniformOutput', false);
    % Y = downsample(c.state_est.tension_f, dsn);
    % ylabel_name = 'tension';
    Y = downsample(c.flight_mode(I), dsn);
    ylabel_name = 'flightmode';

    plot_quartersphere(1.01*system_params.tether.length);
    hold on;
    plot3c(X.x, X.y, X.z, double(Y));
    axis equal;
    set(gca, 'ZDir', 'reverse', 'XDir', 'reverse');
    ylabel(ylabel_name);

    n_RotMat = 60;
    I_v = 1:round(length(Y)/n_RotMat):length(Y);
    for i = I_v
      plot_rotation_matrix(DCM(:, :, i)', [X.x(i), X.y(i), X.z(i)]', 4);
    end

    [Xg_ti_origin, dcm_g2ti, ti_origin_azimuth] = ...
        find_trans_in_frame(s2m(c.state_est.Xg)', ...
                            c.flight_mode, system_params, control_params);
    plot_rotation_matrix(dcm_g2ti, Xg_ti_origin, [150, 10, 10]);

    if contains('trans_in', plot_names)
      % view(-ti_origin_azimuth*180/pi, control_params.trans_in.r3_yaw2*180/pi);
      view(-ti_origin_azimuth*180/pi, 0);
    end
  end
end

try
  if contains({'trans_in', 'control', 'all'}, plot_names)
    mfig('c: Transition-in position', 'Time [s]', 'Position (ti) [m]');
    plot(t, s2m(c.trans_in.wing_pos_ti));
    plot(t, s2m(c.trans_in.wing_vel_ti), '--');
    legend('Position (ti)', '', '', 'Velocity (ti)', '', '', ...
           'location', 'northwest');
  end
end

try
  if contains({'trans_in', 'control', 'all'}, plot_names)
    mfig('c: Transition-in Euler angles', 'Time [s]', 'Angles [rad]', '');
    plot(t, s2m(c.trans_in.eulers_ti));
    plot(t, [ones(size(t)) * control_params.trans_in.roll_cmd, ...
             c.trans_in.pitch_cmd, ...
             ones(size(t)) * control_params.trans_in.yaw_cmd], '--');
    plot(t, c.trans_in.pitch_cmd_base, 'g-.');
    plot(t, c.state_est.apparent_wind.sph.alpha, 'c--');
    plot(t, c.trans_in.elevation_angle, 'k');
    legend('roll (ti)', 'pitch (ti)', 'yaw (ti)', ...
           'roll cmd', 'pitch cmd', 'yaw cmd', ...
           'pitch cmd (base)', 'alpha', 'elevation', ...
           'location', 'northwest');
  end
end

try
  if contains({'trans_in', 'thrust_moment', 'control', 'all'}, plot_names)
    mfig('c: Transition-in thrust-moment', 'Time [s]', ...
         'Thrust/10 [N] - Moment [N-m]', '');
    plot(t, [c.trans_in.thrust_moment.thrust / 10, ...
             s2m(c.trans_in.thrust_moment.moment).']);
    plot(t, [c.trans_in.available_thrust_moment.thrust / 10, ...
             s2m(c.trans_in.available_thrust_moment.moment).'], '.');
    actual_thrust_moment = control_params.rotor_control.thrusts_to_U
    legend('thrust (cmd)', 'roll moment (cmd)', 'pitch moment (cmd)', ...
           'yaw moment (cmd)', 'thrust (avail)', 'roll moment (avail)', ...
           'pitch moment (avail)', 'yaw moment (avail)', ...
           'location', 'northwest');
  end
end

print_error_without_termination();
mlinkaxes(ax);
end
