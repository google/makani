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

function ax = plot_sim(plot_names, data, t)
% plot_sim -- Makes a standard set of plots from the controller data.
%
% plot_names: Cell array of strings describing what to plot
% data: Data structure containing data and parameters.
% t: Optional time vector.
%
% Example:
%   data = load_h5log('last.h5');
%   plot_sim({'tether'}, data, t);
make_naming_shortcuts(data);
lasterror('reset');
ax = [];
if isempty(S)
  return
end
if nargin < 3
  t = S.message.time;
end

% Some handy helper functions for plotting
hline = @(y) plot(xlim, [y y], '--', 'Color', [0.7, 0.7, 0.7]);

try
  if contains({'pos', 'all'}, plot_names)
    mfig('s: Xg', 'Time [s]', 'Xg [m]')
    plot(s.time, s2m(s.wing.Xg))
  end
end

try
  if contains({'vel', 'all'}, plot_names)
    mfig('s: Vb', 'Time [s]', 'Vb [m]')
    plot(s.time, s2m(s.wing.Vb))
  end
end

try
  if contains({'tether', 'all'}, plot_names)
    mfig('s: tether');
    msub(3, 1, 1, 'tether nodes', [], 'zg [m]');
    for i = 1:length(s.tether.Xg_nodes)
      plot(t, s.tether.Xg_nodes(i).z);
    end
    hline(sim_params.gs_sim.z_ground);  % Ground level.
    msub(3, 1, 2, 'winch pos', [], '[m]');
    plot(t, s.perch.theta_wd * system_params.winch.r_drum);
    msub(3, 1, 3, 'tension', [], '[N]');
    plot(t, s.tether.tension);
  end
end

try
  if contains({'perch', 'all'}, plot_names)
    mfig('s: perch');
    msub(3, 1, 1, 'perch winch ang', [], '[rad]', [], false);
    plot(t, s.perch.theta_p, 'b', ...
         t, s.perch.theta_wd, 'g', ...
         c.time, c.control_input.perch.perch_azi, 'b--');
    legend('theta\_p sim', 'sim theta\_wd sim', 'azi\_perch', ...
           'sim theta_{w\_cmd}');
    msub(3, 1, 2, 'perch winch rate', [], '[rad/s]');
    plot(t, s.perch.omega_p, 'b', t, s.perch.omega_wd, 'g');
  end
end

try
  if contains({'perch', 'all'}, plot_names)
    mfig('c: perch', 'time (s)', 'pos (m)', '');
    plot(s.time, s2m(s.wing.Xg));
    plot(c.time, s2m(c.estimator.Xg_gps), '--');
    plot(c.time, s2m(c.state_est.Xg), ':');

    legend('s.Xg', '', '', 'c.Xg\_gps', '', '', 'c.Xg', '', '');
  end
end

try
  if contains({'perch', 'all'}, plot_names)
    mfig('c: horz pos', 'time (s)', 'sqrt(x^2+y^2) (m)', '');
    plot(s.time, sqrt(s.wing.Xg.x.^2 + s.wing.Xg.y.^2));
    plot(c.time, sqrt(c.estimator.Xg_gps.x.^2 + ...
                      c.estimator.Xg_gps.y.^2), '--');
  end
end

try
  if contains({'perch', 'all'}, plot_names)
    mfig('c: horz pos', 'time (s)', 'sqrt(x^2+y^2) (m)', '');
    plot(s.time, sqrt(s.wing.Xg.x.^2 + s.wing.Xg.y.^2));
    plot(c.time, sqrt(c.estimator.Xg_gps.x.^2 + ...
                      c.estimator.Xg_gps.y.^2), '--');
  end
end

try
  if contains({'sim', 'power', 'all'}, plot_names)
    mfig('s: Aerodynamic rotor power', 'Time [s]', 'Aerodynamic power [kW]');
    plot(t, s.rotors.aero_power / 1e3, ...
         t, 600 / 8, 'k', t, -600 / 8, 'k');
  end
end

try
  if contains({'sim', 'power', 'all'}, plot_names)
    mfig('s: Aerodynamic rotor torque', 'Time [s]', 'Aerodynamic torque [N-m]');
    plot(t, s.rotors.aero_torque, ...
         t, sim_params.power_sys_sim.max_motor_torque, 'k', ...
         t, -sim_params.power_sys_sim.max_motor_torque, 'k');
  end
end

try
  if contains({'levelwind', 'disengage_engage', 'all'}, plot_names)
    vnormxy = @(X) sqrt(X.x.^2 + X.y.^2);
    mfig('simulation levelwind');
    msub(3, 1, 1, 'anchors norm(xy)', [], '');
    plot(t, vnormxy(s.perch.Xg_anchor), 'k');
    plot(t, vnormxy(s.perch.Xg_gsg), 'k:');
    plot(t, vnormxy(s.perch.Xg_lw), 'k--');
    legend('|Xg\_anchor|', '|Xg\_gsg|', '|Xg\_lw|');

    msub(3, 1, 2, '', [], 'tether free len [m]');
    plot(t, s.perch.tether_free_len);

    msub(3, 1, 3, 'anchors', [], '[m]');
    plot(t, s2m(s.perch.Xg_anchor));
    plot(t, s2m(s.perch.Xg_gsg), '-.');
    plot(t, s2m(s.perch.Xg_lw), '--');
    plot(t, struct_norm(s.perch.Xg_anchor), 'k');
    plot(t, struct_norm(s.perch.Xg_gsg), 'k-.');
    plot(t, struct_norm(s.perch.Xg_lw), 'k--');
    legend('Xg\_anchor', '', '', 'Xg\_gsg', '', '', 'Xg\_lw', '', '', ...
           '|Xg\_anchor|', '|Xg\_gsg|', '|Xg\_lw|');
  end
end

servo_names = {'a1', 'a2', 'a3', 'a4', 'a5', 'a6', ...
               'ele1', 'ele2', 'rud1', 'rud2'};

try
  if contains({'servos', 'all'}, plot_names)
    mfig('s: servo pos', 'Time [s]', 'Position [rad]');
    plot(t, s.servo_sensor.theta_flaps);
    legend(servo_names);
  end
end

try
  if contains({'servos', 'all'}, plot_names)
    mfig('s: servo vel', 'Time [s]', 'Velocity [rad/s]');
    plot(t, s.servo_sensor.omega_flaps);
    legend(servo_names);
  end
end

try
  if contains({'servos', 'all'}, plot_names)
    mfig('s: servo aero torq', 'Time [s]', 'Aero torque [Nm]');
    plot(t, s.servo_sensor.aero_torq_flaps);
    legend(servo_names);
  end
end

try
  if contains({'servos', 'servo power', 'all'}, plot_names)
    mfig('s: servo power', 'Time [s]', 'Elec power [W]');
    power = s.servo_sensor.motor_powers;
    plot(t, power, t, sum(power, 1), 'k.-');
    legend(servo_names);
  end
end

try
  if contains({'servos', 'all'}, plot_names)
    for i = 1:s.servo_sensor.num_servos(1)
      mfig(['s: ' servo_names{i} ' aero torq v pos'], 'Theta [rad]', ...
           'torq [Nm]', '', false);
      plot(s.servo_sensor.theta_flaps(i, :), ...
           s.servo_sensor.aero_torq_flaps(i, :));
    end
  end
end

print_error_without_termination();
mlinkaxes(ax);
end
