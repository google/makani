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

function ax = plot_faults(plot_names, data, t)
% plot_faults -- Makes a standard set of plots from the fault detection data.
%
% ax = plot_faults(plot_names, data, t)
% Makes predefined plots labeled by various keywords (e.g. 'faults').  This
% is intended to be called by plot_telem, which is the general plotting
% routine.  Also, links axes of plots.
%
% plot_names: Cell array of strings describing what to plot.
%
% Example:
%   data = load_h5log('20130307-143445.log');
%   plot_faults({'faults'}, data);

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
hline = @(y) plot(xlim, [y y], '--', 'Color', [0.7, 0.7, 0.7]);

try
  if contains({'comms', 'power', 'joystick', 'faults', 'fault names', ...
               'all'}, plot_names)
    mfig('fd: Comms and Power Faults', 'Time [s]', 'Fault code [#]');
    plot_fault_subset(t, c.faults.code, {'kSubsysHvBus', ...
                                         'kSubsysJoystick'});
  end
end

try
  if contains({'position', 'faults', 'fault names', 'all'}, plot_names)
    mfig('fd: Position Sensor Faults', 'Time [s]', 'Fault code [#]');
    plot_fault_subset(t, c.faults.code, {'kSubsysGsgAAzi', ...
                                         'kSubsysGsgAEle', ...
                                         'kSubsysGsgATwist', ...
                                         'kSubsysGsgBAzi', ...
                                         'kSubsysGsgBEle', ...
                                         'kSubsysGsgBTwist', ...
                                         'kSubsysGpsPos', ...
                                         'kSubsysGpsVel'});
  end
end

try
  if contains({'imu', 'faults', 'fault names', 'all'}, plot_names)
    mfig('fd: IMU Faults', 'Time [s]', 'Fault code [#]');
    plot_fault_subset(t, c.faults.code, {'kSubsysImuAAcc', ...
                                         'kSubsysImuAGyro', ...
                                         'kSubsysImuAMag', ...
                                         'kSubsysImuBAcc', ...
                                         'kSubsysImuBGyro', ...
                                         'kSubsysImuBMag', ...
                                         'kSubsysImuCAcc', ...
                                         'kSubsysImuCGyro', ...
                                         'kSubsysImuCMag'});
  end
end

try
  if contains({'loadcell', 'faults', 'fault names', 'all'}, plot_names)
    mfig('fd: Loadcell Faults', 'Time [s]', 'Fault code [#]');
    plot_fault_subset(t, c.faults.code, {'kSubsysLoadcellSensorPort0', ...
                                         'kSubsysLoadcellSensorPort1', ...
                                         'kSubsysLoadcellSensorStarboard0', ...
                                         'kSubsysLoadcellSensorStarboard1'});
  end
end

try
  if contains({'pitot', 'wind', 'faults', 'fault names', 'all'}, plot_names)
    mfig('fd: Pitot and Wind Faults', 'Time [s]', 'Fault code [#]');
    plot_fault_subset(t, c.faults.code, {'kSubsysWindSensor', ...
                                         'kSubsysPitotStatic', ...
                                         'kSubsysPitotAlpha', ...
                                         'kSubsysPitotBeta', ...
                                         'kSubsysPitotDynamic'});
  end
end

try
  if contains({'perch', 'all'}, ...
              plot_names)
    mfig('fd: Levelwind and Perch', 'Time [s]', 'Fault code [#]');
    plot_fault_subset(t, c.faults.code, {'kSubsysLevelwindEleA', ...
                                         'kSubsysLevelwindEleB', ...
                                         'kSubsysPerchAziA', ...
                                         'kSubsysPerchAziB'});
  end
end

try
  if contains({'rotor', 'winch', 'release', 'faults', 'fault names', 'all'}, ...
              plot_names)
    mfig('fd: Rotor, Winch and Release Faults', 'Time [s]', 'Fault code [#]');
    plot_fault_subset(t, c.faults.code, {'kSubsysTetherRelease', ...
                                         'kSubsysWinch', ...
                                         'kSubsysMotorSbo', ...
                                         'kSubsysMotorSbi', ...
                                         'kSubsysMotorPbi', ...
                                         'kSubsysMotorPbo', ...
                                         'kSubsysMotorPto', ...
                                         'kSubsysMotorPti', ...
                                         'kSubsysMotorSti', ...
                                         'kSubsysMotorSto'});
  end
end

try
  if contains({'servo', 'faults', 'fault names', 'all'}, ...
              plot_names)
    mfig('fd: Servo Faults', 'Time [s]', 'Fault code [#]');
    plot_fault_subset(t, c.faults.code, {'kSubsysServoA1', ...
                                         'kSubsysServoA2', ...
                                         'kSubsysServoA4', ...
                                         'kSubsysServoA5', ...
                                         'kSubsysServoA7', ...
                                         'kSubsysServoA8', ...
                                         'kSubsysServoE1', ...
                                         'kSubsysServoE2', ...
                                         'kSubsysServoR1', ...
                                         'kSubsysServoR2'});
  end
end

try
  if contains({'winch faults', 'faults', 'all'}, plot_names)
    mfig('fd: Wing Distance from Perch', 'Time [s]', 'Distance [m]');

    data_length = size(c.time);
    Xg = s2m(c.state_est.Xg);
    wing_pos_from_perch = zeros([3, data_length]);
    for k = 1:max(data_length);
      dcm_g2p = angle_to_dcm(c.control_input.perch.perch_azi(k), 0, 0, 'zxy');
      wing_pos_from_perch(:, k) = ( ...
          dcm_g2p * Xg(:, k) ...
          - s2m(control_params.hover.path.perched_wing_pos_p));
    end
    wing_dis_from_Xg = sqrt(sum(wing_pos_from_perch.^2))';
    wing_dis_from_winch = system_params.tether.length ...
                          + pi * system_params.winch.r_drum ...
                          + c.control_input.perch.winch_pos;

    plot(t, wing_dis_from_Xg);
    plot(t, wing_dis_from_winch);
    plot(t, 10 * abs(wing_dis_from_winch - wing_dis_from_Xg));
    hline(10 * control_params.fault_detection.winch.pos_diff_threshold);
    legend('Estimate from Xg', 'Estimate from winch position', ...
           '10 X difference');
  end
end

% TODO: Add plots for servo and motor actuation errors, GSG
% unwinding, estimated versus measured apparent airspeed, zg_press vs
% Xg plot(c.time, [v_norm(c.Vapp) c.Vapp_pitot]); ylabel('v [m/s]').

print_error_without_termination();
mlinkaxes(ax);
end
function plot_fault_subset(t, faults, subset)
  subsystem_labels = {'kSubsysControllerA',
                      'kSubsysControllerB',
                      'kSubsysControllerC',
                      'kSubsysGpsPos',
                      'kSubsysGpsVel',
                      'kSubsysGsAcc',
                      'kSubsysGsGyro',
                      'kSubsysGsMag',
                      'kSubsysGsCompass',
                      'kSubsysGsGpsPos',
                      'kSubsysGsGpsVel',
                      'kSubsysGsgAAzi',
                      'kSubsysGsgAEle',
                      'kSubsysGsgATwist',
                      'kSubsysGsgBAzi',
                      'kSubsysGsgBEle',
                      'kSubsysGsgBTwist',
                      'kSubsysHvBus',
                      'kSubsysImuAAcc',
                      'kSubsysImuAGyro',
                      'kSubsysImuAMag',
                      'kSubsysImuBAcc',
                      'kSubsysImuBGyro',
                      'kSubsysImuBMag',
                      'kSubsysImuCAcc',
                      'kSubsysImuCGyro',
                      'kSubsysImuCMag',
                      'kSubsysJoystick',
                      'kSubsysLevelwindEle',
                      'kSubsysLoadcellSensorPort0',
                      'kSubsysLoadcellSensorPort1',
                      'kSubsysLoadcellSensorStarboard0',
                      'kSubsysLoadcellSensorStarboard1',
                      'kSubsysMotorSbo',
                      'kSubsysMotorSbi',
                      'kSubsysMotorPbi',
                      'kSubsysMotorPbo',
                      'kSubsysMotorPto',
                      'kSubsysMotorPti',
                      'kSubsysMotorSti',
                      'kSubsysMotorSto',
                      'kSubsysPerchAzi',
                      'kSubsysPitotStatic',
                      'kSubsysPitotAlpha',
                      'kSubsysPitotBeta',
                      'kSubsysPitotDynamic',
                      'kSubsysProximitySensor',
                      'kSubsysServoA1',
                      'kSubsysServoA2',
                      'kSubsysServoA4',
                      'kSubsysServoA5',
                      'kSubsysServoA7',
                      'kSubsysServoA8',
                      'kSubsysServoE1',
                      'kSubsysServoE2',
                      'kSubsysServoR1',
                      'kSubsysServoR2',
                      'kSubsysTetherRelease',
                      'kSubsysWinch',
                      'kSubsysWindSensor'};

  % Number of fault types.
  fault_types = {'None', ...
                 'Disabled', ...
                 'Disagreement', ...
                 'Implausible', ...
                 'No Update', ...
                 'Out of Range', ...
                 'Thrown Error'};
  nf = length(fault_types);

  % Create axes.
  set(gca, 'YTickLabel', fault_types, ...
           'YTick', 0:nf);

  index = [];
  line_color = get(gca, 'ColorOrder');
  for i = 1:length(subset)
    % Plot subsystem faults.
    fault_subset = faults(find(ismember(subsystem_labels, subset{i})), :);
    fault_bits = bsxfun(@times, '1' == dec2bin(fault_subset, nf), nf:-1:1);
    color_ind = mod(i, length(line_color)) + 1;
    for j = 1:nf
      plot_fd((i - 1)*nf + j) = plot(t, fault_bits(:, j), ...
                                     'Marker', '.', 'Color', ...
                                     line_color(color_ind, :));
    end
    % Set subsystem label.
    index(i) = (i - 1)*nf + 1;
    set(plot_fd(index(i)), 'DisplayName', strrep(subset{i}, '_', ' '));
  end

  legend_fd = legend(plot_fd(index), 'Location', 'EastOutside');
end
