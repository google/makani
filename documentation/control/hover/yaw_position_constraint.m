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

function [max_reals, max_reals_closed, max_reals_closed_special] = ...
    yaw_position_constraint()
  constraint_tension_fractions = linspace(0, 1, 50);
  constraint_lengths = linspace(1, 20, 50);
  motor_bandwidth_hz = 10;

  max_reals = zeros(length(constraint_tension_fractions), ...
                    length(constraint_lengths));
  max_reals_closed = zeros(length(constraint_tension_fractions), ...
                           length(constraint_lengths));
  max_reals_closed_special = zeros(length(constraint_tension_fractions), ...
                                   length(constraint_lengths));
  for k1 = 1:length(constraint_tension_fractions)
    for k2 = 1:length(constraint_lengths)
      [sys, sys_closed, sys_closed_special] = constraint_sys( ...
          constraint_tension_fractions(k1), constraint_lengths(k2), ...
          motor_bandwidth_hz);
      max_reals(k1, k2) = max(real(pole(sys)));
      max_reals_closed(k1, k2) = max(real(pole(sys_closed)));
      max_reals_closed_special(k1, k2) = max(real(pole(sys_closed_special)));
    end
  end

  figure('Position', [100, 100, 1000, 500]);
  subplot(1, 2, 1);
  imagesc(constraint_lengths, constraint_tension_fractions, max_reals, [0, 0.1]);
  axis square;
  title('Open loop');
  xlabel('Constraint length [m]');
  ylabel('Tension fraction [#]');
  h = colorbar;
  %ylabel(h, 'Maximum real component of pole [1/s]');

  subplot(1, 2, 2);
  imagesc(constraint_lengths, constraint_tension_fractions, max_reals_closed, [0, 0.1]);
  axis square;
  title('Closed yaw loop');
  xlabel('Constraint length [m]');
  %ylabel('Tension fraction [#]');
  h = colorbar;
  ylabel(h, 'Maximum real component of pole [1/s]');

  print(gcf, '-dpdf', 'yaw_position_constraint.pdf');

  %% figure(3);
  %% imagesc(constraint_lengths, constraint_tension_fractions, ...
  %%         max_reals_closed_special, [-0.1, 0.3]);
  %% title('Closed constraint-tuned yaw loop');
  %% xlabel('Constraint length [m]');
  %% ylabel('Tension fraction [#]');
  %% h = colorbar;
  %% ylabel(h, 'Maximum real component of pole [1/s]');
end

function [sys, sys_closed, sys_closed_special] = constraint_sys( ...
    constraint_tension_fraction, constraint_length, motor_bandwidth_hz)
  g = 9.81;
  m = 1422;
  Izz = 35200;
  lprob = 2;

  lc = constraint_length;
  tc = m*g * constraint_tension_fraction;

  A = [0, 1, 0, 0;
       -tc/(lc * m), 0, (g - tc*(lc + lprob)/(m*lc)), 0;
       0, 0, 0, 1;
       -tc*lprob/(Izz*lc), 0, -tc*lprob^2/(lc*Izz), 0];

  B = [0, 0, 0, 1/Izz]';
  C = [0, 0, 1, 0];

  sys = ss(A, B, C, 0);

  motor_sys = tf([2 * pi * motor_bandwidth_hz], ...
                 [1, 2 * pi * motor_bandwidth_hz]) / 10;
  yaw_sys = tf([1], [Izz, 0, 0]);
  yaw_pid = pidtune(motor_sys * yaw_sys, 'pid', ...
                    2 * pi * max(motor_bandwidth_hz / 20, 0.3), ...
                    pidtuneOptions('PhaseMargin', 60));
  yaw_pid_special = pidtune(motor_sys * sys, 'pid', ...
                            2 * pi * max(motor_bandwidth_hz / 20, 0.3), ...
                            pidtuneOptions('PhaseMargin', 60));

  sys_closed = feedback(yaw_pid * motor_sys * sys, 1);
  sys_closed_special = feedback(yaw_pid_special * motor_sys * sys, 1);
end
