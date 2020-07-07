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

function is_valid = validate_controller(plant, controller, ...
                                        desired_crossover_hz, ...
                                        min_gain_margin_db, ...
                                        min_phase_margin_deg, ...
                                        min_delay_margin_s, ...
                                        max_sensitivity_db, ...
                                        max_complementary_sensitivity_db)
% validate_controller -- Validates controller gain and phase margins.
%
% is_valid = validate_controller(plant, controller, ...
%                                desired_crossover_hz, ...
%                                min_gain_margin_db, ...
%                                min_phase_margin_deg, ...
%                                min_delay_margin_s, ...
%                                max_sensitivity_db, ...
%                                max_complementary_sensitivity_db)
%
% Validates a controller by checking that we have achieved a specific
% crossover frequency, gain margin, phase margin, sensitivity, and
% complementary sensitivity.

  if nargin < 8
    max_complementary_sensitivity_db = 6;
  end
  if nargin < 7
    max_sensitivity_db = 6;
  end
  if nargin < 6
    min_delay_margin_s = [];
  end
  if nargin < 5
    min_phase_margin_deg = 30;
  end
  if nargin < 4
    min_gain_margin_db = 6;
  end
  if nargin < 3
    desired_crossover_hz = [];
  end

  open_loop_sys = plant * controller;
  is_valid = true;

  % Where allmargin returns information about ALL crossover frequencies and
  % margins, margin only returns the most restrictive gain and phase
  % margins.
  info = allmargin(open_loop_sys);
  [gain_margin, phase_margin_deg, ~, ~] = margin(open_loop_sys);
  delay_margin_s = min(info.DelayMargin);

  if length(info.PMFrequency) > 2,
    % The radial loop provides damping only, and therefore has two unity
    % gain crossings.  All other loops have only one crossover frequency,
    % except for one special case where we evaluate the tension loop with a
    % spring constant of zero.
    warning('Expected at most two crossover frequencies.');
    is_valid = false;
  end
  omega_crossover = max(info.PMFrequency);

  if ~info.Stable
    warning('Not closed loop stable.');
    is_valid = false;
  end

  if ~isempty(min_delay_margin_s)
    if min_delay_margin_s > delay_margin_s
      warning(sprintf(['Minimum time delay margin was not achieved: ', ...
                       'desired = %f s, actual = %f s.'], ...
                      min_delay_margin_s, delay_margin_s));
      is_valid = false;
    end
  end

  if ~isempty(desired_crossover_hz)
    crossover_error = abs(desired_crossover_hz - omega_crossover / (2 * pi)) ...
                      / desired_crossover_hz;
    if crossover_error > 0.05
      warning(sprintf(['Cross-over frequency was not achieved: ', ...
                       'desired = %f Hz, actual = %f Hz.'], ...
                      desired_crossover_hz, omega_crossover / (2 * pi)));
      is_valid = false;
    end
  end

  if ~isempty(min_gain_margin_db)
    gain_margin_db = abs(20 * log10(gain_margin));
    if gain_margin_db < min_gain_margin_db * 0.99
      warning(sprintf(['Gain margin was not achieved: ', ...
                       'desired = %f dB, actual = %f dB.'], ...
                      min_gain_margin_db, gain_margin_db));
      is_valid = false;
    end
  end

  if ~isempty(min_phase_margin_deg)
    if phase_margin_deg < min_phase_margin_deg * 0.99
      warning(sprintf(['Phase margin was not achieved: ', ...
                       'desired = %0.1f deg, actual = %0.1f deg.'], ...
                      min_phase_margin_deg, phase_margin_deg));
      is_valid = false;
    end
  end

  if ~isempty(max_sensitivity_db)
    peak_sensitivity_db = 20 * log10(norm(feedback(1, open_loop_sys), 'Inf'));
    if peak_sensitivity_db > max_sensitivity_db
      warning(sprintf(['Peak sensitivity was not achieved: ', ...
                       'desired = %0.1f dB, actual = %0.1f dB.'], ...
                      max_sensitivity_db, peak_sensitivity_db));
      is_valid = false;
    end
  end

  if ~isempty(max_complementary_sensitivity_db)
    peak_complementary_sensitivity_db = 20 * log10(norm( ...
        feedback(open_loop_sys, 1), 'Inf'));
    if peak_complementary_sensitivity_db > max_complementary_sensitivity_db
      warning(sprintf(['Peak complementary sensitivity was not achieved: ', ...
                       'desired = %0.1f dB, actual = %0.1f dB.'], ...
                      max_complementary_sensitivity_db, ...
                      peak_complementary_sensitivity_db));
      is_valid = false;
    end
  end
end
