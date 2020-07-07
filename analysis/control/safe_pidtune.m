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

function [compensator, filtered_compensator] = ...
    safe_pidtune(plant, desired_crossover_hz, desired_phase_margin, ...
                 compensator_type, derivative_filter, validation_params)
% safe_pidtune -- Safe version of pidtune that validates controller margins.
%
% compensator = safe_pidtune(plant, desired_crossover_hz, ...
%                            desired_phase_margin, compensator_type)
  if nargin < 4
    compensator_type = 'pid';
  end
  if nargin < 5,
    derivative_filter = 1.0;
  end
  if nargin < 6,
    validation_params.min_gain_margin_db = 6;
    validation_params.min_phase_margin_deg = max(50, desired_phase_margin - 15);
    validation_params.min_delay_margin_s = 0.04;
    validation_params.max_sensitivity_db = 6;
  end
  compensator = pidtune(plant, compensator_type, ...
                        2 * pi * desired_crossover_hz, ...
                        pidtuneOptions('PhaseMargin', desired_phase_margin));

  validate_controller(plant, compensator, desired_crossover_hz, 6, ...
                      desired_phase_margin, 0.04);

  s = tf('s');
  filtered_compensator = compensator.Kp + compensator.Ki / s + ...
      compensator.Kd * s * derivative_filter;
  validate_controller(plant, filtered_compensator, [], ...
                      validation_params.min_gain_margin_db, ...
                      validation_params.min_phase_margin_deg, ...
                      validation_params.min_delay_margin_s, ...
                      validation_params.max_sensitivity_db);
end
