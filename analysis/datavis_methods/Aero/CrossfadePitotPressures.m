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

function [dyn_press, alpha_press, beta_press, stat_press] = ...
    CrossfadePitotPressures(pitots, pitot_params)
% CrossfadePitotPressures - Fade between low and high speed Pitot data.
%
% pitot_params = system_params.pitot;
% pitots = control_dataset.message.control_input.pitots
% [dyn_press, alpha_press, beta_press, stat_press] = ...
%   CrossfadePitotPressures(pitots, pitot_params)
%
% The M600 contains one main air data probe that is plumbed to two
% different sets of pressure measurement transducers: one high-precision,
% low-range, and one lower precision, higher range.  As dynamic pressure
% increases, the estimator switches from using the low speed sensor to
% using the high speed sensor. See CalcPitotApparentWindSph in
% control/estimator/estimator_apparent_wind.c.

% TODO(b/31995416): Somehow encode enumerations in h5 log files.
kPitotSensorHighSpeed = 1;
kPitotSensorLowSpeed = 2;
high_pressure = pitot_params.sensors.max_pressure(kPitotSensorLowSpeed);

% TODO: Create CrossfadeStruct function.
dyn_press = Crossfade(pitots.diff.dyn_press(kPitotSensorLowSpeed, :), ...
                      pitots.diff.dyn_press(kPitotSensorHighSpeed, :), ...
                      pitots.diff.dyn_press(kPitotSensorHighSpeed, :), ...
                      0.5 * high_pressure, high_pressure);

alpha_press = Crossfade(pitots.diff.alpha_press(kPitotSensorLowSpeed, :), ...
                      pitots.diff.alpha_press(kPitotSensorHighSpeed, :), ...
                      pitots.diff.dyn_press(kPitotSensorHighSpeed, :), ...
                      0.5 * high_pressure, high_pressure);

beta_press = Crossfade(pitots.diff.beta_press(kPitotSensorLowSpeed, :), ...
                      pitots.diff.beta_press(kPitotSensorHighSpeed, :), ...
                      pitots.diff.dyn_press(kPitotSensorHighSpeed, :), ...
                      0.5 * high_pressure, high_pressure);

stat_press  = pitots.stat_press(kPitotSensorLowSpeed, :);