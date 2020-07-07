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

function [ins, out] = ins_step_resolver(ins, xhat_1, data, out)
p = ins.param;
xhat = ins_xhat_interpolate(p, xhat_1, ins.xhat, data.t);

% Apply virtual corrections while stationary.
if abs(data.ll_vel) < p.stationary.max_velocity ...
    && xhat.t - ins.t_stationary > p.stationary.update_period
  [ins.dx, out] = ins_correct_stationary(p, xhat, ins.dx, ins.inputs, out);
  ins.t_stationary = xhat.t;
end

if xhat.t - ins.t_wheel > p.wheel.update_period
  ins.t_wheel = xhat.t;

  % Since the Lexus does not have wheel speed sensors, we use its resolver to
  % apply a single odometry correction at the differential position. We use
  % the differential position rather than both rear wheels because we don't
  % want to imply a straight line trajectory.
  r_bdiff_b = (p.geometry.r_bwheelrl_b + p.geometry.r_bwheelrr_b) / 2;

  % Use the rear-left error state.
  wheel = p.wheel.WHEEL_RL;

  % Apply correction.
  [H, R, dy] = ins_output_wheel(p, xhat, ins.inputs, r_bdiff_b, ...
                                data.ll_vel / p.geometry.wheel_radius, wheel);
  [ins.dx, out] = ins_dx_correct(p, ins.dx, H, R, dy, 3*3, data.t, out, ...
                                 p.meas.WHEEL_VEL(wheel));
end
