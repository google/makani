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

function [ins, out] = ins_step_imu(ins, data, out)

%fprintf(1, 'Step IMU t=%.3f\n', data.t);

% Propagate navigation state forward.
xhat_1 = ins.xhat;
ins.xhat = ins_xhat_propagate(ins.param, ins.xhat, ins.gps, data);

% Propagate error state about trajectory.
[ins.dx, ins.inputs] = ins_dx_propagate(ins.param, ins.xhat, ins.dx, ...
                                        xhat_1, ins.gps, data);

% Process aiding measurements.
t_aid = data.t;
while ins.pps.tail ~= ins.pps.head && ins.pps.data(ins.pps.tail).t < t_aid
  [ins, out] = ins_step_pps(ins, xhat_1, ins.pps.data(ins.pps.tail), out);
  ins.pps.tail = 1 + mod(ins.pps.tail, ins.pps.length);
end
while ins.eph.tail ~= ins.eph.head && ins.eph.data(ins.eph.tail).t < t_aid
  [ins, out] = ins_step_eph(ins, xhat_1, ins.eph.data(ins.eph.tail), out);
  ins.eph.tail = 1 + mod(ins.eph.tail, ins.eph.length);
end
while ins.iono.tail ~= ins.iono.head && ins.iono.data(ins.iono.tail).t < t_aid
  [ins, out] = ins_step_iono(ins, xhat_1, ins.iono.data(ins.iono.tail), out);
  ins.iono.tail = 1 + mod(ins.iono.tail, ins.iono.length);
end
while ins.obs.tail ~= ins.obs.head && ins.obs.data(ins.obs.tail).t_obs < t_aid
  [ins, out] = ins_step_obs(ins, xhat_1, ins.obs.data(ins.obs.tail), out);
  ins.obs.tail = 1 + mod(ins.obs.tail, ins.obs.length);
end
while ~ins.param.wheel.has_wheel && ins.resolver.tail ~= ins.resolver.head ...
    && ins.resolver.data(ins.resolver.tail).t < t_aid
  [ins, out] = ins_step_resolver(ins, xhat_1, ...
                                 ins.resolver.data(ins.resolver.tail), out);
  ins.resolver.tail = 1 + mod(ins.resolver.tail, ins.resolver.length);
end
while ins.wheel.tail ~= ins.wheel.head ...
    && ins.wheel.data(ins.wheel.tail).t < t_aid
  [ins, out] = ins_step_wheel(ins, xhat_1, ins.wheel.data(ins.wheel.tail), out);
  ins.wheel.tail = 1 + mod(ins.wheel.tail, ins.wheel.length);
end

% Handle Phi propagation timeout.
ins.dx = ins_dx_timeout(ins.dx);

% Correct navigation state.
if isempty(ins.dx.Phi_count) || max(ins.dx.Phi_count) == 0
  if norm(ins.dx.dx_minus) > 0
    fprintf(1, 'Correct navigation state\n');
    ins.xhat = ins_xhat_correct(ins.param, ins.xhat, ins.dx.dx_minus);
    ins.dx.dx_minus(:) = 0;
  end
end

% Re-initialize system.
if ins.reinit && (ins.xhat.t - ins.gps.xhat.t) <= data.dt
  ins = ins_step_reinit(ins);
end

% Output.
out = ins_log_state(ins, data.index, out);
if (ins.xhat.t - ins.gps.xhat.t) <= data.dt
  out = ins_log_gps(ins.param, ins.gps, ins.inputs, out);
end
