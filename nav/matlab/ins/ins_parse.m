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

function [ins, out] = ins_parse(ins, input, end_time, out)
% ins_parse  Iterate the INS state for each data point.
%
% [ins, out] = ins_parse(ins, input, end_time, message_types, out)
%
% Arguments:
%
% ins: Initial state. Use ins_init to initialize the default structure.
% input: A properly formatted input data structure to process.
% end_time: Optional. Process input data until specified end time.
% out: Optional. Output data structure.
%
% Return values:
%
% ins: Final state of inertial navigation system. Use ins_parse to resume
%     processing from the last iteration.
% out: Output data structure.

% Handle optional input arguments.
if ~exist('end_time', 'var') || isempty(end_time)
  end_time = Inf;
end
if ~exist('out', 'var') || isempty(out)
  out = struct();
end

% Determine iterations to process.
iters = (ins.iter + 1):length(input.order.id);
iters = iters(ismember(input.order.id(iters), ins.message_types));
iters = iters(input.order.t(iters) <= end_time);

% Allocate memory for output.
out = ins_log_init(ins.param, input, input.order.t(iters(1)), end_time, out);

% Determine first occurrence of each message type.
types = unique(input.order.id);
idx = zeros(max(types), 1);
for i = 1:length(types)
  idx(types(i)) = 1 + length(find(input.order.id(1:iters(1) - 1) == types(i)));
end

% Process each message sequentially in the order logged.
t_start = tic;
for i = 1:length(iters)
  ins.iter = iters(i);
  type = input.order.id(ins.iter);
  name = input.order.name{type};

  % Store data in circular buffers.
  msg = get_message(input.(name), idx(type));
  ins.(name) = ins_cbuf_insert(ins.(name), msg);

  % Lag INS by a specified number of IMU samples.
  if type == input.order.map.imu ...
      && length(ins.imu.data) >= ins.param.lag_imu_samples
    tail = ins.imu.head + ins.imu.length - ins.param.lag_imu_samples;
    tail = 1 + mod(tail, ins.imu.length);
    while ins.imu.tail ~= tail
      [ins, out] = ins_step_imu(ins, ins.imu.data(ins.imu.tail), out);
      ins.imu.tail = 1 + mod(ins.imu.tail, ins.imu.length);
    end
  end
  idx(type) = idx(type) + 1;
end
t_elapsed = toc(t_start);
fprintf(1, 'Processed %d packets in %.1f seconds (%.1f packets/s)\n', ...
        length(iters), t_elapsed, length(iters) / t_elapsed);
