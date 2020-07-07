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

function [messages, t, t0] = GetMotorMessages(filename, kMessageType, ...
                                              timebase, offset, motor_list, ...
                                              rescale_time)
% GetMotorMessages -- Extract motor messages and return cell array of structs.
%
% [messages, t, t0] = GetMotorMessages(filename, kMessageType, ...
%                                              timebase, offset)
% Extract desired motor messages according to message type.  Will base time off
% either first message extracted or can accept initial time offset given
% externally.
%
% Arguments
%
% filename: Name of H5 file with flight data.
% kMessageType: Aio message type with motor data
% timebase: If zero, then use capture_header time, else, this is a multiplier to
%           multiply aio_header sequence number with.
% offset: Externally calculated time offset.  Useful particularly when aligning
%         data to external event.
% motor_list: Array of motor indeces, indicating which motors to extract (1-8)
% rescale_time: Optional boolean to attempt to clean up noisy time data
%
% Return values
%
% messages: Cell array of structure motor data.
% t: Cell array of timestamps for message data.
% t0: Time of first sample after transition from flight_mode == 1.
%
% Required toolboxes: None.

  if nargin < 4
    offset = 0;
  end

  if nargin < 5
    motor_list = 1:8;
  end

  if nargin < 6
    rescale_time = false;
  end

  motor_str = {'MotorSbo', 'MotorSbi', 'MotorPbi', 'MotorPbo', ...
               'MotorPto', 'MotorPti', 'MotorSti', 'MotorSto', ...
               'DynoMotorSbo', 'DynoMotorSbi', 'DynoMotorPbi', 'DynoMotorPbo', ...
               'DynoMotorPto', 'DynoMotorPti', 'DynoMotorSti', 'DynoMotorSto'};

  % Initialize cell arrays
  t = cell(1,16);
  messages = cell(1,16);

  for i = motor_list
    messages{i} = GetUniqueData(filename, motor_str{i}, kMessageType);
  end

  % Extract time vectors
  firstTime = 1;
  for i = motor_list
    if firstTime
      if ~isfield(messages{i}, 'null')
        [t{i}, t0] = GetTimeFromStruct(messages{i}, offset, timebase, ...
                                       rescale_time);
        firstTime = 0;
      end
    else
      t{i} = GetTimeFromStruct(messages{i}, t0, timebase, rescale_time);
    end
    messages{i}.message.capture_time = t{i};
  end

end
