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

function [ins, out] = ins_main(param, input, time_interval, message_types)
% ins_main  Inertial navigation system main entry point.
%
% [ins, out] = ins_main(param, input, time_interval, message_types)
% Processes input data with the inertial navigation system algorithm.
%
% Arguments:
%
% param: Specify algorithm configuration parameters. Use ins_param to create
%     the default structure.
% input: A properly formatted input data structure to process.
% time_interval: Optional. Process a subset of data given begin and end
%     processing times. Use [A, B] to specify both begin and end processing
%     times (in seconds), or B to specify the end processing time.
% message_types: Optional. Process a subset of the available data message
%     types. Specify an array of message types to process.
%
% Return values:
%
% ins: Final state of inertial navigation system. Use ins_parse to resume
%     processing from the last iteration.
% out: Output data structure.

% Handle optional input arguments.
if ~exist('time_interval', 'var') || isempty(time_interval)
  time_interval = [0; Inf];
elseif length(time_interval) == 1
  time_interval = [0; time_interval];
end
if ~exist('message_types', 'var') || isempty(message_types)
  message_types = unique(input.order.id);
end

% Initialize INS structure. This routine sets ins.iter to the start of the
% processing interval.
ins = ins_init(param, input, min(time_interval), message_types);

% Process data until end of processing interval.
[ins, out] = ins_parse(ins, input, max(time_interval));
