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

function [motor_names] = GetMotorShortNames(motor_index)
% GetMotorShortNames -- Get motor short names corresponding to motor_index.
%
% [motor_names] = GetMotorShortNames(motor_index)
%
% Return the short names of motors (e.g. 'sbo', 'sbi', ...) corresponding to
% motor_index. The convention followed by AppConfigGetIndex() is used except
% that the index starts at 1 instead of 0, e.g. sbo is 1, pbo is 4, pto is 5,
% and sto is 8.
%
% Arguments:
%
% motor_index: A 1xN or Nx1 vector of motor indices in the range [1, 8].
%
% Return Values:
%
% motor_names: A 1xN or Nx1 cell array with short names corresponding to
%              motor_index.
  motor_names = {'sbo', 'sbi', 'pbi', 'pbo', 'pto', 'pti', 'sti', 'sto', ''};

  % Deal with out of range indices by assigning them to index 9 or ''.
  motor_index(motor_index <= 0 | 8 < motor_index) = 9;
  motor_names = motor_names(motor_index);
end
