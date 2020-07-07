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

function s_out = struct_op(op, s1, s2)
% struct_op -- Perform binary operation on two structs.
%
% s_out = struct_op(op, s1, s2)
% Perform a binary operation on two structs that have the same fields
% and dimensions.
%
% Arguments
%
% op: A function handle to a 2-argument function. Type "help bsxfun" to
%     see a list of the basic built-in binary operators.
% s1, s2: Structures with same fields and dimensions.
%
% Return Values
%
% s_out: Result of the binary operation.  A structure with the same fields
%        and dimensions of s1 and s2.
%
% Example
%
% Vg_app = struct_op(@minus, c.state_est.wind_g, c.state_est.Vg)

if isstruct(s1)
  fn = fieldnames(s1);
  for k = 1:length(fn)
    s_out.(fn{k}) = struct_op(op, s1.(fn{k}), s2.(fn{k}));
  end
else
  s_out = op(s1, s2);
end
