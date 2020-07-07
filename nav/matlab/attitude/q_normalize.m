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

function [q_plus] = q_normalize(q_minus)
% q_normalize -- Scale a quaternion to have a unit norm.

n = q_norm(q_minus);
q_plus = struct();
q_plus.s = ones(1, length(n));
q_plus.v = zeros(3, length(n));
i = find(n > 0);
j = i(q_minus.s(1, i) < 0);
n(j) = -n(j);
q_plus.s(1, i) = q_minus.s(1, i) ./ n(i);
q_plus.v(1, i) = q_minus.v(1, i) ./ n(i);
q_plus.v(2, i) = q_minus.v(2, i) ./ n(i);
q_plus.v(3, i) = q_minus.v(3, i) ./ n(i);
end
