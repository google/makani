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

function [p] = q_to_mrp(q)
% q_to_mrp -- Convert a quaternion to a modified Rodrigues parameter.

i = q.s < 0;
q.s(:, i) = -q.s(:, i);
q.v(:, i) = -q.v(:, i);

p = zeros(3, size(q.s, 2));
p(1, :) = q.v(1, :) ./ (q.s + 1);
p(2, :) = q.v(2, :) ./ (q.s + 1);
p(3, :) = q.v(3, :) ./ (q.s + 1);
end
