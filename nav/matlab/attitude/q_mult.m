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

function [q] = q_mult(b, c)
% q_mult -- Perform quaternion multiplication q = b (X) c.

q = struct();
q.s = b.s .* c.s - dot(b.v, c.v);
q.v = -cross(b.v, c.v);
q.v(1, :) = q.v(1, :) + b.v(1, :) .* c.s + b.s .* c.v(1, :);
q.v(2, :) = q.v(2, :) + b.v(2, :) .* c.s + b.s .* c.v(2, :);
q.v(3, :) = q.v(3, :) + b.v(3, :) .* c.s + b.s .* c.v(3, :);

i = q.s < 0;
q.s(:, i) = -q.s(:, i);
q.v(:, i) = -q.v(:, i);
end
