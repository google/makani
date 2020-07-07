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

function [dcm] = q_to_dcm(q)
% q_to_dcm -- Convert a quaternion to a direction cosine matrix.
%
% This function defines a quaternion according to Shuster's convention such
% that the quaternion multiplication order is consistent with the direction
% cosine multiplication order.
%
%   q_to_dcm(q_mult(p, q)) == q_to_dcm(p) * q_to_dcm(q)

q = q_normalize(q);
s = q.s .* q.s - dot(q.v, q.v);
dcm = zeros(3, 3, length(s));
dcm(1, 1, :) = 2 * q.v(1, :) .* q.v(1, :) + s;
dcm(1, 2, :) = 2 * q.v(2, :) .* q.v(1, :) + 2 * q.s .* q.v(3, :);
dcm(1, 3, :) = 2 * q.v(3, :) .* q.v(1, :) - 2 * q.s .* q.v(2, :);
dcm(2, 1, :) = 2 * q.v(1, :) .* q.v(2, :) - 2 * q.s .* q.v(3, :);
dcm(2, 2, :) = 2 * q.v(2, :) .* q.v(2, :) + s;
dcm(2, 3, :) = 2 * q.v(3, :) .* q.v(2, :) + 2 * q.s .* q.v(1, :);
dcm(3, 1, :) = 2 * q.v(1, :) .* q.v(3, :) + 2 * q.s .* q.v(2, :);
dcm(3, 2, :) = 2 * q.v(2, :) .* q.v(3, :) - 2 * q.s .* q.v(1, :);
dcm(3, 3, :) = 2 * q.v(3, :) .* q.v(3, :) + s;
end
