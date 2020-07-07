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

function [q] = dcm_to_q(dcm)
% dcm_to_q -- Convert a direction cosine matrix to a quaternion.
%
% This function defines a quaternion according to Shuster's convention such
% that the quaternion multiplication order is consistent with the direction
% cosine multiplication order.
%
%   dcm_to_q(a * b) == q_mult(dcm_to_q(a), dcm_to_q(b))

q = struct();
q.s = ones(1, size(dcm, 3));
q.v = zeros(3, size(dcm, 3));

d11 = dcm(1, 1, :);
d22 = dcm(2, 2, :);
d33 = dcm(3, 3, :);

i = d22 >= -d33 & d11 >= -d22 & d11 >= -d33;
if any(i)
  q.s(1, i) = 0.5 * sqrt(1 + d11(i) + d22(i) + d33(i));
  q.v(1, i) = squeeze(dcm(2, 3, i) - dcm(3, 2, i))' ./ (4 * q.s(1, i));
  q.v(2, i) = squeeze(dcm(3, 1, i) - dcm(1, 3, i))' ./ (4 * q.s(1, i));
  q.v(3, i) = squeeze(dcm(1, 2, i) - dcm(2, 1, i))' ./ (4 * q.s(1, i));
end

i = ~i & d22 <= -d33 & d11 >= d22 & d11 >= d33;
if any(i)
  q.v(1, i) = 0.5 * sqrt(1 + d11(i) - d22(i) - d33(i));
  q.s(1, i) = squeeze(dcm(2, 3, i) - dcm(3, 2, i))' ./ (4 * q.v(1, i));
  q.v(2, i) = squeeze(dcm(1, 2, i) + dcm(2, 1, i))' ./ (4 * q.v(1, i));
  q.v(3, i) = squeeze(dcm(3, 1, i) + dcm(1, 3, i))' ./ (4 * q.v(1, i));
end

i = ~i & d22 >= d33 & d11 <= d22 & d11 <= -d33;
if any(i)
  q.v(2, i) = 0.5 * sqrt(1 - d11(i) + d22(i) - d33(i));
  q.s(1, i) = squeeze(dcm(3, 1, i) - dcm(1, 3, i))' ./ (4 * q.v(2, i));
  q.v(1, i) = squeeze(dcm(1, 2, i) + dcm(2, 1, i))' ./ (4 * q.v(2, i));
  q.v(3, i) = squeeze(dcm(2, 3, i) + dcm(3, 2, i))' ./ (4 * q.v(2, i));
end

i = ~i & d22 <= d33 & d11 <= -d22 & d11 <= d33;
if any(i)
  q.v(3, i) = 0.5 * sqrt(1 - d11(i) - d22(i) + d33(i));
  q.s(1, i) = squeeze(dcm(1, 2, i) - dcm(2, 1, i))' ./ (4 * q.v(3, i));
  q.v(1, i) = squeeze(dcm(3, 1, i) + dcm(1, 3, i))' ./ (4 * q.v(3, i));
  q.v(2, i) = squeeze(dcm(2, 3, i) + dcm(3, 2, i))' ./ (4 * q.v(3, i));
end

q = q_normalize(q);
end
