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

function [v] = mtimesx(m, x)
% mtimesx -- Multiply an ndarray of matrices against a vector.
%
% Arguments:
%
% m: An ndarray of matrices, arranged according to m(:, :, i) where i represents
%    the i-th matrix.
% x: A column array of vectors.
%
% Return values:
%
% v: The matrix multiplication of v(:, i) = m(:, :, i) * x(:, i).

m1 = size(m, 1);
m3 = size(m, 3);
x1 = size(x, 1);
x2 = size(x, 2);
assert(m3 == x2 | m3 == 1 | x2 == 1);
assert(size(m, 2) == x1);
v = reshape(sum(bsxfun(@times, m, reshape(x, 1, x1, x2)), 2), m1, max(x2, m3));
end
