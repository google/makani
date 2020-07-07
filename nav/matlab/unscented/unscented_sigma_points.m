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

function [chi, L] = unscented_sigma_points(x, Pxx, lambda)

% Generate 2*n+1 symmetric sigma points.
n = size(x, 1);
L = chol((n + lambda) * Pxx, 'lower');
chi = zeros(n, 2*n + 1);
chi(:,1) = x;
for i = 1:n
  chi(:,1+i) = x - L(:,i);
  chi(:,1+n+i) = x + L(:,i);
end
