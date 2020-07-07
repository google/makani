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

function [out] = taylor_series_trig(x2, f0)
% Compute Taylor series for sin/cos functions of increment angles.
delta = 1 / factorial(f0);
out = delta;

if x2 > 1
  error('Invalid increment angle x2=%g.', x2);
end

while (abs(delta) > eps())
  fact = (f0 + 1) * (f0 + 2);
  delta = -delta * (x2 / fact);
  out = out + delta;
  f0 = f0 + 2;
end
