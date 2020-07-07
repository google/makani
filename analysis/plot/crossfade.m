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

%  CROSSFADE   Saturated linear interpolation.
%   The output is a linear combination of z0 and z1.
%   All inputs are scalar except for x
function y = crossfade(z0, z1, x, x_low, x_high)
    y = mix(z0, z1, (x - x_low)/max(x_high - x_low, 1.0e-6));
end

function y = mix(x0, x1, c_in)
    c = saturate(c_in, 0.0, 1.0);
    y = (1.0 - c)*x0 + c*x1;
end

function y = saturate(x, low, high)
    y = min(max(x, low), high);
end
