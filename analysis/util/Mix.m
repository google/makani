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

function x = Mix(x0, x1, c)
% MIX   Fade from X0 to X1 as C goes from 0 to 1.
%
% X = MIX(X0, X1, C) implements the mixing function:
%
%          | x0                     if c <= 0
%   f(c) = | (1 - c) * x0 + c * x1  if 0 < c < 1
%          | x1                     if c >= 1
%
% This directly emulates the behavior of the Mix function in the
% Makani C library (common/c_math_util.h).

c_sat = Saturate(c, 0, 1);
x = (1 - c_sat) .* x0 + c_sat .* x1;