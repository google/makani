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

function x = Saturate(x, low, high)
% SATURATE   Restrict a value to a particular interval.
%
% Y = SATURATE(X, LOW, HIGH) replaces values in X outside of the interval
% [LOW, HIGH] with LOW or HIGH.
%
% This directly emulates the behavior of the Saturate function in the
% Makani C library (common/c_math_util.h).
x(x > high) = high;
x(x < low) = low;