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

function y = Crossfade(y0, y1, x, x_low, x_high)
% CROSSFADE   Fade from Y0 to Y1 as X goes from X_LOW to X_HIGH.
%
% Y = CROSSFADE(Y0, Y1, X, X_LOW, X_HIGH)
%
% Args:
%   Y0: Signal to use when x <= x_low.
%   Y1: Signal to use when x > x_high.
%   X: Control value used to set the fraction of each signal.
%   X_LOW: The threshold for x below which y0 is used completely.
%   X_HIGH: The threshold for x above which y1 is used completely.
%
% Returns:
%   If x <= x_low, returns y0.  If x > x_high, returns y1.
%   Otherwise, returns a linear combination of y0 and y1 based on the
%   fraction of the distance of x between x_low and x_high.
%
% Crossfade is similar to MATLAB's INTERP1 except that (1) it only
% interpolates between two points; and (2) values are saturated outside of
% the given interval, where interp1 will either extrapolate or return NaN.
%
% This directly emulates the behavior of the Crossfade function in the
% Makani C library (common/c_math_util.h).
y = Mix(y0, y1, (x - x_low) ./ (x_high - x_low));