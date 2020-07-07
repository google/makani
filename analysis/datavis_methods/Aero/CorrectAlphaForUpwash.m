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

function [alpha_corrected] = CorrectAlphaForUpwash(alpha, alpha_scale, alpha_bias)
% CorrectAlphaForUpwash -- apply correction to alpha for wing upwash.
% alpha_corrected = CorrectAlphaForUpwash(alpha, alpha_scale, alpha_bias)
% This directly applies the correction factors available in the h5 logs.
% alpha_corrected = (alpha - alpha_bias)/alpha_scale
%
% Arguments-
%
% alpha       -- [1 x n] angle of attack [rad]
% alpha_scale -- scaling factor for upwash correction
% alpha_bias  -- bias for upwash correction [rad]
%
% Output Values-
%
% alpha_corrected -- [1 x n] angle of attack corrected for upwash [rad]
%

alpha_corrected = (alpha - alpha_bias) / alpha_scale;