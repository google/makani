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

function [xhat] = ins_xhat_interpolate(p, xhat0, xhat1, t)
r = (t - xhat0.t) / (xhat1.t - xhat0.t);
dx = ins_dx_compute(p, xhat0, xhat1);
xhat = ins_xhat_correct(p, xhat0, r * dx);
xhat.t = t;
