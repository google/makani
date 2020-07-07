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

function [jump_msec] = gps_jump(tow_1, tow)
% Minimum update period (msec).
period_msec = 20;
jump_msec = mod((tow - tow_1) * 1000, period_msec);
if jump_msec > period_msec / 2
  jump_msec = jump_msec - period_msec;
end
if abs(jump_msec) ~= 1
  jump_msec = 0;
end
