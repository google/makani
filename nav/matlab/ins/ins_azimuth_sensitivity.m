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

function [out] = ins_azimuth_sensitivity(data, time_interval)
if ~exist('time_interval', 'var')
  time_interval = [0; Inf];
end

d2r = pi / 180;
r2d = 180 / pi;

out = {};
steps = 8;
for step = 1:steps
  angle = (step - ceil(steps / 2)) * (360 / steps) * d2r;
  fprintf(1, 'Assuming initial azimuth angle of %.0f degrees\n', angle * r2d);

  p = ins_param(data, angle);
  [~, out{step}] = ins_main(p, data, time_interval);
end
