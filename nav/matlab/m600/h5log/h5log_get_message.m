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

function [out] = h5log_get_message(in, index)

n = length(in.capture_header.tv_sec);
out = get_message(in, n, index);

function [out] = get_message(in, n, index)

field_names = fields(in);
for f = 1:length(field_names)
  field = field_names{f};
  s = size(in.(field));
  if isstruct(in.(field))
    out.(field) = get_message(in.(field), n, index);
  elseif s(1) == n
    out.(field) = in.(field)(index);
  elseif s(2) == n
    out.(field) = in.(field)(:, index);
  elseif s(3) == n
    out.(field) = in.(field)(:, :, index);
  end
end
