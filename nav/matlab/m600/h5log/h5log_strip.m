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

function [out] = h5log_strip(in, t_offset)

field_names = fields(in);
for f = 1:length(field_names)
  field = field_names{f};
  if isstruct(in.(field))
    if isfield(in.(field), 'capture_header')
      out.(field) = h5log_strip_message(in.(field), t_offset);
    else
      out.(field) = h5log_strip(in.(field), t_offset);
    end
  else
    out.(field) = in.(field);
  end
end
