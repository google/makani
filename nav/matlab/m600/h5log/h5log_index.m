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

function [out] = h5log_index(in)
[out, time_id] = build_index(in, [], 1);
[~, ii] = sort(time_id(:, 1));
out.t_unix = time_id(ii, 1);
out.order = time_id(ii, 2);


function [out, time_id, id] = build_index(in, time_id, id)
out = [];
field_names = fields(in);
for f = 1:length(field_names)
  field = field_names{f};
  if isstruct(in.(field))
    if isfield(in.(field), 'capture_header')
      out.(field) = id;
      time = h5log_get_time(in.(field))';
      time_id = [time_id; time, ones(size(time)) * id];
      id = id + 1;
    else
      [out_field, time_id, id] = build_index(in.(field), time_id, id);
      if ~isempty(out_field)
        out.(field) = out_field;
      end
    end
  end
end
