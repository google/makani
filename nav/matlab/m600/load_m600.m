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

function [out] = load_m600(filename)
out = {};
files = dir(filename);
for f = 1:length(files)
  filename = files(f).name;
  fprintf('Loading %s...\n', filename);
  out{f} = load_single(filename);
end


function [data] = load_single(filename)
% Load HDF5 log into a usable form.
data = load_h5log(filename);
data = h5log_fix_arrays(data);
data = h5log_dedup(data);
index = h5log_index(data);
t_offset = index.t_unix(1);
data = h5log_strip(data, t_offset);
data = cast_float(data);
data.t_offset = t_offset;
data.type = index;
data.filename = filename;

% Improve data format.
data = h5log_fix_gps(data);
