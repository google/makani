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

function files = get_files(search_path)
% get_files -- Recursively retrieve all files in a given search path.
%
% files = get_files(search_path)
%
% Arguments
%
% search_path: Parent directory to search for files.
%
% Return Values
%
% files: All files under the given search path.

curr_dir_data = dir(search_path);
dir_indices = [curr_dir_data.isdir];
files = {curr_dir_data(~dir_indices).name}';

if ~isempty(files)
  files = cellfun(@(x) fullfile(search_path, x), files, ...
    'UniformOutput', false);
end

sub_dirs = {curr_dir_data(dir_indices).name};
valid_index = ~ismember(sub_dirs, {'.', '..'});

for i = find(valid_index)
  next_dir = fullfile(search_path, sub_dirs{i});
  files = [files; get_files(next_dir)];  %#ok
end
