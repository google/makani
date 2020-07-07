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

function [] = save_all_figures(format, directory)
if ~exist('format', 'var') || isempty(format)
  format = 'fig';
end
if ~exist('directory', 'var') || isempty(directory)
  directory = '.';
end

handles = get(0, 'children');
for h = 1:length(handles)
  handle = handles(h);
  filename = get_filename(handle, directory);
  if ~exist(directory, 'dir')
    mkdir(directory);
  end
  saveas(handle, filename, format);
end


function [filename] = get_filename(handle, directory)
name = get(handle, 'Name');
name = strrep(name, '/', '');
number = get(handle, 'Number');
filename = sprintf('%s/Fig %02d: %s', directory, number, name)
