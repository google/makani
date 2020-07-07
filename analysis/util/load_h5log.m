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

function data = load_h5log(filename)
% load_h5log -- Load M600 HDF5 logs into a struct.
%
% data = load_h5log(filename)
% Load M600 HDF5 logs into a struct. The file extension (.h5) can be
%     omitted from the filename.
%
% Examples:
%     data = load_h5log('20130307-143445.h5');
%     data = load_h5log('20130307-143445');
%
% Arguments
%
% filename: A string containing path to the HDF5 file.
%
% Return Values
%
% data: A struct, structured according to the following rules:
%     /parameters/<param> ==> data.parameters.<param>
%     /messages/kAioNode<node>/kMessageType<msg> ==> data.<node>.<msg>

  % Append file extension if one is ommitted.
  ext = '.h5';
  indices = strfind(filename, ext);
  if ~any(indices == length(filename) - length(ext) + 1)
    filename = [filename, ext];
  end

  % Load h5 into a containers.Map indexed by full h5 path name.
  datasets = h5load(filename, '/');

  % Reformat the map into a struct with appopriate renaming.
  keys = datasets.keys();
  data = struct();
  for k = 1:length(keys)
    key = keys{k};
    value = datasets(key);
    field_path = get_field_path(key);
    if ~isempty(field_path)
      data = setfield(data, {1}, field_path{:}, {1}, value);
    end
  end
  make_naming_shortcuts(data);
end


function field_path = get_field_path(key)
  field_path = {};
  section = '/parameters';
  idx = strfind(key, section);
  if any(idx == 1)
    % /parameters/<param_name> ==> data.parameters.<param_name>
    slash_pos = strfind(key, '/');
    field_name = key(slash_pos(length(slash_pos)) + 1 : length(key));
    field_path{1} = 'parameters';
    field_path{2} = field_name;
  end
  section = '/messages/kAioNode';
  idx = strfind(key, section);
  if any(idx == 1)
    % /messages/kAioNode<node>/kMessageType<msg> ==> data.<node>.<msg>
    subsection = '/kMessageType';
    sub_idx = strfind(key, subsection);
    if ~isempty(sub_idx)
      slash_pos = strfind(key, '/');
      node_name = key(length(section) + 1:sub_idx(1)-1);
      msg_type = key(slash_pos(length(slash_pos)) ...
                     + length(subsection) : length(key));
      field_path{1} = node_name;
      field_path{2} = msg_type;
    end
  end
end
