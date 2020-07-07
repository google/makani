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

function [compiled_data_structure] = append_to_data_structure (compile_into_this_data_structure, append_this_data) 
% append_to_data_structure -- appends data to existing structure without overwriting at hierarchical location 
% [compiled_data_structure] = append_to_data_structure (compile_into_this_data_structure, append_this_data)
%
% Arguments-
%
% compile_into_this_data_structure: the structure data needs to be appended into
% append_this_data: new data that needs to be appended
%
% Return Values-
%
% compiled_data_structure: structure same as compile_into_this_data_structure with data at hierarchical location from append_this_data  
%

data_read = append_this_data;
messages = compile_into_this_data_structure;

% this nested loop cycles down to the sensor type to write the data for various locations
node_types = fieldnames (data_read);
for ii = 1:numel(node_types);
  if isempty (['data_read.' node_types{ii,1}]) ~= 1
    msg_types = eval(['fieldnames(data_read.' node_types{ii,1} ')']);
    for jj = 1:numel(msg_types);
      if isempty (['data_read.' node_types{ii,1} '.' msg_types{jj,1}]) ~= 1
        sen_types = eval(['fieldnames(data_read.' node_types{ii,1} '.' msg_types{jj,1} ')']);
        for kk = 1:numel(sen_types);
        	if isempty (['data_read.' node_types{ii,1} '.' msg_types{jj,1} '.' sen_types{kk,1} ]) ~= 1
        		sen_locs = eval(['fieldnames(data_read.' node_types{ii,1} '.' msg_types{jj,1} '.' sen_types{jj,1} ')']);
        		for ll = 1:numel(sen_locs);
        			vrnm = [node_types{ii,1} '.' msg_types{jj,1} '.' sen_types{kk,1} '.' sen_locs{ll,1}];
        			eval (['messages.' vrnm '= data_read.' vrnm ';']);
        		end
        	end
        end
      end
    end
  end
end
clear data_read ii jj kk;

compiled_data_structure = messages;
