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

function [message_struct] = GenerateDerivedValue(message_struct, var_math, ...
                            motor_list)
% GenerateDerivedValue - Generate new field for plotting based on math
%
% [message_struct] = GenerateDerivedValue(message_struct, descriptor)
% Build and add new variable to message for plotting purposes.  Will overwrite
% variable if previously generated to ensure we don't reuse names with different
% math.  In the future, we could assume same definitions and check for
% existence.
%
% Note: Doesn't currently support data which is in arrays or if it does, it's
% only by accident.
%
% Arguments
%
% message_struct: Message generated from H5 read from Makani flight data.
% var_math: Description of what's needed to generate new variable.
%    .math: Anonymous function string describing calculation of new value.
%    .input: Ordered cell array of strings with variable names.
%    .output: Name of variable to create.
%    .header: Default is 'message'
% motor_list: Array of motor indeces.
%
% Return value
%
% message_struct: message_struct with additional data added.
%
% Required toolboxes: None.
%

% Short circuit out if no variable needs to be generated
if isempty(var_math)
  return
end

if isempty(motor_list)
  motor_list = 1:8;
end

% Generate math expression
f = str2func(var_math.math);

% Verify source variables exist
if (isa(var_math.input, 'cell'))
  vars = cell(length(var_math.input), max(motor_list));
  i = 0;
  for plot_var_i = var_math.input
    i = i+1;
    for j = motor_list
      if (isfield(var_math, 'header') && (ischar(var_math.header)))
        if (isempty(var_math.header))
          [stat, vars{i,j}] = IsDeepField(message_struct{j}, ...
                                      [plot_var_i{1}]);
        else
          [stat, vars{i,j}] = IsDeepField(message_struct{j}, ...
                                      [var_math.header{i}, '.', plot_var_i{1}]);
        end
      else
        [stat, vars{i,j}] = IsDeepField(message_struct{j}, ...
                                      ['message.',plot_var_i{1}]);
      end
      if ~stat
        disp(['Field not available: ', plot_var_i{1}]);
        return;
      end
    end
  end

else
  vars = cell(1, max(motor_list));

  for j = motor_list
    if (isfield(var_math, 'header') && (ischar(var_math.header)))
      if (isempty(var_math.header)) % '' Lets us access the counter
        [stat, vars{j}] = IsDeepField(message_struct{j}, ...
                                 [var_math.input]);
      else
        [stat, vars{j}] = IsDeepField(message_struct{j}, ...
                                 [var_math.header, '.', var_math.input]);
      end
    else % Default to using message
      [stat, vars{j}] = IsDeepField(message_struct{j}, ...
                                 ['message.', var_math.input]);
    end
    if ~stat
      disp(['Field not available: ', plot_var]);
      return;
    end
  end

end

% Generate new variable
for i = motor_list
      message_struct{i}.message.(var_math.output) = ...
          feval(f, vars{:,i});
end
end


 % Test if varString indicates a variable in the structure
function [status, var] = IsDeepField(dataStruct, varString)
  status = false;
  var = [];
  tokens = strsplit(varString, '.');
  testStruct = dataStruct;
  for t = tokens
    if (~isfield(testStruct, t{1}))
      return;
    end
    testStruct = testStruct.(t{1});
  end
  status = true;
  var = testStruct;
  return;
end
