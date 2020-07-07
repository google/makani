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

function find_fields(s, expr, prefix)
% find_fields -- Recursively search struct for field names matching regexp.
%
% find_fields(my_struct, expression)
% Recursively search a structure and its substructures for fields matching
% a given regular expression, printing out the names of matching fields.
%
% my_struct: The root structure to search.
% expression: The regular expression to match.
%
% Example:
%   data = load_h5log('logs/last.h5')
%   find_fields(data, 'winch')

if nargin == 2
  prefix = inputname(1);
  if isempty(prefix)
    prefix = '<anonymous>';
  end
end

f = fields(s);
for ii = 1:length(f)
  if ~isempty(regexp(f{ii}, expr, 'once'))
    h = regexprep(f{ii}, ['(' expr ')'], '<strong>$1</strong>');
    fprintf('%s.%s\n', prefix, h);
  end
end

for ii = 1:length(f)
  if isstruct(s.(f{ii}))
    find_fields(s.(f{ii}), expr, [prefix '.' f{ii}]);
  end
end
