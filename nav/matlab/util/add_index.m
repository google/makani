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

function [p] = add_index(p, count, symbol, name, unit, scale)
if ~exist('name', 'var') || isempty(name)
  name = symbol;
end
if ~exist('unit', 'var') || isempty(unit)
  unit = '#';
end
if ~exist('scale', 'var') || isempty(scale)
  scale = 1;
end
if ~isfield(p, 'symbol')
  p.symbol = {};
  p.symbol_escaped = {};
  p.name = {};
  p.name_escaped = {};
  p.unit = {};
  p.scale = [];
  p.select = [];
  p.count = 0;
end
offset = p.count;
p.count = offset + count;
p.(upper(symbol)) = (offset + 1):(offset + count);
for i = 1:count
  if count > 1
    p.name{offset + i} = sprintf('%s(%d)', name, i);
    p.symbol{offset + i} = sprintf('%s(%d)', symbol, i);
  else
    p.name{offset + i} = name;
    p.symbol{offset + i} = symbol;
  end
  p.name_escaped{offset + i} = escape_name(p.name{offset + i});
  p.symbol_escaped{offset + i} = escape_name(p.symbol{offset + i});
  p.unit{offset + i} = unit;
  p.scale(offset + i) = scale(min(length(scale), i));
  p.select = [p.select, offset + i];
end
p.nselect = 1:p.count;
p.nselect(p.select) = [];
p.map = zeros(p.count, 1);
p.map(p.select) = 1:length(p.select);


function [name] = escape_name(name)
name = strrep(name, '\', '\\');
name = strrep(name, '_', '\_');
