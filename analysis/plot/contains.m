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

function bool = contains(lst1, lst2)
%  CONTAINS   Returns true if lst2 contains one of the strings in lst1.
%   Grouping strings in lst2 acts as an AND operator such that lst1 must
%   contain both of those strings.
%
%   Examples:
%       contains({'a', 'b', 'c', 'd'}, {'a', 'e'}) returns true
%       contains({'a', 'b', 'c', 'd'}, {{'b', 'e'}}) returns false
%       contains({'a', 'b', 'c', 'd'}, {{'b', 'c'}, 'f'}) returns true
%       contains({'a', 'b', 'c', 'd'}, {{'b', 'e'}, 'd'}) returns true
    if isstr(lst1), lst1 = {lst1}; end
    if isstr(lst2), lst2 = {lst2}; end
    bool = any(cellfun(@(x) contains_helper(lst1, x), lst2));
end


function bool = contains_helper(lst1, lst2)
%  CONTAINS_HELPER   Returns true if lst1 contains all the strings in lst2
    if ~iscell(lst2), lst2 = {lst2}; end
    bool = all(cellfun(@(s1) any(cellfun(@(s2) strcmpi(s1, s2), lst1)), lst2));
end
