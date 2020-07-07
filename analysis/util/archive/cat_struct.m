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

function s = cat_struct(s1, s2)
%  CAT_STRUCT   Concatenates the arrays in a nested structure.
%   S = CAT_STRUCT(S1, S2)
%
%   Looks in S2 for the fields found in S1 and and concatonates the arrays to
%   form a structure, S, with the same fields as S1.

s = merge_struct_rec(s1, s2, 1);
end

function s = merge_struct_rec(s, s2, r_depth)

n = fieldnames(s);

% Check that the fields in s1 are contained within s2
if isstruct(s2)
    n2 = fieldnames(s2);
    for i = 1:length(n)
        if ~strcmp(n{i}, n2)
            error_s_not_contained_in_s2(n, n2, n{i});
        end
    end
else
    error_s_not_contained_in_s2(n, '<empty>', n{1});
end

% If the field is a structure, then recurse. If the field is a value,
% then concatonate.
for i = 1:length(n),
    ni = n{i};

    if is_param_struct(ni, r_depth)
        break  % don't concatonate parameter struct
    elseif (isstruct(s.(ni)))
        s.(ni) = merge_struct_rec(s.(ni), s2.(ni), r_depth + 1);
    else
        if ndims(s.(ni)) == 3
            s.(ni) = cat(3, s.(ni), s2.(ni));
        else
            s.(ni) = [s.(ni); s2.(ni)];
        end
    end
end
end


% TODO: instead of this error function, it would be nice to pass the
% failing fieldname up the recursion such that the full structure path to
% the failing field can be displayed at the command prompt
function error_s_not_contained_in_s2(n, n2, ni)
disp('Error:')
disp('The base structure contains these fields:')
disp(n)
disp('The new structure (to be concatonated) contains these fields:')
disp(n2)
disp(['Field: "' ni '" is missing in the new structure'])
error()
end


% recursion_depth protects the names control_params, system_params,
% and sim_params from getting interperated as parameters if the same
% names re-appear deep in the structure.
function bool = is_param_struct(ni, recursion_depth)
bool = any(strcmp({'control_params', 'system_params', 'sim_params'}, ni)) ...
       && recursion_depth == 1;
end
