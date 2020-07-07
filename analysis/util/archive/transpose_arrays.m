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

function xt = transpose_arrays(x)
%  TRANSPOSE_ARRAYS   Recursively traverse a structure and transpose arrays
%   to column vectors. This only works if the record length is longer than
%   the maximum array dimension.
    if ~isstruct(x),
        xt = x';
        return;
    end
    xt = x;
    fields = fieldnames(x);
    for i = 1:numel(fields)
        f = x.(fields{i});
        size_f = size(f);
        if isstruct(f)
            xt.(fields{i}) = transpose_arrays(f);
        elseif size_f(2) == length(f)
            xt.(fields{i}) = f';
        end
    end
end
