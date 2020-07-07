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

function m = s2m(s)
%  S2M  Converts a struct of arrays to a matrix.
%   M = S2M(S) converts a struct with N fields each with an M element vector
%   to an NxM matrix.
%
%    Example:
%        eulers_rpy_mat = s2m(c.state_est.eulers_rpy);
    m = cell2mat(struct2cell(structfun(@(x) x', s, 'UniformOutput', false)));
end
