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

function y = downsample(x, N, phase)
% DOWNSAMPLE Downsample input signal.
%   This function is re-written since the Signal Processing
%   Toolbox is not available to all of Makani.
%
%   DOWNSAMPLE(X,N) downsamples input signal X by keeping every
%   N-th sample starting with the first. If X is a matrix, the
%   downsampling is done along the columns of X.
%
%   DOWNSAMPLE(X,N,PHASE) specifies an optional sample offset.
%   PHASE must be an integer in the range [0, N-1].

if nargin < 3; phase = 0; end

dim = find(size(x) ~= 1, 1);
if isempty(dim), dim = 1; end

assert(0 <= phase && phase <= N - 1);
assert(mod(N, 1) == 0);
assert(mod(phase, 1) == 0);

I = (phase + 1):N:size(x, dim);
if dim == 1
    y = x(I,:);
elseif dim == 2
    y = x(:,I);
else
    error('Arrays max(size(x))>2 not supported');
end
