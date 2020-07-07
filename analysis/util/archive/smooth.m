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

function y = smooth(U, n, shape, dim)
% SMOOTH   n-point average (with no phase lag) of the rows or columns of U
%
%   U is the input
%   n is the number of points to average over
if nargin < 3 || isempty(shape)
    shape = 'same';
end

if nargin < 4
    dim = find(size(U) ~= 1, 1);
    if isempty(dim)
        dim = 1;  % default to smoothing along columns
    end
end

win = ones(round(n), 1)/round(n);  % create averaging window
if dim == 1
    y = conv2(win, 1, U, shape);
elseif dim == 2
    y = conv2(1, win, U, shape);
else
    error('dim must be <= 2');
end
