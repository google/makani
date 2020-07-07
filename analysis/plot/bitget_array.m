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

function bitarray  = bitget_array(data, bits0, offset_scale)
%  BITGET_ARRAY   Creates a matrix of bits from a vector of integers
%
%   BITGET_ARRAY(DATA, BITS0, OFFSET_SCALE)
%   DATA = (n,1) uint column vector
%   BITS0 = an array of bit locations to return (zero indexed)
%   OFFSET_SCALE = for ease of plotting each bit array is offset by the bit
%                location * ofset_scale (default is 0, recommend 0.1 for
%                plots)
% 
%   Example:
%   data = uint8([1 1 1 1 1 2 2 2 2 2 3 3 3 3 3]');
%   bitarray = bitget_array(data, [0 1 2 3], 0.1)
%   plot(bitarray)
if nargin < 3
    offset_scale = 0;
end

nm = [size(data,1), length(bits0)];

% Initialize logical array
if offset_scale == 0
    bitarray = false(nm);
else
    bitarray = single(zeros(nm));
end

% Typecast to unsigned if needed (bitget only takes unsigned arguments)
types = {'int8', 'int16', 'int32'};
for k = 1:length(types)
    if isa(data, types{k})
        data = typecast(data, ['u', types{k}]);
    end
end

for k = 1:length(bits0)
    bitarray(:,k) = single(bitget(data, bits0(k))) + offset_scale*bits0(k);
end
