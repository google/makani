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

function [data] = ins_cbuf_unwrap(cbuf)
if isfield(cbuf, 'data')
  len = length(cbuf.data);
  tail = 1 + mod(cbuf.head-1, len);
  data(1:len-tail+1) = cbuf.data(tail:len);
  data(len-tail+2:len) = cbuf.data(1:tail-1);
else
  data = [];
end
