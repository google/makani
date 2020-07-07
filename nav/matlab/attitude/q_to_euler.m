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

function [e] = q_to_euler(b)
e(1, 1) = atan2(2*(b.v(2)*b.v(3) + b.s*b.v(1)), ...
                b.v(3)*b.v(3) - b.v(2)*b.v(2) - b.v(1)*b.v(1) + b.s*b.s);
e(2, 1) = -asin(2*(b.v(1)*b.v(3) - b.s*b.v(2)));
e(3, 1) = atan2(2*(b.v(1)*b.v(2) + b.s*b.v(3)), ...
                b.v(1)*b.v(1) + b.s*b.s - b.v(3)*b.v(3) - b.v(2)*b.v(2));
