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

function out = ForwardBackwardFilter(t,u,sys,derivoption)
% Apply the filter specified by sys to the signal u(t).
% If derivoption == 1, also take a finite difference time derivative.

% Space the time evenly
t = linspace(0,t(end)-t(1),numel(t));

% Forward filter
out = lsim(sys,u,t);
% Backward filter
out = lsim(sys,out(end:-1:1),t);
% Reoganize front to back
out = out(end:-1:1);

% Reshape the time array
t = reshape(t,size(out));

% Take a forward difference time derivative if necessary
for idx = 1:derivoption
   out = diff(out)./diff(t);
   out = out([1:end end]);
end