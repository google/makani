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

function [dx, handle] = ins_dx_push(dx, timeout, time)
% Push new error state transition matrix into propagation routines.

% Generate a unique handle.
handle = rand();
while ~isempty(find(dx.Phi_handle == handle))
  handle = rand();
end

% Insert new Phi matrix.
% Phi_{k, 0} = Phi_{k, k-1} * Phi_{k-1, k-2} * ... * Phi_{1, 0}.
Phi_k0 = eye(size(dx.Pxx_minus));
if ~isempty(dx.Phi_k0)
  Phi_k0 = cat(3, Phi_k0, dx.Phi_k0);
end
dx.Phi_k0 = Phi_k0;
dx.Phi_handle = [handle, dx.Phi_handle];
dx.Phi_timeout = [timeout, dx.Phi_timeout];
dx.Phi_time = [time, dx.Phi_time];
dx.Phi_count = [0, dx.Phi_count];
