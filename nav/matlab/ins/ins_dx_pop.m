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

function [dx, Phi_k0] = ins_dx_pop(dx, handle)
% Pop propagated error state transition matrix from propagation routines.
Phi_k0 = [];
if ~isempty(handle)
  idx = find(dx.Phi_handle == handle);
  if ~isempty(idx)
    % Phi_{k, 0} = Phi_{k, k-1} * Phi_{k-1, k-2} * ... * Phi_{1, 0}.
    Phi_k0 = prod(dx.Phi_k0(:, :, 1:idx), 3);
    dx.Phi_timeout(idx) = 0;
  end
end
