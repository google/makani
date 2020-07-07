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

function [dx, inputs] = ins_dx_propagate(p, xhat, dx_1, xhat_1, gps, data)
dx = dx_1;
dx.t = xhat.t;

% Linearize system.
[Phi, Qd, inputs] = ins_dx_system(p, xhat, gps, data);

% Propagate error state.
dx.dx_minus = Phi * dx_1.dx_minus;
dx.Pxx_minus = Phi * dx_1.Pxx_minus * Phi' + Qd;
if length(dx_1.Phi_count)
  % Phi_{k, 0} = Phi_{k, k-1} * Phi_{k-1, k-2} * ... * Phi_{1, 0}.
  dx.Phi_k0(:, :, 1) = Phi * dx_1.Phi_k0(:, :, 1);
  dx.Phi_count(1) = dx_1.Phi_count(1) + 1;
end
