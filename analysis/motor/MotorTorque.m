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

function [torque] = MotorTorque(id, iq, params)
% MotorTorque -- Calculate motor torque from direct and quadrature current.
%
% [torque] = MotorTorque(id, iq, params)
%
% Arguments:
%
% id:     n x m matrix of direct currents.
% iq:     n x m matrix of quadrature currents.
% params: Motor parameter structure.
%
% Return Values:
%
% torque: n x m matrix of motor electromechanical torque.
  Ld = params.Ld;
  Lq = params.Lq;
  npp = params.npp_elec;

  % Calculate the flux linkage as a function of phase current.
  lambda = polyval(params.lambda(end:-1:1), hypot(id, iq));

  % Calculate torque.
  torque = 1.5 * npp * (lambda .* iq + (Ld - Lq) * id .* iq);

  % TODO: Add correction for dq-frame shift.
  % TODO: Add alternate methods of calculating torque.
end
