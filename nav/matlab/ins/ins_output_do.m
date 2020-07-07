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

function [H, R, dy] = ins_output_do(p, xhat, inputs, sv, obs)
s = p.states;

% We compute the line-of-sight vectors such that R_ie represents the relative
% rotation over the transmission delay. The relative rotation for the user
% position is then zero by definition.

% Antenna offset.
r_bgps_b = p.geometry.r_bgps_b;

Omega_ie_e = vcross(xhat.omega_ie_e);
Omega_eb_b = vcross(inputs.omega_eb_b);

% Compute the measurement sensitivity matrix.

H_r = zeros(3, s.count);
H_r(:, s.THETA_BI) = -xhat.R_eb * vcross(r_bgps_b);
H_r(:, s.R_IB_E) = eye(3);

H_dr = zeros(3, s.count);
H_dr(:, s.THETA_BI) = -Omega_ie_e * xhat.R_eb * vcross(r_bgps_b) ...
    - xhat.R_eb * vcross(Omega_eb_b * r_bgps_b);
H_dr(:, s.R_IB_E) = Omega_ie_e;
H_dr(:, s.V_IB_E) = eye(3);
H_dr = H_dr - xhat.R_eb * vcross(r_bgps_b) * inputs.H_omega_eb_b;

H = -sv.dlos_eci * H_r - sv.los_eci * H_dr;
H(:, s.CF_BIAS) = 1;
H(:, s.CF_WALK) = 1;

% Compute the measurement residual.
y = -obs.do * obs.lambda;
yhat = sv.do;
dy = y - yhat;

% Compute the measurement covariance.
R = p.gps.sigma_do^2;
