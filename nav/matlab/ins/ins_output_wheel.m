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

function [H, R, dy] = ins_output_wheel(p, xhat, inputs, r_bw_b, omega, wheel)
s = p.states;

% Compute wheel radius estimate.
radius = p.geometry.wheel_radius + xhat.dradius(wheel);

% Specify velocity unit vector in body frame.
u_b = [1; 0; 0];

% Compute velocity of wheel hub.
u_e = xhat.R_eb * u_b;
v_iw_e = xhat.v_ib_e + xhat.R_eb * cross(inputs.omega_eb_b, r_bw_b);
v_e = u_e' * v_iw_e;

% Compute measurement sensitivity matrix.
H = zeros(1, s.count);
H(:, s.THETA_BI) = (-xhat.R_eb * vcross(u_b))' * v_iw_e / radius;
H(:, s.V_IB_E) = (u_e' ...
    - u_e' * xhat.R_eb * vcross(cross(inputs.omega_eb_b, r_bw_b))) / radius;
H(:, s.WHEEL_RADIUS(wheel)) = -v_e / radius^2;
H = H - u_e' * xhat.R_eb * vcross(r_bw_b) * inputs.H_omega_eb_b / radius;

% Compute measurement residual.
y = omega;
yhat = v_e / radius;
dy = y - yhat;

% Compute measurement covariance.
R = p.wheel.sigma_omega^2;
