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

function [H, R, dy] = ins_output_pr(p, xhat, sv, obs)
s = p.states;

% We compute the line-of-sight vectors such that R_ie represents the relative
% rotation over the transmission delay. The relative rotation for the user
% position is then zero by definition.

% Antenna offset.
r_bgps_b = p.geometry.r_bgps_b;

% Compute the measurement sensitivity matrix.
H = zeros(1, s.count);
H(:, s.THETA_BI) = sv.los_eci * xhat.R_eb * vcross(r_bgps_b);
H(:, s.R_IB_E) = -sv.los_eci;
H(:, s.CB_PHASE) = 1;
H(:, s.GPS_PR(obs.prn)) = 1;

% Compute the measurement residual.
y = obs.pr;
yhat = sv.pr + xhat.gps_pr(obs.prn);
dy = y - yhat;

% Compute the measurement covariance.
R = p.gps.sigma_pr^2;
