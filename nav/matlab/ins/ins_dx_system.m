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

function [Phi, Qd, inputs] = ins_dx_system(p, xhat, gps, imu)
s = p.states;
u = p.inputs;

% Compute general inertial error state.
[A, B, Q, inputs] = ins_dx_system_inertial(p, xhat, imu);

% Compute wheel radius error state.
A(s.WHEEL_RADIUS, s.WHEEL_RADIUS) = -eye(4) * diag(1 ./ p.wheel.tc_radius);
B(s.WHEEL_RADIUS, u.WHEEL_RADIUS) = eye(4);
Q(u.WHEEL_RADIUS, u.WHEEL_RADIUS) = eye(4) * diag(p.wheel.qc_radius.^2);

% Compute GPS clock phase error state.
A(s.CB_PHASE, s.CF_BIAS) = 1;
A(s.CB_PHASE, s.CF_WALK) = 1;
B(s.CB_PHASE, u.CLOCK_N) = 1;
Q(u.CLOCK_N, u.CLOCK_N) = p.clock.sigma_N^2;

% Compute GPS clock frequency instability error state.
A(s.CF_BIAS, s.CF_BIAS) = -1 / p.clock.tc_freq;
B(s.CF_BIAS, u.CLOCK_B) = 1;
Q(u.CLOCK_B, u.CLOCK_B) = p.clock.qc_freq^2;

% Compute GPS clock frequency random walk state.
B(s.CF_WALK, u.CLOCK_K) = 1;
Q(u.CLOCK_K, u.CLOCK_K) = p.clock.sigma_K^2;

% Compute GPS pseudo-range error state.
A(s.GPS_PR, s.GPS_PR) = -eye(32) * diag(1 ./ gps.tc_ura);
B(s.GPS_PR, u.GPS_PR) = eye(32);
Q(u.GPS_PR, u.GPS_PR) = eye(32) * diag(gps.qc_ura.^2);

% Select states.
A(s.nselect, s.nselect) = 0;
B(s.nselect, u.nselect) = 0;
Q(u.nselect, u.nselect) = 0;

% Compute discrete time system.
dt = imu.dt;
Phi = eye(size(A)) + A*dt + (A*dt)^2/2;
Qd = B*Q*B'*dt + 1/2*A*B*Q*B'*(dt)^2 + 1/2*B*Q*B'*A'*(dt)^2 ...
    + 1/3*A*B*Q*B'*A'*(dt)^3;
