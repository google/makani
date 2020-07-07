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

function [spec] = adis16488_spec(dt)
mg = 1000 / 9.80665;
d2r = pi / 180;
hr = 3600;
tau0 = 1e-2;
tau1 = 1e4;
spec.dt = dt;

% Accelerometer datasheet specification.
Q = 1.221e-8 * 9.80665 / sqrt(12);
N = 0.029 * 1/sqrt(hr);
B = 0.07 / sqrt(2*log(2)/pi) * 1/mg;
K = 0;
R = 0;
spec.accel = imu_spec(dt, Q, N, B, K, R, tau0, tau1);

% Gyro datasheet specification.
Q = 3.052e-7 * d2r / sqrt(12);
N = 0.26 * d2r/sqrt(hr);
B = 5.1 / sqrt(2*log(2)/pi) * d2r/hr;
K = 0;
R = 0;
spec.gyro = imu_spec(dt, Q, N, B, K, R, tau0, tau1);

% Magnetometer datasheet specification.
Q = 0.1 * 1e-3 / sqrt(12);
N = 0.04 * 1e-3;
B = 0;
K = 0;
R = 0;
spec.mag = imu_spec(0.01, Q, N, B, K, R, tau0, tau1);
