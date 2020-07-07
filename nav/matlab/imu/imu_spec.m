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

function [spec] = imu_spec(dt, Q, N, B, K, R, tau0, tau1)
res = 0.01;

spec.Q = Q;
spec.N = N;
spec.B = B;
spec.K = K;
spec.R = R;
spec.tau = 10.^(log10(tau0):res:log10(tau1))';
spec.sigma = imu_ideal_avar(spec.tau, spec);
spec.freq = (0:res:ceil(0.5/dt))';
spec.spectra = imu_ideal_psd(spec.freq, dt, spec);
