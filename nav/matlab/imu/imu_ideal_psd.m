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

function [spec, s_Q, s_N, s_B, s_K, s_R] = imu_ideal_psd(freq, dt, model)
spectra_Q = (model.Q^2) * (2 * pi * freq).^2 * dt;
spectra_N = (model.N^2) * ones(size(freq));
spectra_B = (model.B^2) ./ (2 * pi * freq);
spectra_K = (model.K^2) ./ (2 * pi * freq.^2);
spectra_R = (model.R^2) ./ (2 * pi * freq.^3);
spectra = spectra_Q + spectra_N + spectra_B + spectra_K + spectra_R;

% Convert to single-sided PSD.
spec = 2 * spectra;
s_Q = 2 * spectra_Q;
s_N = 2 * spectra_N;
s_B = 2 * spectra_B;
s_K = 2 * spectra_K;
s_R = 2 * spectra_R;
