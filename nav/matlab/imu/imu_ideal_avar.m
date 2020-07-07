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

function [sigma, s_Q, s_N, s_B, s_K, s_R] = imu_ideal_avar(tau, model)
var_Q = (model.Q^2) * 3 * tau.^-2;
var_N = (model.N^2) * tau.^-1;
var_B = (model.B^2) * (2 * log(2) / pi) * tau.^0;
var_K = (model.K^2) / 3 * tau.^1;
var_R = (model.R^2) / 2 * tau.^2;
var = var_Q + var_N + var_B + var_K + var_R;

sigma = sqrt(var);
s_Q = sqrt(var_Q);
s_N = sqrt(var_N);
s_B = sqrt(var_B);
s_K = sqrt(var_K);
s_R = sqrt(var_R);
