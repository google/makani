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

function [dx, dy, out] = ins_correct_dr(p, xhat, dx, Phi_0k, xhat_0, H_0, ...
                                        u_0, obs_0, xhat_k, H_k, u_k, ...
                                        obs_k, jump, out, mm)
s = p.states;

H = H_k - H_0 * Phi_0k;
y = obs_k.cp * obs_k.lambda - obs_0.cp * obs_0.lambda;
yhat = (u_k.cp + xhat_k.gps_pr(obs_k.prn)) ...
    - (u_0.cp + xhat_0.gps_pr(obs_0.prn) + jump);
dy = y - yhat;

R = 0.2^2;

[dx, out] = ins_dx_correct(p, dx, H, R, dy, 3*3, obs_k.t_obs, out, mm);
