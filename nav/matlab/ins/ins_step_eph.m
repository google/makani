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

function [ins, out] = ins_step_eph(ins, xhat_1, data, out)
ins.gps.eph{data.prn} = data;

sigma = sqrt(data.ura^2 - ins.param.gps.sigma_pr^2);
[qc_ura, tc_ura] = ins_bi_model(sigma, ins.param.gps.pr_bias_tau);
ins.gps.qc_ura(data.prn) = qc_ura;
ins.gps.tc_ura(data.prn) = tc_ura;
