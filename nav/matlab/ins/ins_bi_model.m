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

function [qc, Tc] = ins_bi_model(B, B_tau)
% Convert bias instability AV parameters to 1st order Markov approximation.
Tc = B_tau / 1.89;
qc = B * 0.6643 ./ (0.437 * sqrt(Tc));
