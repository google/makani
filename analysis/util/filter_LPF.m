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

function Y = filter_LPF(U, fc, ts)
% FILTER_LPF
%   Math identical to the LPF() function used throughout the controller
%   code to create a filter of cut off, Hz_cut.
%   Then uses the matlab filter command to execute it.
%
%   Example:
%   Cut off frequency, fc, is in Hz
%   Sample time, ts, is in seconds
%   >fc = 2.5 %Hz
%   >ts = 0.01
%   >LatAccFlt = filter_LPF(LatAcc,fc,ts)

p_filt = 2*pi*fc*ts;

% create the numerator
B = [p_filt, 0];

% create the denominator
A = [1, p_filt - 1];

% filter input U
Y = filter(B, A, U, U(1,:));

