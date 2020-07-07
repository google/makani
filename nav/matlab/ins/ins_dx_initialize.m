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

function [dx] = ins_dx_initialize(Pxx_minus, dx_minus, start_time, valid)
dx = struct();
dx.t = start_time;
dx.Pxx_minus = Pxx_minus;  % Covariance.
dx.dx_minus = dx_minus;    % Mean.
dx.Phi_k0 = [];       % Array of state transition matrices from time k to 0.
dx.Phi_handle = [];   % Array of unique handles.
dx.Phi_timeout = [];  % Array of timeouts.
dx.Phi_time = [];     % Array of start times.
dx.Phi_count = [];    % Array of iteration counts.
dx.innov = {};
dx.valid = valid;
