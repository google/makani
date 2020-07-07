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

function [dx] = ins_dx_timeout(dx)
elapsed = dx.t - dx.Phi_time;
ii = 1:find(elapsed < dx.Phi_timeout, 1, 'last');
dx.Phi_k0 = dx.Phi_k0(:, :, ii);
dx.Phi_handle = dx.Phi_handle(ii);
dx.Phi_timeout = dx.Phi_timeout(ii);
dx.Phi_time = dx.Phi_time(ii);
dx.Phi_count = dx.Phi_count(ii);
