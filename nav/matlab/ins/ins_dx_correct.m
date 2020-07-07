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

function [dx, out] = ins_dx_correct(p, dx, H, R, dy, chi2, time, out, mm)
if ~dx.valid
  return
end

% Disable unselected states.
H(:, p.states.nselect) = 0;

% Compute innovation (dz) and correction (dx).
Pzz = H * dx.Pxx_minus * H' + R;
invPzz = inv(Pzz);
dz = dy - H * dx.dx_minus;
K = dx.Pxx_minus * H' * invPzz;
K(p.states.nselect, :) = 0;
dx_plus = K * dz;

% To prevent trajectory divergence as a result of rejecting all measurements
% through our innovation likelihood test, we consider the mean of past
% innovations. We expect the mean to be zero. It is possible, however, that
% our estimate has diverged and the mean is no longer zero. In this case, we
% consider +/- 3 sigma from that mean as valid.
if length(dx.innov) < mm || ~isfield(dx.innov{mm}, 't')
  dx.innov{mm}.t = [];
  dx.innov{mm}.dz = [];
end
min_points = 5;
max_age = 10;
i = length(dx.innov{mm}.t) + 1;
dx.innov{mm}.t(i) = dx.t;
dx.innov{mm}.dz(i, :) = dz';
ii = find(dx.innov{mm}.t < dx.t - max_age);
dx.innov{mm}.t(ii) = [];
dx.innov{mm}.dz(ii, :) = [];
avg_dz = mean(dx.innov{mm}.dz, 1)';
points = length(dx.innov{mm}.t);

% Test innovation likelihood.
like2 = dz' * invPzz * dz;
like2_from_mean = (dz - avg_dz)' * invPzz * (dz - avg_dz);
valid = dx.valid && (chi2 < 0 || like2 < chi2 ...
                     || (points > min_points && like2_from_mean < chi2));

% Apply correction.
% TODO: Implement MRP shadow parameters to avoid potential singularity.
if valid
  I = eye(size(dx.Pxx_minus));
  dx.dx_minus = dx.dx_minus + dx_plus;
  dx.Pxx_minus = (I - K * H) * dx.Pxx_minus * (I - K * H)' + K * R * K';
end

% Log innovation and correction for analysis.
out = ins_log_innovation(Pzz, dz, avg_dz, dx_plus, like2, valid, time, out, mm);

% Abort execution if invalid.
Pxx_minus = dx.Pxx_minus(p.states.select, p.states.select);
if min(eig(Pxx_minus)) <= 0
  error('Invalid covariance matrix (min eigenvalue=%g).', min(eig(Pxx_minus)));
end
