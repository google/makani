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

function [y_bar, Pyy, Pxy] = ...
    unscented_transform(x, Pxx, alpha, beta, kappa, f, param)

% Compute sigma points.
nx = size(x,1);
lambda = alpha^2*(nx + kappa) - nx;
chi = unscented_sigma_points(x, Pxx, lambda);
n_sigma_pts = size(chi,2);

% Compute output mean.
w_mean = [lambda/(nx+lambda) 0.5/(nx+lambda)*ones(1, 2*nx)];
Y = f(chi(:,1), param);
ny = size(Y,1);
Y = [Y, zeros(ny, n_sigma_pts-1)];
y_bar = w_mean(1)*Y(:,1);
for i = 2:n_sigma_pts
  Y(:,i) = f(chi(:,i), param);
  y_bar = y_bar + w_mean(i)*Y(:,i);
end

% Compute output covariance.
w_cov = [lambda/(nx+lambda)+(1-alpha^2+beta) 0.5/(nx+lambda)*ones(1, 2*nx)];
Pyy = zeros(ny,ny);
Pxy = zeros(nx,ny);
for i = 1:n_sigma_pts
  dy = Y(:,i) - y_bar;
  dx = chi(:,i) - x;
  Pyy = Pyy + w_cov(i)*(dy*dy');
  Pxy = Pxy + w_cov(i)*(dx*dy');
end
