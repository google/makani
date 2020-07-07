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

function [] = sqrt_kalman_comparison(output)
% sqrt_kalman_comparison -- Generates data using a conventional Kalman filter.
%
% sqrt_kalman_comparison(output)
% Simulates a linear system then filters its outputs using a conventional Kalman
% filter then writes the system description and results to a file.
%
% Arguments
%
% output: The path to write the data to.
%
  kIter = 100;
  nx = 4;
  nz = 3;
  nw = 2;
  nu = 1;

  F = randn(nx, nx);
  F = F/max(abs(eig(F)));  % Make a marginally stable system.
  G = randn(nx, nw);
  B = randn(nx, nu);
  H = randn(nz, nx);
  T = randn(nw, nw);  % Square root of process noise covariance.
  U = randn(nz, nz);
  R = U'*U;
  U = chol(R); % Square root of measurement noise covariance.

  % Simulate the system to get a test data set.
  z = NaN*ones(nz, kIter);
  x = NaN*ones(nx, kIter);
  x(:, 1) = rand(nx, 1);
  u = randn(nu, kIter);
  for i = 1:kIter-1,
    x(:, i+1) = F*x(:, i) + G*T*randn(nw, 1) + B*u(:, i);
    z(:, i) = H*x(:, i+1) + U*randn(nz, 1);
  end
  x_mean = B*u;

  Q = T'*T;
  P = NaN*ones(nx, nx, kIter);
  K = NaN*ones(nx, nz, kIter);
  x_hat = NaN*ones(nx, kIter);
  P(:, :, 1) = eye(nx);
  x_hat(:, 1) = zeros(nx, 1);
  for i = 1:kIter-1
    [P_pred, x_pred] = conventional_propagate(P(:, :, i), F, G, Q, ...
                                            x_hat(:, i), ...
                                            x_mean(:, i));
    [P(:, :, i+1), K(:, :, i), x_hat(:, i+1)] ...
    = conventional_update(P_pred, F, H, R, z(:, i), x_pred);
  end
  dlmwrite(output, [kIter; nx; nw; nz; F(:); G(:)
                    H(:); T(:); U(:); x_hat(:, 1)
                    reshape(chol(squeeze(P(:, :, 1))), [], 1)
                    reshape([z; x_hat; x_mean; reshape(P, nx*nx, [])], [], 1)]);
end

function [P_prior, x_prior] = conventional_propagate(P, F, G, Q, x, x_mean)
  if nargin >= 5,
    if nargin < 6, x_mean = zeros(length(x),1); end
    x_prior = F*x + x_mean;
  end
  P_prior = F*P*F' + G*Q*G';
end

function [P, K, x] = conventional_update(P_prior, F, H, R, z, x_prior)
  K = P_prior*H'*inv(R + H*P_prior*H');
  P = (eye(size(P_prior)) - K*H)*P_prior;
  if nargin == 6,
    x = x_prior + K*(z-H*x_prior);
  end
end
