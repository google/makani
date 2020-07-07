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

function [r_u_ecef, b_u, drho, v_u_ecef, f_u, drhodot, gdop, sv] = ...
    gps_point(eph, iono, obs, r_u_ecef, b_u, v_u_ecef, f_u)
earth = earth_wgs84();

% State indicies.
X_POSITION = 1:3;
X_CLOCK_BIAS = 4;
X_VELOCITY = 5:7;
X_CLOCK_FREQ = 8;

% Initial conditions.
if ~exist('r_u_ecef','var') || isempty(r_u_ecef)
  r_u_ecef = zeros(3, 1);
end
if ~exist('b_u','var') || isempty(b_u)
  b_u = 0;
end
if ~exist('v_u_ecef','var') || isempty(v_u_ecef)
  v_u_ecef = zeros(3, 1);
end
if ~exist('f_u','var') || isempty(f_u)
  f_u = 0;
end

% Allocate memory.
n = length(obs);
drho = zeros(n, 1);
drhodot = zeros(n, 1);
G = zeros(2*n, 8);

% Define satellite weighting.
sigma_pr = cellfun(@(x) x.ura, eph);  % [m]
sigma_do = 0.3 * ones(1, n);  % [m/s]
W = diag([1 ./ sigma_pr.^2, 1 ./ sigma_do.^2]);

% Compute user position and velocity in ECI coordinates.
[r_u_eci, v_u_eci] = ecef_to_eci(r_u_ecef, 0, v_u_ecef, earth.omega);

% Iterate until convergence.
iterations = 10;
for iter = 1:iterations
  % Iterate for each observation.
  for i = 1:n
    % Receiver local measurement time (time of reception).
    t_recv = obs{i}.tow;

    % Compute line-of-sight vectors.
    sv(i) = gps_los(eph{i}, iono, r_u_ecef, b_u, v_u_ecef, f_u, t_recv);

    % Formulate geometric matrix.
    G(i, X_POSITION) = -sv(i).los_eci;
    G(i, X_CLOCK_BIAS) = 1;
    G(n + i, X_POSITION) = -sv(i).dlos_eci;
    G(n + i, X_VELOCITY) = -sv(i).los_eci;
    G(n + i, X_CLOCK_FREQ) = 1;

    % Compute pseudo-range residual.
    drho(i) = obs{i}.pr - sv(i).pr;

    % Compute doppler residual (receiver measurement = -(v_s - v_u).1*f0/c).
    drhodot(i) = (-obs{i}.do * obs{i}.lambda) - sv(i).do;
  end

  % Compute weighted least-squares solution.
  invGtG = inv(G' * W * G);
  dx = invGtG * G' * W * [drho; drhodot];
  r_u_eci = r_u_eci + dx(X_POSITION);
  b_u = b_u + dx(X_CLOCK_BIAS);
  v_u_eci = v_u_eci + dx(X_VELOCITY);
  f_u = f_u + dx(X_CLOCK_FREQ);

  % Convert back to ECEF coordinates.
  [r_u_ecef, v_u_ecef] = eci_to_ecef(r_u_eci, 0, v_u_eci, earth.omega);

  % Compute geometric dilustion of precision.
  gg = [X_POSITION, X_CLOCK_BIAS];
  dops = invGtG(gg, gg);
  gdop = trace(dops);

  % Check for convergence.
  delta = dx' * dx;
  if delta < 1e-6
    break;
  end
end
