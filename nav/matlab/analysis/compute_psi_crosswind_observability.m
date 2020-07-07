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

function [] = compute_psi_crosswind_observability()
% compute_psi_crosswind_observability -- Compute observability for psi filter.
%
% Edit this script to analyze the observability of our attitude filter (a psi
% filter) over the crosswind trajectory.

p = define_parameters();

inc_figure(0, 'Crosswind trajectory');
fprintf('Observability for one measurement correction:\n');
W = compute_trajectory_observability(p, @correct_one_vector, ...
                                     @draw_crosswind_trajectory);
[U, S, V] = svd(W);
U
sigma = sqrt(diag(S))'


fprintf('Observability for two measurement corrections:\n');
W = compute_trajectory_observability(p, @correct_two_vectors, []);
[U, S, V] = svd(W);
U
sigma = sqrt(diag(S))'
end


function [p] = define_parameters()
p = crosswind_trajectory_params();
p.center_antenna_offset = [3; 0; 0];  % [m]
p.starboard_antenna_offset = [0; 12; -0.5];  % [m]
p.port_antenna_offset = [0; -12; -0.5];  % [m]
p.dt = 0.01;  % [s]
p.correct_period = 2.5;  % [s]
end


function [W] = compute_trajectory_observability(p, correct_fn, draw_fn)
% Compute the observability grammian.
W = zeros(6, 6);
Phi_k_0 = eye(6);
for t = 0:p.dt:p.loop_period
  x = compute_crosswind_trajectory(p, t);
  if mod(t, p.correct_period) < p.dt/2
    if ~isempty(draw_fn)
      draw_fn(x);
    end
    [H_k, R_k] = correct_fn(p, x);
    W = W + Phi_k_0' * H_k' * inv(R_k) * H_k * Phi_k_0;
  end
  Phi_k_k_z1 = compute_state_transition_matrix(x, p.dt);
  Phi_k_0 = Phi_k_k_z1 * Phi_k_0;
end
end


function [Phi] = compute_state_transition_matrix(x, dt)
Phi = [eye(3), x.dcm_bn * dt; zeros(3, 3), eye(3)];
end


function [H, R] = correct_one_vector(p, x)
r_n = x.dcm_nb * (p.starboard_antenna_offset - p.port_antenna_offset);
H = nav_vector_sensitivity_matrix(x, r_n);
R = blkdiag(eye(3) * 0.2^2);
end


function [H, R] = correct_two_vectors(p, x)
r1_n = x.dcm_nb * (p.center_antenna_offset - p.port_antenna_offset);
r2_n = x.dcm_nb * (p.starboard_antenna_offset - p.port_antenna_offset);
H = [nav_vector_sensitivity_matrix(x, r1_n);
     nav_vector_sensitivity_matrix(x, r2_n)];
R = blkdiag(eye(6) * 0.2^2);
end


function [H] = nav_vector_sensitivity_matrix(x, r_n)
H = [-x.dcm_bn * vcross(r_n), zeros(3, 3)];
end
