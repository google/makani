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

function [] = constrained_least_squares_comparison(output)
% constrained_least_squares_comparison -- Outputs random constrained LS problems
% solved using qp.
%
% constrained_least_squares_comparison(output)
% Generates random constrained least squares problems to be compared
% with algorithms in optim.c.
%
% Arguments
%
% output: The path to write the data to.
%
  problems = {solve_random_problem(4, 4, 4, 4, 4)
              solve_random_problem(4, 4, 2, 4, 4)
              solve_random_problem(10, 4, 4, 4, 4)
              solve_random_problem(10, 4, 2, 4, 4)
              solve_random_problem(4, 4, 4, 4, 2)
              solve_random_problem(4, 4, 4, 10, 4)
              solve_random_problem(4, 4, 4, 10, 2)
              solve_random_problem(4, 4, 2, 4, 2)};

  dlmwrite(output, [length(problems); vertcat(problems{:})]);
end

% Generate random problem instances with specific dimensions
% and ranks for A and c.
function serial = solve_random_problem(nx, nb, ra, nc, rc)
  if rc > 0,
    A = randn(nb, ra)*randn(ra, nx);
  else,
    A = zeros(nb, nx);
  end
  if rc > 0,
    C = randn(nc, rc)*randn(rc, nx);
  else
    C = zeros(nc, nx);
  end
  b = randn(nb, 1);
  x0 = randn(nx, 1);
  lower = C*x0 - rand(nc, 1);
  upper = C*x0 + rand(nc, 1);

  serial = solve_and_serialize(A, b, C, lower, upper, x0);
end

function serial = solve_and_serialize(A, b, C, lower, upper, x0)
  x = qp(x0, A'*A, -2*A'*b, [], [], [], [], lower, C, upper);
  serial = [length(x0); length(b); length(lower); ...
            A(:);  b(:); C(:); lower(:); upper(:); x0(:); x(:)];
end
