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

function [] = mat_divide_comparison(output)
% mat_divide_comparison -- Outputs random matrix division problems.
%
% mat_divide_comparison(output)
% Solves a handful of random of problems using '\' and outputs
% Octave's solutions.
%
% Arguments
%
% output: The path to write the data to.
  problems = {solve_random_problem(4, 4, 4, 4)
              solve_random_problem(4, 4, 4, 10)
              solve_random_problem(10, 4, 4, 4)
              solve_random_problem(10, 4, 4, 10)
              solve_random_problem(4, 10, 4, 4)
              solve_random_problem(4, 10, 4, 4)};
  dlmwrite(output, [length(problems)
                    vertcat(problems{:})]);
end

% Generate random problem instances with specific dimensions
% and ranks for A and c.
function serial = solve_random_problem(nr, nc, ra, nb)
  A = randn(nr, ra)*randn(ra, nc);
  B = randn(nr, nb);
  X = A\B;
  serial = [nr; nc; ra; nb; A(:); B(:); X(:)];
end
