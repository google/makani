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

function convert_ref(filename)
  data = load(filename);
  if isfield(data, 'ref')
    name = 'ref';
    airfoil = data.ref;
  elseif isfield(data, 'oref')
    name = 'oref';
    airfoil = data.oref;
  end

  flap_deflections = pi / 180 * (-20:5:10);
  alphas = pi / 180 * airfoil.results.zero.alpha;

  data.lift_coeffs = zeros(length(flap_deflections), length(alphas));
  data.lift_coeffs(1, :) = remove_nans(airfoil.results.n20.CL);
  data.lift_coeffs(2, :) = remove_nans(airfoil.results.n15.CL);
  data.lift_coeffs(3, :) = remove_nans(airfoil.results.n10.CL);
  data.lift_coeffs(4, :) = remove_nans(airfoil.results.n5.CL);
  data.lift_coeffs(5, :) = remove_nans(airfoil.results.zero.CL);
  data.lift_coeffs(6, :) = remove_nans(airfoil.results.p5.CL);
  data.lift_coeffs(7, :) = remove_nans(airfoil.results.p10.CL);

  data.drag_coeffs = zeros(length(flap_deflections), length(alphas));
  data.drag_coeffs(1, :) = remove_nans(airfoil.results.n20.CD);
  data.drag_coeffs(2, :) = remove_nans(airfoil.results.n15.CD);
  data.drag_coeffs(3, :) = remove_nans(airfoil.results.n10.CD);
  data.drag_coeffs(4, :) = remove_nans(airfoil.results.n5.CD);
  data.drag_coeffs(5, :) = remove_nans(airfoil.results.zero.CD);
  data.drag_coeffs(6, :) = remove_nans(airfoil.results.p5.CD);
  data.drag_coeffs(7, :) = remove_nans(airfoil.results.p10.CD);

  data.moment_coeffs = zeros(length(flap_deflections), length(alphas));
  data.moment_coeffs(1, :) = remove_nans(airfoil.results.n20.CM);
  data.moment_coeffs(2, :) = remove_nans(airfoil.results.n15.CM);
  data.moment_coeffs(3, :) = remove_nans(airfoil.results.n10.CM);
  data.moment_coeffs(4, :) = remove_nans(airfoil.results.n5.CM);
  data.moment_coeffs(5, :) = remove_nans(airfoil.results.zero.CM);
  data.moment_coeffs(6, :) = remove_nans(airfoil.results.p5.CM);
  data.moment_coeffs(7, :) = remove_nans(airfoil.results.p10.CM);

  data.converged = zeros(length(flap_deflections), length(alphas));
  data.converged(1, :) = remove_nans(airfoil.results.n20.valid);
  data.converged(2, :) = remove_nans(airfoil.results.n15.valid);
  data.converged(3, :) = remove_nans(airfoil.results.n10.valid);
  data.converged(4, :) = remove_nans(airfoil.results.n5.valid);
  data.converged(5, :) = remove_nans(airfoil.results.zero.valid);
  data.converged(6, :) = remove_nans(airfoil.results.p5.valid);
  data.converged(7, :) = remove_nans(airfoil.results.p10.valid);

  fid = fopen([name, '.json'], 'w');
  fprintf(fid, '{');
  fprintf(fid, '\n  "name": "%s",', name);
  fprintf(fid, '\n  "flap_deflections": [');
  fprintf(fid, '%s', print_vector(flap_deflections, 6, '   '));
  fprintf(fid, '\n  ],');
  fprintf(fid, '\n  "alphas": [');
  fprintf(fid, '%s', print_vector(alphas, 6, '   '));
  fprintf(fid, '\n  ],');
  fprintf(fid, '\n  "lift_coeffs": [');
  for k = 1:length(flap_deflections)
    fprintf(fid, '\n    [');
    fprintf(fid, '%s', print_vector(data.lift_coeffs(k, :), 6, '     '));
    if k < length(flap_deflections)
      fprintf(fid, '\n    ],');
    else
      fprintf(fid, '\n    ]');
    end
  end
  fprintf(fid, '\n  ],');

  fprintf(fid, '\n  "drag_coeffs": [');
  for k = 1:length(flap_deflections)
    fprintf(fid, '\n    [');
    fprintf(fid, '%s', print_vector(data.drag_coeffs(k, :), 6, '     '));
    if k < length(flap_deflections)
      fprintf(fid, '\n    ],');
    else
      fprintf(fid, '\n    ]');
    end
  end
  fprintf(fid, '\n  ],');

  fprintf(fid, '\n  "moment_coeffs": [');
  for k = 1:length(flap_deflections)
    fprintf(fid, '\n    [');
    fprintf(fid, '%s', print_vector(data.moment_coeffs(k, :), 6, '     '));
    if k < length(flap_deflections)
      fprintf(fid, '\n    ],');
    else
      fprintf(fid, '\n    ]');
    end
  end
  fprintf(fid, '\n  ],');

  fprintf(fid, '\n  "converged": [');
  for k = 1:length(flap_deflections)
    fprintf(fid, '\n    [');
    fprintf(fid, '%s', print_vector(data.converged(k, :), 6, '     '));
    if k < length(flap_deflections)
      fprintf(fid, '\n    ],');
    else
      fprintf(fid, '\n    ]');
    end
  end
  fprintf(fid, '\n  ]');
  fprintf(fid, '\n}');
  fclose(fid);
end

function y = remove_nans(x)
  y = x;
  y(isnan(x)) = interp1(1:length(x), x, find(isnan(x)), 'pchip');
end

function str = print_vector(vector, elements_per_line, indent_str)
  str = '';
  for k = 1:elements_per_line:(length(vector) - elements_per_line)
    str = [str, sprintf('\n'), indent_str, ...
           sprintf(' % f,', vector(k:(k + elements_per_line - 1)))];
  end
  str = [str, sprintf('\n'), indent_str, ...
         sprintf(' % f,', vector((k + elements_per_line):end))];
  str = str(1:(end - 1));
end
