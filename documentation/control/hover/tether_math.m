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

function tether_math()
  tether_length = linspace(5, 450, 1000);

  T0 = 3400;
  [catenary_spring_const, horizontal_length] = calc_catenary_spring_const( ...
      T0, tether_length);
  elastic_spring_const = calc_tether_spring_const(tether_length);

  %% figure(22);plot(tether_length, horizontal_length);

  %% figure(1);
  %% subplot(1, 2, 1);
  %% plot(tether_length, catenary_spring_const, ...
  %%      tether_length, elastic_spring_const);
  %% ylim([0, 1e7]);
  %% grid on;
  %% xlabel('Payout [m]');
  %% ylabel('Spring constant [N/m]');
  %% legend('Catenary spring constant', 'Elastic spring constant');

  figure(2);
  clf;
  plot(tether_length, catenary_spring_const / 1e3);
  ylim([0, 10]);
  grid on;
  xlabel('Payout [m]');
  ylabel('Spring constant [kN/m]');
  title('Catenary and elastic spring constants');

  % Create inset for elastic spring constant.
  axes('Position', [0.4, 0.4, 0.5, 0.5]);
  xlabel('Payout [m]');
  ylabel('Spring constant [kN/m]');
  box on;
  plot(tether_length, catenary_spring_const / 1e3, ...
       tether_length, elastic_spring_const / 1e3);
  ylim([0, 1e4]);
  xlabel('Payout [m]');
  ylabel('Spring constant [kN/m]');
  legend('Catenary', 'Elastic');

  print(gcf, '-dpdf', 'tether_spring.pdf');
end

function [spring_const, horizontal_length] = calc_catenary_spring_const( ...
    T0, tether_length)
  lambda = 1;  % [kg/m]
  g = 9.81;
  a = T0 / (lambda * g);
  r = calc_horizontal_length(T0, tether_length);
  x = r / (2 * a);
  spring_const = lambda * g / 2 * cosh(x) ./ (x .* cosh(x) - sinh(x));

  horizontal_length = r;
end

function spring_const = calc_tether_spring_const(tether_length)
  E = 150e9;
  A = pi * (0.023 / 2)^2;
  spring_const = E * A ./ tether_length;
end

function horizontal_length = calc_horizontal_length(T0, tether_length)
  lambda = 1;  % [kg/m]
  g = 9.81;
  a = T0 / (lambda * g);
  frac_length_func = @(x, l) sinh(l / (2 * a) * x) - l / (2 * a);
  frac_length = zeros(size(tether_length));
  for k = 1:length(tether_length)
    frac_length(k) = fzero(@(x) frac_length_func(x, tether_length(k)), 1.0);
  end
  horizontal_length = frac_length .* tether_length;
end
