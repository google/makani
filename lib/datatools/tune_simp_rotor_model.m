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

function tune_simp_rotor_model(prop_data)
% tune_simp_rotor_model -- Find a simple model of the XROTOR database.
%
% tune_simp_rotor_model(prop_data)
% Fits the following 4-parameter model of thrust to the XROTOR database.
%     T = k_T(J) * omega^2
% where,
%     k_T(J) = a1*(J - J_neutral)
%              + a2*(J - J_neutral)^2
%              + a3*(J - J_neutral)^3
% where,
%     J: Advance ratio, 2*pi*Vapp./(omega*D).
%     J_neutral: Zero thrust advance ratio.
%     J_max: Thrust/power drop off rapidly above this value due to stall.
%
% Arguments
%
% prop_data: The XROTOR database in a Makani wing parameters .mat file.

try
  v_app = prop_data.v_inf;
  w = prop_data.omega;
  thrust = prop_data.thrust;
  torque = prop_data.torque;
catch
  error('The required prop_data field names are not present.');
end

J = calc_advance_ratio(v_app, w, 2.0 * prop_data.R);
J_max = find_J_max(thrust, J);
J_neutral = calc_J_neutral(thrust, J);

a = advance_ratio_best_fit(v_app(:), w(:), 2.0 * prop_data.R, ...
                           J_neutral, J_max, thrust(:));
a_torque = advance_ratio_best_fit(v_app(:), w(:), 2.0 * prop_data.R, ...
                                  J_neutral, J_max, torque(:));

% Find the result of the simple model over its valid range.
dJ = J - J_neutral;
thrust_fit = polyval([a', 0], dJ) .* w.^2;
torque_fit = polyval([a_torque', 0], dJ) .* w.^2;
thrust_fit(J > J_max) = NaN;
torque_fit(J > J_max) = NaN;

% Plots.
axes_color_order = get(0, 'DefaultAxesColorOrder');
set(0, 'DefaultAxesColorOrder', hsv(20));

figure(1);
surf(v_app, w, thrust_fit);
hold on;
plot3(v_app, w, thrust, 'k.');
hold off;
title('XROTOR database and simple model fit (thrust)');
xlabel('V\_app [m/s]');
ylabel('omega  [rad/s]');
zlabel('thrust [N]');

figure(2);
surf(v_app, w, torque_fit);
hold on;
plot3(v_app, w, torque, 'k.');
hold off;
title('XROTOR database and simple model fit (torque)');
xlabel('V\_app [m/s]');
ylabel('omega  [rad/s]');
zlabel('torque [N-m]');

figure(3);
H1 = plot(dJ, thrust ./ w.^2, 'b'); hold on;
H2 = plot(dJ, polyval([a', 0], dJ), 'g');
H3 = plot(dJ, thrust_fit ./ w.^2, 'r'); hold off;
title('Least squares fit');
xlabel('J - J\_neutral [#]');
ylabel('thrust/w^2 [N*s^2]');
ylim([-10, 10]);
legend([H1(1), H2(1), H3(1)], 'XROTOR', 'fit', 'fit bounded by J\_max');

figure(4);
H1 = plot(dJ, torque ./ w.^2, 'b'); hold on;
H2 = plot(dJ, polyval([a_torque', 0], dJ), 'g');
H3 = plot(dJ, torque_fit ./ w.^2, 'r'); hold off;
title('Least squares fit');
xlabel('J - J\_neutral [#]');
ylabel('torque/w^2 [N*s^2]');
ylim([-10, 10]);
legend([H1(1), H2(1), H3(1)], 'XROTOR', 'fit', 'fit bounded by J\_max');

% Reset the default axis color order.
set(0, 'DefaultAxesColorOrder', axes_color_order);

% Print the controller coefficients.
fprintf('''thrust_coeffs'': [%.2e, %.2e, %.2e],\n', a(1), a(2), a(3));
fprintf('''torque_coeffs'': [%.2e, %.2e, %.2e],\n', ...
        a_torque(1), a_torque(2), a_torque(3));
fprintf('''J_neutral'': %2.3f,\n', J_neutral);
fprintf('''J_max'': %2.2f,\n', J_max);


function J_neutral = calc_J_neutral(thrust, J)
% Finds the advance ratio, J, that corresponds to neutral thrust.
J_neutral_all = zeros(size(J, 2), 1);
for j = 1:size(J, 2)
  for i = 1:(size(J, 1) - 1)
    if sign(thrust(i, j)) == 1 && sign(thrust(i + 1, j)) == -1
      J_neutral_all(j) = calc_x_intercept(J(i, j), thrust(i, j), ...
                                          J(i + 1, j), thrust(i + 1, j));
    end
  end
end
J_neutral = mean_excluding_outliers(J_neutral_all);


function x = calc_x_intercept(x1, y1, x2, y2)
M = [x1, y1; x2, y2];
a = M \ [1; 1];
x = 1 / a(1);


function J_max = find_J_max(thrust, J)
J_max_all = zeros(size(J, 1), 1);
for k = 1:size(J, 1)
  [~, ind] = min(thrust(k, :));
  J_max_all(k) = J(k, ind);
end

% Subtract 0.1 from J_max to stay a healthy distance away from rotor
% stall.
J_max = mean_excluding_outliers(J_max_all) - 0.1;


function a = advance_ratio_best_fit(v_app, w, D, J_neutral, J_max, ...
                                    thrust_or_torque)
J = calc_advance_ratio(v_app, w, D);

% Only include the points less than J_max in the fit.
I = J < J_max;
J = J(I);
thrust_or_torque = thrust_or_torque(I);
w = w(I);

% Perform least squares fit.
dJ = J - J_neutral;
b = thrust_or_torque ./ w.^2;
A = [dJ.^3, dJ.^2, dJ];
a = A \ b;


function J = calc_advance_ratio(v_app, omega, D)
J = 2 * pi * v_app ./ (omega * D);


function mu = mean_excluding_outliers(x)
I = ((mean(x) + std(x)) > x) & (x > (mean(x) - std(x)));
mu = mean(x(I));
