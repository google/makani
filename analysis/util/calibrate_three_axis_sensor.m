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

function [A, b, data_cal, final_cost] = ...
  calibrate_three_axis_sensor(data, r)
% calibrate_three_axis_sensor -- Find sensor scales and offsets.
%
% [A, b, data_cal, final_cost] = calibrate_three_axis_sensor(data, r)
% Find the best-fit scales and offsets to map an ellipsoid data set onto
% a sphere centered at the origin.  This is useful for finding the gains
% and biases for 3-axis magnetometers and accelerometers.
%
% Arguments
%
% data: Multiple readings from a three-axis sensor.  The data must contain
%       at least six measurements to solve for the 3 gains and 3 biases.
%       Usually data is a time series taken as the sensor is rotated.
% r: The radius of the sphere to fit the data to.  For accelerometers this
%    should be 9.81, for magnetometers, somewhere around 0.4895.
%
% Return Values
%
% A: 3x3 diagonal matrix with each gain representing a gain for a
%    primary axis.
% b: Column vector with the 3 biases.
% data_cal: Input data corrected with the gains and biases.
%
% Example:
% [A, b, mag_cal] = calibrate_three_axis_sensor(imu_mag, 0.4895);
%
% data_cal = A*imu_mag(1, :)' + b           % Correct a single data point.
% data_cal = bsxfun(@plus, A*imu_mag', b)'  % Correct a data set.

data = double(data);

% Function for calculating the norm of each row of a matrix.
vnorm = @(x) sqrt(sum(x.^2, 2));

% If an 'r' is not provided, use the average norm of the data.
if nargin < 2, r = mean(vnorm(data)); end

% Optimize over the 6-vector, theta.
% theta0 = [a11 a22 a33 b1 b2 b3];
theta = [1 1 1 0 0 0];  % Initial guess.

% Parameterize the cost function for fminsearch.
cost = @(th) ellipsoid_cost(th, r, data);

opt = optimset('TolX', 1e-12, 'TolFun', 1e-12, 'FunValCheck', 'on', ...
  'MaxFunEvals', 5e4);
[theta, final_cost, ~] = fminsearch(cost, theta, opt);

% fminsearch is sensitive to initial guess and can exit even if it's not at
% a stationary point!  I suspect it has to do with auto-scaling based
% on initial guess.  This is lame!  Probably best to replace it with an
% an in-house gradient descent method (using numerical gradient).
errmsg = ['theta = [' num2str(theta, 4) ']' ...
  '\nis not a stationary point, try a different initial condition.' ...
  '\nfinal_cost = ' num2str(final_cost, 4)];
assert(check_stationary_point(cost, theta, r), sprintf(errmsg));

% Vectorized version of data_cal = A*data + b
A = diag(theta(1:3));
b = theta(4:6)';
data_cal = bsxfun(@plus, A*data', b)';

% Plot uncalibrated and calibrated ellipsoids and x/y/z/magnitude data.
figure(1)
plot_ellipsoid(data, r)
title('Uncalibrated measurements in 3D');

figure(2)
plot_ellipsoid(data_cal, r)
title('Calibrated measurements in 3D');

figure(3)
subplot(2, 1, 1)
plot(vnorm(data), 'k:'); hold on
plot(vnorm(data_cal), 'k'); hold off
legend('|XYZ_{uncal}|', '|XYZ_{cal}|')

subplot(2, 1, 2)
plot(data, ':'); hold on
plot(data_cal); hold off
ylabel('X,Y,Z')

grid on

figure(4)
[n1, x1] = hist(vnorm(data), 50, 'b');
[n2, x2] = hist(vnorm(data_cal), 50, 'g');
plot(x1, n1, 'b--', x2, n2, 'b'); hold on
mean_data_cal = mean(vnorm(data_cal));
plot([r r], ylim, 'r', [mean_data_cal mean_data_cal], ylim, 'g--')
hold off
grid on
title('Histogram of norm for calibrated and uncalibrated data')
xlabel('norm')
legend('|XYZ_{uncal}|', '|XYZ_{cal}|', 'r', 'mean(|XYZ_{cal}|)')


function L = ellipsoid_cost(theta, r, data)
% Cost function for quantifying the deviation of a data set from a sphere.
% The zero-cost surface is a sphere of radius, r, any deviation from that
% sphere increases the cost.
%
% Arguments
%
% theta: A vector of offsets and gains used to modify the data set.
% r: The radius of the zero-cost sphere.
% data: Data from a 3-axis sensor.
%
% Return Values
%
% L: The "cost" calculated as the sum of the errors square.

% Offsets.
b = theta(4:6)';

% Gains (axes are assumed orthogonal when A is set to diagonal).
A = diag(theta(1:3));

% Assume each row is a sample point.
y = bsxfun(@plus, A*data', b)';  % Corrected points.
r_y = sqrt(sum(y.*y, 2));  % Radius of each point.
L = sum((r_y - r).^2);  % Sum of the errors squared.


function plot_ellipsoid(X, r)
% Plot data on sphere.

plot3(X(:,1), X(:,2), X(:,3), '*');
[x, y, z] = sphere(200);
hold on; id = surf(x*r, y*r, z*r); hold off;
set(id, 'FaceAlpha', 0.5, 'LineStyle', 'none');
axis equal; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z')


function stationary = check_stationary_point(cost, theta, r)
% Check that a proposed solution is a stationary point to the cost function.
% This is necessary because fminsearch() does not guarantee convergence.

stationary = true;
% Perturb each of the theta values by a percentage of r.
for i = 1:length(theta)
  th_perturb = zeros(size(theta));
  th_perturb(i) = 0.001*r;
  dcostp = cost(theta) - cost(theta + th_perturb);
  dcostn = cost(theta) - cost(theta - th_perturb);
  if dcostp > 0 || dcostn > 0
    stationary = false;
    break
  end
end
