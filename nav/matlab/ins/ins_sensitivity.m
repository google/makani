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

function [sense] = ins_sensitivity(data, time_interval)
if ~exist('time_interval', 'var') || isempty(time_interval)
  time_interval = [0; Inf];
end

% Define useful constants.
deg = pi / 180;
s = 1;
hr = 3600;
g = 9.80665;
mg = g / 1000;
ppm = 1e-6;
mm = 1e-3;

% Define number of evenly spaced points in trajectory to compare.
output_points = 100;

% Compute reference trajectory.
[~, ref] = ins_main(ins_param(data), data, time_interval);
ii_ref = find(ref.init > 0);
ii_ref = ii_ref(mod(ii_ref, ceil(length(ii_ref) / output_points)) == 0);

% Define input parameters.
v = struct();
v = add_input(v, 1, 'azimuth', 90 * deg);
v = add_input(v, 3, 'accel_turn_on', 16 * mg);
v = add_input(v, 3, 'accel_misalign', 3 * deg);
v = add_input(v, 3, 'accel_ortho', 0.035 * deg);
v = add_input(v, 3, 'accel_scale', 5000 * ppm);
v = add_input(v, 3, 'accel_offset_y', 0.1 * mm);
v = add_input(v, 3, 'accel_offset_z', 0.1 * mm);
v = add_input(v, 3, 'gyro_turn_on', 0.02 * deg/s);
v = add_input(v, 3, 'gyro_misalign', 0.05 * deg);
v = add_input(v, 3, 'gyro_ortho', 0.05 * deg);
v = add_input(v, 3, 'gyro_scale', 10000 * ppm);
v = add_input(v, 9, 'gyro_g_sense', 0.009 * deg/s/g);

% Ignore these error sources.
v = select_index(v, v.ACCEL_ORTHO, []);
v = select_index(v, v.ACCEL_OFFSET_Y, []);
v = select_index(v, v.ACCEL_OFFSET_Z, []);
v = select_index(v, v.GYRO_MISALIGN, []);
v = select_index(v, v.GYRO_ORTHO, []);
v = select_index(v, v.GYRO_G_SENSE, []);

% Define input distribution.
Pvv = diag(v.scale(v.select));
vbar = zeros(length(v.select), 1);

% Compute error state distribution.
[xbar, Pxx, Pvx] = unscented_transform(vbar, Pvv, 1e-1, 2, 0, @sim_fn, ...
                                       {v, ref, ii_ref, data, time_interval});

% Prepare output.
names = {'x', 'y', 'z', 'azi'};
n = length(ii_ref);
sense = v;
sense.ref = ref;
sense.ii_ref = ii_ref;
sense.t = ref.t(ii_ref);
for i = 1:length(names);
  name = names{i};
  xbar_name = sprintf('xbar_%s', name);
  pxx_name = sprintf('pxx_%s', name);
  pxv_name = sprintf('pxv_%s', name);
  ii = ((i - 1) * n + 1):(i * n);
  sense.(xbar_name) = xbar(ii);
  sense.(pxx_name) = diag(Pxx(ii, ii));
  sense.(pxv_name) = zeros(n, v.count);
  sense.(pxv_name)(:, v.select) = Pvx(:, ii)';
end


function [p] = add_input(p, count, symbol, sigma)
p = add_index(p, count, symbol, [], [], sigma.^2);


function [dx] = sim_fn(vv, param)
[v, ref, ii_ref, data, time_interval] = deal(param{:});

vv_full = zeros(v.count, 1);
vv_full(v.select) = vv;
vv = vv_full;

data.imu = perturb_imu(v, vv, data.imu);
[~, est] = ins_main(ins_param(data, vv(v.AZIMUTH)), data, time_interval);
dx = compute_error(ref, est, ii_ref);


function [imu] = perturb_imu(v, vv, imu)
dt = imu.dt * [1, 1, 1];
[ya_a, yg_g] = perturb_inertial(v, vv, imu.dvsf ./ dt, imu.phi ./ dt);
imu.dvsf = ya_a .* dt;
imu.phi = yg_g .* dt;


function [ya_a, yg_g] = perturb_inertial(v, vv, f_ib_b, omega_ib_b)
% Accelerometer error models.
K_a = diag(1 + vv(v.ACCEL_SCALE));
R_ba = mrp_to_dcm(vv(v.ACCEL_MISALIGN) / 4);
T_aa = ins_ortho(vv(v.ACCEL_ORTHO));
c_a = vv(v.ACCEL_TURN_ON);
f_ba_bx = [0; 0; 0];
r_ba_ay = vv(v.ACCEL_OFFSET_Y);
r_ba_az = vv(v.ACCEL_OFFSET_Z);
M_ab = K_a * inv(R_ba * T_aa);

% Gyro error models.
K_g = diag(1 + vv(v.GYRO_SCALE));
R_ag = mrp_to_dcm(vv(v.GYRO_MISALIGN) / 4);
T_gg = ins_ortho(vv(v.GYRO_ORTHO));
c_g = vv(v.GYRO_TURN_ON);
F_g = reshape(vv(v.GYRO_G_SENSE), 3, 3);
M_gb = K_g * inv(T_gg) * R_ag' * R_ba';

% Compute output.
ya_a = zeros(size(f_ib_b));
yg_g = zeros(size(omega_ib_b));
for i = 1:size(ya_a, 1)
  Omega_ib_b = vcross(omega_ib_b(i, :)');
  f_ba_bx = [0; 0; 0];
  f_ba_by = Omega_ib_b * Omega_ib_b * R_ba * r_ba_ay;
  f_ba_bz = Omega_ib_b * Omega_ib_b * R_ba * r_ba_az;
  f_ba_b = [f_ba_bx(1); f_ba_by(2); f_ba_bz(3)];
  f_ia_b = f_ib_b(i, :)' - f_ba_b;
  ya_a(i, :) = (M_ab * f_ia_b + c_a)';
  yg_g(i, :) = (M_gb * (omega_ib_b(i, :)' + F_g * f_ia_b) + c_g)';
end


function [dx] = compute_error(ref, est, ii)
dr = ref.r_ib_e(ii, :) - est.r_ib_e(ii, :);
dazi = wrap_angle(ref.azimuth(ii, :) - est.azimuth(ii, :));
dx = [dr(:, 1); dr(:, 2); dr(:, 3); dazi];
