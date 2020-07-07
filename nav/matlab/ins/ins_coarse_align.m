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

function [xhat, dx] = ins_coarse_align(param, time, meas)
% ins_coarse_align  Estimate the initial navigation state and error state.
%
% [xhat, dx] = ins_coarse_align(param, time, meas)
%
% Estimate the initial navigation state given measurement observations y,
%
% xhat_0 = init_fn(y),
%
% then perturb measurement observations and model parameters to explore the
% variation of the initial navigation state estimate. Here, we define
% distribution v ~ N(0, Pvv) to describe the initialization function input
% parameters, then use the unscented transformation to relate perturbations
% in those parameters to the initial error state and covariance. For each
% perturbation, we compute
%
% x_i = xhat_0 + xtilde(v),
%
% where x_i represents the hypothetical true state, xhat_0 represents the
% initial navigation state estimate, and xtilde represents the navigation state
% perturbation from v. We then estimate the perturbed measurement observations
% given x_i and measurement observation perturbation ytilde from v as
%
% y_i = h(x_i, y_0 + ytilde(v)).
%
% We now re-evaluate the initial navigation state estimate, xhat, using the
% same initialization function,
%
% xhat_i = init_fn(y_i).
%
% We then compute the navigation state error,
%
% dx_i = xhat_i - x_i,
%
% to complete the unscented transformation.
%
% Arguments:
%
% param: Specify algorithm configuration parameters. Use ins_param to create
%     the default structure.
% time: Specify the initialization time.
% measurements: Specify the initialization measurement observations.
%
% Return values:
%
% xhat: The initial state estimate.
% dx: The initial error state.

% Define useful constants.
deg = pi / 180;
hr = 3600;
g = 9.80665;
ug = g / 1e6;
mg = g / 1000;
ppm = 1e-6;
earth = earth_wgs84();

% Estimate the initial navigation state given recent measurement observations.
xhat_0 = ins_xhat_initialize(param, time, meas);

% Define input parameters.
ura = cellfun(@(x) x.ura, meas.eph);
p = struct();
% TODO: Implement MRP shadow parameters to avoid potential singularity.
p = add_input(p, 1, 'azimuth', 180 * deg);
p = add_input(p, 3, 'a_ib_e', 0.01);
p = add_input(p, 3, 'omega_lb_b', 1 * deg/hr);
p = add_input(p, 3, 'accel_noise', 1.29 * mg);
p = add_input(p, 3, 'accel_bias', param.accel.sigma_B);
p = add_input(p, 3, 'accel_turn_on', 1 * mg);
p = add_input(p, 3, 'accel_misalign', 1 * deg);
p = add_input(p, 3, 'accel_ortho', 0.035 * deg);
p = add_input(p, 3, 'accel_scale', 500 * ppm);
p = add_input(p, 3, 'accel_offset_y', 1e-4);
p = add_input(p, 3, 'accel_offset_z', 1e-4);
p = add_input(p, 3, 'gyro_noise', 0.135 * deg);
p = add_input(p, 3, 'gyro_bias', param.gyro.sigma_B);
p = add_input(p, 3, 'gyro_turn_on', [50; 50; 1] * deg/hr);
p = add_input(p, 3, 'gyro_misalign', 0.05 * deg);
p = add_input(p, 3, 'gyro_ortho', 0.05 * deg);
p = add_input(p, 3, 'gyro_scale', 500 * ppm);
p = add_input(p, 9, 'gyro_g_sense', 0.009 * deg/g);
p = add_input(p, 4, 'wheel_radius', 0.001);
p = add_input(p, 1, 'clock_phase', 10);
p = add_input(p, 1, 'clock_freq_bias', param.clock.sigma_B);
p = add_input(p, 1, 'clock_freq_turn_on', 10);
p = add_input(p, length(param.gps.prns), 'gps_pr_bias', ura);
p = add_input(p, length(meas.obs), 'pseudo_range', param.gps.sigma_pr);
p = add_input(p, length(meas.obs), 'doppler', 0.5);

% Define input distribution.
Pvv = diag(p.scale(p.select));
vbar = zeros(length(p.select), 1);

% Compute error state distribution.
[xbar, Pxx] = unscented_transform(vbar, Pvv, 1e-1, 2, 0, @coarse_align_fn, ...
                                  {param, p, xhat_0, meas});
if min(eig(Pxx)) <= 0
  error('Invalid coarse alignment (min eigenvalue=%g).', min(eig(Pxx)));
end

xbar_full = zeros(param.states.count, 1);
xbar_full(param.states.select) = xbar;
Pxx_full = zeros(param.states.count, param.states.count);
Pxx_full(param.states.select, param.states.select) = Pxx;

% Correct navigation state estimate with error state.
xhat = ins_xhat_correct(param, xhat_0, xbar_full);
dx = ins_dx_initialize(Pxx_full, zeros(param.states.count, 1), time, 1);



function [p] = add_input(p, count, symbol, sigma)
p = add_index(p, count, symbol, [], [], sigma.^2);


function [dx] = coarse_align_fn(v, param)
[param, p, xhat_0, meas] = deal(param{:});

v_full = zeros(p.count, 1);
v_full(p.select) = v;
v = v_full;

% Compute x_i = xhat_0 + xtilde(v).
x = compute_x(param, p, v, xhat_0);

% Compute y_i = h(x_i, y_0 + ytilde(v)).
dt = meas.imu.dt;
ya_a = compute_accel(v, x);
yg_g = compute_gyro(v, x);
yobs = compute_gps(param, p, v, xhat_0, x, meas.iono, meas.eph, meas.obs);

% Create init structure.
init = struct();
init.imu = struct();
init.imu.dt = dt;
init.imu.accel = ya_a';
init.imu.gyro = yg_g';
init.imu.dvsf = ya_a' * dt;
init.imu.phi = yg_g' * dt;
init.iono = meas.iono;
init.eph = meas.eph;
init.obs = yobs;

% Compute navigation estimate from measurements.
xhat_i = ins_xhat_initialize(param, x.t, init);

% Compute error between truth state and estimate.
dx = ins_dx_compute(param, x, xhat_i);
dx = dx(param.states.select);


function [x] = compute_x(param, p, v, xhat0)
x = struct();

% Time.
x.t = xhat0.t;

% Perturb accelerometer inputs.
x.omega_a = v(p.ACCEL_NOISE);
x.b_a = xhat0.b_a + v(p.ACCEL_BIAS);
x.c_a = xhat0.c_a + v(p.ACCEL_TURN_ON);
x.mrp_sa = mrp_mult(xhat0.mrp_sa, v(p.ACCEL_MISALIGN) / 4);
x.R_bs = xhat0.R_bs;
x.R_ba = xhat0.R_bs * mrp_to_dcm(x.mrp_sa);
x.no_a = xhat0.no_a + v(p.ACCEL_ORTHO);
x.T_aa = ins_ortho(x.no_a);
x.k_a = xhat0.k_a + v(p.ACCEL_SCALE);
x.K_a = diag(1 + x.k_a);
x.r_ba_ay = xhat0.r_ba_ay + v(p.ACCEL_OFFSET_Y);
x.r_ba_az = xhat0.r_ba_az + v(p.ACCEL_OFFSET_Z);

% Perturb gyro inputs.
x.omega_g = v(p.GYRO_NOISE);
x.b_g = xhat0.b_g + v(p.GYRO_BIAS);
x.c_g = xhat0.c_g + v(p.GYRO_TURN_ON);
x.mrp_ag = mrp_mult(xhat0.mrp_ag, v(p.GYRO_MISALIGN) / 4);
x.R_ag = mrp_to_dcm(x.mrp_ag);
x.no_g = xhat0.no_g + v(p.GYRO_ORTHO);
x.T_gg = ins_ortho(x.no_a);
x.k_g = xhat0.k_g + v(p.GYRO_SCALE);
x.K_g = diag(1 + x.k_a);
x.F_g = xhat0.F_g + reshape(v(p.GYRO_G_SENSE), 3, 3);

% Perturb wheel encoder inputs.
x.dradius = xhat0.dradius + v(p.WHEEL_RADIUS);

% Perturb GPS inputs.
x.cb_phase = xhat0.cb_phase + v(p.CLOCK_PHASE);
x.cf_bias = xhat0.cf_bias + v(p.CLOCK_FREQ_BIAS);
x.cf_walk = xhat0.cf_walk + v(p.CLOCK_FREQ_TURN_ON);
x.gps_pr = xhat0.gps_pr;
x.gps_pr(param.gps.prns) = xhat0.gps_pr(param.gps.prns) + v(p.GPS_PR_BIAS);

% Compute position.
x.r_ix_e = xhat0.r_ix_e;
x.dr_xb_e = xhat0.dr_xb_e;
x.r_ib_e = double(x.r_ix_e) + x.dr_xb_e;

% Compute local gravity.
x.g_ib_e = earth_gravity(x.r_ib_e);

% Perturb initial azimuth rotation.
R_azi = [cos(v(p.AZIMUTH)), -sin(v(p.AZIMUTH)), 0;
         sin(v(p.AZIMUTH)), cos(v(p.AZIMUTH)), 0;
         0 0 1];

% Compute attitude.
[~, ~, ~, R_le] = ecef_to_llh(x.r_ib_e);
R_lb = R_azi * R_le * xhat0.R_eb;
x.R_eb = R_le' * R_lb;
x.q_eb = dcm_to_q(x.R_eb);

% Compute angular rate.
x.omega_ie_e = xhat0.omega_ie_e;
x.omega_lb_b = v(p.OMEGA_LB_B);
x.omega_ib_b = x.omega_lb_b + x.R_eb' * x.omega_ie_e;
x.omega_eb_b = x.omega_ib_b - x.R_eb' * x.omega_ie_e;

% Compute velocity.
x.v_ib_e = xhat0.v_ib_e;

% Compute specific force.
x.a_ib_e = v(p.A_IB_E);
x.f_ib_e = x.a_ib_e - x.g_ib_e + 2 * cross(x.omega_ie_e, x.v_ib_e);
x.f_ib_b = x.R_eb' * x.f_ib_e;
Omega_ib_b = vcross(x.omega_ib_b);
f_ba_bx = [0; 0; 0];
f_ba_by = Omega_ib_b * Omega_ib_b * x.R_ba * x.r_ba_ay;
f_ba_bz = Omega_ib_b * Omega_ib_b * x.R_ba * x.r_ba_az;
x.f_ba_b = [f_ba_bx(1); f_ba_by(2); f_ba_bz(3)];
x.f_ia_b = x.f_ib_b - x.f_ba_b;


function [ya_a] = compute_accel(v, x)
% Compute accelerometer measurement model.
ya_a = x.K_a * inv(x.R_ba * x.T_aa) * (x.f_ib_b + x.f_ba_b) + x.b_a ...
    + x.c_a + x.omega_a;


function [yg_g] = compute_gyro(v, x)
% Compute gyro measurement model.
yg_g = x.K_g * inv(x.T_gg) * x.R_ag' * x.R_ba' * ...
    (x.omega_ib_b + x.F_g * x.f_ia_b) + x.b_g + x.c_g + x.omega_g;


function [yhat] = compute_gps(param, p, v, xhat_0, x, iono, eph, obs)
% Compute position and velocity of GPS antenna.
r_igps_ecef = x.r_ib_e + x.R_eb * param.geometry.r_bgps_b;
v_igps_ecef = x.v_ib_e + x.R_eb * cross(x.omega_eb_b, param.geometry.r_bgps_b);

% Compute pseudo-range and doppler measurements.
yhat = obs;
for idx = 1:length(obs)
  prn = obs{idx}.prn;
  lambda = obs{idx}.lambda;
  rho = obs{idx}.pr - xhat_0.cb_phase - xhat_0.gps_pr(prn);
  rhodot = obs{idx}.do - (xhat_0.cf_bias + xhat_0.cf_walk) / lambda;
  yhat{idx}.pr = rho + x.cb_phase + x.gps_pr(prn) + v(p.PSEUDO_RANGE(idx));
  yhat{idx}.do = rhodot + (x.cf_bias + x.cf_walk + v(p.DOPPLER(idx))) / lambda;
end
