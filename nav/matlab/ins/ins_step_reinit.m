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

function [ins] = ins_step_reinit(ins)
p = ins.param;

% Check for sufficient IMU data.
if length(ins.imu.data) < p.lag_imu_samples * 2
  fprintf(1, 'Reinit: waiting for IMU data.\n');
  return
end

% Check for GPS ionosphere corrections.
if isempty(ins.gps.iono)
  fprintf(1, 'Reinit: waiting for GPS ionosphere corrections.\n');
  return
end

% Check for GPS ephemerides.
[eph, obs] = gps_collate(ins.gps.eph, ins.gps.obs, ins.gps.valid);
if length(obs) < p.init.satellites
  fprintf(1, 'Reinit: waiting for satellites (found=%d, require=%d).\n', ...
          length(obs), p.init.satellites);
  return
end

% Extract inertial data about initialization time (+/-lag).
imu = ins_cbuf_unwrap(ins.imu);
time = [imu.t];
ii = find(time >= time(end) - p.lag_imu_samples * 2);
imu_avg = struct();
imu_var = struct();
fields = fieldnames(imu);
for f = 1:length(fields)
  field = fields{f};
  v = double(reshape([imu.(field)], size(imu(1).(field), 2), length(imu)))';
  imu_avg.(field) = mean(v(ii, :));
  imu_var.(field) = var(v(ii, :));
end

% Store initialization data (averaged about initialization time).
init = struct();
init.imu = imu_avg;
init.iono = ins.gps.iono;
init.eph = eph;
init.obs = obs;

% Initialize navigation state.
xhat = ins_xhat_initialize(ins.param, ins.xhat.t, init);

% Check for quasi-stationary condition about processing time (t - lag).
dvsf_err = abs(norm(imu_avg.dvsf) - norm(xhat.g_ib_e) * imu_avg.dt);
phi_err = norm(imu_avg.phi - 0);
if norm(imu_var.dvsf) > p.init.dvsf_var || dvsf_err > p.init.dvsf_err ...
    || norm(imu_var.phi) > p.init.phi_var || phi_err > p.init.phi_err
  fprintf(1, 'Reinit: waiting for quasi-stationary interval.\n');
  return
end

% Ready to perform coarse alignment.
fprintf(1, 'Reinit: performing coarse alignment at t=%f.\n', xhat.t);
[ins.xhat, ins.dx] = ins_coarse_align(p, xhat.t, init);
ins.init = ins.init + 1;
ins.reinit = 0;
ins.gps.xhat = ins.xhat;
