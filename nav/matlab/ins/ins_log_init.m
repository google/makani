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

function [out] = ins_log_init(p, data, start_time, end_time, out)
if ~exist('out', 'var') || isempty(out)
  out = struct();
end
if ~isfield(out, 'gps')
  out.gps = struct();
end

out = log_state_init(p, data, end_time, out);
out.param = p;
out.innov = {};
out.gps = log_gps_init(data, start_time, end_time, out.gps);



if isfield(data, 'pose')
  out.pose = log_data_init(data.pose, end_time);
end
if isfield(data, 'resolver')
  out.resolver = log_data_init(data.resolver, end_time);
end
if isfield(data, 'wheel')
  out.wheel = log_data_init(data.wheel, end_time);
end
if isfield(data, 'posllh')
  out.posllh = log_posllh_init(data, end_time);
end
if isfield(data, 'velned')
  out.velned = log_data_init(data.velned, end_time);
end
if isfield(data, 'clock')
  out.clock = log_clock_init(data, end_time);
end




function [out] = log_state_init(p, data, end_time, out)
% Output time/status.
count = length(data.imu.t(data.imu.t <= end_time)) - p.lag_imu_samples + 1;
out.t = data.imu.t(1:count);
out = append_output(out, 'iter', count, 1);
out = append_output(out, 'init', count, 1);

% Output position.
out = append_output(out, 'r_ib_e', count, 3);
out = append_output(out, 'lat', count, 1);
out = append_output(out, 'lon', count, 1);
out = append_output(out, 'height', count, 1);

% Output GPS antenna position.
out = append_output(out, 'antenna_r_igps_e', count, 3);
out = append_output(out, 'antenna_lat', count, 1);
out = append_output(out, 'antenna_lon', count, 1);
out = append_output(out, 'antenna_height', count, 1);

% Output velocity.
out = append_output(out, 'v_ib_e', count, 3);
out = append_output(out, 'v_ib_l', count, 3);
out = append_output(out, 'v_ib_b', count, 3);

% Output acceleration.
out = append_output(out, 'a_ib_e', count, 3);
out = append_output(out, 'a_ib_l', count, 3);
out = append_output(out, 'a_ib_b', count, 3);

% Output inertial inputs.
out = append_output(out, 'f_ib_b', count, 3);
out = append_output(out, 'omega_ib_b', count, 3);

% Output attitude.
out = append_output(out, 'mrp_be', count, 3);
out = append_output(out, 'theta_be', count, 3);
out = append_output(out, 'q_eb', count, 4);
out = append_output(out, 'q_lb', count, 4);
out = append_output(out, 'euler', count, 3);
out = append_output(out, 'azimuth', count, 1);

% Output accelerometer parameters.
out = append_output(out, 'b_a', count, 3);
out = append_output(out, 'c_a', count, 3);
out = append_output(out, 'k_a', count, 3);
out = append_output(out, 'mrp_sa', count, 3);
out = append_output(out, 'no_a', count, 3);
out = append_output(out, 'r_ba_ay', count, 3);
out = append_output(out, 'r_ba_az', count, 3);

% Output gyro parameters.
out = append_output(out, 'b_g', count, 3);
out = append_output(out, 'c_g', count, 3);
out = append_output(out, 'k_g', count, 3);
out = append_output(out, 'mrp_ag', count, 3);
out = append_output(out, 'no_g', count, 3);
out = append_output(out, 'F_g', count, 9);

% Output wheel encoder parameters.
out = append_output(out, 'radius', count, 4);

% Output GPS parameters.
out = append_output(out, 'cb_phase', count, 1);
out = append_output(out, 'cf_bias', count, 1);
out = append_output(out, 'cf_walk', count, 1);
out = append_output(out, 'gps_pr', count, 32);

% Output covariance.
out = append_output(out, 'sigma_xx', count, p.states.count);



function [out] = log_gps_init(data, start_time, end_time, out)
% Output time/status.
count = length(find(diff(data.obs.tow(data.obs.t <= end_time)) ~= 0));
out = append_output(out, 't', count, 1);
ii = 1:find(out.t > 0, 1, 'last');
ii = ii(out.t(ii) < start_time);
out.count = length(ii);
out.prns = unique(data.obs.prn);
out.prns = out.prns(out.prns <= 32);

% Output GPS least squares point solution.
out = append_output(out, 'pt_r_u_ecef', count, 3);
out = append_output(out, 'pt_v_u_ecef', count, 3);
out = append_output(out, 'pt_b_u', count, 1);
out = append_output(out, 'pt_f_u', count, 1);
out = append_output(out, 'pt_lat', count, 1);
out = append_output(out, 'pt_lon', count, 1);
out = append_output(out, 'pt_height', count, 1);
out = append_output(out, 'pt_drho', count, 32);
out = append_output(out, 'pt_gdop', count, 1);
out = append_output(out, 'pt_sats', count, 1);
out = append_output(out, 'pt_azimuth', count, 32);
out = append_output(out, 'pt_elev', count, 32);
out = append_output(out, 'pt_dt_path', count, 32);

% Output INS solution.
out = append_output(out, 'ins_r_u_ecef', count, 3);
out = append_output(out, 'ins_v_u_ecef', count, 3);
out = append_output(out, 'ins_lat', count, 1);
out = append_output(out, 'ins_lon', count, 1);
out = append_output(out, 'ins_height', count, 1);


function [out] = log_posllh_init(data, end_time)
% Output POSLLH solution.
ii = find(data.posllh.t <= end_time);
ii = ii(data.posllh.lat(ii) ~= 0);
ii = ii(data.posllh.lon(ii) ~= 0);
out = struct();
out.t = data.posllh.t(ii);
out.lat = data.posllh.lat(ii);
out.lon = data.posllh.lon(ii);
out.height = data.posllh.height(ii);


function [out] = log_clock_init(data, end_time)
% Output CLOCK solution.
earth = earth_wgs84();
ii = find(data.clock.t <= end_time);
out = struct();
out.t = data.clock.t(ii);
out.bias = data.clock.bias(ii) * earth.c * 1e-9;
out.drift = data.clock.drift(ii) * earth.c * 1e-9;


function [out] = log_data_init(data, end_time)
out = struct();
ii = find(data.t <= end_time);
fields = fieldnames(data);
for f = 1:length(fields)
  field = fields{f};
  out.(field) = data.(field)(ii, :);
end


function [out] = append_output(out, name, rows, cols)
data = zeros(rows, cols);
if isfield(out, name)
  m = min(size(out.(name), 1), rows);
  n = min(size(out.(name), 2), cols);
  data(1:m, 1:n) = out.(name)(1:m, 1:n);
end
out.(name) = data;
