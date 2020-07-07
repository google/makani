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

function [out] = h5log_fix_gps(in)
out = [];
fields = fieldnames(in);
for f = 1:length(fields)
  field = fields{f};
  if isstruct(in.(field))
    if strcmp(field, 'GpsSatellites')
      out.(field) = fix_gps_satellites(in.(field));
    elseif strcmp(field, 'NovAtelObservations')
      out.(field) = fix_novatel_observations(in.(field));
    elseif strcmp(field, 'SeptentrioObservations')
      out.(field) = fix_septentrio_observations(in.(field));
    else
      out.(field) = h5log_fix_gps(in.(field));
    end
  else
    out.(field) = in.(field);
  end
end

% Add GPS azimuth and elevation angles.
if isfield(out, 'GpsSatellites') && isstruct(out.GpsSatellites)
  if isfield(out, 'NovAtelSolution')
    ii = find(out.NovAtelSolution.best_xyz.pos_type > 0);
    t = out.NovAtelSolution.t(:, ii);
    r_u_ecef = [out.NovAtelSolution.best_xyz.pos_x(:, ii);
                out.NovAtelSolution.best_xyz.pos_y(:, ii);
                out.NovAtelSolution.best_xyz.pos_z(:, ii)];
    tow = out.NovAtelSolution.best_xyz.timestamp.tow(:, ii) / 1000;
  elseif isfield(out, 'SeptentrioSolution')
    ii = find(out.SeptentrioSolution.pvt_cartesian.mode > 0);
    t = out.SeptentrioSolution.t(:, ii);
    r_u_ecef = [out.SeptentrioSolution.pvt_cartesian.x(:, ii);
                out.SeptentrioSolution.pvt_cartesian.y(:, ii);
                out.SeptentrioSolution.pvt_cartesian.z(:, ii)];
    tow = out.SeptentrioSolution.pvt_cartesian.timestamp.tow(:, ii) / 1000;
  else
    t = [];
    r_u_ecef = [];
    tow = [];
  end
  out.GpsSatellites = add_gps_angles(out.GpsSatellites, t, tow, r_u_ecef);
end


function [data] = fix_gps_satellites(data)
gps_pi = pi;
data.eph = h5log_fix_gps_prn(data.eph, data.eph.prn);
data.eph.delta_n = data.eph.delta_n * gps_pi;
data.eph.m_0 = data.eph.m_0 * gps_pi;
data.eph.omega_0 = data.eph.omega_0 * gps_pi;
data.eph.i_0 = data.eph.i_0 * gps_pi;
data.eph.omega = data.eph.omega * gps_pi;
data.eph.omega_dot = data.eph.omega_dot * gps_pi;
data.eph.i_dot = data.eph.i_dot * gps_pi;


function [data] = fix_novatel_observations(data)
prn = double(data.range.prn);
data.range = h5log_fix_gps_prn(data.range, prn);


function [data] = fix_septentrio_observations(data)
data.meas_epoch.signal_type = bitand(uint32(data.meas_epoch.type), 31);
prn = double(data.meas_epoch.svid) + double(data.meas_epoch.signal_type) / 1000;
data.meas_epoch = h5log_fix_gps_prn(data.meas_epoch, prn);


function [sat] = add_gps_angles(sat, sol_t, sol_tow, sol_xyz)
sat.elev = zeros(size(sat.eph.prn));
sat.azi = zeros(size(sat.eph.prn));
if isempty(sol_t)
  return
end

t_transmit = interp1(sol_t, sol_tow, sat.t, 'nearest');
r_u_ecef = [interp1(sol_t, sol_xyz(1, :), sat.t);
            interp1(sol_t, sol_xyz(2, :), sat.t);
            interp1(sol_t, sol_xyz(3, :), sat.t)];

for p = 1:size(sat.eph.prn,1)
  ii = find(sat.eph.prn(p, :) > 0);
  ii = [ii(1); ii(diff(sat.eph.t_oe(p, ii)) ~= 0); ii(end)];
  for jj = 1:length(ii) - 1
    eph_p = gps_eph(sat.eph, p, ii(jj));
    for kk = ii(jj):ii(jj + 1)
      r_s_ecef = gps_position(eph_p, t_transmit(kk));
      [~, ~, ~, elev, azi] = gps_angles(r_u_ecef(:, kk), r_s_ecef);
      sat.elev(p, kk) = elev;
      sat.azi(p, kk) = azi;
    end
  end
end
