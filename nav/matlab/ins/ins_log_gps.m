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

function [out] = ins_log_gps(p, gps, inputs, out)
[eph, obs] = gps_collate(gps.eph, gps.obs, gps.valid);
if length(obs) >= 4 && ~isempty(gps.iono)
  out.gps.count = out.gps.count + 1;
  index = out.gps.count;

  % Log observation time.
  out.gps.t(index, :) = gps.xhat.t;

  % Log least squares point solution.
  [r_u_ecef, b_u, drho, v_u_ecef, f_u, ~, gdop, sv] = ...
      gps_point(eph, gps.iono, obs);
  [lat, lon, height] = ecef_to_llh(r_u_ecef);
  out.gps.pt_r_u_ecef(index, :) = r_u_ecef';
  out.gps.pt_v_u_ecef(index, :) = v_u_ecef';
  out.gps.pt_b_u(index, :) = b_u';
  out.gps.pt_f_u(index, :) = f_u';
  out.gps.pt_lat(index, :) = lat';
  out.gps.pt_lon(index, :) = lon';
  out.gps.pt_height(index, :) = height';
  out.gps.pt_drho(index, cellfun(@(x) x.prn, obs)) = drho';
  out.gps.pt_gdop(index, :) = gdop';
  out.gps.pt_sats(index, :) = length(obs);
  for i = 1:length(sv)
    prn = sv(i).prn;
    out.gps.pt_azimuth(index, prn) = sv(i).azimuth;
    out.gps.pt_elev(index, prn) = sv(i).elev;
    out.gps.pt_dt_path(index, prn) = sv(i).delay.path;
  end

  % Log INS solution.
  [r_u_ecef, v_u_ecef] = ins_gps_antenna(p, gps.xhat, inputs);
  [lat, lon, height] = ecef_to_llh(r_u_ecef);
  out.gps.ins_r_u_ecef(index, :) = r_u_ecef';
  out.gps.ins_v_u_ecef(index, :) = v_u_ecef';
  out.gps.ins_lat(index, :) = lat';
  out.gps.ins_lon(index, :) = lon';
  out.gps.ins_height(index, :) = height';
end
