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

function [ins, out] = ins_step_obs(ins, xhat_1, data, out)
p = ins.param;

% Check for corresponding ephemeris.
[eph, data] = gps_collate(ins.gps.eph, data);
if isempty(eph) || isempty(data) || isempty(ins.gps.iono)
  return
end
eph = eph{1};
data = data{1};

% Handle new measurement epoch.
new_epoch = ins.gps.tow ~= data.tow;
if new_epoch
  % Store previous measurement epoch for delta-range corrections.
  ins.gps_1 = ins.gps;

  % Initialize new measurement epoch.
  ins.gps.valid(:) = 0;
  ins.gps.tow = data.tow;

  % Handle 1-msec receiver clock phase corrections when the receiver's
  % receive time-of-week differs from expected.
  % TODO: Validate with point solution, handle extended dropouts.
  earth = earth_wgs84();
  jump_msec = gps_jump(ins.gps_1.tow, ins.gps.tow);
  ins.gps.jump = 1e-3 * jump_msec * earth.c;
  xhat_1.cb_phase = xhat_1.cb_phase + ins.gps.jump;
  ins.xhat.cb_phase = ins.xhat.cb_phase + ins.gps.jump;

  % Interpolate navigation state to observation time.
  ins.gps.xhat = ins_xhat_interpolate(p, xhat_1, ins.xhat, data.t_obs);

  % Compute antenna state.
  [ins.gps.r_u_ecef, ins.gps.v_u_ecef] = ...
      ins_gps_antenna(p, ins.gps.xhat, ins.inputs);
  ins.gps.b_u = ins.gps.xhat.cb_phase;
  ins.gps.f_u = ins.gps.xhat.cf_bias + ins.gps.xhat.cf_walk;

  % Pop state transition matrix propagation since last measurement.
  [ins.dx, Phi_k0] = ins_dx_pop(ins.dx, ins.gps.handle);
  ins.gps.Phi_0k = inv(Phi_k0);

  % Start propagating error state until measurement.
  [ins.dx, ins.gps.handle] = ...
      ins_dx_push(ins.dx, p.gps.timeout, data.t_obs);
end

% Compute line-of-sight vector and sensitivity matrix for pseudo-range and
% delta-range measurement corrections.
sv = gps_los(eph, ins.gps.iono, ins.gps.r_u_ecef, ins.gps.b_u, ...
             ins.gps.v_u_ecef, ins.gps.f_u, data.tow);
H_pr = ins_output_pr(p, ins.gps.xhat, sv, data);

% Store recent observations for initialization routine and delta-range
% measurement corrections.
ins.gps.obs{data.prn} = data;
ins.gps.sv{data.prn} = sv;
ins.gps.H_pr(data.prn, :) = H_pr;
ins.gps.valid(data.prn) = 1;

% Validate mask angle.
if sv.elev < p.gps.mask_angle
  return
end

% Perform measurement corrections.
fprintf(1, 'Correct GPS observations sv=%d t=%f\n', data.prn, data.t_obs);

% Correct pseudo-range.
[ins.dx, out] = ins_correct_pr(p, ins.gps.xhat, ins.dx, sv, data, out, ...
                               p.meas.GPS_PR(data.prn));

% Correct delta-range or Doppler. Delta-range utilizes the accumulated Doppler
% residual (carrier phase integral) over the measurement epoch, and provides
% a less noisy signal than the instantaneous Doppler measurements, especially
% in the presents of vibration.
if data.lli == 0 && ~isempty(ins.gps.Phi_0k) && ins.gps_1.valid(data.prn) ...
  && data.qual > 5 && ins.gps_1.obs{data.prn}.qual > 5
  % Correct delta-range.
  [ins.dx, dy_dr, out] = ins_correct_dr(p, ins.gps.xhat, ins.dx, ...
                                        ins.gps.Phi_0k, ...
                                        ins.gps_1.xhat, ...
                                        ins.gps_1.H_pr(data.prn, :), ...
                                        ins.gps_1.sv{data.prn}, ...
                                        ins.gps_1.obs{data.prn}, ...
                                        ins.gps.xhat, H_pr, sv, data, ...
                                        ins.gps.jump, out, ...
                                        p.meas.GPS_DR(data.prn));
else
  % Correct Doppler (fallback from delta-range).
  [ins.dx, out] = ins_correct_do(p, ins.gps.xhat, ins.dx, ...
                                 ins.inputs, sv, data, out, ...
                                 p.meas.GPS_DO(data.prn));
end
