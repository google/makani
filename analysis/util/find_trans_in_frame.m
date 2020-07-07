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

function [Xg_ti_origin, dcm_g2ti, ti_origin_azimuth] = ...
  find_trans_in_frame(Xg, flight_mode, system_params, control_params)
% FIND_TRANS_IN_FRAME  find the trans-in frame from a data run

I = find(flight_mode == 6, 1);

if isempty(I)
  Xg_ti_origin = [];
  dcm_g2ti = [];
  ti_origin_azimuth = [];
  return
end

Xg = Xg(I, :)';
% this code is ripped out of e_ControlTransin.m
ti_origin_azimuth = atan2(-Xg(2), -Xg(1));
dcm = angle_to_dcm(ti_origin_azimuth, ...
                   -control_params.trans_in.start_elevation, 0, 'zyx');
Xg_ti_origin = dcm' * [-system_params.tether.length, 0, 0]';
dcm_g2ti = angle_to_dcm(ti_origin_azimuth, ...
                        control_params.trans_in.r2_angle_of_ascent, ...
                        control_params.trans_in.r3_yaw2, 'zyz');
