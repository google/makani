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

function [apparent_wind_p] = ...
    PitotPressuresToApparentWindUsingCalibrations(alpha_press, beta_press, ...
                                                  dyn_press, stat_press, ...
                                                  rho, pitot_cal)
% Get apparent wind vector in pitot frame using calibrations for a 5-port
% pitot sensor.
%
% Arguments-
% alpha_press: [1 x n] differential alpha pressure [Pa]
% beta_press:  [1 x n] differential beta pressure [Pa]
% dyn_press:   [1 x n] dynamic pressure [Pa]
% stat_press:  [1 x n] static pressure [Pa]
% rho:         [1 x n] air density [kg/m^3]
% pitot_cal:   [struct] pitot calibration information
%              likely generated from GetPitotCalibrationCoefficients.m
%
% Output Values-
% apparent_wind_p: [3 x n] apparent wind vector [Vx; Vy; Vz] in the pitot
%                  frame [m/s]
%

% Parse the pitot calibration data to identify reference wind speeds and
% corresponding calibration coefficients.
all_cals = fieldnames(pitot_cal);
for ii = 1:length(all_cals)
  v_cal(ii)       = pitot_cal.(all_cals{ii}).speed;
  cal_mat(ii,:,:) = pitot_cal.(all_cals{ii}).coeff;
end
clear ii all_cals;

% Convert the dynamic pressure to wind speed.
v = sqrt(2 * abs(dyn_press ./ rho));

% Divide the differential pressures by static pressure.
cp_alpha = alpha_press ./ stat_press;
cp_beta  = beta_press  ./ stat_press;

% Collect the pressure ratios for applying calibration.
cal_input = [cp_alpha;    cp_beta;    ...
             cp_alpha.^2; cp_beta.^2; ...
             cp_alpha.^3; cp_beta.^3];

% Apply calibrations one data point at a time.
% TODO: This can be improved for better execution speed.
for ii = 1:length(v)
  use_cal = interp1(v_cal, cal_mat, v(ii), 'linear', 'extrap');
  aero_angles = squeeze(use_cal) * cal_input(:,ii);
  aoa(ii) = aero_angles(1);
  aos(ii) = aero_angles(2);
end

% Bound the aoa and aos to 20 deg, this is the range of the current pitot probe.
aoa(aoa >  20) =  20;
aos(aos >  20) =  20;

aoa(aoa < -20) = -20;
aos(aos < -20) = -20;

% Convert the aero angles to apparent wind in the pitot frame.
apparent_wind_x = v .* cosd(aoa) .* cosd(aos);
apparent_wind_y = v .* sind(aos);
apparent_wind_z = v .* sind(aoa) .* cosd(aos);
apparent_wind_p = -[apparent_wind_x; apparent_wind_y; apparent_wind_z];
