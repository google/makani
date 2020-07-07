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

function [pitot_cal] = GetPitotCalibrationCoefficients(pitot_cal_file)
% GetPitotCalibrationCoefficients -- calibration coefficients from pitot calibration file.
% [pitot_cal] = GetPitotCalibrationCoefficients(pitot_cal_file)
%
% The factory pitot calibrations are provided for a combination of alpha and
% beta at a given wind speed. This function produces the coefficients for a
% third order fit on differential pressure data from a five port + static pitot
% probe. The fit is assumed to of the form, for a given wind speed:
% [alpha, beta] = [k: 2 x 6][cp_alpha,
%                            cp_beta,
%                            cp_alpha^2,
%                            cp_beta^2,
%                            cp_alpha^3,
%                            cp_beta^3   ]'; where
% cp_alpha is differential horizontal ports pressure normalized by static pressure
% cp_beta is differential vertical ports pressure normalized by static pressure
%
% Input Arguments-
% pitot_cal_file: [char] string contains calibration filename with path (*.pcf)
%
% Output Values-
% pitot_cal : [struct] structure with pitot calibration information
%         .filename -- [char] filename of the calibration file
%         .speed    -- wind speed for this calibration [m/s]
%         .coeff    -- [2 x 6] calibration coefficients
%         .R2       -- fit quality, must be greater than 0.999 for good fit
%

% The general structure of .pcf file is as follows:
% Mach no info is on line 8
% calibration data starts at line 15
% col1: Theta[deg]
% col2: Phi  [deg]
% col3: P1  [Torr]
% col4: P2  [Torr]
% col5: P3  [Torr]
% col6: P4  [Torr]
% col7: P5  [Torr]
% col8: P6  [Torr]
% col9: Pt  [Torr]
% col10:Ps  [Abs,Torr]
% col11:Ts  [C]

% Parse the PCF header. This is difficult to do using 'textscan' as some of
% the values (e.g. the date) contain embedded tabs.
fid = fopen(pitot_cal_file);
header = struct();
while ~feof(fid)
    str = fgetl(fid);
    pairs = strsplit(str, '=');
    if length(pairs) ~= 2
        break
    end
    key = pairs{1};
    value = strtrim(pairs{2});
    % Convert numeric values into numbers.
    if ismember(key, {'NHL', 'NP', 'Mach', 'CalPts', 'TstPts'})
        value = sscanf(value, '%f');
    end
    header.(key) = value;
end
fclose(fid);

% Verify that the header contains the expected number of lines.
assert(header.NHL == 14, 'Number of header lines not as expected.');

% Parse the column headers from the last read line.
column_labels = regexprep(strsplit(str, '\t'), '[\W]', '');

% Read data file and parse parameters.
dataread = dlmread(pitot_cal_file, '\t', header.NHL, 0);
pcf_data = struct();
for ii = 1:length(column_labels)
  field_name = genvarname(column_labels{ii});
  pcf_data.(field_name) = dataread(:, ii);
end
clear field_name

% Convert the test angle settings to aero angles.
aoa_deg = atand(tand(pcf_data.Thetadeg) .* sind(pcf_data.Phideg));
aos_deg = asind(sind(pcf_data.Thetadeg) .* cosd(pcf_data.Phideg));

% Calculate non-dimensional pressure coefficients defined as
% differential pressures normalized by static pressure
% Cross section of Pitot tube.            Frontal view of the probe.
%
%        ..(1)..                                    .---------.
%     (5)   |   (4)                                '    (5)    `
%    .` \ a | a / `.                              '             '
%   `    \  |  /    `                             | (3) (1) (2) |
%  `      \ | /      `    a: port_angle           |             |
%  '       \|/       '                            ',    (4)    ,'
%  |        x        |                              '---------'
%
% cp_alpha is the pressure difference between ports 5 and 4, and
% cp_beta is the difference between ports 3 and 2.
cp_alpha = (pcf_data.P4Torr - pcf_data.P5Torr) ./ (pcf_data.PsAbsTorr);
cp_beta  = (pcf_data.P2Torr - pcf_data.P3Torr) ./ (pcf_data.PsAbsTorr);

% find coefficients for third order fitting
cal_mat = [aoa_deg, aos_deg]' / [cp_alpha,    cp_beta, ...
                                 cp_alpha.^2, cp_beta.^2, ...
                                 cp_alpha.^3, cp_beta.^3]';

% use the fit coefficients to get aero angles from pressure data
out_f = cal_mat * [cp_alpha,    cp_beta, ...
                   cp_alpha.^2, cp_beta.^2, ...
                   cp_alpha.^3, cp_beta.^3]';
aoa_deg_f = out_f(1,:)';
aos_deg_f = out_f(2,:)';

% check the quality of fit
R2_1 = 1 - sum((aoa_deg - aoa_deg_f).^2) / sum((aoa_deg - mean(aoa_deg)).^2);
R2_2 = 1 - sum((aos_deg - aos_deg_f).^2) / sum((aos_deg - mean(aos_deg)).^2);
R2 = min(R2_1, R2_2);

% Calculate the speed of sound from temperature as shown in the ref:
% https://en.wikipedia.org/wiki/Speed_of_sound#Practical_formula_for_dry_air
v_sound = 331.3*sqrt(mean(pcf_data.TsC/273.15 + 1));

% Get the Mach number.
speed = header.Mach * v_sound;

% write the output structure
pitot_cal.coeff    = cal_mat;
pitot_cal.R2       = R2;
pitot_cal.filename = pitot_cal_file;
pitot_cal.speed    = speed;