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

function [pitot_cal] = CollectAllPitotCalibrations(pitot_serial_number)
% CollectAllPitotCalibrations -- struct with pitot calibration coefficients.
% pitot_cal = CollectAllPitotCalibrations(pitot_serial_number)
%
% This function outputs a structure with pitot calibration coefficients from all
% available calibration files for the specified serial number. The calibrations
% in makani/database/AeroProbe are used.
% GetPitotCalibrationCoefficients.m is used to calculate the coefficients from
% each available calibration file.
%
% Arguments-
% pitot_serial_number - [char] AeroProbe pitot identifier serial number
%
% Output Values-
% pitot_cal - [struct] structure with pitot calibration coefficients
%       .Mxxx [struct] -- data from each available calibration file at Mach xxx
%            .filename -- [char] filename of the calibration file
%            .speed    -- wind speed for this calibration [m/s]
%            .coeff    -- [2 x 6] calibration coefficients
%            .R2       -- fit quality, must be greater than 0.999 for good fit
%

% Check if Makani repo is added to the env path.
MAKANI_HOME = getenv('MAKANI_HOME');
if isempty(MAKANI_HOME)
  fprintf('Set MAKANI_HOME, to access pitot calibration files. \n')
  fprintf('Use setenv(''MAKANI_HOME'', ''<local path to makani git repo>'').\n')
  fprintf('Test using getenv(''MAKANI_HOME'').\n')
  error  ('Please set MAKANI_HOME.')
else
  pitot_cal_folder = fullfile(MAKANI_HOME, 'database', 'AeroProbe');
end
clear MAKANI_HOME;

% Find all the files in the AeroProbe folder, return an error if none are found.
dir_info = dir(fullfile(pitot_cal_folder, [pitot_serial_number '_M*.pcf']));

if isempty(dir_info)
  error('Check if calibration file(s) for pitot ''%s'' exists.', pitot_serial_number);
else
  cal_files = {dir_info.name};
  % calibration Mach number from the filename for identification only
  mach_id = extractBetweenStrings(cal_files, pitot_serial_number, '.pcf');
end

% Compile pitot calibrations,
for ii = 1:length(cal_files)
  pcf = fullfile(pitot_cal_folder, cal_files{ii});
  case_lbl = ['cal', mach_id{ii}];
  pitot_cal.(case_lbl) = GetPitotCalibrationCoefficients(pcf);
end
clear ii pcf case_lbl mach_id cal_files;