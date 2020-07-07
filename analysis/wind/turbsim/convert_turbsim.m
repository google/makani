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

function convert_turbsim(input_filename, output_filename)
% Convert a TurbSim generated bladed output file into a .mat
% file importable by Python.
%
% Arguments
%
% input_filename: Input path to the TurbSim generate file.
%
% output_filename: Path to write the .mat file output to.

addpath('/opt/makani/third_party/TurbSim/Test')

[velocity, y, z, nz, ny, dz, dy, dt, zHub, z1, SummVars] = ...
    readBLgrid(input_filename);

disp(output_filename)
save('-7', output_filename, 'velocity', 'y', 'z', 'nz', 'ny', 'dz', ...
     'dy', 'dt', 'zHub', 'z1', 'SummVars')

end
