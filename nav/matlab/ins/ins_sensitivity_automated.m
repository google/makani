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

function [] = ins_sensitivity_automated(files)

close all;
for f = 1:length(files)
  file = files{f};

  i = strfind(sprintf('/%s', file), '/');
  filename = file(i(end):end);
  i = strfind(sprintf('%s', filename), '.');
  basename = filename(1:i(end) - 1);
  if ~exist(basename, 'dir')
    mkdir(basename);
  end

  data = load(file);
  sense = ins_sensitivity(data);
  azimuth = ins_azimuth_sensitivity(data);
  datafile = sprintf('%s/%s_output.mat', basename, basename);
  save(datafile, 'data', 'sense', 'azimuth');

  close all;
  plot_ins_inputs(data);
  dirname = sprintf('%s/inputs', basename);
  save_all_figures('fig', dirname);
  save_all_figures('png', dirname);

  close all;
  plot_ins_dx(sense.ref);
  dirname = sprintf('%s/dx', basename);
  save_all_figures('fig', dirname);
  save_all_figures('png', dirname);

  close all;
  plot_ins_dz(sense.ref);
  dirname = sprintf('%s/dz', basename);
  save_all_figures('fig', dirname);
  save_all_figures('png', dirname);

  close all;
  plot_ins_output(sense.ref);
  dirname = sprintf('%s/output', basename);
  save_all_figures('fig', dirname);
  save_all_figures('png', dirname);

  close all;
  plot_ins_sensitivity(sense);
  dirname = sprintf('%s/sense', basename);
  save_all_figures('fig', dirname);
  save_all_figures('png', dirname);

  close all;
  plot_ins_azimuth_sensitivity(azimuth);
  dirname = sprintf('%s/azimuth', basename);
  save_all_figures('fig', dirname);
  save_all_figures('png', dirname);
end
close all;
