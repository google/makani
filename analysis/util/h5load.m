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

function datasets = h5load(filename, obj_path, excludes)
% h5load -- Load HDF5 data into a containers.Map.
%
% datasets = h5load(filename[, obj_path, excludes])
% Load HDF5 data into a containers.Map, with each dataset indexed
% by their path.
%
% Arguments
%
% filename: A string containing the path to the HDF5 file.
% obj_path: Optional string containing the root path within the HDF5 file
%           to be scanned (default: '/').
% excludes: A cell array of strings specifying what dataset NOT to load.
%           This is optional. (default: {}).
%
% Return Values
%
% datasets: A containers.Map of datasets, indexed by their h5 paths.

  if nargin == 1
    obj_path = '/';
    excludes = {};
  end

  if nargin == 2
    excludes = {};
  end

  datasets = containers.Map();
  dataset_names = h5list(filename, obj_path);
  for n = 1 : length(dataset_names)
    name = dataset_names{n};
    if ismember(name, excludes)
      continue;
    end
    datasets(name) = h5read(filename, name);
  end
end
