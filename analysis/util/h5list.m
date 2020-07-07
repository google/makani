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

function dataset_names = h5list(filename, obj_path)
% h5list -- List HDF5 data into a cell array of dataset paths.
%
% dataset_names = h5list(filename[, obj_path])
%
% List HDF5 data into a cell array of dataset paths. The scan starts at
%     obj_path within the HDF5 data.
%
% Arguments
%
% filename: A string containing the path to the HDF5 file.
% obj_path: Optional string containing the root path within the HDF5 file
%           to be scanned (default: '/').
%
% Return Values
%
% dataset_names: A cell array of dataset paths within the HDF5 file.

  file_id = H5F.open(filename, 'H5F_ACC_RDONLY', 'H5P_DEFAULT');
  if nargin == 1
    gid = H5G.open(file_id, '/');
    dataset_names = list_group(gid);
    H5G.close(gid);
    H5F.close(file_id);
  else
    oid = H5O.open(file_id, obj_path, 'H5P_DEFAULT');
    info = H5O.get_info(oid);
    switch (info.type)
      case H5ML.get_constant_value('H5O_TYPE_GROUP')
        dataset_names = list_group(oid);
      case H5ML.get_constant_value('H5O_TYPE_DATASET')
        dataset_names = list_dataset(oid);
      otherwise
        error('Unknown object type %d', info.type);
    end
    H5O.close(oid);
  end
end


function [status, opdata_out]= H5Literator_func(gid, name, opdata)
  try
    obj_id = H5O.open(gid, name, 'H5P_DEFAULT');
  catch me
    switch (me.identifier)
      case {'MATLAB:hdf5lib2:H5G_traverse_real:failure', ...
            'MATLAB:hdf5lib2:H5Oopen:failure'}
        error('Cannot open object %d', gid);
      otherwise
        rethrow(me);
    end
  end

  info = H5O.get_info(obj_id);
  switch (info.type)
    case H5ML.get_constant_value('H5O_TYPE_GROUP')
      datasets = list_group(obj_id);
    case H5ML.get_constant_value('H5O_TYPE_DATASET')
      datasets = {list_dataset(obj_id)};
    otherwise
      error('unknown object type %d', info.type);
  end
  H5O.close(obj_id);
  opdata_out = [opdata, datasets];
  status = 0;
end


function datasets = list_group(gid)
  datasets = {};
  opdata = datasets;
  [~, ~, opdata_out] = H5L.iterate(gid, 'H5_INDEX_NAME', ...
      'H5_ITER_INC', 0, @H5Literator_func, opdata);
  datasets = opdata_out;
end


function dset_name = list_dataset(dset_id)
  dset_name = H5I.get_name(dset_id);
end
