# Copyright 2020 Makani Technologies LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Utilities to ease importing from and exporting to HDF5."""

import contextlib
import os

import h5py
from makani.lib.python.h5_utils import numpy_utils
import numpy


class H5IndexError(Exception):
  """An exception raised when an index cannot be found in a log."""

  def __init__(self, root, record, message=''):
    Exception.__init__(self, root, record, message)
    self.root = root
    self.record = record
    self.message = message


def H5List(filename, node='/', list_all=False):
  """Lists groups and datasets rooted at a node in an HDF5 file.

  Args:
    filename: Path to the HDF5 file.
    node: Path to the HDF5 node to scan.
    list_all: Both groups and datasets are listed if True. Otherwise, only the
        datasets are listed.

  Returns:
    catalog: A list of strings, each is a path to an HDF5 object.
             None if node doesn't exist.

  Raises:
    H5IndexError: Indicates an invalid node name.
  """

  def TraverseNode(obj, path, list_all):
    """Traverses a node in HDF5 and builds the catalog."""
    catalog = []
    if isinstance(obj, h5py.Group):
      if list_all:
        catalog.append(path)
      for child in obj.iterkeys():
        child_path = os.path.join(path, child)
        catalog += TraverseNode(obj[child], child_path, list_all)
    elif isinstance(obj, h5py.Dataset):
      catalog.append(path)
    return catalog

  with contextlib.closing(h5py.File(filename, 'r')) as f:
    if node not in f:
      raise H5IndexError(node, [], 'Cannot index %s' % node)
    else:
      catalog = TraverseNode(f[node], node, list_all)
  return catalog


def H5ReadDataset(filename, node):
  """Reads an HDF5 dataset as a numpy array.

  Args:
    filename: Path to the HDF5 file.
    node: Path to the HDF5 node to scan.

  Raises:
    H5IndexError: If the node cannot be found.

  Returns:
    A numpy object, or None if node doesn't exist.
  """
  f = h5py.File(filename, 'r')
  if node not in f:
    raise H5IndexError(node, [], 'Cannot index %s' % node)
  dataset = f[node].value
  f.close()
  return dataset


def H5LoadDatasets(filename, node='/'):
  """Loads datasets rooted at a node in an HDF5 file.

  Args:
    filename: Path to the HDF5 file.
    node: Path to the HDF5 node to scan.

  Returns:
    datasets: A dictionary of datasets indexed by their node name.
              Each dataset is a numpy object.
  """
  catalog = H5List(filename, node, list_all=False)
  datasets = {}
  f = h5py.File(filename, 'r')
  for name in catalog:
    datasets[name] = f[name].value
  f.close()
  return datasets


def H5FileToDict(filename, root='/', record=None):
  """Loads a portion of an HDF5 file into a dict of plain Numpy arrays.

  Args:
    filename: Filename to load.
    root: Optional path in the HDF5 pseudo-file system to load
        (default value: '/').
    record: List of strings.  Further indexing into compound types located
        at root.

  Raises:
    H5IndexError: If the root or record index cannot be found.

  Returns:
    A dict whose values are either ndarray-like objects (raw data) or further
    dict-like objects.

  Example:
    H5FileToDict('logs/last.h5',
                 '/messages/kAioNodeControllerA/',
                 ['message', 'control'])
  """

  def _ConvertNumpyToDict(data):
    """Converts dict of compound Numpy structs into that of plain arrays."""
    if isinstance(data, dict):
      for key, value in data.iteritems():
        data[key] = _ConvertNumpyToDict(value)
      return data
    elif isinstance(data, numpy.ndarray):
      return numpy_utils.IndexAsDict(data)
    else:
      return data

  data = H5Load(filename, root)
  if isinstance(data, numpy.ndarray):
    data = numpy_utils.IndexAsDict(data, record)
  elif record:
    raise H5IndexError(root, record)
  else:
    data = _ConvertNumpyToDict(data)
  return data


def H5Load(filename, root='/'):
  """Loads a portion of an HDF5 file into a nested dict of datasets.

  The leaves are Numpy structs correspond to the HDF5 datasets.

  Args:
    filename: Filename to load.
    root: Optional path in the HDF5 pseudo-file system to load
        (default value: '/').

  Raises:
    H5IndexError: If the root index cannot be found.

  Returns:
    A dict whose values are either ndarray-like objects (raw data) or further
    dict-like objects.
  """
  f = h5py.File(filename, 'r')
  try:
    selection = f[root]
  except KeyError:
    raise H5IndexError(root, [], 'Cannot index %s' % root)

  if isinstance(selection, h5py.Group):
    return H5GroupToDict(selection)
  else:
    # Return the dataset as a Numpy object.
    return selection.value


def H5Dump(filename, datasets, compression_level=None):
  """Dumps datasets to an HDF5 file.

  Args:
    filename: Path to the HDF5 file.
    datasets: A dictionary of datasets indexed by their node name.
              Each dataset is a numpy object.
    compression_level: Level for gzip compression, an integer from 0-9.
        Empirically, 5 is a good compromise between compression and speed.

  Raises:
    ValueError: Indicates incorrectly structured datasets.
  """

  def _DumpGroup(parent, datasets):
    """Traverses datasets and builds HDF5 nodes."""
    for key, value in datasets.iteritems():
      if isinstance(value, dict):
        child = parent.create_group(key)
        _DumpGroup(child, value)
      elif value is None:
        child = parent.create_group(key)
      else:
        compression_kwargs = {}
        if compression_level:
          compression_kwargs = {'compression': 'gzip',
                                'compression_opts': compression_level}
        parent.create_dataset(key, data=value, **compression_kwargs)

  if not isinstance(datasets, dict):
    raise ValueError('H5Dump: datasets must be a (nested) dictionary of '
                     'numpy arrays')
  f = h5py.File(filename, 'w')
  _DumpGroup(f, datasets)
  f.close()


def H5GroupToDict(g):
  """Private function for transforming an HDF5 group to a dict."""
  return {k: H5GroupToDict(g[k]) if isinstance(g[k], h5py.Group) else g[k].value
          for k in g.iterkeys()}
