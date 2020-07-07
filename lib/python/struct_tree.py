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

"""Add StructTree to easily inspect a complex dictionary and extract values."""

import contextlib
import ctypes

import h5py
from makani.lib.python import ctype_util
from makani.lib.python.h5_utils import h5_io
from makani.lib.python.h5_utils import numpy_utils
import numpy


# Types for compound data content.
_ST_H5PY_GROUP = 0
_ST_NUMPY_NDARRAY = 1
_ST_H5PY_DATASET = 2
_ST_CTYPES_STRUCTURE = 3
_ST_CTYPES_ARRAY = 4
_ST_DICT = 5
_ST_LIST = 6
_ST_NONE = 7
_ST_STRING = 8
# Types for indices other than _ST_STRING and _ST_LIST.
_ST_SCALAR = 9
_ST_SET = 10
_ST_TUPLE = 11

_ST_NDARRAY_LIKE = [_ST_H5PY_DATASET, _ST_NUMPY_NDARRAY]
_ST_DICT_LIKE = [_ST_H5PY_GROUP, _ST_DICT]
_ST_LIST_LIKE = [_ST_LIST, _ST_CTYPES_ARRAY]


def _TypeOfObj(obj):
  obj_type = type(obj)
  if obj_type == h5py.Group:
    return _ST_H5PY_GROUP
  elif obj_type == h5py.Dataset:
    return _ST_H5PY_DATASET
  elif obj_type == numpy.ndarray:
    return _ST_NUMPY_NDARRAY
  elif obj_type == dict:
    return _ST_DICT
  elif obj_type == list:
    return _ST_LIST
  elif obj_type == set:
    return _ST_SET
  elif obj_type == tuple:
    return _ST_TUPLE
  elif obj_type in (str, unicode):
    return _ST_STRING
  elif obj is None:
    return _ST_NONE
  elif isinstance(obj, ctypes.Structure):
    return _ST_CTYPES_STRUCTURE
  elif isinstance(obj, ctypes.Array):
    return _ST_CTYPES_ARRAY
  else:
    return _ST_SCALAR


class StructTree(object):
  """A class to retrieve and reorganize elements in complex dictionaries.

  This class enables users to index a JSON object or a HDF5 file with a single
  string composed of slices and attribute lookup. E.g.,
  message['node.messages[:].sequence'].

  Slices can be used for both dictionaries and lists. Slices for lists have the
  same conventional format (e.g., [:], [start:], [:end]: [start:end],
  [start:end:stride]). Slices for dictionaries can be [:] (to extract the
  entire dictionary) or [{key1, key2}] (to extract a sub-dictionary with the
  specified keys). The extracted result is further squashed.

  Keys in this struct can include any characters other than ',', ':', '[', ']',
  '{', and '}'. Typically, keys follow the format of variable names.
  """

  def __init__(self, seed, fail_silently=False, readonly=False):
    """Initialize the StructTree.

    Args:
      seed: The nested JSON dictionary (used only if mode is "JSON"), or the
          path to the HDF5 file (used only if mode is "HDF5").
      fail_silently: If True, operations returns None rather than raising
          exceptions.
      readonly: True if data is guaranteed to stay unchanged during the lifetime
          of this StructTree object. This is not strictly enforced, but
          attempting to modify a read-only StructTree may yield inconsistent
          results.

    Raises:
      ValueError: Raised if seed is invalid.
    """

    seed_type = _TypeOfObj(seed)
    if seed_type in [_ST_CTYPES_ARRAY, _ST_CTYPES_STRUCTURE, _ST_DICT, _ST_LIST,
                     _ST_NONE, _ST_NUMPY_NDARRAY]:
      # The nested JSON dictionary (used only if mode is "JSON"), or the path
      # to the HDF5 file (used only if mode is "HDF5").
      self._seed = seed
      # The mode for the struct tree, can be "HDF5" or "JSON".
      self._mode = 'JSON'
    elif seed_type in [_ST_H5PY_DATASET, _ST_H5PY_GROUP, _ST_STRING]:
      self._seed = seed
      self._mode = 'HDF5'
    else:
      raise ValueError('Invalid seed type "%s".' % type(seed))

    # If True, operations returns None rather than raising exceptions.
    self._fail_silently = fail_silently
    self._cache = {}
    self._readonly = readonly

  def _IsValidSubtreeSeed(self, seed):
    """Test whether an object can be used as a sub-tree's seed.

    Args:
      seed: The object to test.

    Returns:
      Boolean telling whether the seed is valid.

    Raises:
      ValueError: raised if the seed type is invalid.
    """
    seed_type = _TypeOfObj(seed)
    if seed_type in [
        _ST_CTYPES_STRUCTURE, _ST_CTYPES_ARRAY, _ST_DICT, _ST_H5PY_DATASET,
        _ST_H5PY_GROUP, _ST_NUMPY_NDARRAY, _ST_LIST, _ST_NONE]:
      return True
    elif seed_type in [_ST_STRING, _ST_SCALAR, _ST_TUPLE, _ST_SET]:
      return False
    else:
      raise ValueError('Invalid seed type "%s".' % seed_type)

  def _ConvertToBasicType(self, obj):
    """Convert a ctypes or HDF5 object to Python basic types."""
    obj_type = _TypeOfObj(obj)
    if obj_type in [_ST_CTYPES_STRUCTURE, _ST_CTYPES_ARRAY]:
      return ctype_util.CTypeToPython(obj)
    elif obj_type == _ST_H5PY_GROUP:
      return h5_io.H5GroupToDict(obj)
    elif obj_type == _ST_H5PY_DATASET:
      return numpy_utils.NumpyToJson(obj.value)
    elif obj_type == _ST_NUMPY_NDARRAY:
      return numpy_utils.NumpyToJson(obj)
    elif obj_type == _ST_LIST:
      return [self._ConvertToBasicType(item) for item in obj]
    elif obj_type == _ST_DICT:
      return {key: self._ConvertToBasicType(value)
              for key, value in obj.iteritems()}
    else:
      return obj

  def Data(self, convert_to_basic_types=False):
    """Returns the full data."""
    if self._mode == 'JSON':
      if convert_to_basic_types:
        return self._ConvertToBasicType(self._seed)
      else:
        return self._seed
    else:  # self._mode == 'HDF5':
      if _TypeOfObj(self._seed) == _ST_STRING:
        seed = h5py.File(self._seed, 'r')['/']
      else:
        seed = self._seed
      if convert_to_basic_types:
        return self._ConvertToBasicType(seed)
      else:
        return seed

  def __nonzero__(self):
    return bool(self.Data(convert_to_basic_types=False))

  def ListSkeleton(self, paths=None):
    skeleton = self.Skeleton(paths)
    return self._Flatten(skeleton)

  def _Flatten(self, tree):
    results = []
    if isinstance(tree, dict):
      for key, value in tree.iteritems():
        sub_results = self._Flatten(value)
        for sk, sv in sub_results:
          results.append(((key + '.' + sk) if sk else key, sv))
    else:
      results.append(('', tree))
    return results

  def Skeleton(self, paths=None, depth=None):
    """Returns a structural skeleton.

    Args:
      paths: A list or string to construct a FlexibleIndex to position the
          starting point.
      depth: The number of levels to probe and extract skeletons.

    Returns:
      A dictionary structured similarly to the object, with values removed.
      The leaves are None. A leaf can also be an empty dictionary/list
      indicating that the node has more details but is now a leaf only because
      of the limited `depth`.

    Raises:
      IOError: Raised if the seed points to a nonexistent HDF5 file.
      KeyError: Raised if the index points to an invalid location and
          self._fail_silently is False.
    """

    paths = FlexibleIndex(paths).Indices()

    if self._mode == 'HDF5':
      if isinstance(self._seed, (unicode, str)):
        with contextlib.closing(h5py.File(self._seed, 'r')) as f:
          node = self._Position(f['/'], paths)
          catalog = self._TraverseSkeleton(node, depth)
      else:
        node = self._Position(self._seed, paths)
        catalog = self._TraverseSkeleton(node, depth)
    else:  # self._mode == 'JSON':
      node = self._Position(self._seed, paths)
      catalog = self._TraverseSkeleton(node, depth)
    return catalog

  def __getitem__(self, indices):
    """Extract values within the dictionary and squash nested chains.

    Args:
      indices: A list or string to construct a FlexibleIndex and extract values.

    Returns:
      The desired values, or None.

    Raises:
      KeyError: Raised if the index points to an invalid location and
          self._fail_silently is False.
    """

    return self.Index(indices)

  def Subtree(self, indices):
    """Get a StructTree object as a subtree of the data structure."""
    node = self.Index(indices)

    if self._IsValidSubtreeSeed(node):
      return StructTree(node, self._fail_silently, self._readonly)
    else:
      return node

  def __contains__(self, indices):
    paths = FlexibleIndex(indices).Indices()

    if self._mode == 'HDF5':
      try:
        if isinstance(self._seed, (unicode, str)):
          with contextlib.closing(h5py.File(self._seed, 'r')) as f:
            return self._IsValidIndices(f['/'], paths)
        else:
          return self._IsValidIndices(self._seed, paths)
      except IOError:
        return False
    else:  # self._mode == 'JSON':
      return self._IsValidIndices(self._seed, paths)

  def Index(self, indices, root=None, convert_to_basic_types=False):
    if self._readonly and indices in self._cache:
      return self._cache[indices]
    result = self._RetrieveAttribute(
        indices, root, convert_to_basic_types=convert_to_basic_types)
    if self._readonly:
      self._cache[indices] = result
    return result

  def _RetrieveAttribute(self, indices, root=None, convert_to_basic_types=True):
    """Extract values within the dictionary and squash nested chains.

    Args:
      indices: A list or string to construct a FlexibleIndex and extract values.
      root: A list or string to construct a FlexibleIndex to position the
          starting point for extraction.
      convert_to_basic_types: If true, leaf values will be converted to basic
          types.

    Returns:
      The desired values, or None.

    Raises:
      KeyError: Raised if the index points to an invalid location and
          self._fail_silently is False.
    """

    paths = FlexibleIndex(indices).Indices()

    if self._mode == 'HDF5':
      if isinstance(self._seed, (unicode, str)):
        # If we do not convert to basic types, the function returns a H5Group
        # object, which will become invalid if we close the file.
        if not convert_to_basic_types:
          node = self._Position(h5py.File(self._seed, 'r')['/'], root)
          return self._TraverseValues(node, paths, indices,
                                      convert_to_basic_types)
        else:
          with contextlib.closing(h5py.File(self._seed, 'r')) as f:
            node = self._Position(f['/'], root)
            return self._TraverseValues(node, paths, indices,
                                        convert_to_basic_types)
      else:
        node = self._Position(self._seed, root)
        return self._TraverseValues(node, paths, indices,
                                    convert_to_basic_types)
    else:  # self._mode == 'JSON':
      node = self._Position(self._seed, root)
      return self._TraverseValues(node, paths, indices, convert_to_basic_types)

  def _ValidateKey(self, obj, key, full_index, obj_type):
    """Returns key if it is in obj, or exits according to fail_silently."""
    if obj_type == _ST_CTYPES_STRUCTURE:
      if hasattr(obj, key):
        return key
    else:
      if obj_type in _ST_NDARRAY_LIKE:
        obj = obj.dtype.names
      if key in obj:
        return key
    return self._RaiseOrNone('Failed to index with "%s": "%s" not in %s.' % (
        full_index, key, str(type(obj))))

  def _RaiseOrNone(self, message):
    if self._fail_silently:
      return None
    else:
      raise KeyError(message)

  def _HasAttribute(self, node, key):
    node_type = _TypeOfObj(node)
    if node_type in _ST_DICT_LIKE:
      return key in node
    elif node_type in _ST_NDARRAY_LIKE:
      return key in node.dtype.names
    elif node_type == _ST_CTYPES_STRUCTURE:
      return hasattr(node, key)
    return False

  def _Position(self, node, indices):
    """Indexes into node using a list of attributes."""

    paths = FlexibleIndex(indices).Indices()

    if not paths:
      return node

    for path in paths:
      if path:
        node_type = _TypeOfObj(node)
        if self._HasAttribute(node, path):
          if node_type == _ST_CTYPES_STRUCTURE:
            node = getattr(node, path)
          else:
            node = node[path]
        else:
          if self._fail_silently:
            return None
          else:
            raise KeyError('Invalid index "%s" for positioning.' % indices)
    return node

  def _IsValidIndices(self, node, indices):
    """Check whether the list of attributes is valid under a node."""

    paths = FlexibleIndex(indices).Indices()

    if not paths:
      return True

    for path in paths:
      if path:
        node_type = _TypeOfObj(node)
        if self._HasAttribute(node, path):
          if node_type == _ST_CTYPES_STRUCTURE:
            node = getattr(node, path)
          else:
            node = node[path]
        else:
          return False
    return True

  def _TraverseSkeleton(self, obj, depth):
    """Traverses a node and builds the skeleton."""
    obj_type = _TypeOfObj(obj)
    if obj_type in [_ST_H5PY_GROUP, _ST_DICT, _ST_CTYPES_STRUCTURE]:
      if obj_type == _ST_CTYPES_STRUCTURE:
        keys = ctype_util.Attributes(obj)
      else:
        keys = obj.keys()
      if depth is not None:
        if depth <= 0:
          if keys:
            # Indicate that it can be further expanded.
            return {}
          else:
            return None
        depth -= 1

      catalog = {}
      for key in keys:
        if obj_type == _ST_CTYPES_STRUCTURE:
          child = getattr(obj, key)
        else:
          child = obj[key]
        catalog[key] = self._TraverseSkeleton(child, depth)
      if not catalog:
        # The skeleton for an empty dict/h5py.Group is None.
        catalog = None
    elif obj_type in _ST_NDARRAY_LIKE:
      catalog = numpy_utils.DictSkeleton(obj, depth=depth)
    elif obj_type in _ST_LIST_LIKE:
      if depth is not None:
        if depth <= 0:
          # Indicate that it can be further expanded.
          return []
        depth -= 1

      if obj:
        catalog = []
        is_expandable = False
        for child in obj:
          sub_catalog = self._TraverseSkeleton(child, depth)
          catalog.append(sub_catalog)
          if sub_catalog is not None:
            is_expandable = True
        if catalog and not is_expandable:
          del catalog[:]
      else:
        # The skeleton for an empty list is None.
        catalog = None
    else:
      return None
    return catalog

  def _GetNodeData(self, obj, convert_to_basic_types):
    if not convert_to_basic_types:
      return obj
    else:
      return self._ConvertToBasicType(obj)

  def _TraverseValues(self, obj, indices, full_index, convert_to_basic_types):
    """Steps into the structure to extract values."""
    if not indices:
      # This is the leaf node.
      return self._GetNodeData(obj, convert_to_basic_types)

    obj_type = _TypeOfObj(obj)
    # Obtain the base object to index.
    if obj_type not in [_ST_DICT, _ST_LIST, _ST_CTYPES_STRUCTURE,
                        _ST_CTYPES_ARRAY, _ST_H5PY_GROUP, _ST_H5PY_DATASET,
                        _ST_NUMPY_NDARRAY]:
      return self._RaiseOrNone('Cannot index into type %s.' % type(obj))
    index = indices[0]
    idx_type = _TypeOfObj(index)
    if idx_type == _ST_STRING:
      # Index single attribute.
      if self._ValidateKey(obj, index, full_index, obj_type) is None:
        return None
      child = (getattr(obj, index) if obj_type == _ST_CTYPES_STRUCTURE
               else obj[index])
      return self._TraverseValues(child, indices[1:], full_index,
                                  convert_to_basic_types)
    elif idx_type == _ST_SET:
      pass
    elif idx_type != _ST_TUPLE:
      index = (index,)

    # Index with slices
    # Index dictionary-like structures.
    if (obj_type in [_ST_DICT, _ST_CTYPES_STRUCTURE] or
        (obj_type in _ST_NDARRAY_LIKE and obj.dtype.names is not None)):
      if len(index) == 1 and index[0] == slice(None, None, None):
        if obj_type == _ST_DICT:
          keys = obj.keys()
        elif obj_type == _ST_CTYPES_STRUCTURE:
          keys = ctype_util.Attributes(obj)
        else:
          keys = obj.dtype.names
      elif idx_type == _ST_SET:
        keys = index
      else:
        raise KeyError('Cannot index a dict-like object with "%s" (%s) in "%s".'
                       % (index, type(index).__name__, full_index))
      result = {}
      for key in keys:
        if self._ValidateKey(obj, key, full_index, obj_type) is None:
          return None
        child = (getattr(obj, key) if obj_type == _ST_CTYPES_STRUCTURE
                 else obj[key])
        result[key] = self._TraverseValues(child, indices[1:], full_index,
                                           convert_to_basic_types)

    # Index list and regular Numpy arrays.
    else:
      is_slice = False
      if idx_type == _ST_TUPLE:
        for index_part in index:
          if isinstance(index_part, slice):
            is_slice = True
            break
      else:
        raise KeyError('Cannot index a list-like object with "%s" (%s) in "%s".'
                       % (index, type(index).__name__, full_index))

      if obj_type in _ST_NDARRAY_LIKE:
        # Get a slice from Numpy array.
        try:
          obj = obj.__getitem__(index)
        except IndexError:
          return self._RaiseOrNone(
              '"%s" in "%s" failed to index the Numpy array.' % (index,
                                                                 full_index))
      elif obj_type in _ST_LIST_LIKE:
        # Get a slice from a list.
        try:
          obj = obj.__getitem__(*index)
        except IndexError:
          return self._RaiseOrNone('"%s" in "%s" failed to index the list.' %
                                   (index, full_index))
      else:
        return self._RaiseOrNone('Cannot index into type %s.' % type(obj))

      if len(indices) == 1:
        result = self._TraverseValues(obj, None, full_index,
                                      convert_to_basic_types)
      elif is_slice:
        result = [self._TraverseValues(item, indices[1:], full_index,
                                       convert_to_basic_types)
                  for item in obj]
      else:
        result = self._TraverseValues(obj, indices[1:], full_index,
                                      convert_to_basic_types)
    return result


def StringToSlices(slice_str):
  """Parse a string into a list of slices."""
  slice_parts = slice_str.split(',')
  input_parts = []
  for part in slice_parts:
    part = part.strip()
    try:
      part = int(part)
    except ValueError:
      try:
        part = slice(*[int(s) if s.strip() else None
                       for s in part.split(':')])
      except ValueError:
        # Cannot parse it as a slice, use it as a string.
        pass
    input_parts.append(part)
  return input_parts


class FlexibleIndex(object):
  """A flexible index that can convert a string to a list of indices.

  The indices are used to extract values from a nested struct.
  """

  def __init__(self, index):
    """Create a flexible index from a list, a string, or None.

    Args:
      index: A list of indices to extract values from the struct. Index has the
          following forms:
          (a) A string (e.g., "key") gets an attribute from the object/dict.
          (b) A slice (e.g., slice(None, 100, 2) gets a slice from a list. If
              it is slice(None, None, None), then it gets everything from a list
              or a dict.
          (c) A set of strings (e.g., {key1, key2}) gets a sub-dictionary that
              only includes the selected keys.
          (d) A tuple of integers and slices indexes a multi-dimensional list
              or regular numpy array.
          The index can also be a string that concatenates all indices. In
          this case string indices become `.<string>` and others become
          `[<slice>]`. E.g., key1.key2[:][{key3, key4}].key5

    Raises:
      ValueError: Raised if input is invalid.
    """

    idx_type = _TypeOfObj(index)
    if idx_type == _ST_LIST:
      self._indices = index
      return
    elif idx_type == _ST_TUPLE:
      self._indices = list(index)
      return
    elif idx_type == _ST_NONE:
      index = ''
    elif idx_type != _ST_STRING:
      raise ValueError('Invalid value to create a flexible index: "%s".'
                       % index)

    # Segment index
    segments = str(index).strip().split('.')
    indices = []
    for segment in segments:
      while True:
        slice_start = segment.find('[')
        if slice_start < 0:
          break
        elif slice_start > 0:
          indices.append(segment[:slice_start])
        segment = segment[slice_start+1:]
        slice_end = segment.index(']')
        slice_str = segment[:slice_end].strip()

        # Detect a set as a slice for dictionaries.
        if slice_str.startswith('{') and slice_str.endswith('}'):
          is_set_slice = True
          slice_str = slice_str[1:-1].strip()
        elif (not slice_str.startswith('{')) and (not slice_str.endswith('}')):
          is_set_slice = False
        else:
          raise ValueError('Invalid set syntax: "%s".' % slice_str)

        input_parts = StringToSlices(slice_str)

        if is_set_slice:
          input_parts = set(input_parts)
        else:
          input_parts = tuple(input_parts)
        indices.append(input_parts)
        segment = segment[slice_end+1:]
      if segment:
        indices.append(segment)
    self._indices = indices

  def Indices(self):
    return self._indices


def DictToD3Tree(skeleton_dict, name, parent_path=''):
  """Convert a dictionary to a format used for D3 visualization.

  Args:
    skeleton_dict: A dict of the StructTree skeleton.
    name: The name of the root node.
    parent_path: The struct_tree.FlexibleIndex of the root node. (default: '')

  Returns:
    A JSON object with information about nodes to be visualized.

  Example:
    If `name` is 'fruit' and `skeleton_dict` is
    {
        'apple': {'color': None},  # None means this is the root.
        'banana': (3,)
    }

    Then the result becomes:
    {
       'name': 'fruit',  # Name of the node.
       'path': '',      # Index of the node.
       'leaf': False,   # If True, this node cannot be further expanded.
       'children': [
           {
               'name': 'apple',
               'path': 'apple',
               'leaf': False,
               'children': [
                   {
                       'name': 'color',
                       'path': 'apple.color',
                       'leaf': True
                   }
               ]
           },
           {
               'name': 'banana[3]',
               'path': 'banana[:]',
               'leaf': True
           }
       ]
    }
  """

  root = {
      'name': name,
      'children': _DictToD3TreeTraverse(skeleton_dict, parent_path),
      'path': parent_path
  }
  root['leaf'] = not root['children']
  return root


def _DictToD3TreeTraverse(obj, path):
  """Traverse a dictionary node and return its corresponding D3 structure."""
  children = []
  if isinstance(obj, dict):
    for key, value in obj.iteritems():
      name = _ChildName(key, value)
      child_path = _ChildPath(path, key, value)
      node = {'name': name, 'path': child_path}
      node['leaf'] = _IsLeaf(value)
      grand_children = _DictToD3TreeTraverse(value, child_path)
      if grand_children is not None:
        node['children'] = grand_children
      children.append(node)
  elif isinstance(obj, list):
    for idx, value in enumerate(obj):
      name = _ChildName(str(idx), value)
      child_path = _ChildPath(path, idx, value)
      node = {'name': idx, 'path': child_path}
      node['leaf'] = _IsLeaf(value)
      grand_children = _DictToD3TreeTraverse(value, child_path)
      if grand_children is not None:
        node['children'] = grand_children
      children.append(node)
  return children if children else None


def _ChildName(key, value):
  name = key
  if value is not None:
    if isinstance(value, tuple):
      # value is a tuple describing the array shape.
      name += str(list(value))
  return name


def _ChildPath(parent_path, key, value):
  name = key
  if value:
    if isinstance(value, tuple):
      if len(value) == 1:
        name += '[:]'
      else:
        name += '[%s /* Choose a dimension! */]' % (
            ', '.join([':'] * len(value)))
  if not parent_path:
    return name
  else:
    return '%s.%s' % (parent_path, name)


def _IsLeaf(value):
  return (value is None) or isinstance(value, tuple)


def IsValidElement(attribute):
  # TODO: Remove this function and just use comparison with None.
  return attribute is not None
