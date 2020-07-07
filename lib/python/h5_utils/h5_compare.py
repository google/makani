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

"""Several utilities for working with HDF5 files in Python."""

import numpy


def _GetKeys(data):
  """Get the keys of a H5Dataset or numpy compound data type."""
  if hasattr(data, 'keys'):
    return set(data.keys())
  elif hasattr(data, 'dtype') and data.dtype.names:
    return set(data.dtype.names)
  else:
    return None


# The following methods generate strings to describe differences
# between entries in the two log files.
def _KeyMissing(side):
  """One log is missing a key present in the other log."""
  return 'Key missing from %s' % side


def _TypeMismatch(a, b):
  """The entries at the same position in A and B have differing types."""
  return 'Types do not match, %s v. %s' % (str(a), str(b))


def _ShapeMismatch(a, b):
  """The entries at the same position in A and B have different shapes."""
  return 'Shapes do not match, %s v. %s' % (str(a), str(b))


def _ValueMismatch(how_much):
  """The values at the same position of A and B do not match."""
  return 'Values mismatch, %s' % how_much


def _RecursiveCompare(a, b, truncate, excludes):
  """Recursively compares two data structures loaded by h5py."""
  if type(a) != type(b):  # pylint: disable=unidiomatic-typecheck
    return _TypeMismatch(type(a), type(b))

  a_keyset = _GetKeys(a)
  b_keyset = _GetKeys(b)

  if a_keyset is not None:
    # This handles the case where a numerical array is being
    # compared to a compound type.
    if b_keyset is None:
      return _TypeMismatch(a.dtype, b.dtype)
    # Compute which keys are common and which are exclusive to either
    # log.
    exclude_set = set([k for k in excludes.keys()
                       if excludes[k] is True])
    a_keyset -= exclude_set
    b_keyset -= exclude_set
    a_missing = b_keyset - a_keyset
    b_missing = a_keyset - b_keyset
    common = a_keyset & b_keyset

    report = {k: _KeyMissing('A') for k in a_missing}
    report.update({k: _KeyMissing('B') for k in b_missing})

    for k in common:
      child_excludes = excludes[k] if k in excludes else {}
      recurse = _RecursiveCompare(a[k], b[k], truncate, child_excludes)
      if recurse:
        report[k] = recurse

    return report if report else None
  elif isinstance(a, numpy.ndarray):
    if a.dtype.name != b.dtype.name:
      return _TypeMismatch(a.dtype, b.dtype)

    a_shape = list(a.shape)
    b_shape = list(b.shape)
    if len(a_shape) == len(b_shape) and truncate:
      n = min(a_shape[0], b_shape[0])
      if n < 0.9 * max(a_shape[0], b_shape[0]):
        return _ShapeMismatch(a_shape, b_shape)
      a_shape[0] = n
      b_shape[0] = n

    if a_shape != b_shape:
      return _ShapeMismatch(a_shape, b_shape)

    a_array = numpy.array(a)[0:a_shape[0], ...]
    b_array = numpy.array(b)[0:a_shape[0], ...]
    if not numpy.all(a_array == b_array):
      first_mismatch = numpy.where(a_array != b_array)[0][0]
      if a_array.dtype == numpy.bool:
        how_much = 'first mismatch: %d.' % first_mismatch
      else:
        percentiles = numpy.percentile(numpy.abs(a_array - b_array), [50, 100])
        how_much = ('first mismatch: %d, abs(a-b): %.2e median, %.2e max.'
                    % (first_mismatch, percentiles[0], percentiles[1]))
      return _ValueMismatch(how_much)

    return None
  else:
    raise SystemExit('Cannot handle type %s.' % str(type(a)))


def CompareLogs(a, b, truncate=False, excludes=None):
  """Produces a structure summarizing the difference between two log files.

  Args:
    a: A tree of dicts with leaves that are of type numpy.ndarray.
    b: A second tree as above to be compared to A.
    truncate: Optional boolean.  When True, the size of numpy.ndarray objects
      being compared will be truncated to the smaller first dimension by up to
      90% (default value is False).
    excludes: Optional tree of dicts whose leaves have the value True.  Any such
      leaf indicates a path that should not be compared
      (default value is {}).

  Returns:
    Returns None a and b are equivalent.  Otherwise returns a tree
    structure of dicts.  Each key maps to a key of one of the two log
    files.  The leaves are strings that summarize the difference
    between the two logs at that point.

  Raises:
    SystemExit: If an unexpected type is encountered.

  """
  excludes = {} if excludes is None else excludes
  assert _GetKeys(a) and _GetKeys(b)
  return _RecursiveCompare(a, b, truncate, excludes)


def _GeneratePathStr(path):
  """Formats a path to a field for printing."""
  return ((len(path) - 1) * '  ') + path[-1] if path else ''


def GenerateReportString(r, path=None, max_depth=None):
  """Generates a human-readable summary of differences generated by CompareLogs.

  Args:
    r: A result from running CompareLogs.
    path: Optional array of strings defining prefix for all paths.
      (default value: [])
    max_depth: Optional maximum depth to walk in the report.  Values that differ
      at a point deeper than max_depth still are reported, just without any
      details.

  Returns:
    A human-friendly summary of the difference.
  """
  path = path if path else []
  max_depth = max_depth - 1 if max_depth is not None else None

  if r is None:  # There were no differences.
    return ''
  elif max_depth is not None and max_depth < 0:  # Summarize below max depth.
    return _GeneratePathStr(path) + ': Children mismatch below max depth.'
  elif isinstance(r, dict):  # Recurse on dict objects.
    lines = [_GeneratePathStr(path)]
    lines += [GenerateReportString(v, path + [k], max_depth)
              for (k, v) in r.items()]
    return '\n'.join([l for l in lines if l])
  else:
    return _GeneratePathStr(path) + ': ' + r
