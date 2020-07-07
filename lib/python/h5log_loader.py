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

"""Provides a more user friendly interface to Makani HDF5 logs.

Typical use case:
  h5 = H5LogLoader()
  h5.Open(['file1.h5', 'file2.h5'])

  # Dictionary interface.
  plt.figure()
  t = h5.capture_time['FcA/FlightComputerSensor']
  d = h5['FcA/FlightComputerSensor/aux/mag']
  plt.plot(t, d)

  # Function call interface.
  plt.figure()
  for f in h5.GetFields(r'^Servo[^/]+/ServoStatus$'):
    t = h5.GetCaptureTime(f)
    d = h5.GetData(f + '/angle_measured')
    plt.plot(t, d, label=h5.GetNodeName(f))
  plt.legend()
"""

# NOTICE: To ease analysis, please do not depend on the Makani repository!
import collections
import re
import sys

import h5py
import numpy as np


def PrintProgress(count, total, suffix=''):
  """Print a progress bar to stdout."""
  bar_len = 60
  filled_len = int(round(bar_len * count / float(total)))

  percent = 100.0 * count / float(total)
  bar = '=' * filled_len + '-' * (bar_len - filled_len)
  sys.stdout.write('[%s] % 4.1f%% ...%s\r' % (bar, percent, suffix))

  if count == total:
    sys.stdout.write('\nDone!\n')

  sys.stdout.flush()


def _H5BuildDtypeTree(dtype, prefix=''):
  """Recursively build an array of paths, starting from an HDF5 dtype."""
  values = [str(prefix).lstrip('/')]
  if dtype.fields:
    for key, item in dtype.fields.iteritems():
      values += _H5BuildDtypeTree(item[0].base, prefix + '/' + key)
  return values


def _H5BuildGroupTree(group, prefix=''):
  """Recursively build an array of paths to each HDF5 dataset."""
  values = []
  for key, item in group.iteritems():
    if isinstance(item, h5py.Dataset):
      values.append(str(prefix + '/' + key).lstrip('/'))
    elif isinstance(item, h5py.Group):
      values += _H5BuildGroupTree(item, prefix + '/' + key)
  return values


def _H5BuildGroupAndDtypeTree(group, prefix=''):
  """Recursively build an array of paths, starting from an HDF5 group."""
  values = []
  for key, item in group.iteritems():
    if isinstance(item, h5py.Dataset):
      values += _H5BuildDtypeTree(item.dtype.base, prefix + '/' + key)
    elif isinstance(item, h5py.Group):
      values += _H5BuildGroupAndDtypeTree(item, prefix + '/' + key)
  return values


def _NormalizeFileNames(filenames):
  if filenames is None:
    filenames = []
  elif not isinstance(filenames, list):
    filenames = [filenames]
  return filenames


class _H5DataCache(object):
  """Provides a simple cache interface to H5*Cache classes."""

  def __init__(self):
    self._data_cache = {}  # Map path to cached data.

  def ClearCached(self):
    """Free cached memory."""
    self._data_cache = {}

  def GetCached(self, path):
    """Get data and cache the result."""
    data = self._data_cache.get(path)
    if data is None:
      data = self.GetData(path)
      self._data_cache[path] = data
    return np.copy(data)  # Copy data to prevent changes to cached values.

  def GetData(self, path):  # pylint: disable=unused-argument
    """Get data without caching the result."""
    raise NotImplementedError


class _H5DataLog(object):
  """Load HDF5 files."""

  def __init__(self):
    # Abstract HDF5 interface to improve access performance.
    self._data_logs = []   # A list of HDF5 file objects.
    self._data_paths = {}  # A mapping from path to HDF5 data path.

  def __del__(self):
    self.Close()

  def Open(self, filenames):
    """Open HDF5 log files.

    Args:
      filenames: A list of log HDF5 files, sorted in time.
    """
    # Python's garbage collection does not always work well with HDF5 data
    # structures. Close all files before reopening them again.
    self.Close()

    # Load HDF5 files as read-only.
    self._data_logs = [h5py.File(f, 'r') for f in filenames]

    # Index HDF5 data structure to improve path lookup performance.
    paths = set()
    for d in self._data_logs:
      paths |= set(_H5BuildGroupAndDtypeTree(d))

    self._data_paths = {self._GetShortPath(p): p for p in paths}

  def _GetShortPath(self, path):
    """Remove overly verbose prefixes."""
    return path.replace('kAioNode', '').replace('kMessageType', '')

  def Save(self, filename, verbose=False):
    """Save data from all input files to a single, merged HDF5."""
    with h5py.File(filename, 'w') as fp:
      # Build paths for each HDF5 dataset.
      paths = set()
      for d in self._data_logs:
        paths |= set(_H5BuildGroupTree(d))
      dataset_paths = {self._GetShortPath(p): p for p in paths}

      dataset_count = len(dataset_paths)
      i = 0
      for short_path, path in dataset_paths.iteritems():
        fp.create_dataset(path, data=self.GetData(short_path))
        i += 1
        if verbose:
          PrintProgress(i, dataset_count, 'Concatenating HDF5 datasets')

  def Close(self):
    """Close all HDF5 log files and free associated memory."""
    while self._data_logs:
      d = self._data_logs.pop()
      d.close()
    self.__init__()

  def GetData(self, path):
    """Load data from HDF5 structure as data[field 0][field 1] ... [field N]."""
    arrays = []
    split_path = self._data_paths[path].split('/')
    for d in self._data_logs:
      try:
        for p in split_path:
          d = d[p]
        arrays.append(d)
      except KeyError:
        pass
    if len(arrays) == 1:
      return arrays[0]
    return np.concatenate(arrays, axis=0)

  def GetPathsRegex(self, re_match, re_sub=r'\g<0>'):
    """Get a list of paths matching the given regex pattern."""
    expr = re.compile(re_match)
    paths = set()
    for path in self._data_paths:
      match = expr.match(path)
      if match:
        paths.add(match.expand(re_sub))
    return sorted(list(paths))


class _H5AioTimeCache(_H5DataCache):
  """Load and cache local node time."""

  def __init__(self, loader):
    super(_H5AioTimeCache, self).__init__()
    self._loader = loader
    self._aio_time_offset = collections.defaultdict(float)

  def GetData(self, path):
    """Get the local node time associated with the given path."""
    path = self._loader.GetAioHeaderPath(path)
    data = self._loader.GetData(path + '/timestamp').astype(long)
    for i in np.where(np.diff(data) < 0)[0]:
      data[i + 1:] += 2**32
    return data.astype(float) * 1e-6 - self._aio_time_offset[path]

  def GetOffset(self, path):
    """Get the local node time offset assicated with the given path."""
    path = self._loader.GetAioHeaderPath(path)
    return self._aio_time_offset[path]

  def SetOffset(self, path, offset):
    """Set the local node time offset assicated with the given path."""
    path = self._loader.GetAioHeaderPath(path)
    if path in self._data_cache:
      self._data_cache[path] += self._aio_time_offset[path] - offset
    self._aio_time_offset[path] = offset

  def ShiftOffset(self, path, delta):
    """Shift the local node time offset assicated with the given path."""
    offset = self.GetOffset(path) + delta
    self.SetOffset(path, offset)


class _H5CaptureTimeCache(_H5DataCache):
  """Load and cache capture time."""

  def __init__(self, loader):
    super(_H5CaptureTimeCache, self).__init__()
    self._loader = loader
    self._capture_time_offset = 0  # Offset common to all datasets.

  def GetData(self, path):
    """Get the capture time associated with the given path."""
    path = self._loader.GetCaptureHeaderPath(path)
    tv_sec = self._loader.GetData(path + '/tv_sec').astype(float)
    tv_usec = self._loader.GetData(path + '/tv_usec').astype(float)
    return tv_sec + tv_usec * 1e-6 - self._capture_time_offset

  def GetOffset(self):
    """Get the global capture time offset."""
    return self._capture_time_offset

  def SetOffset(self, offset):
    """Set the global capture time offset."""
    for t in self._data_cache.itervalues():
      t += self._capture_time_offset - offset
    self._capture_time_offset = offset

  def ShiftOffset(self, delta):
    """Shift the global capture time offset."""
    offset = self.GetOffset() + delta
    self.SetOffset(offset)


class _H5GpsTimeCache(_H5DataCache):
  """Load and cache GPS time."""

  def __init__(self, loader):
    super(_H5GpsTimeCache, self).__init__()
    self._loader = loader
    self._gps_time_offset = 0  # Offset common to all datasets.
    self._gps_time_cal = None  # Calibration from capture time to GPS time.

  def GetData(self, path):
    """Get the GPS time associated with the given path."""
    if self._gps_time_cal is None:
      t_cap = []
      t_gps = []
      loader = self._loader
      fields = loader.GetPathsRegex(
          r'^messages/[^/]+/NovAtelObservations/message$')
      for f in fields:
        time_status = loader.GetData(f + '/range/timestamp/time_status')
        time_of_week_ms = loader.GetData(f + '/range/timestamp/tow').astype(int)
        pps_latency_us = loader.GetData(f + '/pps_latency_usec').astype(int)

        # See NovAtel OEM6 docs for "GPS Reference Time Status" on page 35.
        i = np.where(time_status >= 100)[0]
        # Validate time-of-week range.
        gps_week_ms = 7 * 24 * 3600 * 1000
        i = i[np.where(time_of_week_ms[i] < gps_week_ms)[0]]
        # Validate PPS latency range.
        i = i[np.where(0 < pps_latency_us[i])[0]]
        i = i[np.where(pps_latency_us[i] < 1000 * 1000)[0]]
        # Remove invalid indices.
        cap_time = loader.GetCaptureTime(f)[i]
        time_of_week_ms = time_of_week_ms[i]
        pps_latency_us = pps_latency_us[i]

        # Handle GPS week rollovers.
        for i in np.where(np.diff(time_of_week_ms) < 0)[0]:
          time_of_week_ms[i + 1:] += gps_week_ms

        # To communicate GPS time precisely, the GPS receiver provides a pulse
        # per second interrupt signal that occurs on the GPS time-of-week [ms]
        # second transition (i.e., when time-of-week % 1000 == 0). The GPS
        # receiver also transmits the time-of-week value in each message. We
        # can then relate the reception time of any message to the time of
        # validity by measuring the time between the PPS interrupt and message
        # reception.
        #                Data D2 valid
        #     PPS        | Data D2 received       PPS
        #      |         | |                       |
        # --D19!-D0-.-D1-.-D2-.-D3-. ... .-D18.-D19!-D0-.-D1-.-D2-- ...
        #      |         |-|  Transport delay
        #      |<--------->|  PPS latency
        #
        transport_delay_us = pps_latency_us - (time_of_week_ms % 1000) * 1000
        # Compute times.
        t_cap.append(cap_time)
        t_gps.append(time_of_week_ms * 1e-3 + transport_delay_us * 1e-6)

      t_cap = np.concatenate(t_cap)
      t_gps = np.concatenate(t_gps)

      # Reject outliers. Loop multiple times to improve estimate.
      for _ in range(3):
        # Compute linear fit coefficients: (gps_time) = m * (capture_time) + b.
        p = np.polyfit(t_cap, t_gps, 1)
        # Compute error in linear fit: (delta) = (measurement) - (estimate).
        delta = t_gps - np.polyval(p, t_cap)
        # Find data with error less than 3 sigma.
        i = np.where(np.abs(delta) < 3.0 * np.std(delta))
        t_cap = t_cap[i]
        t_gps = t_gps[i]
      self._gps_time_cal = np.polyfit(t_cap, t_gps, 1)

    # Evaluate linear fit: (gps_time) = m * (capture_time) + b.
    return np.polyval(
        self._gps_time_cal,
        self._loader.GetCaptureTime(path)) - self._gps_time_offset

  def GetOffset(self):
    """Get the global GPS time offset."""
    return self._gps_time_offset

  def SetOffset(self, offset):
    """Set the global GPS time offset."""
    for t in self._data_cache.itervalues():
      t += self._gps_time_offset - offset
    self._gps_time_offset = offset

  def ShiftOffset(self, delta):
    """Shift the global GPS time offset."""
    offset = self.GetOffset() + delta
    self.SetOffset(offset)


class H5DataLoader(object):
  """Load and cache log data."""

  def __init__(self, filenames=None):
    self._filenames = _NormalizeFileNames(filenames)
    self._data_log = _H5DataLog()
    self._aio_time_cache = _H5AioTimeCache(self)
    self._capture_time_cache = _H5CaptureTimeCache(self)
    self._gps_time_cache = _H5GpsTimeCache(self)
    self._relative_time_cache = _H5CaptureTimeCache(self)

  def __enter__(self):
    self.Open(self._filenames)
    return self

  def __exit__(self, *unused_args):
    self.Close()

  def Open(self, filenames=None):
    """Open HDF5 log files.

    Args:
      filenames: A list of log HDF5 files, sorted in time.
    """
    self.Close()

    if filenames is not None:
      self._filenames = _NormalizeFileNames(filenames)
    if self._filenames is not None:
      self._data_log.Open(self._filenames)
    if self._data_log.GetPathsRegex('^info/min_tv_[u]?sec'):
      min_sec = self._data_log.GetData('info/min_tv_sec').astype(float)
      min_usec = self._data_log.GetData('info/min_tv_usec').astype(float)
      offset = min_sec + min_usec * 1e-6
      self._relative_time_cache.SetOffset(offset)

  def Save(self, filename, verbose=False):
    """Save data from all input files to a single, merged HDF5."""
    self._data_log.Save(filename, verbose)

  def Close(self):
    """Close all HDF5 log files and free associated memory."""
    self._data_log.Close()
    self._filenames = []
    self.ClearCached()

  def ClearCached(self):
    """Free cached memory."""
    for t in self.__dict__.values():
      if isinstance(t, _H5DataCache):
        t.ClearCached()

  def GetData(self, path):
    """Get data associated with the given path."""
    return self._data_log.GetData(path)

  def GetAioTime(self, path):
    """Get the local node time associated with the given path."""
    return self._aio_time_cache.GetCached(self.GetAioHeaderPath(path))

  def GetAioTimeOffset(self, path):
    """Get the local node time offset associated with the given path."""
    return self._aio_time_cache.GetOffset(self.GetAioHeaderPath(path))

  def SetAioTimeOffset(self, path, offset):
    """Set the local node time offset associated with the given path."""
    self._aio_time_cache.SetOffset(self.GetAioHeaderPath(path), offset)

  def ShiftAioTimeOffset(self, path, delta):
    """Shift the local node time offset associated with the given path."""
    self._aio_time_cache.ShiftOffset(self.GetAioHeaderPath(path), delta)

  def GetCaptureTime(self, path):
    """Get the capture time associated with the given path."""
    return self._capture_time_cache.GetCached(self.GetCaptureHeaderPath(path))

  def GetCaptureTimeOffset(self):
    """Get the global capture time offset."""
    return self._capture_time_cache.GetOffset()

  def SetCaptureTimeOffset(self, offset):
    """Set the global capture time offset."""
    self._capture_time_cache.SetOffset(offset)

  def ShiftCaptureTimeOffset(self, delta):
    """Shift the global capture time offset."""
    self._capture_time_cache.ShiftOffset(delta)

  def GetGpsTime(self, path):
    """Get the GPS time associated with the given path."""
    return self._gps_time_cache.GetCached(self.GetCaptureHeaderPath(path))

  def GetGpsTimeOffset(self):
    """Get the global GPS time offset."""
    return self._gps_time_cache.GetOffset()

  def SetGpsTimeOffset(self, offset):
    """Set the global GPS time offset."""
    self._gps_time_cache.SetOffset(offset)

  def ShiftGpsTimeOffset(self, delta):
    """Shift the global GPS time offset."""
    self._gps_time_cache.ShiftOffset(delta)

  def GetRelativeTime(self, path):
    """Get the relative time associated with the given path."""
    return self._relative_time_cache.GetCached(self.GetCaptureHeaderPath(path))

  def GetRelativeTimeOffset(self):
    """Get the global relative time offset."""
    return self._relative_time_cache.GetOffset()

  def SetRelativeTimeOffset(self, offset):
    """Set the global relative time offset."""
    self._relative_time_cache.SetOffset(offset)

  def ShiftRelativeTimeOffset(self, delta):
    """Shift the global relative time offset."""
    self._relative_time_cache.ShiftOffset(delta)

  def GetAioHeaderPath(self, path):
    """Get the AioHeader base path from the given path."""
    if not self.IsMessagePath(path):
      raise ValueError('Invalid path specified:', path)
    return '/'.join(path.split('/')[0:3] + ['aio_header'])

  def GetCaptureHeaderPath(self, path):
    """Get the CaptureHeader base path from the given path."""
    if not self.IsMessagePath(path):
      raise ValueError('Invalid path specified:', path)
    return '/'.join(path.split('/')[0:3] + ['capture_header'])

  def IsNodePath(self, path):
    """Determine if the path contains a valid node path."""
    return re.match(r'^messages/[^/]+(/.+)?$', path)

  def IsMessagePath(self, path):
    """Determine if the path contains a valid message path."""
    return re.match(r'^messages/[^/]+/[^/]+(/.+)?$', path)

  def IsDataPath(self, path):
    """Determine if the path contains a valid data path."""
    return re.match(r'^messages/[^/]+/[^/]+/message/.+$', path)

  def GetPathsRegex(self, re_match, re_sub=r'\g<0>'):
    """Get a list of paths matching the given regex pattern."""
    return self._data_log.GetPathsRegex(re_match, re_sub)

  def GetNodeName(self, path):
    """Get the node name associated with the given path."""
    if not self.IsNodePath(path):
      raise ValueError('Invalid path specified:', path)
    return path.split('/')[1]

  def GetMessageName(self, path):
    """Get the message name associated with the given path."""
    if not self.IsMessagePath(path):
      raise ValueError('Invalid path specified:', path)
    return path.split('/')[2]

  def GetDataName(self, path):
    """Get the data field name associated with the given path."""
    if not self.IsDataPath(path):
      raise ValueError('Invalid path specified:', path)
    return path.split('/', 4)[4:]

  @property
  def filenames(self):
    """Get HDF5 file names."""
    return self._filenames


class H5DataDict(object):
  """Creates a path abstraction to the H5DataLoader object."""

  def __init__(self, loader, get_data_function, re_match_sub_dict):
    """Initialize the H5DataDict object.

    Args:
      loader: A H5DataLoader object.
      get_data_function: A H5DataLoader function to map self.GetData().
      re_match_sub_dict: A dict mapping path regex pattern to substitution.
    """
    self._loader = loader
    self._get_data_function = get_data_function
    self._re_match_sub_dict = re_match_sub_dict
    self._dict = {}

  def BuildDict(self):
    """Build the dictionary of data paths."""
    self._dict = {}
    for re_match, re_sub in self._re_match_sub_dict.iteritems():
      expr = re.compile(re_match)
      for path in self._loader.GetPathsRegex(expr):
        self._dict[expr.sub(re_sub, path)] = path

  def GetPathsRegex(self, pattern):
    """Get a list of paths matching the given regex pattern."""
    expr = re.compile(pattern)
    return sorted([f for f in self._dict if expr.match(f)])

  def GetPaths(self, prefix):
    """Get a list of paths with the given prefix."""
    return self.GetPathsRegex(r'^(' + prefix + r')(/.+)?$')

  def GetSubpaths(self, prefix, recursive=False):
    """Get a list of subpaths of the given prefix."""
    if recursive:
      return self.GetPathsRegex(r'^(' + prefix + r')/.+$')
    else:
      return self.GetPathsRegex(r'^(' + prefix + r')/[^/]+$')

  def GetData(self, path):
    """Get data associated with the given path."""
    return self._get_data_function(self._dict[path])

  def GetAioTime(self, path):
    """Get the local node time associated with the given path."""
    return self._loader.GetAioTime(self._dict[path])

  def GetAioTimeOffset(self, path):
    """Get the local node time offset associated with the given path."""
    return self._loader.GetAioTimeOffset(self._dict[path])

  def SetAioTimeOffset(self, path, offset):
    """Set the local node time offset associated with the given path."""
    self._loader.SetAioTimeOffset(self._dict[path], offset)

  def ShiftAioTimeOffset(self, path, delta):
    """Shift the local node time offset associated with the given path."""
    self._loader.ShiftAioTimeOffset(self._dict[path], delta)

  def GetCaptureTime(self, path):
    """Get the capture time associated with the given path."""
    return self._loader.GetCaptureTime(self._dict[path])

  def GetCaptureTimeOffset(self):
    """Get the global capture time offset."""
    return self._loader.GetCaptureTimeOffset()

  def SetCaptureTimeOffset(self, offset):
    """Set the global capture time offset."""
    self._loader.SetCaptureTimeOffset(offset)

  def ShiftCaptureTimeOffset(self, delta):
    """Shift the global capture time offset."""
    self._loader.ShiftCaptureTimeOffset(delta)

  def GetGpsTime(self, path):
    """Get the GPS time associated with the given path."""
    return self._loader.GetGpsTime(self._dict[path])

  def GetGpsTimeOffset(self):
    """Get the global GPS time offset."""
    return self._loader.GetGpsTimeOffset()

  def SetGpsTimeOffset(self, offset):
    """Set the global GPS time offset."""
    self._loader.SetGpsTimeOffset(offset)

  def ShiftGpsTimeOffset(self, delta):
    """Shift the global GPS time offset."""
    self._loader.ShiftGpsTimeOffset(delta)

  def GetRelativeTime(self, path):
    """Get the relative time associated with the given path."""
    return self._loader.GetRelativeTime(self._dict[path])

  def GetRelativeTimeOffset(self):
    """Get the global relative time offset."""
    return self._loader.GetRelativeTimeOffset()

  def SetRelativeTimeOffset(self, offset):
    """Set the global relative time offset."""
    self._loader.SetRelativeTimeOffset(offset)

  def ShiftRelativeTimeOffset(self, delta):
    """Shift the global relative time offset."""
    self._loader.ShiftRelativeTimeOffset(delta)

  def GetNodeName(self, path):
    """Get the node name associated with the given path."""
    return self._loader.GetNodeName(self._dict[path])

  def GetMessageName(self, path):
    """Get the message name associated with the given path."""
    return self._loader.GetMessageName(self._dict[path])

  def GetDataName(self, path):
    """Get the data field name associated with the given path."""
    return self._loader.GetDataName(self._dict[path])

  def keys(self):  # pylint: disable=invalid-name
    """Get all possible paths, used for dictionary self[] auto-completion."""
    return sorted(self._dict.keys())

  def __contains__(self, path):
    """Provide 'in' interface."""
    return path in self._dict

  def __getitem__(self, path):
    """Provide self[] dictionary access to data."""
    return self.GetData(path)


class H5LogLoader(H5DataDict):
  """Abstract a HDF5 log files to simplify interface."""

  def __init__(self, filenames=None):
    self._data_loader = H5DataLoader(filenames)

    super(H5LogLoader, self).__init__(
        self._data_loader, self._data_loader.GetData,
        {
            r'^messages/([^/]+/[^/]+)$': r'\1',
            r'^messages/([^/]+/[^/]+)/message/(.+)$': r'\1/\2',
        })

    self._aio_header_dict = H5DataDict(
        self._data_loader, self._data_loader.GetData,
        {
            r'^messages/([^/]+/[^/]+)$': r'\1',
            r'^messages/([^/]+/[^/]+)/aio_header/(.+)$': r'\1/\2',
        })

    self._aio_time_dict = H5DataDict(
        self._data_loader, self._data_loader.GetAioTime,
        {r'^messages/([^/]+/[^/]+)$': r'\1'})

    self._bad_packets_dict = H5DataDict(
        self._data_loader, self._data_loader.GetData,
        {r'^bad_packets/(.+)$': r'\1'})

    self._capture_header_dict = H5DataDict(
        self._data_loader, self._data_loader.GetData,
        {
            r'^messages/([^/]+/[^/]+)$': r'\1',
            r'^messages/([^/]+/[^/]+)/capture_header/(.+)$': r'\1/\2',
        })

    self._capture_time_dict = H5DataDict(
        self._data_loader, self._data_loader.GetCaptureTime,
        {r'^messages/([^/]+/[^/]+)$': r'\1'})

    self._gps_time_dict = H5DataDict(
        self._data_loader, self._data_loader.GetGpsTime,
        {r'^messages/([^/]+/[^/]+)$': r'\1'})

    self._info_dict = H5DataDict(
        self._data_loader, self._data_loader.GetData,
        {r'^info/(.+)$': r'\1'})

    self._relative_time_dict = H5DataDict(
        self._data_loader, self._data_loader.GetRelativeTime,
        {r'^messages/([^/]+/[^/]+)$': r'\1'})

    self._param_dict = H5DataDict(
        self._data_loader, self._data_loader.GetData,
        {r'^parameters/(.+)$': r'\1'})

  def __enter__(self):
    self.Open()
    return self

  def __exit__(self, *unused_args):
    self.Close()

  def Open(self, filenames=None):
    """Open HDF5 log files.

    Args:
      filenames: A list of log HDF5 files, sorted in time.
    """
    self._data_loader.Open(filenames)
    self.BuildDict()

  def Save(self, filename, verbose=False):
    """Save data from all input files to a single, merged HDF5."""
    self._data_loader.Save(filename, verbose)

  def Close(self):
    """Close all HDF5 log files and free associated memory."""
    self._data_loader.Close()
    self.ClearCached()
    self.BuildDict()

  def ClearCached(self):
    """Free cached memory."""
    self._data_loader.ClearCached()

  def BuildDict(self):
    """Build the dictionaries of data paths."""
    super(H5LogLoader, self).BuildDict()
    for t in self.__dict__.values():
      if isinstance(t, H5DataDict):
        t.BuildDict()

  def GetNodes(self):
    """Get a list of nodes found in the log file."""
    pattern = r'^messages/([^/]+)/[^/]+/.+'
    return self._data_loader.GetPathsRegex(pattern, r'\1')

  def GetMessageTypes(self, node=r'[^/]+'):
    """Get a list of message types found in the log file."""
    pattern = r'^messages/' + node + r'/([^/]+)/.+$'
    return self._data_loader.GetPathsRegex(pattern, r'\1')

  @property
  def filenames(self):
    """Get HDF5 file names."""
    return self._data_loader.filenames

  @property
  def aio_header(self):
    return self._aio_header_dict

  @property
  def aio_time(self):
    return self._aio_time_dict

  @property
  def bad_packets(self):
    return self._bad_packets_dict

  @property
  def capture_header(self):
    return self._capture_header_dict

  @property
  def capture_time(self):
    return self._capture_time_dict

  @property
  def data(self):
    return self

  @property
  def gps_time(self):
    return self._gps_time_dict

  @property
  def info(self):
    return self._info_dict

  @property
  def param(self):
    return self._param_dict

  @property
  def relative_time(self):
    return self._relative_time_dict
