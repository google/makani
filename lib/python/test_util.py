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

"""Test helpers."""

import contextlib
import copy
import functools
import logging
import os
import sys
import tempfile

import gflags
import h5py
import makani
import mock
import numpy as np


class _ContextDecorator(object):
  """Base class for a context manager that also acts as a decorator.

  The name is stolen from Python 3's contextlib.ContextDecorator.

  Note that the _ContextDecorator's constructor must be called when using it as
  a decorator.  In particular, if FooClass is a _ContextDecorator whose
  constructor takes no args (other than `self`), then, @FooClass() must be used
  instead of @FooClass.
  """

  def __call__(self, func):
    @functools.wraps(func)
    def _Wrapper(*args, **kwargs):
      with self:
        return func(*args, **kwargs)
    return _Wrapper

  def __enter__(self):
    raise NotImplementedError()

  def __exit__(self, *unused_args):
    raise NotImplementedError()


class LogDisabler(_ContextDecorator):
  """Temporarily disables logging.

  All logging of severity less than or equal to the specified level is disabled
  while the LogDisabler is active.
  """

  def __init__(self, severity):
    self._severity = severity

  def __enter__(self):
    logging.disable(self._severity)

  def __exit__(self, exception_type, exception_value, traceback):
    logging.disable(logging.NOTSET)


def DisableWarnings():
  return LogDisabler(logging.WARNING)


class FlagValueSaver(_ContextDecorator):
  """Saves flag values on entry and restores them on exit.

  This is a light version of google3's FlagContext (see
  //testing/pybase/flagsaver.py), which additionally supports addition/deletion
  of flags and modification of flag metadata.
  """

  def __init__(self, flags=None):
    self._flags = gflags.FLAGS if flags is None else flags

  def __enter__(self):
    self._original_values = copy.deepcopy(self._flags.FlagValuesDict())

  def __exit__(self, exception_type, exception_value, traceback):
    for flag, value in self._original_values.items():
      setattr(self._flags, flag, value)


class SampleLogFileMissingError(Exception):
  pass


def CreateSampleHDF5File(file_name, num_samples):
  """Creates a new HDF5 file based on a reference file.

  Populates the /data and /parameters HDF5 groups with datasets that
  have the appropriate dtypes determined from a sample HDF5 file.

  Args:
    file_name: Path to HDF5 file to be created.
    num_samples: Number of samples for each dataset in '/data'.

  Returns:
    New h5py file object.

  Raises:
    SampleLogFileMissingError: The sample log file doesn't exist.
  """
  log_file = h5py.File(file_name, 'w')
  sample_log_path = os.path.join(makani.HOME, 'lib/pcap_to_hdf5/empty_log.h5')
  if not os.path.isfile(sample_log_path):
    raise SampleLogFileMissingError('The sample log file (%s) does not exist.' %
                                    sample_log_path)

  with contextlib.closing(h5py.File(sample_log_path, 'r')) as sample_log:
    parameters = log_file.create_group('parameters')
    for key in sample_log['parameters'].keys():
      parameters.create_dataset(key, shape=(1,),
                                dtype=sample_log['parameters'][key].dtype)

    messages = log_file.create_group('messages')
    for n in sample_log['messages'].keys():
      node = messages.create_group(n)
      for t in sample_log['messages'][n].keys():
        node.create_dataset(t, shape=(num_samples,),
                            dtype=sample_log['messages'][n][t].dtype)

  return log_file


class H5DatasetWriter(object):
  """Context manager to simplify writing to h5py File objects.

  In h5py, assigning to datasets of compound type is finicky.  Values
  for individual fields are not assignable (nothing happens, silently,
  if you try).  Given a h5py dataset, this context manager returns a
  numpy array whose fields can be assigned individually and then
  copies its contents.

  E.g.
    with H5DatasetWriter(log_file['parameters']['sim_params']) as sim_params:
      sim_params['phys_sim']['wind_speed'] = 5.0
  """

  def __init__(self, dataset):
    self._dataset = dataset
    self._temp = np.zeros_like(dataset)

  def __enter__(self):
    return self._temp

  def __exit__(self, exception_type, exception_value, traceback):
    # Without the slice operation, `_dataset` will bind to `_temp`
    # rather than copying its contents.
    self._dataset[:] = self._temp


class StdoutPatch(object):
  """Redirects sys.stdout to a tempfile.

  Useful for capturing print output.

  A single StdoutPatch may be reused in multiple contexts, but old contents will
  be clobbered each time a new context is entered.
  """

  def __init__(self):
    self._tempfile = None

  def __enter__(self):
    if self._tempfile is not None:
      self._tempfile.close()

    self._tempfile = tempfile.TemporaryFile()
    self._patch = mock.patch.object(sys, 'stdout', self._tempfile)
    self._patch.start()
    return self

  def __exit__(self, *args):
    self._patch.stop()
    self._tempfile.seek(0)
    self.contents = self._tempfile.read()

  def Read(self):
    self._tempfile.seek(0)
    return self._tempfile.read()
