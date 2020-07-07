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

"""Base classes for check lists and items to check.

Classes defined here will be used for both real-time monitor and log analysis.
"""

import collections
import copy
import inspect
import json
import logging
import os
import string

import makani
from makani.analysis.checks import check_range
from makani.analysis.checks import log_util
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.receiver import aio_util
from makani.lib.python import ctype_util
from makani.lib.python import debug_util
from makani.lib.python import dict_util
from makani.lib.python import struct_tree
import numpy

_DEFAULT_MIN_GAP = 1000


def RegisterSpecs(func):
  """The decorator to register the specification for each check item object.

  The decorator first tests whether it is involved in the outmost call of the
  check item object. If so, it then goes through the args, kwargs, and defaults
  to populate the specification.

  Args:
    func: The __init__ function of a check item class.

  Returns:
    The wrapper function.
  """
  def Wrapper(self, *args, **kwargs):
    """The function wrapper to extract argument for CheckItems."""
    frame_stack = inspect.stack()
    if len(frame_stack) > 1:
      # Check if the caller is a method of BaseCheckItem. If so,
      # `func` itself is not the outmost call to extract parameters.
      frame = frame_stack[1][0]
      frame_locals = frame.f_locals

      if ('self' in frame_locals and
          isinstance(frame_locals['self'], BaseCheckItem)):
        return func(self, *args, **kwargs)
    # Record the args and kwargs into a dict.
    params = {}
    # Get the arguments for the function.
    # Example:
    #     def f(a, b=1, c=2, *pos, **named):
    #         pass
    # This returns:
    #     ArgSpec(args=['a', 'b', 'c'], varargs='pos', keywords='named',
    #             defaults=(1, 2))
    argspec = inspect.getargspec(func)

    # If an arg has default, the corresponding index in the `defaults` array is
    # N - (num_args_without_default), where N is the index of the arg in
    # the `args` array.
    # We started N as 1 to count for the `self` argument.
    default_idx = (1 - (len(argspec.args) - len(argspec.defaults))
                   if argspec.defaults else None)

    # For class member functions, the first item in args is `self`. Skip it.
    for idx, arg_name in enumerate(argspec.args[1:]):
      if idx < len(args):
        arg_value = args[idx]
      elif arg_name in kwargs:
        arg_value = kwargs[arg_name]
      elif (argspec.defaults and default_idx >= 0
            and default_idx < len(argspec.defaults)):
        arg_value = argspec.defaults[default_idx]
      else:
        raise ValueError('Missing argument "%s" for "%s".'
                         % (arg_name, self.__class__.__name__))
      if argspec.defaults:
        default_idx += 1
      if isinstance(arg_value, check_range.BaseRange):
        arg_value = arg_value.RawArgs()
      params[arg_name] = arg_value
    # Assign the parameters.
    self.SetSpecs(params)
    # Call the original function.
    obj = func(self, *args, **kwargs)
    return obj
  return Wrapper


class _ListOfChecksError(Exception):
  pass


def SeparateFieldIndex(field_index):
  """Separate structural indices from slices.

  Example: 'a[:].b[0].c' => ['a', 'b', 'c'], [':', '0']

  Args:
    field_index: A string index to a nested structure.

  Returns:
    output_segments: A list of structural indices.
    slices: A list of slices (in their string description).
  """

  slices = []
  segments = field_index.split('.')
  # Remove list slices from the struct_tree_index.
  output_segments = []
  for segment in segments:
    segment = segment.strip()
    while ']' in segment:
      end_pos = segment.find(']')
      start_pos = segment.find('[')
      slices.append(segment[(start_pos + 1):end_pos])
      segment = segment[:start_pos] + segment[end_pos + 1:]

    if segment:
      output_segments.append(segment)

  return output_segments, slices


def _Hdf5IndexFromStructure(field):
  """Converts a message structure index to an HDF5 index.

  Example: 'voltage[2].min' => 'voltage.min[:, 2]', where ':' selects the
  entire time sequence.

  Args:
    field: A nested field index in the form of a string.

  Returns:
    The same field index for HDF5.
  """
  segments, slices = SeparateFieldIndex(field)
  slices.insert(0, ':')  # Select the entire time sequence.
  return '.'.join(segments), struct_tree.StringToSlices(','.join(slices))


class BaseCheckItem(object):
  """A base class for a check."""

  LOG_TEMPLATE = string.Template(
      'messages.kAioNode$aio_node.kMessageType$message_type')
  AIO_TEMPLATE = string.Template('$message_type.$aio_node')

  # Index to message data as a namedtuple of (message_name, source, field).
  # E.g., [DataIndex('MotorStatus', 'MotorSbo', 'voltage[2].min'), ...]
  # A special case for `field` is 'timestamp', where 'timestamp' does not
  # exist but it will be computed from capture_header's tv_sec and tv_usec.
  DataIndex = collections.namedtuple(  # pylint: disable=invalid-name
      'DataIndex', ['message_name', 'source', 'field'])

  @RegisterSpecs
  def __init__(self, for_log, normal_ranges=check_range.AllInclusiveRange(),
               warning_ranges=check_range.AllInclusiveRange(), name=None,
               sort_by_sequence=True):
    """Initialize a check.

    Args:
      for_log: True if this check targets log data. False if it checks
          AIO messages.
      normal_ranges: A BaseRange object in which values are regarded as normal.
      warning_ranges: A BaseRange object in which values should raise warnings,
          if they fall out of normal_ranges. Values falling out of
          warning_ranges should raise errors.
      name: Name of the check.
      sort_by_sequence: If true, messages will be sorted by sequence number,
          instead of orderd by time of receiving.

    Raises:
      ValueError: Raised if the __init__ function is not decorated with
          RegisterSpecs.
    """
    # Values that fall within any of the normal ranges are normal.
    self._normal_ranges = check_range.BuildRanges(normal_ranges)
    # Values that fall outside of all normal ranges but inside any warning
    # ranges raise warnings. If the values fall even out of the warning
    # ranges, then they raise errors.
    self._warning_ranges = check_range.BuildRanges(warning_ranges)
    # Name of the check.
    self._name = name
    # True if the inputs should be sorted by sequence (marked at sender).
    self._sort_by_sequence = sort_by_sequence
    # Results of the check.
    if for_log:
      # If the check processes a log file, the result is formed as
      # {<check_name>:
      #     {<warning_level>: {"total": ..., "count": ..., "range": ...}}}.
      self._results = {}
    else:
      # If the check processes AIO messages, the resut is formed as
      # [{"name": ..., "value": ..., "stoplight": ...}, ...]
      self._results = []
    # Inputs needed by the check, in the form of a list of DataIndex objects.
    self._fields = self._RegisterInputs()
    # True if the check is performed over log data instead of AIO messages.
    self._for_log = for_log
    if not hasattr(self, '_kwargs'):
      raise ValueError(
          '%s.__init__ is missing the "@base.check.RegisterSpecs" decorate.'
          % type(self).__name__)

    # Merge selected regions into one section if their distance is less than
    # this parameter.
    self._min_gap = _DEFAULT_MIN_GAP

  def Name(self):
    """Return the name of the check. Default to the class name if empty."""
    return self._name if self._name else type(self).__name__

  def SetSpecs(self, params):
    """Set the specification of a check."""
    self._kwargs = params

  def _GetSpecs(self):
    """Get the specification of a check."""
    module_file = os.path.relpath(inspect.getfile(self.__class__),
                                  makani.HOME)
    module_path = 'makani.' + module_file[
        :module_file.rfind('.py')].replace('/', '.')
    class_path = '.'.join([module_path, self.__class__.__name__])
    return class_path, copy.deepcopy(self._kwargs)

  def GetDeterministicSpecs(self):
    """Get the deterministic specification of a check.

    Returns:
      The classpath and the parameters of the Check.
        The parameters are formatted so that the warning_ranges
        and normal_ranges are lists ordered as follows:
          -Non-duplicate ranges, sorted from least to greatest by the first
          number of each pair.
          -Non-duplicate set of values.
          -Value (if there are no sets and only one value).

        Example:
        makani.analysis.checks.collection.fake_checks.ExampleGradebookSpeed
        {'for_log': True, 'warning_ranges': [[0, 180], [6]],
                          'normal_ranges': [[0, 10], [80, 150],
                                            set([1, 2, -1, 14])]}
    Raises:
      ValueError: An error occurred with the the types in the warning_ranges
                  or normal_ranges.
    """

    class_path, parameters = self._GetSpecs()
    for category in ['warning_ranges', 'normal_ranges']:
      if category not in parameters:
        continue
      ranges_and_sets = []
      combined_set = set()

      for elem in parameters[category]:
        if isinstance(elem, list) and len(elem) > 1:
          ranges_and_sets.append(elem)
        elif isinstance(elem, set):
          combined_set = combined_set.union(elem)
        elif isinstance(elem, list) and len(elem) == 1:
          combined_set.update(elem)
        else:
          raise ValueError('Incorrect type in "%s" of check "%s": %s.'
                           % (category, self.Name(), elem))

      if combined_set:
        ranges_and_sets.append(combined_set)
      parameters[category] = ranges_and_sets
    return class_path, parameters

  def _GetLogData(self, data, message_name, aio_node, field):
    """Get message data from log files.

    Args:
      data: A StructTree object associated with the HDF5.
      message_name: Name of the message.
      aio_node: Name of the AIO node.
      field: String index within the message.

    Returns:
      A NumPy array as a time sequence of the field values.
      None if the field doesn't exist.
    """
    if aio_node is None:
      return data[message_name]

    prefix = self.LOG_TEMPLATE.substitute(
        message_type=message_name, aio_node=aio_node)
    if prefix not in data:
      logging.warn('Invalid message "%s.%s".', message_name, aio_node)
      return None

    if self._sort_by_sequence:
      # De-dupicate and order the message sequence.
      indices = log_util.MessageOrderedIndices(data, message_name, aio_node,
                                               self.LOG_TEMPLATE)
    else:
      indices = log_util.MessageDeduppedIndices(data, message_name, aio_node,
                                                self.LOG_TEMPLATE)
    if field == 'timestamp':
      return log_util.LogTimestamp(data, message_name, aio_node, indices)
    else:
      if field:
        path, slices = _Hdf5IndexFromStructure(field)
        values = data['%s.message.%s' % (prefix, path)]
        if values is not None:
          slices[0] = indices
          values = values[tuple(slices)]
        else:
          logging.error('Invalid field "%s.%s.%s".', message_name, aio_node,
                        field)
      else:
        values = data['%s.message' % prefix]
      return values

  def _GetAioData(self, data, message_name, aio_node, field):
    """Get message data from AIO.

    Args:
      data: A StructTree object associated with the HDF5.
      message_name: Name of the message.
      aio_node: Name of the AIO node.
      field: String index within the message.

    Returns:
      Value of the appointed message field. None if the field doesn't exist.
    """

    prefix = self.AIO_TEMPLATE.substitute(
        message_type=message_name, aio_node=aio_node)

    if field == 'timestamp':
      return data['%s.capture_info.timestamp' % prefix]
    else:
      return data['%s.%s' % (prefix, field)]

  def Populate(self, messages, cache=None):
    """Instantiate a list of data arguments from the messages.

    Args:
      messages: A StructTree object containing the nested dict of all messages
          and AIO nodes.
      cache: A dict of previously looked-up fields for caching.

    Returns:
      The list of values needed to compute the check results.
    """
    return self._PopulateRawData(messages, cache)

  def _PopulateRawData(self, messages, cache=None):
    """Instantiate a list of raw data from the messages."""
    use_cache = cache is not None
    values = []
    for info in self._fields:
      if self._for_log:
        if use_cache:
          if info not in cache:
            cache[info] = self._GetLogData(
                messages, info.message_name, info.source, info.field)
          values.append(cache[info])
        else:
          values.append(self._GetLogData(
              messages, info.message_name, info.source, info.field))
      else:
        values.append(self._GetAioData(
            messages, info.message_name, info.source, info.field))
    return values

  def Check(self, *args):
    """Clear old results and check for new results.

    Args:
      *args: The list of data arguments produced by Populate().

    Raises:
      _ListOfChecksError: Raised if the results are invalid.
    """

    if self._results:
      # Clear previous results.
      if isinstance(self._results, list):
        del self._results[:]
      elif isinstance(self._results, dict):
        self._results.clear()
      else:
        raise _ListOfChecksError('Invalid result type.')
    try:
      self._Check(*args)
    except Exception:  # pylint: disable=broad-except
      if self._for_log:
        self._results[self.Name()] = {
            log_util.DEFAULT_EXCEPTION_NAME: {
                'traceback': debug_util.FormatTraceback()
            }
        }
      else:
        self._results.append({
            'name': self.Name(),
            'value': 'EXCEPTION!',
            'stoplight': stoplights.STOPLIGHT_UNAVAILABLE
        })

  def _RegisterInputs(self):
    """Register inputs as a list of DataIndex objects."""
    raise NotImplementedError

  def _Check(self, *args):
    """Logics to check results."""
    raise NotImplementedError

  def HasExceptions(self):
    """Returns True if check results contain exceptions."""
    if self._for_log:
      for result in self._results.itervalues():
        if log_util.DEFAULT_EXCEPTION_NAME in result:
          return True
    else:
      for result in self._results:
        if result['stoplight'] == stoplights.STOPLIGHT_UNAVAILABLE:
          return True
    return False

  def HasWarnings(self):
    """Returns True if check results contain warnings."""
    if self._for_log:
      for result in self._results.itervalues():
        if log_util.DEFAULT_WARNING_NAME in result:
          return True
    else:
      for result in self._results:
        if result['stoplight'] == stoplights.STOPLIGHT_WARNING:
          return True
    return False

  def HasErrors(self):
    """Returns True if check results contain errors."""
    if self._for_log:
      for result in self._results.itervalues():
        if log_util.DEFAULT_ERROR_NAME in result:
          return True
    else:
      for result in self._results:
        if result['stoplight'] == stoplights.STOPLIGHT_ERROR:
          return True
    return False

  def GetResults(self):
    """Accessor of the results."""
    return self._results

  def TextSummary(self, precision):
    """Return a list of strings as the text summary of this check."""
    def StrWithPrecision(obj):
      if isinstance(obj, list):
        return '[' + ', '.join(StrWithPrecision(v) for v in obj) + ']'
      elif isinstance(obj, set):
        return '{' + ', '.join(StrWithPrecision(v) for v in obj) + '}'
      elif isinstance(obj, tuple):
        return '(' + ', '.join(StrWithPrecision(v) for v in obj) + ')'
      elif isinstance(obj, dict):
        return '{' + ', '.join(
            StrWithPrecision(k) + ': ' + StrWithPrecision(obj[k])
            for k in sorted(obj)) + '}'
      elif precision is not None and (
          isinstance(obj, float) or (isinstance(obj, numpy.number) and
                                     numpy.issubdtype(obj, float))):
        return str(round(obj, precision))
      else:
        return str(obj)

    if not self._results:
      return []

    indent = '  '
    lines = []
    for name, result in self._ResultGenerator():
      lines.append('%s:' % name)
      for field in sorted(result.keys()):
        values = result[field]
        lines.append(indent + '%s:' % field)
        line = indent * 2
        line += StrWithPrecision(values)
        lines.append(line)
    return lines

  def _ResultGenerator(self):
    """Return a generator to iterate through the results."""
    if isinstance(self._results, list):
      return enumerate(self._results)
    elif isinstance(self._results, dict):
      return self._results.iteritems()
    else:
      raise _ListOfChecksError('Result must be a list or dict.')

  def _CheckByRange(self, name, values, normal_ranges, warning_ranges,
                    warning_name='warning', error_name='error'):
    """Check whether `values` are normal, or should raise a warning/error.

    This is a utility called by derived classes.

    Args:
      name: Name of the check.
      values: Values to check.
      normal_ranges: A BaseRange object in which values are regarded as normal.
      warning_ranges: A BaseRange object in which values should raise warnings
          if they fall out of normal_ranges. Values falling out of
          warning_ranges should raise errors.
      warning_name: Name of the warning field.
      error_name: Name of the error field.
    """
    if values is None:
      return

    if self._for_log:
      result = log_util.CheckByRange(values, normal_ranges, warning_ranges,
                                     warning_name, error_name=error_name,
                                     min_gap=self._min_gap)
      if result:
        assert isinstance(result, dict)
        self._results[name] = result
    else:
      result = aio_util.CheckByRange(values, normal_ranges, warning_ranges,
                                     name)
      if result:
        assert isinstance(result, list)
        self._results += result

  def _CheckForFailure(self, name, values, target_ranges,
                       error_if_fail=True):
    """Checks for whether the values fall into the target_ranges.

    This is a utility called by derived classes.

    Args:
      name: Name of the check.
      values: Values to check.
      target_ranges: A BaseRange object in which values are regarded as normal.
        It can also be a list of basic objects that create BaseRange objects.
        Please refer to check_range.RangeChecker for details.
      error_if_fail: True if failing the range test should result in an error,
        otherwise a failure will result in only a warning.
    """
    if values is None:
      return

    if self._for_log:
      result = log_util.CheckForFailure(values, target_ranges, error_if_fail,
                                        error_name=None, min_gap=self._min_gap)
      if result:
        assert isinstance(result, dict)
        self._results[name] = result
    else:
      result = aio_util.CheckForFailure(values, target_ranges,
                                        error_if_fail, name)
      if result:
        assert isinstance(result, list)
        self._results += result

  def _Arg(self, message_name, aio_node, field):
    """Return a tuple representing a data argument of the check."""
    return self.DataIndex(message_name, aio_node, field)

  def SetMinGap(self, min_gap):
    """Set the min gap to merge warning fragments into larger sections."""
    self._min_gap = min_gap


class FilteredCheckItem(BaseCheckItem):
  """A check list item that acts on a filtered signal."""

  @RegisterSpecs
  def __init__(self, for_log, window_size, window_step,
               normal_ranges, warning_ranges=check_range.AllInclusiveRange(),
               name=None):
    super(FilteredCheckItem, self).__init__(for_log, normal_ranges,
                                            warning_ranges, name)
    self._window_size = window_size
    assert window_size >= 1
    self._window_step = window_step if window_step else window_size
    if not self._for_log:
      self._history = collections.defaultdict(
          lambda: collections.deque(maxlen=window_size))

  def _GetWindow(self, key, data):
    """Get a filtered window of data.

    The function needs to be overridden by derived classes that need filtering.

    Args:
      key: The unique identifier of the data field. The key is used as an index
          to store and load historic values of the field.
      data: Data to be filtered.

    Returns:
      The filtered data window.
    """

    if self._for_log:
      if data is None or data.size == 0:
        return data
      else:
        return log_util.Rolling(data, self._window_size, self._window_step)
    else:
      if data is None:
        self._history[key].clear()
        return numpy.array([])
      else:
        self._history[key].append(ctype_util.CTypeToPython(data))
        return numpy.array(self._history[key])

  def Populate(self, messages, cache=None):
    """Instantiate a list of data arguments from the messages."""
    return [self._GetWindow(pos, v)
            for pos, v in enumerate(self._PopulateRawData(messages, cache))]


class BitmaskErrorCheck(BaseCheckItem):
  """A checklist item to check an error or warning flag."""

  @RegisterSpecs
  def __init__(self, for_log, message_type, sources, field,
               enum_helper, error_if_fail=True, errors=None, name=None):
    """Specify which error or warning flag to check.

    Args:
      for_log: True if this check targets log data. False if it checks
          AIO messages.
      message_type: The message where the desired flag is in
          e.g. 'MotorStatus'
      sources: The aio nodes to get messages from, can be one node or a list of
          nodes, e.g. 'MotorSti' or ['MotorSti', 'MotorPbo']
      field: String providing path in message struct to the desired flag
          e.g. 'motor_error'
      enum_helper: A c_helpers.EnumHelper object for the desired error type
          e.g. c_helpers.EnumHelper('MotorErrorFlag', flags,
                                    prefix='kMotorError')
      error_if_fail: If False, will raise warnings and not errors upon failure.
      errors: A list of error short names to check (if None then defaults to
          checking all errors) e.g. ['OverSpeed', 'UnderVoltage']
      name: Name of the check.
    """
    self._message_type = message_type
    self._message_sources = sources if isinstance(sources, list) else [sources]
    self._field = field
    self._enum_helper = enum_helper
    self._error_if_fail = error_if_fail

    if errors is not None:
      self._errors = [(e, enum_helper.Value(e)) for e in errors]
    else:
      self._errors = [(enum_helper.ShortName(e[0]), e[1]) for e in enum_helper]

    super(BitmaskErrorCheck, self).__init__(for_log, name=name)

  def _RegisterInputs(self):
    inputs = []
    # Add an input for every message source provided.
    if isinstance(self._message_sources, list):
      for source in self._message_sources:
        inputs.append(self._Arg(self._message_type, source, self._field))
    else:
      inputs.append(self._Arg(self._message_type,
                              self._message_sources, self._field))
    return inputs

  def _CheckErrorFlag(self, values, error_name_suffix=''):
    """Helper to check the value(s) provided for errors.

    Args:
      values: The values to check against the error bitmask. Can be one
          number or a numpy.ndarray of numbers.
      error_name_suffix: A string that will prefix the name of the error
          raised by the check system if an error is found in values.
          For example, if an error 'kMotorErrorUnderVoltage' is found,
          and error_name_prefix is 'MotorSti', then the error will show up
          in the check results as 'kMotorErrorUnderVoltage (MotorSti)'
    """
    if values is None:
      return

    if isinstance(values, list):
      values = numpy.array(values)

    for error_short_name, error_bitmask in self._errors:
      # Get the set of values to check by performing a bitwise and
      # between each value in the motor_error_flag sequence and the
      # bitmask associated with the error we are checking.
      values_to_check = self._GetMaskedValues(values, error_bitmask)
      zero_range = check_range.Singleton(0)
      name = error_short_name + '(' + error_name_suffix + ')'
      self._CheckForFailure(name, values_to_check, zero_range,
                            self._error_if_fail)

  def _GetMaskedValues(self, values, error_bitmask):
    return values & error_bitmask

  def _Check(self, *values):
    """Checks for errors or warnings that could be indicated in the flag."""
    # Use the source as the prefix for the error, so if the source
    # is 'MotorSti' and we find an error 'kMotorErrorUnderVoltage'
    # then the error will show up in the check results as
    # 'MotorSti kMotorErrorUnderVoltage'. This is used to differentiate
    # between identically named errors that come from different aio_nodes.
    assert len(values) == len(self._message_sources)
    for value, source in zip(values, self._message_sources):
      self._CheckErrorFlag(value, source)


class BitmaskErrorLatchCheck(BitmaskErrorCheck):
  """Only detect errors when they start to appear."""

  def _GetMaskedValues(self, values, error_bitmask):
    masked_values = values & error_bitmask
    # Assume that the beginning of the time series should not be a latch-on
    # event, even if the error bit is set. Most likely this is due to the
    # continuation of a previous log.
    result = numpy.diff(masked_values) > 0
    return numpy.insert(result, 0, False)


class FieldRangeCheck(BaseCheckItem):
  """Check whether a particular field's value fall in certain ranges."""

  @RegisterSpecs
  def __init__(self, for_log, message_type, sources, field,
               normal_ranges, warning_ranges):
    self._sources = sources
    self._message_type = message_type
    self._field = field
    name = self.__class__.__name__ + '.%s.%s %s' % (
        message_type, field, sources)
    super(FieldRangeCheck, self).__init__(
        for_log, normal_ranges=normal_ranges, warning_ranges=warning_ranges,
        name=name)

  def _RegisterInputs(self):
    inputs = []
    for source in self._sources:
      inputs.append(self._Arg(self._message_type, source, self._field))
    return inputs

  def _Check(self, *values):
    assert len(values) == len(self._sources)
    for idx, source in enumerate(self._sources):
      name = ('%s (%s)' % (self._name, source)  if len(self._sources) > 1
              else self._name)
      if values[idx] is None:
        continue
      self._CheckByRange(name, values[idx],
                         self._normal_ranges, self._warning_ranges)


class ListOfChecks(object):
  """A base class for a check list."""

  def __init__(self):
    self._items_to_check = []

  def __len__(self):
    return len(self._items_to_check)

  def SetMinGap(self, min_gap):
    """Set the min gap to merge warning fragments into larger sections."""
    for item in self._items_to_check:
      item.SetMinGap(min_gap)

  def List(self):
    return self._items_to_check

  def Append(self, check):
    """Append a CheckItem object to the list of checks."""
    self._items_to_check.append(check)

  def BatchAppend(self, check_items):
    self._items_to_check += check_items

  def Concatenate(self, check_list):
    self._items_to_check += check_list.List()

  def MergeResults(self):
    """Merge the results of the check items within this list."""
    results = None
    for item in self._items_to_check:
      per_item_results = copy.copy(item.GetResults())
      if per_item_results is None:
        continue
      if results is None:
        results = per_item_results
      else:
        assert (isinstance(results, type(per_item_results)) and
                isinstance(results, (list, dict)))
        if isinstance(results, list):
          results += per_item_results
        else:  # isinstance(results, dict):
          results.update(per_item_results)
    return results

  def TextSummary(self, precision=2):
    """Return a list of strings as the text summary of this check list."""
    report = []
    for item in self._items_to_check:
      report += item.TextSummary(precision)
    return '\n'.join(report)

  def GetSpecs(self):
    return [item.GetDeterministicSpecs() for item in self._items_to_check]


def SerializeCheckSpecs(specs):
  """Serialize the deterministic checks into JSON strings.

  If there is a "set" object in the warning_ranges or error_ranges, it will be
  turned into a list and ordered from least to greatest. The parameters
  dictionary will then be orderered and JSONified.

  Args:
    specs: A dictionary that is the parameters of the check.

  Returns:
    JSON string that contains the spec information.

  Example:
    Input:
      {"sort_by_sequence": True, "for_log": True,
       "warning_ranges": [[[0, 10]], set([5, 4]), "name": None,
       "normal_ranges": {[[1, 3]]}}
    Output:
      '{"for_log": true, "name": null, "normal_ranges": {"ranges": [[1, 3]]}
        "sort_by_sequence": true,
        "warning_ranges": {"ranges": [[0, 10]], "values": [4, 5]}}'
  """
  for category in ['warning_ranges', 'normal_ranges']:
    if category not in specs:
      continue

    element_list = specs[category]
    range_dict = {}
    range_list = []
    for elem in element_list:
      if isinstance(elem, set):
        range_dict['values'] = sorted(list(elem))
      elif isinstance(elem, list):
        range_list.append(elem)
    if range_list:
      range_dict['ranges'] = range_list
    specs[category] = range_dict

  ordered_dict = dict_util.OrderDict(specs)
  return json.dumps(ordered_dict)


def ParseCheckSpecs(json_spec):
  """Parses the JSON string into the Checks parameters.

  Args:
    json_spec: the JSON string that contains the parameters of the check.

  Returns:
    The parameters of the Check, with properly instantiated set if
    there is one.
  """
  specs = json.loads(json_spec, object_pairs_hook=collections.OrderedDict)
  for category in ['warning_ranges', 'normal_ranges']:
    if category not in specs:
      continue

    parameters = []
    current_range = specs[category]
    if 'ranges' in current_range:
      parameters += current_range['ranges']
    if 'values' in current_range:
      parameters.append(set(current_range['values']))

    specs[category] = parameters

  return specs


def SkipIfAnyInputIsNone(func):
  """Decorator for check objects' _Check() to skip upon any missing input.

  Mark that the check should be skipped if any of its input fields are not
  present in the flight log.

  Args:
    func: A Check's _Check() member function to apply this decorator.

  Returns:
    The decorator's wrapper function.
  """
  def Wrapper(self, *attributes):
    for attrib in attributes:
      if attrib is None:
        return
    return func(self, *attributes)
  return Wrapper
