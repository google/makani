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

"""Utility functions to analyze logs."""

import re
import string

from makani.analysis.checks import check_range
import numpy


_LOG_TEMPLATE = string.Template(
    'messages.kAioNode$aio_node.kMessageType$message_type')
_SEQUENCE_WRAPAROUND = 65536
DEFAULT_ERROR_NAME = 'error'
DEFAULT_WARNING_NAME = 'warning'
DEFAULT_EXCEPTION_NAME = 'exception'


def _SelectionToSections(selection, min_gap):
  """Identify sections of selected regions defined by a boolean mask array.

  Args:
    selection: A boolean array used for selection.
    min_gap: Merge selected regions into one section if their distance is less
        than min_gap.

  Returns:
    A list of sorted pairs telling the start/end of selected sections
    [(start, end), ...]
  """

  if len(selection.shape) > 1:
    selection = numpy.any(selection, axis=0)

  selection = numpy.insert(selection, selection.size, 0)
  selection = numpy.insert(selection, 0, 0)

  diff = numpy.diff(selection.astype(int))
  start_positions = numpy.where(diff == 1)[0]
  end_positions = numpy.where(diff == -1)[0]
  assert start_positions.size == end_positions.size
  regions = zip(start_positions, end_positions)

  sections = [regions[0]]
  for region in regions[1:]:
    if region[0] - sections[-1][1] < min_gap:
      sections[-1] = (sections[-1][0], region[1])
    else:
      sections.append(region)

  return sections


def CheckByRange(values, normal_ranges, warning_ranges,
                 warning_name=DEFAULT_WARNING_NAME,
                 error_name=DEFAULT_ERROR_NAME, min_gap=1000):
  """Checking values against normal and warning ranges.

  Args:
    values: Values of the field.
    normal_ranges: A BaseRange object in which values are regarded as normal.
        It can also be a list of basic objects that create BaseRange objects.
        Please refer to check_range.RangeChecker for details.
    warning_ranges: A BaseRange object in which values should raise warnings,
        if they fall out of normal_ranges. Values falling out of
        warning_ranges should raise errors.
        Please refer to check_range.RangeChecker for details.
    warning_name: Name of the warning.
    error_name: Name of the error.
    min_gap: Merge selected regions into one section if their distance is less
        than min_gap.

  Returns:
    Check results in the form of {
        warning_name: {
            'total': ...,  # Total number of samples.
            'count': ...,  # Number of samples in this range.
            'range': [min_value, max_value],
        },
        error_name: {...}
    }
  """
  assert values.size > 0
  normal_ranges_obj = check_range.BuildRanges(normal_ranges)
  warning_ranges_obj = check_range.BuildRanges(warning_ranges)
  # Check value ranges.
  normal_selection = normal_ranges_obj.Select(values)
  warning_selection = warning_ranges_obj.Select(values)
  warning_selection &= ~normal_selection
  error_selection = ~(normal_selection | warning_selection)

  results = {}
  if not numpy.all(normal_selection):
    total = normal_selection.size
    warning_values = values[warning_selection]
    error_values = values[error_selection]
    if warning_values.size:
      results[warning_name] = {
          'total': total,
          'count': numpy.sum(warning_selection),
          'range': [numpy.min(warning_values), numpy.max(warning_values)],
          'expecting': str(normal_ranges_obj),
          'sections': _SelectionToSections(warning_selection, min_gap),
      }
    if error_values.size:
      results[error_name] = {
          'total': total,
          'count': numpy.sum(error_selection),
          'range': [numpy.min(error_values), numpy.max(error_values)],
          'expecting': str(warning_ranges_obj),
          'sections': _SelectionToSections(error_selection, min_gap),
      }
  return results


def Rolling(a, window, step, start_from_beginning=True):
  """Construct rolling windows."""
  num_windows = (a.shape[0] - window) / step + 1
  if not start_from_beginning:
    length = (num_windows - 1) * step + window
    a = a[-length:]
  shape = (num_windows, window) + a.shape[1:]
  strides = (a.strides[0] * step,) + (a.strides[0],) + a.strides[1:]
  return numpy.lib.stride_tricks.as_strided(a, shape=shape, strides=strides)


def DedupSelection(sequence, wraparound=None):
  """Return a bit-mask marking values to keep for a de-duplicated sequence."""
  deduped_indices = IndicesToOrderAndDedup(sequence, wraparound)
  selection = numpy.zeros(sequence.shape, dtype=bool)
  selection[deduped_indices] = True
  return selection


def IndicesToDedup(sequence, wraparound=None):
  """Return the indices that guarantee de-duplicated sequence."""
  deduped_indices = IndicesToOrderAndDedup(sequence, wraparound)
  deduped_indices.sort()
  return deduped_indices


def UnwrapSequence(sequence, wraparound=_SEQUENCE_WRAPAROUND):
  seq_step = numpy.diff(sequence, axis=0)
  half_round = wraparound / 2
  seq_step = seq_step + (seq_step < -half_round) * wraparound - (
      seq_step > half_round) * wraparound
  return numpy.insert(numpy.cumsum(seq_step), 0, 0)


def IndicesToOrderAndDedup(sequence, wraparound=_SEQUENCE_WRAPAROUND):
  """Return the indices that guarantee monotonic ordering with unique IDs."""
  if wraparound is not None:
    sequence = UnwrapSequence(sequence, wraparound)
  # sort_indices[n] is the index to the n-th smallest elemnt in `seq`.
  sort_indices = numpy.argsort(sequence)
  # dedup_indices[n] is True if the n-th element in the sorted sequence is
  # not a duplicate.
  dedup_indices = numpy.insert(numpy.diff(sequence[sort_indices]), 0, 1) != 0
  return sort_indices[dedup_indices]


def GetMessageSequence(data, message_name, aio_node,
                       prefix_template=_LOG_TEMPLATE, slice_string=''):
  """Get the message sequence.

  Args:
    data: A StructTree object representing the HDF5 log.
    message_name: Name of the message type.
    aio_node: Name of the AIO node.
    prefix_template: The string template representing the prefix of a message.
    slice_string: A string describing a slice to pre-select the sequence.

  Returns:
    The sequence of a given message name from an AIO node.
  """
  prefix = prefix_template.substitute(
      message_type=message_name, aio_node=aio_node)
  sequence = data['%s.aio_header.sequence%s' % (prefix, slice_string)]
  if sequence is None:
    # Handle old logs.
    sequence = data['%s.header.sequence%s' % (prefix, slice_string)]
  return sequence


def MessageOrderedIndices(data, message_name, aio_node,
                          prefix_template=_LOG_TEMPLATE,
                          wraparound=_SEQUENCE_WRAPAROUND,
                          slice_string=''):
  """Get message indices that guarantee monotonic ordering with unique IDs."""
  sequence = GetMessageSequence(data, message_name, aio_node, prefix_template,
                                slice_string)
  assert sequence is not None
  return IndicesToOrderAndDedup(sequence, wraparound)


def MessageDeduppedIndices(data, message_name, aio_node,
                           prefix_template=_LOG_TEMPLATE,
                           wraparound=_SEQUENCE_WRAPAROUND,
                           slice_string=''):
  """Get message indices that guarantee monotonic ordering with unique IDs."""
  sequence = GetMessageSequence(data, message_name, aio_node, prefix_template,
                                slice_string)
  assert sequence is not None
  return IndicesToDedup(sequence, wraparound)


def MessageDedupSelection(data, message_name, aio_node,
                          prefix_template=_LOG_TEMPLATE,
                          wraparound=_SEQUENCE_WRAPAROUND):
  """Get the bit-mask to select a de-duplicated message sequence."""
  sequence = GetMessageSequence(data, message_name, aio_node, prefix_template)
  assert sequence is not None
  return DedupSelection(sequence, wraparound)


def LogTimestamp(data, message_name, aio_node, selection=None,
                 slice_string='', prefix_template=_LOG_TEMPLATE):
  """Get a sequence of timestamps from the log data.

  Args:
    data: A StructTree object representing the HDF5 log.
    message_name: Name of the message type.
    aio_node: Name of the AIO node.
    selection: A list of indices to select a sublist of timestamps.
    slice_string: A string describing a slice to pre-select the timestamp
        sequence before applying the selection.
    prefix_template: The string template representing the prefix of a message.

  Returns:
    The timestamp sequence of a given message name from an AIO node.
  """

  prefix = prefix_template.substitute(
      message_type=message_name, aio_node=aio_node)
  tv_sec = data['%s.capture_header.tv_sec%s' % (prefix, slice_string)]
  tv_usec = data['%s.capture_header.tv_usec%s' % (prefix, slice_string)]
  if tv_sec is None or tv_usec is None:
    return
  if selection is not None:
    tv_sec = tv_sec[selection]
    tv_usec = tv_usec[selection]
  return tv_sec + 1e-6 * tv_usec


def _GetAioNodeAndMessageNameFromField(field_path):
  match = re.search(r'kAioNode(?P<aio_node>\w+)\.kMessageType(?P<message>\w+)',
                    field_path)
  if match is not None:
    return match.group('aio_node'), match.group('message')
  else:
    return None, None


def _SliceStringFromFieldPath(field):
  if field.endswith(']'):
    return '[%s]' % (field[field.rfind('[') + 1 : -1].split(',')[0])
  else:
    return ''


def _MessageTimestamp(struct_tree_obj, field, rebase=True,
                      prefix_template=_LOG_TEMPLATE):
  """Get the timestamp of message fields."""

  aio_node, message_name = _GetAioNodeAndMessageNameFromField(field)
  if aio_node is None or message_name is None:
    return None

  timestamp = LogTimestamp(struct_tree_obj, message_name, aio_node,
                           slice_string=_SliceStringFromFieldPath(field),
                           prefix_template=prefix_template)

  if timestamp is None:
    return None
  if rebase:
    # Rebase the timestamp according to the first message in the sequence.
    timestamp -= timestamp[0]
  return timestamp


def GetOrderedDedupDataAndTimeByField(log_data, field, rebase,
                                      prefix_template=_LOG_TEMPLATE,
                                      wraparound=_SEQUENCE_WRAPAROUND):
  """Get the ordered and de-duplicated data field and its timestamps."""
  aio_node, message_name = _GetAioNodeAndMessageNameFromField(field)
  if aio_node is None or message_name is None:
    return None, None

  try:
    data = log_data[field]
  except IOError:
    return None, None

  timestamps = _MessageTimestamp(log_data, field, rebase, prefix_template)

  if data is None or timestamps is None:
    return None, None

  indices = MessageOrderedIndices(log_data, message_name, aio_node,
                                  prefix_template=prefix_template,
                                  wraparound=wraparound,
                                  slice_string=_SliceStringFromFieldPath(field))
  return data[indices], timestamps[indices]


def CheckForFailure(values, target_ranges, error_if_fail, error_name=None,
                    min_gap=1000):
  """Checking values against normal ranges.

  Args:
    values: Values of the field.
    target_ranges: A BaseRange object in which values are regarded as normal.
        It can also be a list of basic objects that create BaseRange objects.
        Please refer to check_range.RangeChecker for details.
    error_if_fail: True if failed values raise errors,
        otherwise they raise warnings.
    error_name: Name of the error/warning.
    min_gap: Merge selected regions into one section if their distance is less
        than min_gap.

  Returns:
    Check results in the form of {
        warning_name: {
            'total': ...,  # Total number of samples.
            'count': ...,  # Number of samples in this range.
            'range': [min_value, max_value],
        },
        error_name: {...}
    }
  """
  if error_if_fail:
    error_name = DEFAULT_ERROR_NAME if not error_name else error_name
    return CheckByRange(values, target_ranges, check_range.AllExclusiveRange(),
                        DEFAULT_WARNING_NAME, error_name, min_gap)
  else:
    error_name = DEFAULT_WARNING_NAME if not error_name else error_name
    return CheckByRange(values, target_ranges, check_range.AllInclusiveRange(),
                        error_name, DEFAULT_ERROR_NAME, min_gap)
