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

"""List of checks for the network."""

import os

import makani
from makani.analysis.checks import base_check
from makani.analysis.checks import log_util
from makani.avionics.common import tether_message_types as tether_message
from makani.avionics.network import network_config
import numpy

_FRAME_MISSING_LIMIT = 16


class BaseMessageCheck(base_check.BaseCheckItem):

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, sources,
               normal_ranges, warning_ranges):
    self._sources = sources
    self._message_type = message_type
    name = self.__class__.__name__ + '.%s.%s' % (message_type, sources)
    super(BaseMessageCheck, self).__init__(
        for_log, normal_ranges=normal_ranges, warning_ranges=warning_ranges,
        name=name, sort_by_sequence=False)


class MessageIntervalCheck(BaseMessageCheck):
  """Base class for message interval checks."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, sources):
    super(MessageIntervalCheck, self).__init__(for_log, message_type, sources,
                                               [[0.0, 1.0]], [[0.0, 1.0]])

  def _RegisterInputs(self):
    inputs = []
    for source in self._sources:
      inputs.append(self._Arg(self._message_type, source, 'timestamp'))
    return inputs

  def _Check(self, *timestamps):
    assert self._for_log
    assert len(timestamps) == len(self._sources) and len(timestamps)
    timestamps = [series for series in timestamps if series is not None]
    if len(self._sources) > 1:
      timestamps = numpy.concatenate(timestamps)
      timestamps.sort()
    else:
      timestamps = timestamps[0]
    interval = numpy.diff(timestamps)
    self._CheckByRange(self._name,
                       interval, self._normal_ranges, self._warning_ranges)


class BaseMissingFrameCheck(BaseMessageCheck):
  """Base class for missing frame checks."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, sources, field,
               normal_ranges, warning_ranges, frame_wraparound):
    self._frame_wraparound = frame_wraparound
    self._field = field
    super(BaseMissingFrameCheck, self).__init__(for_log, message_type, sources,
                                                normal_ranges, warning_ranges)

  def _RegisterInputs(self):
    inputs = []
    for source in self._sources:
      inputs.append(self._Arg(self._message_type, source, self._field))
    for source in self._sources:
      inputs.append(self._Arg(self._message_type, source, 'timestamp'))
    return inputs

  def _Check(self, *data):
    assert self._for_log
    assert len(data) == len(self._sources) * 2
    frame_index = [series for series in data[:len(self._sources)]
                   if series is not None]
    timestamps = [series for series in data[len(self._sources):]
                  if series is not None]

    if len(self._sources) > 1:
      timestamps = numpy.concatenate(timestamps)
      frame_index = numpy.concatenate(frame_index)
      frame_index = frame_index[timestamps.argsort()]
    else:
      frame_index = frame_index[0]

    frame_index = frame_index.astype(int)
    frame_index = log_util.UnwrapSequence(frame_index, self._frame_wraparound)
    frame_index.sort()
    frame_index_diff = numpy.diff(frame_index)
    self._CheckByRange(self._name, frame_index_diff,
                       self._normal_ranges, self._warning_ranges)


class MissingFrameCheck(BaseMissingFrameCheck):

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, sources,
               frame_wraparound=tether_message.TETHER_FRAME_INDEX_ROLLOVER):
    super(MissingFrameCheck, self).__init__(
        for_log, message_type, sources, 'frame_index',
        [[0, _FRAME_MISSING_LIMIT]], [[0, None]], frame_wraparound)


class MissingReceivedFrameCheck(BaseMissingFrameCheck):

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, sources,
               frame_wraparound=tether_message.TETHER_FRAME_INDEX_ROLLOVER):
    super(MissingReceivedFrameCheck, self).__init__(
        for_log, message_type, sources, 'received_frame_index',
        [[0, _FRAME_MISSING_LIMIT]], [[0, None]], frame_wraparound)


class DuplicateFrameCheck(BaseMissingFrameCheck):

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, sources,
               frame_wraparound=tether_message.TETHER_FRAME_INDEX_ROLLOVER):
    super(DuplicateFrameCheck, self).__init__(
        for_log, message_type, sources, 'frame_index',
        [[1, None]], [[1, None]], frame_wraparound)


class DuplicateReceivedFrameCheck(BaseMissingFrameCheck):

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, sources,
               frame_wraparound=tether_message.TETHER_FRAME_INDEX_ROLLOVER):
    super(DuplicateReceivedFrameCheck, self).__init__(
        for_log, message_type, sources, 'received_frame_index',
        [[1, None]], [[1, None]], frame_wraparound)


class FrameUpdateIntervalCheck(BaseMessageCheck):
  """Check for the frame update rate."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, source,
               frame_index_increment, target_frequency,
               target_interval_tolerance=0.1,
               frame_wraparound=tether_message.TETHER_FRAME_INDEX_ROLLOVER):
    """Check the time interval between frame updates.

    Args:
      for_log: True if this check is to analyze logs, not realtime AIO.
      message_type: The short name of the message type.
      source: The short name of the AIO node sending the message.
      frame_index_increment: The increment of frame updates.
      target_frequency: The expected frequency that frames got updated.
      target_interval_tolerance: The factor by which frame update intervals
          can deviate from the expected.
      frame_wraparound: The bound of the frame index beyond which the index
          wraps around from 0.
    """
    self._frame_wraparound = frame_wraparound
    target_interval = 1.0 / target_frequency
    acceptable_deviation = target_interval * target_interval_tolerance
    interval_limit = [[target_interval - acceptable_deviation,
                       target_interval + acceptable_deviation]]
    self._frame_index_increment = frame_index_increment
    super(FrameUpdateIntervalCheck, self).__init__(
        for_log, message_type, [source], interval_limit, [[None, None]])

  def _RegisterInputs(self):
    inputs = []
    for source in self._sources:
      inputs.append(self._Arg(self._message_type, source, 'frame_index'))
    for source in self._sources:
      inputs.append(self._Arg(self._message_type, source, 'timestamp'))
    return inputs

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, frame_index, timestamps):
    assert self._for_log

    frame_index = frame_index.astype(int)
    frame_index = log_util.UnwrapSequence(frame_index, self._frame_wraparound)

    frame_index_diff = numpy.diff(frame_index)
    frame_update_selection = (frame_index_diff > 0)
    update_timestamps = timestamps[1:][frame_update_selection]
    self._CheckByRange('Frame Ever Updated (%s)' % self._name,
                       numpy.array([update_timestamps.size]), [[1, None]],
                       [[1, None]])

    if update_timestamps.size > 0:
      frame_increment_selection = (
          frame_index_diff == self._frame_index_increment)
      # A bit mask for all the frame updates, where 1 means frame increments
      # without loosing any in between.
      frame_increment_indices = (
          frame_increment_selection[frame_update_selection])

      update_timestamp_diff = numpy.diff(update_timestamps)
      update_timestamp_diff = update_timestamp_diff[frame_increment_indices[1:]]
      self._CheckByRange(self._name,
                         update_timestamp_diff, self._normal_ranges,
                         self._warning_ranges)


class TetherUpChecks(base_check.ListOfChecks):
  """The GPS checklist."""

  def __init__(self, for_log):
    self._items_to_check = []

    groups = [
        {
            'message': 'TetherUp',
            'sources': ['CsGsA'],
        },
        {
            'message': 'TetherUp',
            'sources': ['CsGsB'],
        },
    ]
    for group in groups:
      self._items_to_check += [
          MessageIntervalCheck(for_log, group['message'], group['sources']),
          MissingFrameCheck(for_log, group['message'], group['sources']),
          DuplicateFrameCheck(for_log, group['message'], group['sources']),
      ]


class TetherDownChecks(base_check.ListOfChecks):
  """The GPS checklist."""

  def __init__(self, for_log):
    self._items_to_check = []

    groups = [
        {
            'message': 'TetherDown',
            'sources': ['CsA'],
        },
        {
            'message': 'TetherDown',
            'sources': ['CsGsA'],
        },
        {
            'message': 'TetherDown',
            'sources': ['CsB'],
        },
    ]
    for group in groups:
      self._items_to_check += [
          MessageIntervalCheck(for_log, group['message'], group['sources']),
          MissingFrameCheck(for_log, group['message'], group['sources']),
          DuplicateFrameCheck(for_log, group['message'], group['sources']),
      ]
      if group['message'] == 'TetherDown':
        self._items_to_check.append(
            MissingReceivedFrameCheck(
                for_log, group['message'], group['sources']))

    groups = [
        {
            'message': 'TetherDown',
            'sources': ['CsGsA'],
        },
    ]
    normal_ranges = [[-92, None]]
    warning_ranges = [[-112, None]]
    for group in groups:
      self._items_to_check += [
          base_check.FieldRangeCheck(
              for_log, group['message'], group['sources'],
              'received_signal_strength', normal_ranges, warning_ranges),
          base_check.FieldRangeCheck(
              for_log, group['message'], group['sources'],
              'comms_status.received_signal_strength',
              normal_ranges, warning_ranges),
      ]


class AggregatedLinkChecks(base_check.ListOfChecks):
  """The GPS checklist."""

  def __init__(self, for_log):
    self._items_to_check = []

    groups = [
        {
            'message': 'TetherDown',
            'sources': ['CsA', 'CsGsA', 'CsB'],
        },
        {
            'message': 'TetherUp',
            'sources': ['CsA', 'CsGsA', 'CsGsB'],
        },
    ]
    for group in groups:
      self._items_to_check += [
          MessageIntervalCheck(for_log, group['message'], group['sources']),
          MissingFrameCheck(for_log, group['message'], group['sources']),
      ]


class FrameUpdateRateChecks(base_check.ListOfChecks):

  def __init__(self, for_log):
    self._items_to_check = []

    config = network_config.NetworkConfig(
        os.path.join(makani.HOME, 'avionics/network/network.yaml'))

    for m in config.all_messages:

      if m.name == 'TetherDown':
        for sender in m.all_senders:
          sender_name = sender.camel_name
          if sender_name.startswith('CsGs'):
            self._items_to_check.append(
                FrameUpdateIntervalCheck(
                    for_log, m.name, sender_name,
                    tether_message.TETHER_RADIO_DECIMATION,
                    m.frequency_hz / tether_message.TETHER_RADIO_DECIMATION))
          else:
            self._items_to_check.append(
                FrameUpdateIntervalCheck(
                    for_log, m.name, sender_name, 1, m.frequency_hz))

      elif m.name == 'TetherUp':
        for sender in m.all_senders:
          sender_name = sender.camel_name
          if not sender_name.startswith('CsGs'):
            self._items_to_check.append(
                FrameUpdateIntervalCheck(
                    for_log, m.name, sender_name,
                    tether_message.TETHER_RADIO_DECIMATION,
                    m.frequency_hz / tether_message.TETHER_RADIO_DECIMATION))
          else:
            self._items_to_check.append(
                FrameUpdateIntervalCheck(
                    for_log, m.name, sender_name, 1, m.frequency_hz))

