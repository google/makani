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

"""Checklist items for motors."""

from makani.analysis.checks import base_check
from makani.analysis.checks import check_range
from makani.avionics.motor.firmware import flags
from makani.avionics.network import aio_labels
from makani.lib.python import c_helpers
import numpy
from scipy import signal

_MOTOR_LABELS_HELPER = c_helpers.EnumHelper('MotorLabel', aio_labels,
                                            prefix='kMotor')
_MOTOR_ERROR_FLAGS_HELPER = c_helpers.EnumHelper('MotorErrorFlag', flags,
                                                 prefix='kMotorError')
_MOTOR_WARNING_FLAGS_HELPER = c_helpers.EnumHelper('MotorWarningFlag', flags,
                                                   prefix='kMotorWarning')


class MotorStackPairVoltageDiff(base_check.BaseCheckItem):
  """Class to check voltage differences between stacking pairs."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, motors):
    """Initialize the object.

    Args:
      for_log: True if this check is performed over a log. False if it is for
          realtime AIO messages.
      motors: Names of the motors to check.
    """
    normal_ranges = check_range.RangeChecker([[None, 15]])
    self._motors = motors
    assert len(motors) == 2
    self._filter_order = 3
    cutoff = 0.25
    self._kernel = signal.butter(self._filter_order, cutoff)
    if not for_log:
      self._zi_1 = signal.lfiltic(self._kernel[0], self._kernel[1], [0.0])
      self._zi_2 = signal.lfiltic(self._kernel[0], self._kernel[1], [0.0])

    super(MotorStackPairVoltageDiff, self).__init__(
        for_log, normal_ranges, name='StackedMotorsVoltageDiff(%s)' % motors)

  def _RegisterInputs(self):
    """Register what fields will be used to calculate the check results."""
    data = []
    for motor in self._motors:
      data.append(self._Arg('MotorDebug', motor, 'bus_voltage'))
      data.append(self._Arg('MotorDebug', motor, 'timestamp'))
    return data

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, values_1, timeline_1, values_2, timeline_2):
    """Calculate check results using the list of registered inputs."""
    if self._for_log:
      if len(values_1) < len(values_2):
        values_2 = numpy.interp(timeline_1, timeline_2, values_2)
      else:
        values_1 = numpy.interp(timeline_2, timeline_1, values_1)
      values_1 = signal.filtfilt(self._kernel[0], self._kernel[1], values_1)
      values_2 = signal.filtfilt(self._kernel[0], self._kernel[1], values_2)
    else:
      values, self._zi_1 = signal.lfilter(
          self._kernel[0], self._kernel[1], [values_1], zi=self._zi_1)
      values_1 = values[0]
      values, self._zi_2 = signal.lfilter(
          self._kernel[0], self._kernel[1], [values_2], zi=self._zi_2)
      values_2 = values[0]

    diff = numpy.abs(values_1 - values_2)
    self._CheckByRange(
        'Voltage diff between %s' % (self._motors),
        diff, self._normal_ranges, self._warning_ranges)


class MotorStackingSequence(base_check.FilteredCheckItem):
  """Class to check stacking sequences received by each motor.

  To coordinate amongst stacked motors, motors must receive messages from each
  other continuously and robustly. This class checks whether any message has
  been lost using the sequence numbers of received messages.
  """

  @base_check.RegisterSpecs
  def __init__(self, for_log):
    self._motors = [_MOTOR_LABELS_HELPER.ShortName(i)
                    for i in range(len(_MOTOR_LABELS_HELPER))]
    # The maximum sequence difference can be 3.
    normal_ranges = check_range.RangeChecker([[0, 3]])
    # Any difference larger than 30 is likely to be caused by logger dropping
    # packets and therefore tends to raise false alarms.
    # Any difference between 4 and 30 is likely to be a real alarm.
    warning_ranges = check_range.RangeChecker([[31, None]])
    self._name = 'Motor Stacking Sequence'
    # To check neighboring pairs of sequences, we apply windowing to log data,
    # and circular buffer to AIO data.
    # `window_size` defines the length of the window, or the circular buffer.
    # and circular buffer to AIO data.
    # `window_size` defines the length of the window, or the circular buffer.
    # E.g., for a time-series of [A, B, C, D] and a window with length as 2 and
    # step as 1, the windowed log data becomes
    # [[A, B], [B, C], [C, D]], and the circular buffer yields [A, B], [B, C],
    # and [C, D] at each time step.
    window_size = 2
    window_step = 1
    super(MotorStackingSequence, self).__init__(
        for_log, window_size, window_step, normal_ranges, warning_ranges,
        name='MotorStackingSequence(%s)' % self._motors)

  def _RegisterInputs(self):
    """Register what fields will be used to calculate the check results."""
    data = []
    for motor in self._motors:
      data.append(self._Arg('MotorDebug', 'Motor' + motor, 'sequence'))
    return data

  def _Check(self, *values):
    """Calculate check results using the list of registered inputs."""
    for motor_id, sequences in enumerate(values):
      if sequences is None:
        continue
      for received_seq_id in range(len(self._motors)):
        if received_seq_id != motor_id:
          if self._for_log:
            # `sequences` has a shape of (#_timesteps, #_motors) in the log.
            # After applying the windowing, it is shaped into
            # (#_timestep-1, 2, #_motors).
            neighbors = sequences[:, :, received_seq_id]
            # Subtract the sequence of the next message by that of the previous.
            sequence_diff = neighbors[:, 1] - neighbors[:, 0]
          else:
            # `sequences` has a shape of (#_motors) in the received AIO.
            # With the circular buffer of length 2, it becomes an array with
            # the shape (2, #_motors).
            if sequences.shape[0] < self._window_size:
              # Skip, waiting for the circular buffer to fill up.
              continue
            neighbors = sequences[:, received_seq_id]
            sequence_diff = neighbors[1] - neighbors[0]
          name = '%s: Sequence from %s' % (
              _MOTOR_LABELS_HELPER.ShortName(motor_id),
              _MOTOR_LABELS_HELPER.ShortName(received_seq_id))
          self._CheckByRange(name, sequence_diff, self._normal_ranges,
                             self._warning_ranges)


class MotorErrorCheck(base_check.BitmaskErrorLatchCheck):
  """Class to check for errors reported by the motors."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, motors=None, errors=None, name=None):
    """Initializes this check instance.

    Args:
      for_log: True if we are checking against logs, False if we are checking
          realtime messages.
      motors: List of motor names, e.g. ['Pbi, 'Sto'].
      errors: List of errors or list of warnings to check for e.g.
          ['OverSpeed', 'UnderVoltage', 'BadCommand']
      name: Name of the check.
    """
    if motors is None:
      motor_labels = _MOTOR_LABELS_HELPER.ShortNames()
      sources = ['Motor' + motor for motor in motor_labels]
    else:
      sources = ['Motor' + motor for motor in motors]

    field = 'motor_error'
    enum_helper = _MOTOR_ERROR_FLAGS_HELPER
    error_if_fail = True

    super(MotorErrorCheck, self).__init__(for_log, 'MotorStatus', sources,
                                          field, enum_helper, error_if_fail,
                                          errors, name)

  def _GetMaskedValues(self, values, error_bitmask):
    # We assume `Timeout` happens after the operator Ctrl-C the controller,
    # which results in motor errors, but these should not be reported.
    timeout = values & _MOTOR_ERROR_FLAGS_HELPER.Value('Timeout')
    return numpy.diff((values & error_bitmask) & (~timeout)) > 0


class MotorWarningCheck(base_check.BitmaskErrorLatchCheck):
  """Class to check for warnings reported by the motors."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, motors=None, errors=None, name=None):
    """See MotorErrorCheck for documentation."""
    if motors is None:
      motor_labels = _MOTOR_LABELS_HELPER.ShortNames()
      sources = ['Motor' + motor for motor in motor_labels]
    else:
      sources = ['Motor' + motor for motor in motors]

    field = 'motor_warning'
    enum_helper = _MOTOR_WARNING_FLAGS_HELPER
    error_if_fail = False

    super(MotorWarningCheck, self).__init__(for_log, 'MotorStatus', sources,
                                            field, enum_helper, error_if_fail,
                                            errors, name)


class MotorChecks(base_check.ListOfChecks):
  """The motor checklist."""

  def __init__(self, for_log):
    self._items_to_check = [
        MotorStackPairVoltageDiff(for_log, ['MotorPto', 'MotorSbo']),
        MotorStackPairVoltageDiff(for_log, ['MotorPbo', 'MotorSto']),
        MotorStackPairVoltageDiff(for_log, ['MotorPti', 'MotorSbi']),
        MotorStackPairVoltageDiff(for_log, ['MotorPbi', 'MotorSti']),
        MotorStackingSequence(for_log),
    ]

    errors_to_ignore = ['Timeout', 'RemoteFault', 'All']

    self._items_to_check += [
        MotorErrorCheck(for_log, errors=[error],
                        name='Motor Error (%s)' % error)
        for error in _MOTOR_ERROR_FLAGS_HELPER.ShortNames()
        if error not in errors_to_ignore
    ] + [
        MotorWarningCheck(for_log, errors=[error],
                          name='Motor Warning (%s)' % error)
        for error in _MOTOR_WARNING_FLAGS_HELPER.ShortNames()
    ]
