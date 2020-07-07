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

"""Sample checks for the log analyzer."""

import os

import makani
from makani.analysis.checks import base_check
from makani.analysis.checks import gradebook
from makani.analysis.checks import gradebook_base_check


class MotorSpeedCheck(base_check.BaseCheckItem):
  """Demo how to write a check from scratch using motor speed as an example.

  Two class methods have to be implemented: _RegisterInputs and _Check.

  This class will work for both log analysis and real-time AIO monitoring.
  """

  @base_check.RegisterSpecs
  def __init__(self, for_log, normal_ranges, warning_ranges):
    """Initialize the checks.

    This function is just to provide an example. It is NOT needed in this
    case because it is doing nothing other than calling the __init__ of the
    base class.

    Args:
      for_log: True if this check is used for log analysis. Otherwise, it is
          for realtime AIO monitoring.
      normal_ranges: Ranges in which values should be regarded as normal.
          This is derived BaseRange object or values to construct one. E.g.
          [[None, 10], [60, 180]]: -infinity - 10, 60 - 180 (inclusive).
          [0]: In the range only if value equals to 0.
          [{1, 3, 5}]: In the range only if value equals to 1, 3, or 5.
      warning_ranges: Ranges in which warning are raised if values fall out ot
          normal_ranges. Values outside of warning_ranges should raise errors.
    """
    super(MotorSpeedCheck, self).__init__(
        for_log, normal_ranges, warning_ranges)

  def _RegisterInputs(self):
    """Indicates what fields are needed for the check.

    Returns:
      A list of arguments.
        Each argument contains three components: message type,
        aio node (short name), and messge field. The message field is a string
        that indexes into the message struct. It can refer to arrays and dicts.
        E.g. "fc_mon.analog_data[2]".
        In addition, "timestamp" will refer to the derived field
        "capture_header.tv_sec + capture_header.tv_usec * 1e-6".
    """
    #                |  Message  |  Aio node  | Message |
    #                |   type    | Short name | field   |
    return [self._Arg('MotorDebug', 'MotorSbi', 'omega'),
            self._Arg('MotorDebug', 'MotorPti', 'omega')]

  def _Check(self, sbi_omega, pti_omega):
    """Produce the result of the check.

    Compute values based on the input arguments and check the results.

    The arguments list to this function is the ordered list of retrieved values
    corresponding to message fields registered by the _RegisterInputs function.

    Args:
      sbi_omega: The speed of SBI. It is a 1D array (time series) for log
          processing, and a scalar for realtime AIO monitoring.
      pti_omega: The speed of PTI.
    """

    # Any computation can be inserted here.
    # You can use self._for_log as a condition to distinguish computation for
    # logged time series and computation for realtime AIO messages.

    # Eventually, call _CheckByRange to generate some internal result.
    # There can be multiple calls to _CheckByRange, so that you can reuse some
    # computed values and run different but closely related exams. You can
    # think of this as sub-checks or attributes of a check.

    # Check only if the message is available and and the field exists.
    if sbi_omega is not None:
      self._CheckByRange(
          'SBI Motor Speed',    # Name of this check.
          abs(sbi_omega),       # Value to check. This can be any computed numpy
                                # array (for logs) or scalar (for realtime AIO).
          self._normal_ranges,  # A derived BaseRange object. Values are
                                # as "normal" if they fall into this range.
          self._warning_ranges  # A derived BaseRange object. Values raise
                                # warnings if they fall out of _normal_ranges
                                # but inside of _warning_ranges. If the values
                                # fall out of even the _warning_ranges, an error
                                # will be raised.
      )

    if pti_omega is not None:
      self._CheckByRange(
          'PTI Motor Speed', abs(pti_omega), self._normal_ranges,
          self._warning_ranges)


class GradebookMotorSpeed(gradebook_base_check.GradebookItem):
  """Demo how to write a simple check.

  Sometimes we just want to apply some range checks to a message field. A
  quicker way is to derive from GradebookItem.
  """

  @base_check.RegisterSpecs
  def __init__(self, for_log, normal_ranges, warning_ranges):
    super(GradebookMotorSpeed, self).__init__(
        'Gradebook Motor Speed',  # Base name of the check.
        self.DataIndex('MotorDebug', 'MotorSbo', 'omega'),  # Message field.
        None,  # Optional function to preprocess the value (e.g. lambda x: -x).
        for_log, normal_ranges, warning_ranges  # Remaining BaseCheckItem args.
    )


class SpeedChecks(base_check.ListOfChecks):
  """A list of checks.

  An easy and programmatic way to gather checks and group them into a job, which
  can be submitted by the log analyzer's sandbox request.
  """

  def __init__(self, for_log):
    self._items_to_check = [
        MotorSpeedCheck(for_log, [[0, 10], [80, 150]], [[0, 180]]),
        GradebookMotorSpeed(for_log, [[0, 10], [80, 150]], [[0, 180]]),
    ]


class MotorThermalChecks(gradebook_base_check.GradebookChecks):
  """A list of checked populated from a dict.

  To quickly populate several checks and run them in the sandbox request, you
  can define a dict structured by index layers [message type, aio node, field].
  """

  # Needed to identify sources of message types when initializing the Gradebook.
  _NETWORK_YAML_FILE = os.path.join(makani.HOME,
                                    'avionics/network/network.yaml')

  def __init__(self, for_log):
    """Initialize the list of checks.

    Args:
      for_log: True if the checks are used for log analysis. Otherwise, they are
          for realtime AIO monitoring.
    """

    super(MotorThermalChecks, self).__init__()

    # Create the Python dict describing what checks to include.
    motor_thermal_check_defs = {
        'imports': {
            'motor_thermal_types':  # Import a module to enable indexing by
                                    # the enum's name (see "temps" field below).
                'makani.avionics.common.motor_thermal_types'
        },
        'checks': {
            'MotorStatus': {
                '(Motor.*)': {  # Regular expressions are allowed here.
                    'bus_voltage': {
                        'normal_ranges': [[1, 200], [820, 900]],
                        'name': 'Voltage (V)'
                    },
                    'temps[motor_thermal_types.kMotorThermalChannelBoard]': {
                        'normal_ranges': [[10, 65]],
                        'name': 'Board Temperature (C)'
                    },
                }
            }
        }
    }
    self.Initialize(
        gradebook.Gradebook(motor_thermal_check_defs, self._NETWORK_YAML_FILE),
        for_log)
