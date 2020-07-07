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

"""Monitor indicators related to control telemetry."""

import collections
import ctypes
import math

from makani.analysis.checks import avionics_util
from makani.analysis.control import geometry
from makani.avionics.common import cvt
from makani.avionics.common import loadcell_types
from makani.avionics.common import pack_avionics_messages as avionics_messages
from makani.avionics.network import aio_labels
from makani.avionics.network import aio_node
from makani.common.c_math import filter as c_math_filter
from makani.common.c_math import util as c_math_util
from makani.control import actuator_util
from makani.control import common as control_common
from makani.control import control_params
from makani.control import control_types
from makani.control import system_params
from makani.control import system_types
from makani.control.experiments import experiment_types
from makani.control.fault_detection import fault_detection_types as fd
from makani.control.hover import hover_angles
from makani.gs.monitor import monitor_common
from makani.gs.monitor import monitor_params
from makani.gs.monitor2.apps.layout import indicator
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.layout import widgets
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import avionics
from makani.lib.python import c_helpers
from makani.lib.python import ctype_util
from makani.lib.python import struct_tree
from makani.lib.python.h5_utils import numpy_utils
from makani.system import labels as system_labels
from makani.system import labels_util
import numpy

_AIO_NODE_HELPER = c_helpers.EnumHelper('AioNode', aio_node)
_FLIGHT_MODE_HELPER = c_helpers.EnumHelper('FlightMode', control_types)
_FLIGHT_PLAN_HELPER = c_helpers.EnumHelper('FlightPlan', system_params)
_INITIALIZATION_STATE_HELPER = c_helpers.EnumHelper(
    'InitializationState', control_types)
_TEST_SITE_HELPER = c_helpers.EnumHelper('TestSite', system_params)
_WING_SERIAL_HELPER = c_helpers.EnumHelper('WingSerial', system_params)
_GS_MODEL_HELPER = c_helpers.EnumHelper('GroundStationModel', system_params)
_SIMULATOR_HITL_LEVEL = c_helpers.EnumHelper(
    'SimulatorHitlLevel', system_params)
_SERVO_LABEL_HELPER = c_helpers.EnumHelper('ServoLabel', aio_labels,
                                           prefix='kServo')
_LOADCELL_NODE_LABEL_HELPER = c_helpers.EnumHelper(
    'LoadcellNodeLabel', aio_labels, prefix='kLoadcellNode')
_SUBSYSTEM_LABEL_HELPER = c_helpers.EnumHelper('Subsys', fd)
_FAULT_TYPE_HELPER = c_helpers.EnumHelper('FaultType', fd)
_LOADCELL_ERROR_HELPER = c_helpers.EnumHelper('LoadcellError', loadcell_types,
                                              prefix='kLoadcellError')
_TETHER_NODE_FLAGS_HELPER = c_helpers.EnumHelper('TetherNodeFlag', cvt)

_EXPERIMENT_TYPE_HELPER = c_helpers.EnumHelper('ExperimentType',
                                               experiment_types)

_CONTROL_PARAMS = control_params.GetControlParams().contents
_MONITOR_PARAMS = monitor_params.GetMonitorParams().contents
_SYSTEM_PARAMS = system_params.GetSystemParams().contents
_GLOBAL_PARAMS = system_params.g_sys


def _IsControllerMessageValid(mode, *attributes):
  if mode == common.SPARSE_COMMS_MODE:
    return (attributes[1] and
            attributes[0].no_update_count <=
            common.MAX_NO_UPDATE_COUNT_CONTROL_TELEMETRY)
  elif mode == common.FULL_COMMS_MODE:
    return (attributes[0] and
            control_common.IsControlSystemRunning(attributes[0].init_state))
  else:
    assert False


def _GetControllerMessageAttributes(mode):
  if mode == common.SPARSE_COMMS_MODE:
    return [
        ('filtered', 'merge_tether_down', 'tether_down.control_telemetry'),
        ('filtered', 'merge_tether_down', 'valid'),
        ('filtered', 'merge_tether_down', 'timestamp_sec'),
        # TetherDown message should be preferred whenever possible.
        ('ControlTelemetry', None),
    ]
  elif mode == common.FULL_COMMS_MODE:
    # TODO: Make "choose any source" more explicit than a None.
    return [('ControlTelemetry', None),]
  else:
    assert False


class ControllerInitStateIndicator(indicator.SingleAttributeIndicator):
  """Indicator about the controller init state."""

  def __init__(self):
    super(ControllerInitStateIndicator, self).__init__(
        ('ControlTelemetry', None), 'Controller State')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry):
    init_state = telemetry.init_state
    text = '%s' % _INITIALIZATION_STATE_HELPER.ShortName(init_state)
    if control_common.IsControlSystemRunning(init_state):
      stoplight = stoplights.STOPLIGHT_NORMAL
    else:
      stoplight = stoplights.STOPLIGHT_WARNING
    return text, stoplight


class BaseControlSlowIndicator(indicator.SingleAttributeIndicator):

  def __init__(self, name):
    super(BaseControlSlowIndicator, self).__init__(
        ('ControlSlowTelemetry', None), name)


class BaseControllerRunningIndicator(indicator.MultiModeIndicator):
  """Base class for controller indicators."""

  def _GetMessageAttributes(self):
    return _GetControllerMessageAttributes(self._mode)

  def _GetTimestamp(self, *args):
    if self._mode == common.FULL_COMMS_MODE:
      return args[0].capture_info['timestamp']
    else:
      return args[2]

  def _IsValidInput(self, *attributes):
    return _IsControllerMessageValid(self._mode, *attributes)


class BaseControllerRunningListChart(indicator.MultiModeListChart):
  """List chart that is active only when a controller is running."""

  def _GetTimestamps(self, *args):
    if self._mode == common.FULL_COMMS_MODE:
      return args[0].capture_info['timestamp']
    elif self._mode == common.SPARSE_COMMS_MODE:
      return args[2]
    else:
      assert False

  def _GetMessageAttributes(self):
    return _GetControllerMessageAttributes(self._mode)

  def _GetControlTelemetry(self, *args):
    """Get the control telemetry message.

    Args:
      *args: The list of attributes passed to the Filter function.
          The list of args are fields whose indices are defined by
          _GetControllerMessageAttributes()

    Returns:
      The ControlTelemetry message.
    """

    if self._mode == common.FULL_COMMS_MODE:
      return args[0]
    elif self._mode == common.SPARSE_COMMS_MODE:
      return args[3]
    else:
      assert False

  def _IsValidInput(self, *attributes):
    return _IsControllerMessageValid(self._mode, *attributes)


class BaseControlSketch2D(indicator.MultiModeSketch2D):

  def _GetMessageAttributes(self):
    return _GetControllerMessageAttributes(self._mode)

  def _IsValidInput(self, *attributes):
    return _IsControllerMessageValid(self._mode, *attributes)


def _GetHoverPositionErrors(control_telemetry):
  """Get hover position errors and stoplight from ControlTelemetry message."""
  hover = control_telemetry.hover
  position_errors = [-(hover.wing_pos_g_cmd.z
                       - control_telemetry.state_est.Xg.z),
                     hover.wing_pos_b_error.y,
                     hover.wing_pos_b_error.z]

  # We use the angle feedback being greater than the maximum angle
  # feedback from the position alone as an indicator that the
  # tangential feedback is saturated.
  #
  # TODO: Determine better method of detecting saturation.
  if not control_common.AnyHoverFlightMode(control_telemetry.flight_mode):
    stoplight = stoplights.STOPLIGHT_ANY
  elif (abs(hover.angles_fb.z)
        >= _CONTROL_PARAMS.hover.position.max_pos_angle_fb):
    stoplight = stoplights.STOPLIGHT_WARNING
  else:
    stoplight = stoplights.STOPLIGHT_NORMAL

  return position_errors, stoplight


def _GetHoverVelocityErrors(control_telemetry):
  """Get hover velocity errors and stoplight from ControlTelemetry message."""
  hover = control_telemetry.hover
  velocity_errors = [
      -(hover.wing_vel_g_cmd.z - control_telemetry.state_est.Vg.z),
      hover.wing_vel_b_error.y, hover.wing_vel_b_error.z]

  # We use the angle feedback being greater than the maximum angle
  # feedback from the velocity alone as an indicator that the
  # tangential feedback is saturated.
  #
  # TODO: Determine better method of detecting saturation.
  if not control_common.AnyHoverFlightMode(control_telemetry.flight_mode):
    stoplight = stoplights.STOPLIGHT_ANY
  elif (abs(hover.angles_fb.z)
        >= _CONTROL_PARAMS.hover.position.max_vel_angle_fb):
    stoplight = stoplights.STOPLIGHT_WARNING
  else:
    stoplight = stoplights.STOPLIGHT_NORMAL

  return velocity_errors, stoplight


class FlightModeIndicator(BaseControllerRunningIndicator):
  """Indicator for flight mode."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(FlightModeIndicator, self).__init__(mode, 'Flight mode')

  def _Filter(self, telemetry, *args):
    if not self._IsValidInput(telemetry, *args):
      if self._mode == common.FULL_COMMS_MODE:
        if telemetry:
          init_state = telemetry.init_state
          text = 'Init: %s' % _INITIALIZATION_STATE_HELPER.ShortName(init_state)
          return text, stoplights.STOPLIGHT_WARNING
        else:
          return '--', stoplights.STOPLIGHT_UNAVAILABLE
      elif self._mode == common.SPARSE_COMMS_MODE:
        return '--', stoplights.STOPLIGHT_UNAVAILABLE
      else:
        assert False

    flight_mode = telemetry.flight_mode
    if flight_mode in _FLIGHT_MODE_HELPER:
      return (_FLIGHT_MODE_HELPER.ShortName(flight_mode),
              stoplights.STOPLIGHT_NORMAL)
    else:
      return 'Invalid', stoplights.STOPLIGHT_ERROR


class ExperimentIndicator(BaseControllerRunningIndicator):
  """Indicator for experiments."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(ExperimentIndicator, self).__init__(mode, 'Experiment')

  @indicator.ReturnIfInputInvalid(
      '--:  case --\nStaged: --', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    if self._mode == common.FULL_COMMS_MODE:
      experiment_type = telemetry.state_est.experiment.active_type
      case_id = telemetry.state_est.experiment.case_id
      staged_experiment = telemetry.state_est.experiment.staged_type
    elif self._mode == common.SPARSE_COMMS_MODE:
      experiment_type = telemetry.experiment_type
      case_id = telemetry.experiment_case_id
      if args[-1]:
        # args[-1] is the ControlTelemetry when available.
        # See _GetControllerMessageAttributes(mode).
        staged_experiment = args[-1].state_est.experiment.staged_type
      else:
        staged_experiment = None
    else:
      assert False

    # TODO: Use GetExperimentCaseDescription() to describe
    # test cases.
    experiment_type = _EXPERIMENT_TYPE_HELPER.ShortName(experiment_type)
    staged_experiment = (
        'unknown' if staged_experiment is None
        else _EXPERIMENT_TYPE_HELPER.ShortName(staged_experiment))
    return (
        '%s: case %d\nStaged: %s' % (
            experiment_type, case_id, staged_experiment),
        stoplights.STOPLIGHT_NORMAL)


class FlightModeGatesIndicator(BaseControllerRunningIndicator):
  """Indicator for flight mode gates."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(FlightModeGatesIndicator, self).__init__(mode, 'Gates')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    if self._mode == common.SPARSE_COMMS_MODE:
      flight_mode = telemetry.flight_mode
      gated_mode = control_common.GetNextFlightMode(flight_mode)
      flight_mode_gates = telemetry.flight_mode_gates
      time = float(telemetry.flight_mode_time) * 0.1
    elif self._mode == common.FULL_COMMS_MODE:
      flight_mode = telemetry.flight_mode
      gated_mode = control_common.GetNextFlightMode(flight_mode)
      flight_mode_gates = telemetry.flight_mode_gates[gated_mode]
      time = telemetry.time
    else:
      assert False

    gate_strs = []
    for i in range(control_common.GetNumFlightModeGates(gated_mode)):
      if flight_mode_gates & (1 << i):
        gate_strs.append(
            ctypes.string_at(monitor_common.GateToString(gated_mode, i)))

    if gate_strs:
      gate_str = gate_strs[int(time) % len(gate_strs)]
    else:
      gate_str = '--'
    return gate_str, stoplights.STOPLIGHT_NORMAL


class FlightPlanIndicator(BaseControlSlowIndicator):
  """Indicator for flight plan."""

  _NORMAL_PLANS = {
      _FLIGHT_PLAN_HELPER.Value(s)
      for s in ['TurnKey', 'HighHover']
  }

  def __init__(self):
    super(FlightPlanIndicator, self).__init__('Flight plan')

  def _Filter(self, control_slow_telemetry):
    flight_plan = control_slow_telemetry.flight_plan
    stoplight = (
        stoplights.STOPLIGHT_NORMAL if flight_plan in self._NORMAL_PLANS
        else stoplights.STOPLIGHT_ERROR)
    return _FLIGHT_PLAN_HELPER.ShortName(flight_plan), stoplight


class ControlTimeIndicator(BaseControllerRunningIndicator):
  """Timing indicator."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(ControlTimeIndicator, self).__init__(mode, 'Time [s]')

  def _FormatTime(self, seconds):
    ss = seconds % 60
    minutes = seconds / 60
    mm = minutes % 60
    hh = minutes / 60
    return '%3d:%02d:%02d' % (hh, mm, ss)

  @indicator.ReturnIfInputInvalid('--\n\n', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    if self._mode == common.FULL_COMMS_MODE:
      control_telemetry = args[0]
      text = 'Total: %s, Mode: %s' % (
          self._FormatTime(control_telemetry.time),
          self._FormatTime(control_telemetry.flight_mode_time))
    elif self._mode == common.SPARSE_COMMS_MODE:
      control_telemetry = args[0]
      text = 'Mode: %s' % self._FormatTime(
          control_telemetry.flight_mode_time * 0.1)
    else:
      assert False
    return text, stoplights.STOPLIGHT_NORMAL


class ControllerTimingIndicator(BaseControllerRunningIndicator):
  """Loop Time indicator."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(ControllerTimingIndicator, self).__init__(mode, 'Loop Time')

  def _GetMessageAttributes(self):
    attributes = _GetControllerMessageAttributes(self._mode)
    if self._mode == common.SPARSE_COMMS_MODE:
      attributes += [('filtered', 'merge_tether_down', 'max_loop_time')]
    return attributes

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    if self._mode == common.FULL_COMMS_MODE:
      control_telemetry = args[0]
      controller_time = (control_telemetry.finish_usec -
                         control_telemetry.start_usec) / 1.0e6
      loop_time = control_telemetry.loop_usec / 1.0e6
      max_loop_time = control_telemetry.max_loop_usec / 1.0e6
    elif self._mode == common.SPARSE_COMMS_MODE:
      tether_control_telemetry = args[0]
      loop_time = tether_control_telemetry.loop_time
      max_loop_time = args[4]
      controller_time = None
    else:
      assert False

    global_ts = _GLOBAL_PARAMS.ts.contents.value

    text = 'Total: {:3.0f}% (max: {:3.0f}%)'.format(
        100.0 * loop_time / global_ts,
        100.0 * max_loop_time / global_ts)

    if controller_time is not None:
      text += '\nCont.: {:3.0f}%'.format(100.0 * controller_time / global_ts)

    if max_loop_time > 0.65 * global_ts:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL
    return text, stoplight


class LoopCountIndicator(BaseControllerRunningIndicator):
  """The indicator to count crosswind loops."""

  def _GetMessageAttributes(self):
    args = super(LoopCountIndicator, self)._GetMessageAttributes()
    args += [
        ('filtered', 'loop_count', 'current_rev_count'),
        ('filtered', 'loop_count', 'total_rev_count')]
    return args

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(LoopCountIndicator, self).__init__(mode, 'Loop Count', font_size=18)

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    flight_mode = telemetry.flight_mode
    current_rev_count = args[-2]
    total_rev_count = args[-1]
    if not (control_common.AnyCrosswindFlightMode(flight_mode) or
            flight_mode == control_types.kFlightModeTransIn):
      text = 'Last:% 3d\nTotal:% 3d' % (
          current_rev_count, total_rev_count)
      return text, stoplights.STOPLIGHT_ANY
    else:
      text = 'Current:% 3d\n' % current_rev_count
      return text, stoplights.STOPLIGHT_NORMAL


class JoystickIndicator(indicator.SingleAttributeIndicator):

  def __init__(self):
    super(JoystickIndicator, self).__init__(
        ('JoystickStatus', 'JoystickA'), 'Joystick')

  def _Filter(self, joystick):
    text = 'T: {: 3.2f} R: {: 3.2f}\nP: {: 3.2f} Y: {: 3.2f}'.format(
        joystick.throttle, joystick.roll, joystick.pitch, joystick.yaw)
    return text, stoplights.STOPLIGHT_NORMAL


class HoverGainRampScaleIndicator(BaseControllerRunningIndicator):

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(HoverGainRampScaleIndicator, self).__init__(
        mode, 'Hover gain ramp')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):

    if self._mode == common.FULL_COMMS_MODE:
      gain_ramp_scale = telemetry.hover.gain_ramp_scale
    elif self._mode == common.SPARSE_COMMS_MODE:
      gain_ramp_scale = telemetry.gain_ramp_scale
    else:
      assert False

    flight_mode = telemetry.flight_mode

    text = '{:4.2f}'.format(gain_ramp_scale)
    if not control_common.AnyHoverFlightMode(flight_mode):
      stoplight = stoplights.STOPLIGHT_ANY
    else:
      stoplight = (stoplights.STOPLIGHT_WARNING if gain_ramp_scale < 1.0
                   else stoplights.STOPLIGHT_NORMAL)
    return text, stoplight


class WindStateEstIndicator(BaseControllerRunningIndicator):

  def __init__(self):
    super(WindStateEstIndicator, self).__init__(
        common.FULL_COMMS_MODE, 'Wind')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, control_telemetry):
    wind_g = control_telemetry.state_est.wind_g
    if wind_g.valid:
      text = '{: 3.0f} deg  {:4.1f} m/s'.format(
          numpy.rad2deg(common.WindAziToWindNed(wind_g.dir_f)),
          wind_g.speed_f)
      stoplight = stoplights.STOPLIGHT_NORMAL
    else:
      text = '--'
      stoplight = stoplights.STOPLIGHT_WARNING
    return text, stoplight


class HoverAnglesChart(BaseControllerRunningListChart):

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode, **widget_kwargs):
    super(HoverAnglesChart, self).__init__(
        mode, ['Roll', 'Pitch', 'Yaw'], 'Hover angles [deg]',
        precision=1, **widget_kwargs)

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    if self._mode == common.FULL_COMMS_MODE:
      control_telemetry = args[0]
      flight_mode = control_telemetry.flight_mode
      angles = control_telemetry.hover.angles
      angles = [angles.x, angles.y, angles.z]
    elif self._mode == common.SPARSE_COMMS_MODE:
      control_telemetry = args[0]
      flight_mode = control_telemetry.flight_mode
      pitch = control_telemetry.pitch
      roll = control_telemetry.roll
      yaw = control_telemetry.yaw
      pos_g = control_telemetry.pos_g

      dcm_g2b = hover_angles.Mat3()
      hover_angles.AngleToDcm(yaw, pitch, roll, hover_angles.kRotationOrderZyx,
                              ctypes.pointer(dcm_g2b))
      angles = hover_angles.Vec3()
      wing_pos_g = hover_angles.Vec3(pos_g[0], pos_g[1], pos_g[2])
      # TODO: Need vessel_pos_g to calculate the hover angles properly.
      vessel_pos_g = hover_angles.kVec3Zero
      hover_angles.HoverAnglesGetAngles(
          ctypes.pointer(wing_pos_g), ctypes.pointer(vessel_pos_g),
          ctypes.pointer(dcm_g2b), ctypes.pointer(angles))
      angles = [angles.x, angles.y, angles.z]
    else:
      assert False

    # Apply limits before converting to degrees.
    stoplight = stoplights.MostSevereStoplight(
        *[stoplights.SetByLimits(
            angles[i],
            self._GetLimits(_MONITOR_PARAMS.est.hover_angles[i]))
          for i in range(3)])

    # Convert to degrees for human consumption.
    angles = [c_math_util.Wrap(numpy.rad2deg(value), -180.0, 180.0)
              for value in angles]

    # Hover Angles are only sensible in flight modes that take place
    # near the hover attitude.
    if not (control_common.AnyHoverFlightMode(flight_mode)
            or flight_mode == _FLIGHT_MODE_HELPER.Value('Perched')):
      stoplight = stoplights.STOPLIGHT_ANY
      angles = None

    return self._GetTimestamps(*args), angles, stoplight


class TetherAnglesChart(indicator.BaseAttributeDictChart):
  """Indicator for tether pitch/roll angle chart and text display."""

  _roll_label = 'Roll'
  _pitch_label = 'Pitch'

  def __init__(self, name='Tether angles [deg]', **widget_kwargs):
    super(TetherAnglesChart, self).__init__(
        [('ControlTelemetry', None, 'capture_info.timestamp'),
         ('ControlTelemetry', None, 'state_est.tether_force_b.sph.roll'),
         ('ControlTelemetry', None, 'state_est.tether_force_b.sph.pitch')],
        [self._roll_label, self._pitch_label], name, precision=1,
        **widget_kwargs)

  @indicator.ReturnIfInputInvalid({}, {}, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, timestamp, roll_rad, pitch_rad):
    if timestamp is None or roll_rad is None or pitch_rad is None:
      return {}, {}, stoplights.STOPLIGHT_UNAVAILABLE
    else:
      angles = {self._roll_label: numpy.rad2deg(roll_rad),
                self._pitch_label: numpy.rad2deg(pitch_rad)}
      timestamps = {self._roll_label: timestamp,
                    self._pitch_label: timestamp}
      pitch_stoplight = stoplights.SetByLimits(
          pitch_rad, self._GetLimits(_MONITOR_PARAMS.est.tether_pitch))
      return timestamps, angles, pitch_stoplight


class HoverPositionErrorsIndicator(BaseControllerRunningIndicator):

  def __init__(self):
    super(HoverPositionErrorsIndicator, self).__init__(
        common.FULL_COMMS_MODE, 'Pos. error [m]')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, control_telemetry):
    errors, stoplight = _GetHoverPositionErrors(control_telemetry)
    return 'alt: {: 3.2f}   tng: {: 3.2f}\nrad: {: 3.2f}'.format(
        errors[0], errors[1], errors[2]), stoplight


class HoverPathErrorsChart(BaseControllerRunningListChart):

  def __init__(self, **widget_kwargs):
    super(HoverPathErrorsChart, self).__init__(
        common.FULL_COMMS_MODE, ['Azi err', 'Ele err', 'Raw azi err'],
        'Path error [deg]',
        precision=1, ylim=[-10.0, 10.0], **widget_kwargs)

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, control_telemetry):
    if not control_common.AnyHoverFlightMode(control_telemetry.flight_mode):
      return None, None, stoplights.STOPLIGHT_ANY
    hover = control_telemetry.hover
    cmd_x = hover.wing_pos_g_cmd.x
    cmd_y = hover.wing_pos_g_cmd.y
    raw_cmd_x = hover.raw_wing_pos_g_cmd.x
    raw_cmd_y = hover.raw_wing_pos_g_cmd.y
    pos_x = control_telemetry.state_est.Xg.x
    pos_y = control_telemetry.state_est.Xg.y
    azi_error = c_math_util.Wrap(
        numpy.arctan2(cmd_y, cmd_x) - numpy.arctan2(pos_y, pos_x),
        -numpy.pi, numpy.pi)

    raw_azi_error = c_math_util.Wrap(
        numpy.arctan2(raw_cmd_y, raw_cmd_x) - numpy.arctan2(pos_y, pos_x),
        -numpy.pi, numpy.pi)

    ele_error = c_math_util.Wrap(
        numpy.arctan2(-hover.wing_pos_g_cmd.z,
                      numpy.linalg.norm((cmd_x, cmd_y))) -
        numpy.arctan2(-control_telemetry.state_est.Xg.z,
                      numpy.linalg.norm((pos_x, pos_y))),
        -numpy.pi, numpy.pi)

    errors = [numpy.rad2deg(azi_error), numpy.rad2deg(ele_error),
              numpy.rad2deg(raw_azi_error)]
    return (self._GetTimestamps(control_telemetry), errors,
            stoplights.STOPLIGHT_NORMAL)


class HoverPositionErrorsChart(BaseControllerRunningListChart):

  def __init__(self, **widget_kwargs):
    super(HoverPositionErrorsChart, self).__init__(
        common.FULL_COMMS_MODE, ['alt', 'tng', 'rad'], 'Pos. error [m]',
        precision=1, **widget_kwargs)

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, control_telemetry):
    if not control_common.AnyHoverFlightMode(control_telemetry.flight_mode):
      return None, None, stoplights.STOPLIGHT_ANY

    position_errors, stoplight = _GetHoverPositionErrors(control_telemetry)
    return self._GetTimestamps(control_telemetry), position_errors, stoplight


class HoverVelocityErrorsIndicator(BaseControllerRunningIndicator):

  def __init__(self):
    super(HoverVelocityErrorsIndicator, self).__init__(
        common.FULL_COMMS_MODE, 'Vel. error [m/s]')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, control_telemetry):
    errors, stoplight = _GetHoverVelocityErrors(control_telemetry)
    return 'v_alt: {: 3.2f}   v_tng: {: 3.2f}\nv_rad: {: 3.2f}'.format(
        errors[0], errors[1], errors[2]), stoplight


class HoverVelocityErrorsChart(BaseControllerRunningListChart):

  def __init__(self, **widget_kwargs):
    super(HoverVelocityErrorsChart, self).__init__(
        common.FULL_COMMS_MODE, ['v_alt', 'v_tng', 'v_rad'], 'Vel. error [m/s]',
        precision=1, **widget_kwargs)

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, control_telemetry):
    if not control_common.AnyHoverFlightMode(control_telemetry.flight_mode):
      return None, None, stoplights.STOPLIGHT_ANY

    velocity_errors, stoplight = _GetHoverVelocityErrors(control_telemetry)
    return self._GetTimestamps(control_telemetry), velocity_errors, stoplight


class TensionChart(BaseControllerRunningListChart):
  """Tension chart."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode, **widget_kwargs):
    super(TensionChart, self).__init__(
        mode, ['Cmd', 'Meas'], 'Tension [kN]', precision=1, **widget_kwargs)

    self._params = _CONTROL_PARAMS.hover.tension
    self._tension_hard_pid = c_helpers.CopyToEquivalentType(
        self._params.tension_hard_pid, c_math_filter.PidParams)
    self._tension_soft_pid = c_helpers.CopyToEquivalentType(
        self._params.tension_soft_pid, c_math_filter.PidParams)

    self._general_ranges = self._GetLimits(
        _MONITOR_PARAMS.tether.tension)
    self._crosswind_ranges = self._GetLimits(
        _MONITOR_PARAMS.tether.tension_crosswind)
    self._hover_ranges = self._GetLimits(
        _MONITOR_PARAMS.tether.tension_hover)

  def _FilterSparseMode(self, telemetry, *args):
    tensions = [None, None]
    tensions[1] = telemetry.tension * 0.001
    flight_mode = telemetry.flight_mode
    # TODO: Add saturation check when using TetherDown.
    if control_common.AnyHoverFlightMode(flight_mode):
      tensions[0] = telemetry.tension_command * 0.001
    return flight_mode, tensions, [], True

  def _FilterFullMode(self, telemetry, *args):
    tensions = [None, None]
    warning_strings = []
    state_est = telemetry.state_est
    is_tension_valid = state_est.tether_force_b.valid
    if is_tension_valid:
      tensions[1] = state_est.tether_force_b.sph.tension * 0.001
    else:
      warning_strings.append('Invalid tension state est.')

    flight_mode = telemetry.flight_mode
    if is_tension_valid and control_common.AnyHoverFlightMode(flight_mode):
      hover = telemetry.hover
      tensions[0] = hover.tension_cmd * 0.001

      if state_est.winch.valid:
        tension_pid = c_math_filter.PidParams()
        c_math_filter.CrossfadePidParams(ctypes.byref(self._tension_hard_pid),
                                         ctypes.byref(self._tension_soft_pid),
                                         state_est.winch.payout,
                                         self._params.hard_spring_payout,
                                         self._params.soft_spring_payout,
                                         ctypes.byref(tension_pid))

        # Check for saturated integrator or output pitch command.
        if c_math_util.IsSaturated(hover.int_pitch,
                                   tension_pid.int_output_min,
                                   tension_pid.int_output_max):
          warning_strings.append('int. sat.')
        elif c_math_util.IsSaturated(hover.pitch_ff + hover.pitch_fb,
                                     hover.pitch_min, hover.pitch_max):
          warning_strings.append('pitch sat.')
      else:
        warning_strings.append('Invalid payout')

    return flight_mode, tensions, warning_strings, is_tension_valid

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    if self._mode == common.FULL_COMMS_MODE:
      flight_mode, tensions_kn, warning_strings, is_tension_valid = (
          self._FilterFullMode(telemetry, *args))
    elif self._mode == common.SPARSE_COMMS_MODE:
      flight_mode, tensions_kn, warning_strings, is_tension_valid = (
          self._FilterSparseMode(telemetry, *args))
    else:
      assert False

    if is_tension_valid:
      stoplight = stoplights.SetByRanges(
          tensions_kn[1] * 1000.0, self._general_ranges['normal'],
          self._general_ranges['warning'])
      if control_common.AnyCrosswindFlightMode(flight_mode):
        crosswind_stoplight = stoplights.SetByRanges(
            tensions_kn[1] * 1000.0, self._crosswind_ranges['normal'],
            self._crosswind_ranges['warning'])
        stoplight = stoplights.MostSevereStoplight(
            stoplight, crosswind_stoplight)
      elif control_common.AnyHoverFlightMode(flight_mode):
        hover_stoplight = stoplights.SetByRanges(
            tensions_kn[1] * 1000.0, self._hover_ranges['normal'],
            self._hover_ranges['warning'])
        stoplight = stoplights.MostSevereStoplight(
            stoplight, hover_stoplight)
    else:
      # Tension is not valid.
      stoplight = stoplights.STOPLIGHT_ERROR

    if warning_strings:
      stoplight = stoplights.MostSevereStoplight(
          stoplight, stoplights.STOPLIGHT_WARNING)
    warning_string = '\n'.join(warning_strings)
    return ([self._GetTimestamps(telemetry, *args)],
            tensions_kn, stoplight, warning_string)


class HoverAngleCommandIndicator(BaseControllerRunningIndicator):
  """Hover angle command indicator."""

  def __init__(self):
    super(HoverAngleCommandIndicator, self).__init__(
        common.FULL_COMMS_MODE, 'Angle command')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, control_telemetry):
    hover = control_telemetry.hover
    text = ('R:% 7.2f rad  int:% 7.2f rad\n'
            'P:% 7.2f rad  int:% 7.2f rad\n'
            'Y:% 7.2f rad  int:% 7.2f rad') % (
                hover.angles_cmd.x, 0.0,
                hover.angles_cmd.y, hover.int_pitch,
                hover.angles_cmd.z, hover.int_angles.z)

    if control_common.AnyHoverFlightMode(control_telemetry.flight_mode):
      stoplight = stoplights.STOPLIGHT_NORMAL
    else:
      stoplight = stoplights.STOPLIGHT_ANY

    return text, stoplight


class HoverThrustMomentIndicator(BaseControllerRunningIndicator):
  """Hover angle command indicator."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(HoverThrustMomentIndicator, self).__init__(mode, 'Thrust-moment')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    int_thrust_str = ''
    int_moment_x_str = ''
    int_moment_y_str = ''
    int_moment_z_str = ''

    if self._mode == common.FULL_COMMS_MODE:
      thrust_cmd = telemetry.thrust_moment.thrust
      moment_cmd = telemetry.thrust_moment.moment
      thrust_avail = telemetry.thrust_moment_avail.thrust
      moment_avail = telemetry.thrust_moment_avail.moment
      moment_cmd = ctype_util.Vec3ToList(moment_cmd)
      moment_avail = ctype_util.Vec3ToList(moment_avail)

      if control_common.AnyHoverFlightMode(telemetry.flight_mode):
        hover = telemetry.hover
        int_moment = hover.int_moment
        int_thrust_str = 'int:% 6.2f kN' % (hover.int_thrust / 1e3)
        int_moment_x_str = 'int:% 6.2f kN-m' % (int_moment.x / 1e3)
        int_moment_y_str = 'int:% 6.2f kN-m' % (int_moment.y / 1e3)
        int_moment_z_str = 'int:% 6.2f kN-m' % (int_moment.z / 1e3)

    elif self._mode == common.SPARSE_COMMS_MODE:
      thrust_cmd = telemetry.thrust
      moment_cmd = telemetry.moment
      thrust_avail = telemetry.thrust_avail
      moment_avail = telemetry.moment_avail

    else:
      assert False

    saturated = {
        'thrust': abs(thrust_cmd - thrust_avail) > 10.0,
        'roll': abs(moment_cmd[0] - moment_avail[0]) > 10.0,
        'pitch': abs(moment_cmd[1] - moment_avail[1]) > 10.0,
        'yaw': abs(moment_cmd[2] - moment_avail[2]) > 10.0,
    }

    if any(saturated[key] for key in ('pitch', 'yaw', 'thrust')):
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    saturated_str = ' '.join(key for key, value in saturated.iteritems()
                             if value)
    if not saturated_str:
      saturated_str = 'none'

    lines = [
        'T:% 6.2f kN     %s' % (thrust_avail / 1e3, int_thrust_str),
        'R:% 6.2f kN-m   %s' % (moment_avail[0] / 1e3, int_moment_x_str),
        'P:% 6.2f kN-m   %s' % (moment_avail[1] / 1e3, int_moment_y_str),
        'Y:% 6.2f kN-m   %s' % (moment_avail[2] / 1e3, int_moment_z_str),
        'Saturated: ' + saturated_str]

    return '\n'.join(lines), stoplight


class WingPosChart(BaseControllerRunningListChart):

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode, **widget_kwargs):
    self._tether_params = _SYSTEM_PARAMS.tether
    super(WingPosChart, self).__init__(
        mode, ['|Xg|', 'xg', 'yg', 'zg'], 'Xg [m]', precision=1,
        **widget_kwargs)

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    if self._mode == common.FULL_COMMS_MODE:
      xg = telemetry.state_est.Xg
      x, y, z = ctype_util.Vec3ToList(xg)
    elif self._mode == common.SPARSE_COMMS_MODE:
      x, y, z = telemetry.pos_g
    else:
      assert False

    xg_norm = numpy.linalg.norm((x, y, z))
    positions = [xg_norm, x, y, z]
    stoplight = stoplights.STOPLIGHT_NORMAL

    # Indicate a warning or error if the position is outside of the
    # tether sphere.
    if xg_norm > 2.0 * self._tether_params.length:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif xg_norm > 1.1 * self._tether_params.length:
      stoplight = stoplights.STOPLIGHT_WARNING

    return self._GetTimestamps(telemetry, *args), positions, stoplight


class AirSpeedChart(BaseControllerRunningListChart):

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode, **widget_kwargs):
    super(AirSpeedChart, self).__init__(
        mode, ['Speed', 'Cmd'], 'Airspeed [m/s]', precision=1, **widget_kwargs)

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    if self._mode == common.FULL_COMMS_MODE:
      flight_mode = telemetry.flight_mode
      airspeed = telemetry.state_est.apparent_wind.sph.v
    elif self._mode == common.SPARSE_COMMS_MODE:
      flight_mode = telemetry.flight_mode
      airspeed = telemetry.airspeed
    else:
      assert False

    control_telemetry = self._GetControlTelemetry(telemetry, *args)

    if (control_common.AnyCrosswindFlightMode(flight_mode) and
        control_telemetry):
      airspeed_cmd = control_telemetry.crosswind.airspeed_cmd

      error_stoplight = stoplights.SetByLimits(
          airspeed - airspeed_cmd,
          self._GetLimits(_MONITOR_PARAMS.est.airspeed_error))

      speed_stoplight = stoplights.SetByLimits(
          airspeed, self._GetLimits(_MONITOR_PARAMS.est.airspeed))

      stoplight = stoplights.MostSevereStoplight(
          speed_stoplight, error_stoplight)

    else:
      airspeed_cmd = None
      stoplight = stoplights.STOPLIGHT_NORMAL
    return (self._GetTimestamps(telemetry, *args),
            [airspeed, airspeed_cmd], stoplight)


class WinchPosIndicator(BaseControllerRunningIndicator):
  """Winch position indicator."""

  def __init__(self):
    super(WinchPosIndicator, self).__init__(
        common.FULL_COMMS_MODE, 'Winch pos | vel')

  @indicator.ReturnIfInputInvalid('', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, control_telemetry):

    text = '{: 4.2f} m | {: 4.2f} m/s'.format(
        control_telemetry.control_input.perch.winch_pos,
        control_telemetry.control_output.winch_vel_cmd)
    return text, stoplights.STOPLIGHT_NORMAL


class PayoutIndicator(BaseControllerRunningIndicator):
  """Payout indicator."""

  def __init__(self):
    super(PayoutIndicator, self).__init__(common.FULL_COMMS_MODE, 'Payout [m]')

  @indicator.ReturnIfInputInvalid('', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, control_telemetry):

    payout = control_telemetry.state_est.winch.payout

    if not control_telemetry.state_est.winch.valid:
      stoplight = stoplights.STOPLIGHT_WARNING
    elif control_telemetry.flight_mode in [
        _FLIGHT_MODE_HELPER.Value('Perched'),
        _FLIGHT_MODE_HELPER.Value('HoverAscend')] and (
            abs(payout) > _MONITOR_PARAMS.est.far_payout_at_perch):
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL
    return '{: 3.2f}'.format(payout), stoplight


class TestSiteIndicator(BaseControlSlowIndicator):
  """Indicator for the test site."""

  def __init__(self):
    super(TestSiteIndicator, self).__init__('Test Site')

  def _Filter(self, control_slow_telemetry):
    return (_TEST_SITE_HELPER.ShortName(control_slow_telemetry.test_site),
            stoplights.STOPLIGHT_NORMAL)


class VersionIndicator(BaseControlSlowIndicator):
  """Indicator about the version of the wing."""

  def __init__(self):
    super(VersionIndicator, self).__init__('Version')

  def _Filter(self, control_slow_telemetry):
    build_info = control_slow_telemetry.build_info
    text = '%s %s\nSN%s %s %s %s' % (
        build_info.date, build_info.time,
        _WING_SERIAL_HELPER.ShortName(control_slow_telemetry.wing_serial),
        _GS_MODEL_HELPER.ShortName(control_slow_telemetry.gs_model),
        _TEST_SITE_HELPER.ShortName(control_slow_telemetry.test_site),
        ctypes.string_at(
            monitor_common.GetHitlDispStr(
                control_slow_telemetry.hitl_config.sim_level,
                control_slow_telemetry.hitl_config.use_software_joystick)))
    # TODO: Remove the wing serial and test site warnings once
    # we begin flying other wings at other places.
    if ((control_slow_telemetry.hitl_config.sim_level ==
         _SIMULATOR_HITL_LEVEL.Value('None')) and
        (control_slow_telemetry.wing_serial in
         [_WING_SERIAL_HELPER.Value('01'),
          _WING_SERIAL_HELPER.Value('04Hover'),
          _WING_SERIAL_HELPER.Value('04Crosswind'),
          _WING_SERIAL_HELPER.Value('06Hover'),
          _WING_SERIAL_HELPER.Value('06Crosswind'),
          _WING_SERIAL_HELPER.Value('07Hover'),
          _WING_SERIAL_HELPER.Value('07Crosswind')]) and
        (control_slow_telemetry.test_site in
         [_TEST_SITE_HELPER.Value('ParkerRanch'),
          _TEST_SITE_HELPER.Value('Norway')])):
      return text, stoplights.STOPLIGHT_NORMAL
    else:
      return text, stoplights.STOPLIGHT_WARNING


class ConstraintWindow(BaseControlSketch2D):
  """The 2D plot showing flight positions in the constraint window."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode, name='Constraint Window'):
    params = _MONITOR_PARAMS.control.constraint_window

    # DCM mapping the g-frame to the indicator frame. Make this an array
    # (with .A) to simplify later operations.
    self._dcm_g2indicator = geometry.AngleToDcm(params.azi_g2indicator,
                                                0.0, 0.0).A

    xlim = [params.plot_xlim[0], params.plot_xlim[1]]
    ylim = [params.plot_ylim[1], params.plot_ylim[0]]

    xy_slice = slice(0, params.num_vertices_xy_cross)
    xs_low = params.xs_low[xy_slice]
    xs_mid = params.xs_mid[xy_slice]
    xs_high = params.xs_high[xy_slice]
    ys_low = params.ys_low[xy_slice]
    ys_mid = params.ys_mid[xy_slice]
    ys_high = params.ys_high[xy_slice]

    xz_slice = slice(0, params.num_vertices_xz_cross)
    zs_vertical = params.zs_vertical[xz_slice]
    xs_vertical = params.xs_vertical[xz_slice]
    polygons = [
        {'x': xs_low, 'y': ys_low, 'color': 'lightblue'},
        {'x': xs_mid, 'y': ys_mid, 'color': 'lightblue'},
        {'x': xs_high, 'y': ys_high, 'color': 'lightblue'},
        {'x': xs_vertical, 'y': zs_vertical, 'color': 'lightgreen'},
    ]
    line_properties = {
        'Horizontal': {'color': 'blue'},
        'Vertical': {'color': 'green'},
    }
    super(ConstraintWindow, self).__init__(
        mode, name, xlim, ylim,
        'x [m]', 'y-blue/z-green [m]', polygons, line_properties, 50, False,
        num_xticks=6)

  @indicator.ReturnCallbackOutputIfInputInvalid(dict)
  def _Filter(self, *args):
    if self._mode == common.FULL_COMMS_MODE:
      x_g = ctype_util.Vec3ToList(args[0].state_est.Xg)
    elif self._mode == common.SPARSE_COMMS_MODE:
      x_g = list(args[0].pos_g)
    else:
      assert False

    # Convert to the indicator frame.
    x, y, z = numpy.dot(self._dcm_g2indicator, x_g)

    axes = ['x', 'y']
    xy = dict(zip(axes, [x, y]))
    xz = dict(zip(axes, [x, z]))
    return {'pointers': {'Horizontal': xy, 'Vertical': xz}}


class RotorPitchYawWindow(BaseControlSketch2D):
  """The 2D plot showing rotor pitch/yaw moments."""

  def __init__(self, name='Rotor Pitch Yaw'):
    xlim = [-10.0, 10.0]
    ylim = [-10.0, 10.0]
    line_properties = {
        'moment': {'color': 'blue'},
        'int moment': {'color': 'green'},
    }
    super(RotorPitchYawWindow, self).__init__(
        common.FULL_COMMS_MODE, name, xlim, ylim,
        'Yaw moment [kN-m]', 'Pitch moment [kN-m]', [], line_properties,
        num_xticks=5, num_yticks=5, history_len=100, show_legend=False, rows=40)

  @indicator.ReturnCallbackOutputIfInputInvalid(dict)
  def _Filter(self, *args):
    assert self._mode == common.FULL_COMMS_MODE
    moment = ctype_util.Vec3ToList(args[0].thrust_moment.moment)
    int_moment = ctype_util.Vec3ToList(args[0].hover.int_moment)

    return {
        'pointers': {
            'moment': {'x': moment[2]/1e3, 'y': moment[1]/1e3},
            'int moment': {'x': int_moment[2]/1e3, 'y': int_moment[1]/1e3},
        }
    }


class CrosswindCircleWindow(BaseControlSketch2D):
  """The 2D plot showing flight positions in the constraint window."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode, name=None):
    # TODO: The path radius should reflect the current path radius from
    # the telemetry.
    params = _CONTROL_PARAMS.crosswind.playbook.entries[0]
    half_width = 1.6 * params.path_radius_target
    xlim = [-half_width, half_width]
    ylim = [-half_width, half_width]
    line_properties = {
        'Current': {'color': 'blue'},
        'Target': {'color': 'green'},
    }
    self._default_radius = params.path_radius_target
    super(CrosswindCircleWindow, self).__init__(
        mode, name, xlim, ylim,
        '-y_cw [m]', '-z_cw [m]', [], line_properties,
        history_len=100, show_legend=False, rows=40,
        num_xticks=7, num_yticks=7)

  @indicator.ReturnCallbackOutputIfInputInvalid(dict)
  def _Filter(self, *args):
    if self._mode == common.FULL_COMMS_MODE:
      current_pos = args[0].crosswind.current_pos_cw
      target_pos = args[0].crosswind.target_pos_cw
      current_point = {'x': -current_pos.x, 'y': -current_pos.y}
      target_point = {'x': -target_pos.x, 'y': -target_pos.y}
      target_radius = args[0].crosswind.path_radius_target
    elif self._mode == common.SPARSE_COMMS_MODE:
      current_pos = args[0].current_pos_cw
      target_pos = args[0].target_pos_cw
      current_point = {'x': -current_pos[0], 'y': -current_pos[1]}
      target_point = {'x': -target_pos[0], 'y': -target_pos[1]}
      target_radius = self._default_radius
    else:
      assert False

    background = {
        'expected': self._PolygonForCircle(target_radius, 0.0, 0.0, 84, 'gray')
    }
    return {
        'pointers': {'Current': current_point, 'Target': target_point},
        'segments': background
    }


class TransOutThreshWindow(BaseControlSketch2D):
  """The 2D plot showing flight positions in the constraint window."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode, name=None):
    params = _CONTROL_PARAMS.crosswind.mode
    self._rotation = numpy.pi
    thresh_x = []
    thresh_y = []
    loop_angle = 0.0
    num_samples = 84
    loop_angle_inc = 2.0 * numpy.pi / num_samples
    while loop_angle < 2.0 * numpy.pi + loop_angle_inc:
      vg_norm_thresh = c_math_util.Interp1(
          params.loop_angle_table, params.max_trans_out_speed_table,
          len(params.loop_angle_table), loop_angle,
          c_math_util.kInterpOptionSaturate)
      x, y = numpy_utils.PolarToCartesian(vg_norm_thresh,
                                          loop_angle + self._rotation)
      thresh_x.append(x)
      thresh_y.append(y)
      loop_angle += loop_angle_inc
    half_width = 40
    # TODO: Add text annotations for radius.
    polygons = [
        self._PolygonForCircle(half_width, 0, 0, num_samples, 'gray'),
        {'x': thresh_x, 'y': thresh_y, 'color': 'green'},
    ]
    xlim = [-half_width, half_width]
    ylim = [-half_width, half_width]
    line_properties = {
        'state': {'color': 'blue'},
    }
    super(TransOutThreshWindow, self).__init__(
        mode, name, xlim, ylim,
        '|Vg| [m/s]', '|Vg| [m/s]', polygons, line_properties,
        history_len=100, show_legend=False, rows=36)

  @indicator.ReturnCallbackOutputIfInputInvalid(dict)
  def _Filter(self, *args):
    if self._mode == common.FULL_COMMS_MODE:
      control_telemetry = args[0]
      loop_angle = control_telemetry.crosswind.loop_angle
      vg = numpy.linalg.norm(control_telemetry.state_est.Vg.values())
    elif self._mode == common.SPARSE_COMMS_MODE:
      control_telemetry = args[0]
      loop_angle = control_telemetry.loop_angle
      vg = numpy.linalg.norm(control_telemetry.vel_g[:])
    else:
      assert False

    x, y = numpy_utils.PolarToCartesian(vg, loop_angle + self._rotation)
    return {'pointers': {'state': {'x': x, 'y': y}}}


class SingleErrorIndicator(indicator.BaseAttributeIndicator):
  """Single-attribute text box showing error between data and cmd."""

  def __init__(self, name, data_msg, cmd_msg, mode_msg, stoplight_limits,
               unit_text, scale_factor=1.0, wrap=None, **base_kwargs):
    sources = [data_msg, cmd_msg, mode_msg]
    self._stoplight_limits = stoplight_limits
    self._unit_text = unit_text
    self._scale_factor = scale_factor
    self._wrap = wrap
    super(SingleErrorIndicator, self).__init__(sources, name, **base_kwargs)

  def _IsValidMode(self, mode):
    """Show stoplight unavailable if this returns False."""
    raise NotImplementedError()

  def _IsValidInput(self, data_attr, cmd_attr, mode):
    """Grey stoplight if invalid mode or not enough data to calculate error."""
    return (self._IsValidMode(mode) and
            data_attr is not None and
            cmd_attr is not None)

  @indicator.ReturnIfInputInvalid(None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, data_attr, cmd_attr, mode):
    error_raw = cmd_attr - data_attr
    if self._wrap is not None:
      error_raw = c_math_util.Wrap(error_raw, self._wrap[0], self._wrap[1])
    # Error limit units must match units of data prior to scaling.
    stoplight = stoplights.SetByLimits(error_raw, self._stoplight_limits)
    text = '{: 6.1f} '.format(self._scale_factor * error_raw) + self._unit_text
    return text, stoplight


class AlphaErrorIndicator(SingleErrorIndicator):
  """Display difference between alpha command and actual."""

  def __init__(self, name='Alpha error'):
    data_msg = ('ControlTelemetry', None, 'state_est.apparent_wind.sph.alpha')
    cmd_msg = ('ControlTelemetry', None, 'crosswind.alpha_cmd')
    mode_msg = ('ControlTelemetry', None, 'flight_mode')
    stoplight_limits = self._GetLimits(_MONITOR_PARAMS.est.alpha_error)
    unit_text = '[deg]'
    scale_factor = numpy.rad2deg(1.0)
    wrap = [-numpy.pi, numpy.pi]
    super(AlphaErrorIndicator, self).__init__(
        name, data_msg, cmd_msg, mode_msg, stoplight_limits, unit_text,
        scale_factor, wrap)

  def _IsValidMode(self, flight_mode):
    if flight_mode is None:
      return False
    return control_common.AnyCrosswindFlightMode(flight_mode)


class BetaErrorIndicator(SingleErrorIndicator):
  """Display difference between beta command and actual."""

  def __init__(self, name='Beta error'):
    data_msg = ('ControlTelemetry', None, 'state_est.apparent_wind.sph.beta')
    cmd_msg = ('ControlTelemetry', None, 'crosswind.beta_cmd')
    mode_msg = ('ControlTelemetry', None, 'flight_mode')
    stoplight_limits = self._GetLimits(_MONITOR_PARAMS.est.beta_error)
    unit_text = '[deg]'
    scale_factor = numpy.rad2deg(1.0)
    wrap = [numpy.deg2rad(-180), numpy.deg2rad(180)]
    super(BetaErrorIndicator, self).__init__(
        name, data_msg, cmd_msg, mode_msg, stoplight_limits, unit_text,
        scale_factor, wrap)

  def _IsValidMode(self, flight_mode):
    if flight_mode is None:
      return False
    return control_common.AnyCrosswindFlightMode(flight_mode)


class AirspeedErrorIndicator(SingleErrorIndicator):
  """Display difference between airspeed command and actual."""

  def __init__(self, name='Airspeed err'):
    data_msg = ('ControlTelemetry', None, 'state_est.apparent_wind.sph.v')
    cmd_msg = ('ControlTelemetry', None, 'crosswind.airspeed_cmd')
    mode_msg = ('ControlTelemetry', None, 'flight_mode')
    stoplight_limits = self._GetLimits(_MONITOR_PARAMS.est.airspeed_error)
    unit_text = '[m/s]'
    super(AirspeedErrorIndicator, self).__init__(
        name, data_msg, cmd_msg, mode_msg, stoplight_limits, unit_text)

  def _IsValidMode(self, flight_mode):
    if flight_mode is None:
      return False
    return control_common.AnyCrosswindFlightMode(flight_mode)


class AeroAnglesChart(BaseControllerRunningListChart):
  """Chart showing wind angles according to pitot."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode, **widget_kwargs):
    super(AeroAnglesChart, self).__init__(
        mode, ['Alpha', 'Beta', 'Alpha Cmd', 'Beta Cmd'], 'Aero angles [deg]',
        precision=1, **widget_kwargs)

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    if self._mode == common.FULL_COMMS_MODE:
      sph = args[0].state_est.apparent_wind.sph
      alpha = sph.alpha
      beta = sph.beta
      flight_mode = args[0].flight_mode
    elif self._mode == common.SPARSE_COMMS_MODE:
      alpha = args[0].alpha
      beta = args[0].beta
      flight_mode = args[0].flight_mode
    else:
      assert False

    if (control_common.AnyCrosswindFlightMode(flight_mode) or
        flight_mode == control_common.kFlightModeOffTether):
      alpha_ranges = self._GetLimits(_MONITOR_PARAMS.est.alpha)
      beta_ranges = self._GetLimits(_MONITOR_PARAMS.est.beta)
      stoplight = stoplights.MostSevereStoplight(
          stoplights.SetByRanges(alpha, alpha_ranges['normal'],
                                 alpha_ranges['warning']),
          stoplights.SetByRanges(beta, beta_ranges['normal'],
                                 beta_ranges['warning']))
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    alpha_deg = numpy.rad2deg(alpha)
    beta_deg = numpy.rad2deg(beta)

    control_telemetry = self._GetControlTelemetry(*args)
    if (control_common.AnyCrosswindFlightMode(flight_mode) and
        control_telemetry):
      alpha_cmd_deg = numpy.rad2deg(control_telemetry.crosswind.alpha_cmd)
      beta_cmd_deg = numpy.rad2deg(control_telemetry.crosswind.beta_cmd)
    else:
      alpha_cmd_deg = beta_cmd_deg = None

    return self._GetTimestamps(*args), [
        alpha_deg, beta_deg, alpha_cmd_deg, beta_cmd_deg], stoplight


class CrosswindDeltasChart(BaseControllerRunningListChart):
  """Chart showing crosswind deltas."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode, **widget_kwargs):
    super(CrosswindDeltasChart, self).__init__(
        mode, ['Ail', 'Flap', 'Ele', 'Rud'],
        'Deltas [deg]', precision=2, **widget_kwargs)

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    if self._mode == common.FULL_COMMS_MODE:
      control_telemetry = args[0]
      deltas = control_telemetry.deltas
      flight_mode = control_telemetry.flight_mode
      degs = numpy.rad2deg([deltas.aileron, deltas.inboard_flap,
                            deltas.elevator, deltas.rudder]).tolist()
    elif self._mode == common.SPARSE_COMMS_MODE:
      control_telemetry = args[0]
      flight_mode = control_telemetry.flight_mode
      degs = numpy.rad2deg([control_telemetry.delta_aileron,
                            control_telemetry.delta_inboard_flap,
                            control_telemetry.delta_elevator,
                            control_telemetry.delta_rudder]).tolist()
    else:
      assert False

    stoplight = (stoplights.STOPLIGHT_NORMAL
                 if control_common.AnyCrosswindFlightMode(flight_mode)
                 else stoplights.STOPLIGHT_ANY)

    return self._GetTimestamps(*args), degs, stoplight


class BodyRatesChart(BaseControllerRunningListChart):
  """Chart showing body's roll/pitch/yaw rate."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode, angles, **widget_kwargs):
    self._angles = angles
    super(BodyRatesChart, self).__init__(
        mode, angles, 'Body rates [deg/s]', precision=2,
        **widget_kwargs)

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    values = []
    if self._mode == common.FULL_COMMS_MODE:
      control_telemetry = args[0]
      for angle in self._angles:
        if angle == 'Roll':
          values.append(control_telemetry.state_est.pqr_f.x)
        elif angle == 'Pitch':
          values.append(control_telemetry.state_est.pqr_f.y)
        elif angle == 'Yaw':
          values.append(control_telemetry.state_est.pqr_f.z)
        else:
          assert False

      flight_mode = control_telemetry.flight_mode
    elif self._mode == common.SPARSE_COMMS_MODE:
      control_telemetry = args[0]
      for angle in self._angles:
        if angle == 'Roll':
          values.append(control_telemetry.pqr[0])
        elif angle == 'Pitch':
          values.append(control_telemetry.pqr[1])
        elif angle == 'Yaw':
          values.append(control_telemetry.pqr[2])
        else:
          assert False

      flight_mode = control_telemetry.flight_mode
    else:
      assert False

    stoplight = (stoplights.STOPLIGHT_NORMAL
                 if control_common.AnyCrosswindFlightMode(flight_mode)
                 else stoplights.STOPLIGHT_ANY)

    return self._GetTimestamps(*args), [
        numpy.rad2deg(v) for v in values], stoplight


class LowBoundLoopAltitudeIndicator(BaseControllerRunningIndicator):
  """Indicator for the lower altitude in the last crosswind loop."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(LowBoundLoopAltitudeIndicator, self).__init__(
        mode, 'Min. Alt.')
    self._lowest_point_in_downstroke = float('inf')
    self._last_loop_lowest_point = float('inf')

  def _Filter(self, telemetry, *args):
    if telemetry is None:
      return self._lowest_point_in_downstroke, stoplights.STOPLIGHT_ANY
    elif self._mode == common.FULL_COMMS_MODE:
      z_g = telemetry.state_est.Xg.z
      loop_angle = telemetry.crosswind.loop_angle
    elif self._mode == common.SPARSE_COMMS_MODE:
      z_g = telemetry.pos_g[2]
      loop_angle = telemetry.loop_angle
    else:
      assert False

    altitude = _SYSTEM_PARAMS.ground_frame.ground_z - z_g

    if numpy.pi < loop_angle < numpy.pi * 1.5:
      # Reset the lowest point tracker as the downstroke starts.
      # This code can be sharpened to make sure the tracked altitude
      # increases only after the kite hits the lowest point.
      self._lowest_point_in_downstroke = altitude
    else:
      if self._lowest_point_in_downstroke >= altitude:
        self._lowest_point_in_downstroke = altitude
      else:
        self._last_loop_lowest_point = self._lowest_point_in_downstroke

    if self._last_loop_lowest_point is None:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE
    else:
      return ('%.1f m' % self._last_loop_lowest_point,
              stoplights.STOPLIGHT_NORMAL)


class AltitudeChart(BaseControllerRunningListChart):
  """Displays altitude."""

  _GROUND_Z = _SYSTEM_PARAMS.ground_frame.ground_z

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode, **widget_kwargs):
    super(AltitudeChart, self).__init__(
        mode, ['[m]'], 'Altitude',
        precision=1, **widget_kwargs)

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    if self._mode == common.FULL_COMMS_MODE:
      z_g = args[0].state_est.Xg.z
    elif self._mode == common.SPARSE_COMMS_MODE:
      z_g = args[0].pos_g[2]
    else:
      assert False
    flight_mode = args[0].flight_mode

    altitude = self._GROUND_Z - z_g
    if control_common.AnyCrosswindFlightMode(flight_mode):
      stoplight = stoplights.SetByStoplightLimits(
          altitude, _MONITOR_PARAMS.est.crosswind_altitude_agl)
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    return [self._GetTimestamps(*args)], [altitude], stoplight


class TransInTrajectoryChart(BaseControlSketch2D):
  """Chart showing glideslope and its incidence with the landing zone."""

  _GROUND_Z = _SYSTEM_PARAMS.ground_frame.ground_z
  _X_LIMITS = [-600.0, 100.0]
  _ELEVATION_LIMITS = [-5, 695.0]
  _NUM_SPHERE_POINTS = 40

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    tether_sphere = self._PolygonForCircle(
        _SYSTEM_PARAMS.tether.length, 0.0, self._GROUND_Z,
        self._NUM_SPHERE_POINTS, 'lightgray', [0.0, numpy.pi])

    tower = {
        'x': [-2.0, -2.0, 5.0, 5.0],
        'y': [0.0, 15.0, 15.0, 0.0],
        'color': 'gray',
    }

    command_center_half_length = 7.0 / 2  # [m]
    command_center_x = 75.0  # [m]
    command_center = {
        'x': [command_center_x - command_center_half_length,
              command_center_x - command_center_half_length,
              command_center_x + command_center_half_length,
              command_center_x + command_center_half_length],
        'y': [0.0, 5.5, 5.5, 0.0],
        'color': 'gray',
    }

    altitude_bar = {
        'x': self._X_LIMITS,
        'y': [307.0, 307.0],
        'color': 'lightgreen',
    }

    ground_bar = {
        'x': self._X_LIMITS,
        'y': [0.0, 0.0],
        'color': 'orange',
    }

    super(TransInTrajectoryChart, self).__init__(
        mode, name='',
        xlim=self._X_LIMITS, ylim=self._ELEVATION_LIMITS,
        xlabel='x [m]', ylabel='Altitude [m]',
        background_polygons=[
            tether_sphere, altitude_bar, ground_bar, tower, command_center],
        line_properties={
            'X-Z Pos': {'color': 'green'},
        }, num_xticks=5, num_yticks=5, history_len=100, show_legend=False)

  def _GetMessageAttributes(self):
    return _GetControllerMessageAttributes(self._mode) + [
        ('GroundEstimate', None),
        ('GroundEstimateSim', None),
    ]

  def _IsValidInput(self, *attributes):
    if not _IsControllerMessageValid(self._mode, *attributes):
      return False
    if (self._mode == common.SPARSE_COMMS_MODE and
        not any([struct_tree.IsValidElement(attributes[-2]),
                 struct_tree.IsValidElement(attributes[-1])])):
      return False
    return True

  @indicator.ReturnCallbackOutputIfInputInvalid(dict)
  def _Filter(self, *args):
    if self._mode == common.FULL_COMMS_MODE:
      kite_pos = ctype_util.Vec3ToList(args[0].state_est.Xg)
      vessel_pos = ctype_util.Vec3ToList(args[0].state_est.vessel.pos_g)
    elif self._mode == common.SPARSE_COMMS_MODE:
      kite_pos = args[0].pos_g
      ground_estimate = (args[-2] if struct_tree.IsValidElement(args[-2])
                         else args[-1])
      vessel_pos = ctype_util.Vec3ToList(ground_estimate.Xg)
    else:
      assert False

    x, y, z = numpy.array(kite_pos) - numpy.array(vessel_pos)
    # The axis angle relative to the -X direction of the ground frame.
    h_axis_angle = numpy.arctan2(y, x) + numpy.pi
    horizontal_m = (x * math.cos(h_axis_angle) +
                    y * math.sin(h_axis_angle))
    range_m = c_math_util.Saturate(horizontal_m, *self._X_LIMITS)
    height_m = c_math_util.Saturate(self._GROUND_Z - z, *self._ELEVATION_LIMITS)
    tether_sphere = self._PolygonForCircle(
        numpy.linalg.norm([x, y, z]),
        0.0, self._GROUND_Z, self._NUM_SPHERE_POINTS, 'gray', [0.0, numpy.pi])
    return {
        'pointers': {'X-Z Pos': {'x': range_m, 'y': height_m}},
        'segments': {'X-Z Tether': tether_sphere},
    }


class ImpactZoneChart(BaseControlSketch2D):
  """Displays top-down position of the wing and the landing zone."""

  _GROUND_Z = _SYSTEM_PARAMS.ground_frame.ground_z

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    params = _MONITOR_PARAMS.landing_zone
    tophat = self._PolygonForCircle(5, 0.0, 0.0, 10, 'gray')
    tether_sphere = self._PolygonForCircle(
        _SYSTEM_PARAMS.tether.length, 0.0, 0.0, 30, 'lightgray')
    # Minimum and maximum crosswind circle azimuth angles [rad].
    azi_allow_start = _SYSTEM_PARAMS.test_site_params.azi_allow_start
    azi_allow_end = _SYSTEM_PARAMS.test_site_params.azi_allow_end
    azi_vector_length = _SYSTEM_PARAMS.tether.length * 1.2
    azi_start_vector = {
        'x': [0.0, math.sin(azi_allow_start) * azi_vector_length],
        'y': [0.0, math.cos(azi_allow_start) * azi_vector_length],
        'color': 'lightgray'
    }
    azi_end_vector = {
        'x': [0.0, math.sin(azi_allow_end) * azi_vector_length],
        'y': [0.0, math.cos(azi_allow_end) * azi_vector_length],
        'color': 'lightgray'
    }
    # Perimeter of cleared pad where ground station sits (no-land zone).
    pr_pad_vertices = (list(params.pr_pad_vertices))
    pr_pad = {
        'x': [v.y for v in pr_pad_vertices],
        'y': [v.x for v in pr_pad_vertices],
        'color': 'gray'
    }
    # Command center blob (no-land zone).
    command_center = self._PolygonForCircle(100.0, -230.0, 400.0, 100, 'gray')

    super(ImpactZoneChart, self).__init__(
        mode, name='',
        xlim=[-600.0, 600.0], ylim=[-600, 600.0],
        xlabel='y [m]', ylabel='x [m]',
        # TODO: Choose polygons based on test site.
        background_polygons=[pr_pad, command_center, tophat, tether_sphere,
                             azi_start_vector, azi_end_vector],
        line_properties={'Position': {'color': 'green'},
                         'Azimuth': {'color': 'blue'}},
        history_len=50, show_legend=False, invert_xaxis=False,
        invert_yaxis=False, num_xticks=5, num_yticks=5)

  @indicator.ReturnCallbackOutputIfInputInvalid(dict)
  def _Filter(self, *args):

    if self._mode == common.FULL_COMMS_MODE:
      x, y, z = ctype_util.Vec3ToList(args[0].state_est.Xg)
      vx, vy, vz = ctype_util.Vec3ToList(args[0].state_est.Vg)
    elif self._mode == common.SPARSE_COMMS_MODE:
      x, y, z = args[0].pos_g
      vx, vy, vz = args[0].vel_g
    else:
      assert False

    # If in crosswind, plot crosswind azimuth.
    if control_common.AnyCrosswindFlightMode(args[0].flight_mode) and args[-1]:
      crosswind_azi_angle = args[-1].command_message.gs_azi_target
      crosswind_azi_vector_length = _SYSTEM_PARAMS.tether.length * 1.1
    else:
      crosswind_azi_angle = 0.0
      crosswind_azi_vector_length = 0.0

    # vz > 0.0 when the kite is descending.
    ground_hit_time = max(
        0.0, -(z - self._GROUND_Z) / vz if vz > 0.0 else float('inf'))
    elapse = [0.0, min(1.0, ground_hit_time)]
    landing_area = {
        'x': [y + vy * s for s in elapse],
        'y': [x + vx * s for s in elapse],
        'color': 'orange',
    }

    h = -(z - self._GROUND_Z + vz * elapse[1])
    g = 9.81
    if h <= 0.0:
      t_impact = 0.0
    else:
      #  A negative vz actually means going up.
      t_impact = -vz / g + math.sqrt((vz / g) ** 2 + 2 * h / g)

    landing_area['x'][1] += t_impact * vy
    landing_area['y'][1] += t_impact * vx
    landing_area['x'] = [0.0] + landing_area['x'] + [0.0]
    landing_area['y'] = [0.0] + landing_area['y'] + [0.0]

    # Vector for current crosswind azimuth.
    azimuth_pointer = {
        'x': math.sin(crosswind_azi_angle) * crosswind_azi_vector_length,
        'y': math.cos(crosswind_azi_angle) * crosswind_azi_vector_length,
    }
    azimuth_vector = {
        'x': [0.0, azimuth_pointer['x']],
        'y': [0.0, azimuth_pointer['y']],
        'color': 'blue'
    }

    return {
        'pointers': {
            'Position': {'x': y, 'y': x},
            'Azimuth': azimuth_pointer,
        },
        'segments': {
            'landing': landing_area,
            'azi-vec': azimuth_vector,
        },
    }


class GlideslopeChart(BaseControlSketch2D):
  """Chart showing glideslope and its incidence with the landing zone."""

  _X_LIMITS = [-2000.0, 500.0]
  _ELEVATION_LIMITS = [-50.0, 500.0]
  _GS_RANGE = -2000.0
  _H_AXIS_ANGLE = numpy.deg2rad(40)  # Horizontal axis is offset from Xg.x axis.
  _GROUND_Z = _SYSTEM_PARAMS.ground_frame.ground_z

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    params = _MONITOR_PARAMS.landing_zone
    slope = numpy.tan(params.glideslope)
    threshold = {
        'x': [params.threshold, self._GS_RANGE],
        'y': [0.0, (self._GS_RANGE - params.threshold) * slope],
        'color': 'gray',
    }
    overrun = {
        'x': [params.overrun, self._GS_RANGE],
        'y': [0.0, (self._GS_RANGE - params.overrun) * slope],
        'color': 'gray',
    }

    super(GlideslopeChart, self).__init__(
        mode, name='Glideslope',
        xlim=self._X_LIMITS, ylim=self._ELEVATION_LIMITS,
        xlabel='x [m]', ylabel='Altitude [m]',
        background_polygons=[threshold, overrun],
        line_properties={'Position': {'color': 'blue'}},
        num_xticks=5, num_yticks=5, history_len=100, show_legend=False)

  @indicator.ReturnCallbackOutputIfInputInvalid(dict)
  def _Filter(self, *args):
    if self._mode == common.FULL_COMMS_MODE:
      x, y, z = ctype_util.Vec3ToList(args[0].state_est.Xg)
    elif self._mode == common.SPARSE_COMMS_MODE:
      x, y, z = args[0].pos_g
    else:
      assert False

    # Dot product gives projection of [x,y] onto 40 degree line.
    horizontal_m = (x * math.cos(self._H_AXIS_ANGLE) +
                    y * math.sin(self._H_AXIS_ANGLE))
    range_m = c_math_util.Saturate(horizontal_m, *self._X_LIMITS)
    elevation_m = c_math_util.Saturate(self._GROUND_Z - z,
                                       *self._ELEVATION_LIMITS)
    return {'pointers': {'Position': {'x': range_m, 'y': elevation_m}}}


class AeroAnglesXYPlot(BaseControlSketch2D):
  """Chart showing aero angles."""

  _ALPHA_DEG_RANGE = [-20.0, 20.0]
  _BETA_DEG_RANGE = [-20.0, 20.0]

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(AeroAnglesXYPlot, self).__init__(
        mode, name='Aero angles',
        xlim=self._BETA_DEG_RANGE, ylim=self._ALPHA_DEG_RANGE,
        xlabel='beta [deg]', ylabel='alpha [deg]',
        line_properties={'Angles': {'color': 'blue'}},
        history_len=50, show_legend=False)

  @indicator.ReturnCallbackOutputIfInputInvalid(dict)
  def _Filter(self, *args):

    if self._mode == common.FULL_COMMS_MODE:
      wind_sph = args[0].state_est.apparent_wind.sph_f
      alpha = wind_sph.alpha
      beta = wind_sph.beta
    elif self._mode == common.SPARSE_COMMS_MODE:
      alpha = args[0].alpha
      beta = args[0].beta
    else:
      assert False

    alpha_deg = c_math_util.Saturate(numpy.rad2deg(alpha),
                                     *self._ALPHA_DEG_RANGE)
    beta_deg = c_math_util.Saturate(numpy.rad2deg(beta), *self._BETA_DEG_RANGE)
    return {'pointers': {'Angles': {'x': beta_deg, 'y': alpha_deg}}}


class ManualStateIndicator(BaseControllerRunningIndicator):
  """Displays state of the manual controller."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(ManualStateIndicator, self).__init__(
        mode, 'Manual state')

  @indicator.ReturnIfInputInvalid('Release:   --\nAutoglide: --',
                                  stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    if self._mode == common.FULL_COMMS_MODE:
      manual = telemetry.manual
      release_latched = manual.release_latched
      auto_glide_active = manual.auto_glide_active
    elif self._mode == common.SPARSE_COMMS_MODE:
      flags = telemetry.flags
      release_latched = (
          flags & avionics_messages.kTetherControlTelemetryFlagReleaseLatched)
      auto_glide_active = (
          flags & avionics_messages.kTetherControlTelemetryFlagAutoGlideActive)
    else:
      assert False

    text = 'Release:   %s\nAutoglide: %s' % (
        ' True' if release_latched else 'False',
        'Active' if auto_glide_active else 'Inactive')
    return text, stoplights.STOPLIGHT_NORMAL


class ApparentWindSpeedIndicator(BaseControllerRunningIndicator):
  """Displays apparent wind speed via TetherDownMessage."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(ApparentWindSpeedIndicator, self).__init__(
        mode, 'Airspeed', font_size=18)

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    if self._mode == common.FULL_COMMS_MODE:
      airspeed = telemetry.state_est.apparent_wind.sph.v
    elif self._mode == common.SPARSE_COMMS_MODE:
      airspeed = telemetry.airspeed
    else:
      assert False
    return ('Airspeed: % 12.1f m/s' % airspeed, stoplights.STOPLIGHT_NORMAL)


class AltitudeIndicator(BaseControllerRunningIndicator):
  """Displays altitude and descent speed via TetherDownMessage."""

  _GROUND_Z = _SYSTEM_PARAMS.ground_frame.ground_z

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(AltitudeIndicator, self).__init__(mode, 'Altitude', font_size=18)

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    if self._mode == common.FULL_COMMS_MODE:
      pos_z = telemetry.state_est.Xg.z
      vel_z = telemetry.state_est.Vg.z
    elif self._mode == common.SPARSE_COMMS_MODE:
      pos_z = telemetry.pos_g[2]
      vel_z = telemetry.vel_g[2]
    else:
      assert False
    text = '\n'.join([
        'Altitude (AGL):% 7.1f m' % (self._GROUND_Z - pos_z),
        'Descent speed: % 7.1f m/s' % vel_z])
    return text, stoplights.STOPLIGHT_NORMAL


class TopDownPositionChart(BaseControlSketch2D):
  """Displays top-down position of the wing and the landing zone."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    params = _MONITOR_PARAMS.landing_zone
    pr_pad_vertices = (list(params.pr_pad_vertices))
    pr_pad = {
        'x': [v.y for v in pr_pad_vertices],
        'y': [v.x for v in pr_pad_vertices],
        'color': 'orange'
    }
    command_center = self._PolygonForCircle(100.0, -230.0, 400.0, 100, 'orange')

    # Draw B4 range cleared area per China Lake test plan as a
    # three-tether-length radius not-to-exceed boundary
    cleared_area = self._PolygonForCircle(_SYSTEM_PARAMS.tether.length * 3.0,
                                          0.0, 0.0, 200, 'orange')

    super(TopDownPositionChart, self).__init__(
        mode, name='Position (Top down)',
        xlim=[-1250.0, 750.0], ylim=[-1250.0, 750.0],
        xlabel='y [m]', ylabel='x [m]',
        # TODO: Choose polygons based on test site.
        background_polygons=[pr_pad, command_center, cleared_area],
        line_properties={'Position': {'color': 'blue'}},
        history_len=50, show_legend=False, invert_xaxis=False,
        invert_yaxis=False, num_xticks=5, num_yticks=5)

  def _GetMessageAttributes(self):
    attributes = _GetControllerMessageAttributes(self._mode)
    attributes += [
        ('GroundEstimate', None),
        ('GroundStationWeather', None),
        ('GroundStationStatus', None, 'status'),
    ]
    return attributes

  @indicator.ReturnCallbackOutputIfInputInvalid(dict)
  def _Filter(self, *args):

    if self._mode == common.FULL_COMMS_MODE:
      x, y, _ = ctype_util.Vec3ToList(args[0].state_est.Xg)
      control_telemetry = args[0]
    elif self._mode == common.SPARSE_COMMS_MODE:
      x, y, _ = args[0].pos_g
      control_telemetry = args[-4]
    else:
      assert False

    ground_estimate = args[-3]
    weather = args[-2]
    gs_status = args[-1]
    est_valid = control_telemetry and control_telemetry.state_est.wind_g.valid
    gs_valid = (struct_tree.IsValidElement(weather) and
                struct_tree.IsValidElement(ground_estimate) and
                (not _SYSTEM_PARAMS.wind_sensor.on_perch or
                 struct_tree.IsValidElement(gs_status)))

    dir_rad = None

    if est_valid:
      # Draw wind velocity vector (pointing to where wind comes from).
      dir_rad = common.WindAziToWindG(
          control_telemetry.state_est.wind_g.dir_f)
      speed = control_telemetry.state_est.wind_g.speed_f
    elif gs_valid:
      velocity = weather.wind.wind_velocity
      wind_g = avionics.WindWsToWindG(velocity, pqr=ground_estimate.pqr,
                                      dcm_g2p=ground_estimate.dcm_g2p)
      # Draw wind velocity vector (pointing to where wind comes from).
      dir_rad = numpy.arctan2(wind_g.y, wind_g.x)
      speed = numpy.linalg.norm(velocity[:])

    if dir_rad is not None:
      wind_vec_len = 100.0 * speed
      wind_vec = {
          'x': [0, numpy.sin(dir_rad) * wind_vec_len],
          'y': [0, numpy.cos(dir_rad) * wind_vec_len],
          'color': 'red',
      }
    else:
      wind_vec = {
          'y': [0.],
          'x': [0.],
          'color': 'red',
      }

    return {
        'pointers': {'Position': {'x': y, 'y': x}},
        'segments': {'wind': wind_vec},
    }


class ThrottleIndicator(indicator.BaseIndicator):
  """Base class with utilities shared by motor indicators."""

  def __init__(self, from_joystick):
    super(ThrottleIndicator, self).__init__('')
    if from_joystick:
      self._throttle_field = 'JoystickStatus.JoystickA.throttle'
    else:
      self._throttle_field = (
          'ControlTelemetry.ControllerA.control_input.joystick.throttle')

  def Plot(self, position, annotation):
    """Plot the widget for this indicator."""
    return widgets.TapeIndicator(
        self._label, 'green', position, annotation,
        aspect_ratio=1.2,
        right_bars=[
            {'position': 0.8, 'textAbove': 'Ascend/Rtn XWind'},
            {'position': 0.5, 'textBelow': 'Trans out'},
            {'position': 0.4, 'textBelow': 'Reel-in/Descend'},
            {'position': 0.08, 'textBelow': 'E-STOP'},
        ],
        y_ticks=[
            # Label 0.98 as "1.0" so the text does not overflow.
            {'position': 0.98, 'label': '1.0'},
            {'position': 0.8, 'label': '0.8'},
            {'position': 0.5, 'label': '0.5'},
            {'position': 0.4, 'label': '0.4'},
            {'position': 0.08, 'label': '0.08'},
            {'position': 0.04, 'label': 'E-STOP'},
        ])

  def Filter(self, messages):
    throttle = messages[self._throttle_field]
    if throttle is None:
      return 0.0, 'N/A'
    else:
      return throttle, '% 3.2f' % throttle


class FilterFrequencyIndicator(indicator.BaseIndicator):
  """Checks that the filters are running at a high enough frequency."""

  def __init__(self):
    self._warning_threshold = 20
    super(FilterFrequencyIndicator, self).__init__(name='Filter Frequency')

  def Filter(self, messages):
    filter_frequency = messages.filtered.filter_frequency
    if not filter_frequency or not filter_frequency.valid:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE
    frequency = filter_frequency.frequency
    return 'freq: %2.0f Hz' % frequency, (
        stoplights.STOPLIGHT_WARNING if frequency < self._warning_threshold
        else stoplights.STOPLIGHT_NORMAL)


class BaseServoIndicator(indicator.MultiModeIndicator):
  """Base class for servo indicators."""

  def __init__(self, mode, name=None):
    super(BaseServoIndicator, self).__init__(mode, name)

  def _GetMessageAttributes(self):
    if self._mode == common.SPARSE_COMMS_MODE:
      return [
          ('filtered', 'merge_tether_down', 'tether_down.servo_statuses'),
          ('filtered', 'merge_tether_down', 'valid'),
      ]
    elif self._mode == common.FULL_COMMS_MODE:
      return [('ControlTelemetry', None),]
    else:
      assert False

  def _IsValidInput(self, *attributes):
    if self._mode == common.SPARSE_COMMS_MODE:
      return attributes[1]
    elif self._mode == common.FULL_COMMS_MODE:
      return attributes[0]
    else:
      assert False


class FlapsIndicator(BaseServoIndicator):
  """Displays flap deflections in degrees."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(FlapsIndicator, self).__init__(mode, 'Flaps [deg]')

    self._servo_params = c_helpers.CopyToEquivalentType(
        _SYSTEM_PARAMS.servos,
        actuator_util.ServoParams * len(_SYSTEM_PARAMS.servos))

  def _Filter(self, *args):
    flaps_deg = ['--' for _ in range(system_labels.kNumFlaps)]

    if not self._IsValidInput(*args):
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

      # NOTE: For better correspondence with the sparse comms case, we
      # should arguably be reading angles directly from ServoStatusMessages.
      # But given that we plan to use sparse comms mode during real flight, it
      # isn't clear that that's worth the effort.
      if self._mode == common.FULL_COMMS_MODE:
        for i, flap_rad in enumerate(args[0].control_input.flaps):
          flaps_deg[i] = '% 3.0f' % numpy.rad2deg(flap_rad)

      elif self._mode == common.SPARSE_COMMS_MODE:
        # Convert servo angles to flap angles.
        servo_angles = (ctypes.c_double * len(_SERVO_LABEL_HELPER))()
        for i, servo_state in enumerate(args[0]):
          servo_angles[i] = servo_state.angle
        flaps_rad = (ctypes.c_double * system_labels.kNumFlaps)()
        actuator_util.ServoAnglesToFlapAngles(
            ctype_util.SizelessArray(servo_angles),
            ctype_util.SizelessArray(self._servo_params),
            ctype_util.SizelessArray(flaps_rad))

        # Determine validity of each flap angle estimate by the validity of
        # the underlying servo angle estimate(s).
        flaps_valid = [True for _ in range(system_labels.kNumFlaps)]
        for i in _SERVO_LABEL_HELPER.Values():
          flap_label = labels_util.ServoToFlap(i)
          flaps_valid[flap_label] &= (args[0][i].no_update_count <=
                                      common.MAX_NO_UPDATE_COUNT_SERVO_STATUS)

        for i, flap_rad in enumerate(flaps_rad):
          if flaps_valid[i]:
            flaps_deg[i] = '% 3.0f' % numpy.rad2deg(flap_rad)
      else:
        assert False

    text = '\n'.join([
        'A1:  %s   A8:  %s' % (flaps_deg[system_labels.kFlapA1],
                               flaps_deg[system_labels.kFlapA8]),
        'A2:  %s   A7:  %s' % (flaps_deg[system_labels.kFlapA2],
                               flaps_deg[system_labels.kFlapA7]),
        'A4:  %s   A5:  %s' % (flaps_deg[system_labels.kFlapA4],
                               flaps_deg[system_labels.kFlapA5]),
        ' E:  %s    R:  %s' % (flaps_deg[system_labels.kFlapEle],
                               flaps_deg[system_labels.kFlapRud])])
    return text, stoplight


class TetherReleaseReadinessIndicator(indicator.BaseAttributeIndicator):
  """Indicator to check tether release readiness."""

  def __init__(self, loadcells):
    super(TetherReleaseReadinessIndicator, self).__init__([
        ('ControlSlowTelemetry', None, 'flight_plan'),
    ] + [
        ('Loadcell', loadcell, 'status') for loadcell in loadcells
    ], 'Tether Release Readiness')
    self._loadcells = loadcells

  def _IsValidInput(self, *attributes):
    """Whether the input is valid (required for @ReturnIfInputInvalid)."""
    for attribute in attributes:
      if attribute:
        return True
    return False

  @indicator.ReturnIfInputInvalid('--\n\n\n', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, flight_plan, *loadcell_status):
    # Key is expected to be plugged in for high altitude flight plans.
    # For SN1, the key will be plugged in always.
    expect_keyed = (flight_plan is None or
                    (not system_params.IsLowAltitudeFlightPlan(flight_plan)) or
                    _SYSTEM_PARAMS.wing_serial == system_types.kWingSerial01)

    any_warning_or_error = False
    errors = collections.defaultdict(list)
    report_by_node = collections.defaultdict(list)
    any_node = False
    node_prefix_len = len('Loadcell')

    for idx, node in enumerate(self._loadcells):
      flags = loadcell_status[idx]
      if flags is None:
        continue
      any_node = True

      if expect_keyed:
        # If keys are expected to be plugged in, then check all error types.
        if common.CheckFlags(node, report_by_node, None, errors, flags,
                             None, _LOADCELL_ERROR_HELPER):
          any_warning_or_error = True
      else:
        # Otherwise, make sure the key is not plugged in by ensuring
        # BatteryDisconnected error is reported.
        if not avionics_util.CheckError(
            flags, _LOADCELL_ERROR_HELPER.Value('BatteryDisconnected')):
          any_warning_or_error = True
          errors['keyed'].append(node[node_prefix_len:])

    if expect_keyed:
      return common.SummarizeWarningsAndErrors(
          any_node, report_by_node, {}, errors, any_warning_or_error,
          node_prefix_len=node_prefix_len, num_lines=3)
    else:
      # The output message is 'No Key' if no keys are plugged in.
      # Otherwise, it shows a list of nodes with keys plugged in. E.g,
      # 'Keyed: PortA, PortB'.
      text = ('Keyed: ' + ', '.join(errors['keyed'])
              if errors['keyed'] else 'No Key')
      stoplight = (stoplights.STOPLIGHT_ERROR
                   if any_warning_or_error else stoplights.STOPLIGHT_NORMAL)
      return text, stoplight


class TetherReleaseIndicator(indicator.BaseAttributeIndicator):
  """Indicator to show tether release status."""

  def __init__(self):
    super(TetherReleaseIndicator, self).__init__([
        ('filtered', 'merge_tether_down', 'tether_down.release_statuses'),
        ('filtered', 'merge_tether_down', 'valid'),
        ('filtered', 'merge_tether_down', 'node_status'),
        ('filtered', 'merge_tether_down', 'node_status_valid'),
        ('ControlSlowTelemetry', None, 'flight_plan'),
    ], 'Tether Release')

  def _IsValidInput(self, release_statuses, valid, *unused_node_status_fields):
    """Whether the input is valid (required for @ReturnIfInputInvalid)."""
    limit = common.MAX_NO_UPDATE_COUNT_TETHER_RELEASE
    updates = [release_statuses[i].no_update_count < limit
               for i in _LOADCELL_NODE_LABEL_HELPER.Values()]
    return valid and any(updates)

  @indicator.ReturnIfInputInvalid('--\n\n\n', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, release_statuses, unused_valid, node_status,
              node_status_valid, flight_plan):

    expect_armed = (flight_plan is None or
                    not system_params.IsLowAltitudeFlightPlan(flight_plan))

    final_stoplight = stoplights.STOPLIGHT_NORMAL
    status = {}
    release_nodes = _LOADCELL_NODE_LABEL_HELPER.ShortNames()
    for name in release_nodes:
      loadcell_node_idx = _LOADCELL_NODE_LABEL_HELPER.Value(name)

      stoplight = stoplights.STOPLIGHT_NORMAL
      if (release_statuses[loadcell_node_idx].no_update_count >=
          common.MAX_NO_UPDATE_COUNT_TETHER_RELEASE):
        status[name] = '--'
        # TODO: This should be STOPLIGHT_UNAVAILABLE but that
        # does not interact properly with MostSevereStoplight.
        stoplight = stoplights.STOPLIGHT_WARNING
      elif release_statuses[loadcell_node_idx].released:
        status[name] = 'Released'
        stoplight = stoplights.STOPLIGHT_ERROR
      else:
        state = release_statuses[loadcell_node_idx].state
        armed = common.IsActuatorStateArmed(state)
        error = common.IsActuatorStateError(state)
        interlock = release_statuses[loadcell_node_idx].interlock_switched

        if error:
          status[name] = 'Error'
          stoplight = stoplights.STOPLIGHT_WARNING
        else:
          status[name] = 'Armed' if armed else 'Disarmed'
          status[name] += ', safety off' if interlock else ', safety on'

          if interlock and not armed:
            # Indicate red for "Disarmed, safety off" because we
            # should never sit like that.
            stoplight = stoplights.STOPLIGHT_ERROR
          elif interlock:
            # Standard-operating-procedure indicates that the
            # interlock should never be unsafed except when we're
            # actually firing the release.
            stoplight = stoplights.STOPLIGHT_WARNING
          elif armed != expect_armed:
            # Indicate yellow if the armed status is not as expected.
            stoplight = stoplights.STOPLIGHT_WARNING

        aio_node_idx = _AIO_NODE_HELPER.Value('Loadcell' + name)
        if node_status_valid[aio_node_idx]:
          flags = node_status[aio_node_idx].flags
          if flags & _TETHER_NODE_FLAGS_HELPER.Value('AnyError'):
            status[name] = 'Error'
            stoplight = stoplights.MostSevereStoplight(
                stoplight, stoplights.STOPLIGHT_ERROR)
      final_stoplight = stoplights.MostSevereStoplight(
          final_stoplight, stoplight)

    lines = []
    for node in release_nodes:
      lines.append('%s: %s' % (node, status[node]))

    return '\n'.join(lines), final_stoplight

  def _CheckBitMask(self, field, position):
    return bool(field & (1 << position))


class FdAllActiveIndicator(indicator.SingleAttributeIndicator):

  def __init__(self):
    super(FdAllActiveIndicator, self).__init__(
        ('ControlTelemetry', None), 'Faults')

  def _Filter(self, control_telemetry):
    faults = {}
    for i in range(fd.kNumSubsystems):
      mask = control_telemetry.faults[i]
      ptr_mask = ctypes.POINTER(fd.FaultMask)(mask)
      if (not fd.HasAnyFault(ptr_mask)
          or fd.HasFault(fd.kFaultTypeDisabled, ptr_mask)):
        continue
      fault_types = []
      for j in range(fd.kNumFaultTypes):
        if fd.HasFault(j, ptr_mask):
          fault_types.append(_FAULT_TYPE_HELPER.ShortName(j))
      faults[_SUBSYSTEM_LABEL_HELPER.ShortName(i)] = fault_types

    return common.SummarizeWarningsAndErrors(
        True, faults, common.InvertDictOfLists(faults), {}, bool(faults))


class FdDisabledIndicator(indicator.SingleAttributeIndicator):

  def __init__(self):
    super(FdDisabledIndicator, self).__init__(
        ('ControlTelemetry', None), 'Disabled')

  def _Filter(self, control_telemetry):
    disabled = []
    for i in range(fd.kNumSubsystems):
      mask = control_telemetry.faults[i]
      ptr_mask = ctypes.POINTER(fd.FaultMask)(mask)
      if not fd.HasFault(fd.kFaultTypeDisabled, ptr_mask):
        continue
      disabled.append(_SUBSYSTEM_LABEL_HELPER.ShortName(i))
    if not disabled:
      disabled.append('None')
    return ', '.join(disabled), stoplights.STOPLIGHT_NORMAL


class FlutterWarningIndicator(BaseControllerRunningIndicator):
  """Indicator to warn of impending propeller whirl flutter."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(FlutterWarningIndicator, self).__init__(mode, 'Flutter')

  def _GetMessageAttributes(self):
    attributes = _GetControllerMessageAttributes(self._mode)
    if self._mode == common.SPARSE_COMMS_MODE:
      attributes += [('filtered', 'merge_tether_down', 'tether_down')]
    return attributes

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    if self._mode == common.FULL_COMMS_MODE:
      omegas = telemetry.control_input.rotors
      airspeed = -telemetry.state_est.apparent_wind.vector.x
    elif self._mode == common.SPARSE_COMMS_MODE:
      omegas = [status.speed for status in args[-1].motor_statuses]
      airspeed = (telemetry.airspeed *
                  numpy.cos(telemetry.alpha) * numpy.cos(telemetry.beta))

    max_omega = max([abs(omega) for omega in omegas])

    # Flutter limits for S/N 3.
    stoplight = stoplights.MostSevereStoplight(
        stoplights.SetByStoplightLimits(max_omega,
                                        _MONITOR_PARAMS.rotors.speed),
        stoplights.SetByStoplightLimits(airspeed,
                                        _MONITOR_PARAMS.est.airspeed))

    message = (
        'Warning!     ' if stoplight == stoplights.STOPLIGHT_ERROR else
        'Getting close' if stoplight == stoplights.STOPLIGHT_WARNING else
        'OK           ')

    if control_common.AnyCrosswindFlightMode(telemetry.flight_mode):
      return ('%2.0f m/s, %3.0f rad/s %s' % (airspeed, max_omega, message),
              stoplight)
    else:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE


class HoverDistanceFromPerch(BaseControllerRunningIndicator):

  @indicator.RegisterModes(common.FULL_COMMS_MODE)
  def __init__(self, mode):
    super(HoverDistanceFromPerch, self).__init__(mode, 'Dist from Perch [m]')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    delta = c_math_util.Vec3()
    c_math_util.Vec3Sub(
        ctype_util.CastPointer(telemetry.state_est.Xg,
                               c_math_util.Vec3),
        ctype_util.CastPointer(telemetry.hover.perched_pos_g,
                               c_math_util.Vec3),
        ctypes.pointer(delta))

    message = 'dx: {x: 7.2f}  dy: {y: 7.2f}  dz: {z: 7.2f}'.format(
        x=delta.x, y=delta.y, z=delta.z)

    if telemetry.flight_mode == _FLIGHT_MODE_HELPER.Value('Perched'):
      perch_offset = c_math_util.Vec3Norm(ctypes.pointer(delta))
      stoplight = stoplights.SetByStoplightLimits(
          perch_offset, _MONITOR_PARAMS.control.perch_offset)
    elif control_common.AnyHoverFlightMode(telemetry.flight_mode):
      stoplight = stoplights.STOPLIGHT_NORMAL
    else:
      message = '--'
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE

    if c_math_util.Vec3Norm(
        ctype_util.CastPointer(telemetry.hover.perched_pos_g,
                               c_math_util.Vec3)) == 0:
      message = 'Not yet perched.'
      stoplight = stoplights.MostSevereStoplight(
          stoplight, stoplights.STOPLIGHT_WARNING)

    return message, stoplight


class CrosswindPlaybookIndicator(indicator.DictAttributeIndicator):
  """Displays playbook-related values (like wind-aloft) as text."""

  def __init__(self):
    self._wind_aloft_label = 'Wind aloft [m/s]'
    self._azi_offset_label = 'Azimuth offset [deg]'
    self._elevation_label = 'Loop elevation [deg]'
    self._radius_label = 'Path radius [m]'
    super(CrosswindPlaybookIndicator, self).__init__(
        [('ControlTelemetry', None)],
        [self._wind_aloft_label, self._azi_offset_label,
         self._elevation_label, self._radius_label],
        precision=1, name='Playbook', mode='vertical')

  @indicator.ReturnIfInputInvalid({}, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry):
    display_values = {}
    wind_aloft = telemetry.state_est.wind_aloft_g.speed_f_playbook
    azi_offset = numpy.rad2deg(telemetry.crosswind.azi_offset)
    elevation = numpy.rad2deg(telemetry.crosswind.elevation)
    path_radius = telemetry.crosswind.path_radius_target
    display_values[self._wind_aloft_label] = wind_aloft
    display_values[self._azi_offset_label] = azi_offset
    display_values[self._elevation_label] = elevation
    display_values[self._radius_label] = path_radius
    return (display_values, stoplights.STOPLIGHT_NORMAL)


class AutoControllerIndicator(indicator.DictAttributeIndicator):
  """Displays autonomous controller status as text."""

  def __init__(self):
    self._wind_aloft_label = 'Wind aloft [m/s @ deg]'
    self._wind_ground_label = 'Wind ground [m/s @ deg]'
    self._auto_control_label = 'Auto. control'
    self._fault_detected_label = 'Fault detected'
    self._launch_window_label = 'Launch window'
    self._landing_window_label = 'Landing window'
    self._countdown_label = 'Launch countdown [s]'
    self._desired_mode_label = 'Desired mode'
    super(AutoControllerIndicator, self).__init__(
        [('ControlTelemetry', None)],
        [self._wind_aloft_label, self._wind_ground_label,
         self._auto_control_label, self._fault_detected_label,
         self._launch_window_label, self._landing_window_label,
         self._countdown_label, self._desired_mode_label],
        precision=1, name='Autonomous Controller', mode='vertical')

  @indicator.ReturnIfInputInvalid({}, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry):
    def _FormatWind(wind_est):
      """Formats wind speed and direction, and converts direction TO to FROM."""
      return '%3.1f @ %03.f' % (wind_est.speed_f_playbook,
                                numpy.rad2deg(wind_est.dir_f_playbook) + 180.0)
    display_values = {
        self._wind_aloft_label: _FormatWind(telemetry.state_est.wind_aloft_g),
        self._wind_ground_label: _FormatWind(telemetry.state_est.wind_g),
        self._auto_control_label: (
            'Active' if telemetry.planner.autonomous_flight_enabled else 'Off'),
        self._fault_detected_label: (
            'Yes' if telemetry.planner.landing_fault_detected else 'No'),
        self._launch_window_label: (
            'In' if telemetry.planner.inside_launch_window else 'Out'),
        self._landing_window_label: (
            'In' if telemetry.planner.inside_landing_window else 'Out'),
        self._countdown_label: (
            '%4.1f' % telemetry.planner.takeoff_countdown_timer),
        self._desired_mode_label: _FLIGHT_MODE_HELPER.ShortName(
            telemetry.planner.desired_flight_mode),
    }
    return (display_values, stoplights.STOPLIGHT_NORMAL)


class WingVelocityIndicator(BaseControllerRunningIndicator):

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(WingVelocityIndicator, self).__init__(mode, 'Velocity [m/s]')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):

    if self._mode == common.FULL_COMMS_MODE:
      velocity_g = [telemetry.state_est.Vg.x,
                    telemetry.state_est.Vg.y,
                    telemetry.state_est.Vg.z]
    elif self._mode == common.SPARSE_COMMS_MODE:
      velocity_g = telemetry.vel_g

    message = 'vx: {x: 7.2f}  vy: {y: 7.2f}  vz: {z: 7.2f}'.format(
        x=velocity_g[0], y=velocity_g[1], z=velocity_g[2])
    stoplight = stoplights.STOPLIGHT_NORMAL

    return message, stoplight


class TetherSphereDeviationIndicator(BaseControllerRunningIndicator):
  """Indicator to signal that the kite flies off the tether sphere."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode):
    super(TetherSphereDeviationIndicator, self).__init__(
        mode, 'Tether Sphere Diff')
    self._violating_loops = []
    self._last_violation_time = 0
    self._last_violation_stoplight = stoplights.STOPLIGHT_ERROR

  def _GetMessageAttributes(self):
    args = super(TetherSphereDeviationIndicator, self)._GetMessageAttributes()
    args.append(('filtered', 'loop_count', 'current_rev_count'))
    return args

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    flight_mode = telemetry.flight_mode
    current_rev_count = args[-1]

    if self._mode == common.FULL_COMMS_MODE:
      position_g = ctype_util.Vec3ToList(telemetry.state_est.Xg)
    elif self._mode == common.SPARSE_COMMS_MODE:
      position_g = telemetry.pos_g[:]
    tether_sphere_radius = numpy.linalg.norm(position_g)

    if control_common.AnyCrosswindFlightMode(flight_mode):
      deviation = (
          tether_sphere_radius -
          _MONITOR_PARAMS.tether.tether_sphere.radius_nom)
      stoplight = stoplights.SetByLimits(
          deviation, self._GetLimits(
              _MONITOR_PARAMS.tether.tether_sphere.deviation))
      text = '% 5.2f [m]' % deviation
      elapsed_time = (self._GetTimestamp(telemetry, *args) -
                      self._last_violation_time)
      if stoplight in [stoplights.STOPLIGHT_WARNING,
                       stoplights.STOPLIGHT_ERROR]:
        if (elapsed_time <=
            _MONITOR_PARAMS.tether.tether_sphere.latch_duration_sec):
          stoplight = stoplights.MostSevereStoplight(
              stoplight, self._last_violation_stoplight)
        self._last_violation_stoplight = stoplight
        self._last_violation_time = self._GetTimestamp(telemetry, *args)
        if current_rev_count not in self._violating_loops:
          self._violating_loops.append(current_rev_count)
        while (len(self._violating_loops) >
               _MONITOR_PARAMS.tether.tether_sphere.history_len):
          self._violating_loops.pop(0)
      else:
        if (elapsed_time <=
            _MONITOR_PARAMS.tether.tether_sphere.latch_duration_sec):
          stoplight = self._last_violation_stoplight
    else:
      stoplight = stoplights.STOPLIGHT_ANY
      text = '--'

    history_text = (
        'Bad loops: %s' % self._violating_loops
        if self._violating_loops else '')
    text += '\n' + history_text
    return text, stoplight


class TensionPilotOffsetIndicator(indicator.SingleAttributeIndicator):
  """Show the offset to horizontal tension command added by the pilot."""

  def __init__(self):
    super(TensionPilotOffsetIndicator, self).__init__(
        ('ControlTelemetry', None), 'Pilot Tension')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry):
    value = telemetry.hover.horizontal_tension_pilot_offset
    # Formatted to line up nicely with BridleJunctionLoadcellIndicator.
    text = 'Offset:     % 7.3f kN' % (value / 1e3)
    if not control_common.AnyHoverFlightMode(telemetry.flight_mode):
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif abs(value) > 1e-3:
      # Feature is active with a nonzero offset.
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL
    return text, stoplight


class VesselPositionIndicator(BaseControllerRunningIndicator):

  @indicator.RegisterModes(common.FULL_COMMS_MODE)
  def __init__(self, mode):
    super(VesselPositionIndicator, self).__init__(mode, 'Position [m]')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):

    vessel = telemetry.state_est.vessel
    pos_g = [vessel.pos_g.x, vessel.pos_g.y, vessel.pos_g.z]

    if vessel.position_valid:
      message = 'x: {x: 7.2f}  y: {y: 7.2f}  z: {z: 7.2f}'.format(
          x=pos_g[0], y=pos_g[1], z=pos_g[2])

      stoplight = stoplights.STOPLIGHT_NORMAL

    else:
      message = 'invalid'
      stoplight = stoplights.STOPLIGHT_WARNING

    return message, stoplight


class VesselAttitudeIndicator(BaseControllerRunningIndicator):

  @indicator.RegisterModes(common.FULL_COMMS_MODE)
  def __init__(self, mode):
    super(VesselAttitudeIndicator, self).__init__(mode, 'Attitude [deg]')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):

    # TODO: Would it make more sense to report the vessel or
    # the platform attitude? Currently reporting vessel attitude for
    # consistency with the naming.
    vessel = telemetry.state_est.vessel
    dcm_g2v = vessel.dcm_g2v.d

    if vessel.attitude_valid:
      yaw, pitch, roll = geometry.DcmToAngle(numpy.matrix(dcm_g2v))

      message = 'r: {roll: 7.1f}  p: {pitch: 7.1f}  y: {yaw: 7.1f}'.format(
          roll=numpy.rad2deg(roll),
          pitch=numpy.rad2deg(pitch),
          yaw=numpy.rad2deg(yaw))

      stoplight = stoplights.STOPLIGHT_NORMAL

    else:
      message = 'invalid'
      stoplight = stoplights.STOPLIGHT_WARNING

    return message, stoplight
