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

"""Monitor Indicators related to estimators."""
import ctypes

from makani.common.c_math import util as c_math_util
from makani.control import common as control_common
from makani.gs.monitor import monitor_params
from makani.gs.monitor2.apps.layout import indicator
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import control
from makani.lib.python import ctype_util
from makani.system import labels
import numpy

_MONITOR_PARAMS = monitor_params.GetMonitorParams().contents


# TODO: Move the function into a library.
def _Vec3Distance(p0, p1):
  diff = c_math_util.Vec3()
  return c_math_util.Vec3Norm(c_math_util.Vec3Sub(
      ctype_util.CastPointer(p0, c_math_util.Vec3),
      ctype_util.CastPointer(p1, c_math_util.Vec3),
      ctypes.pointer(diff)))


class EstimatorGyroDiffIndicator(control.BaseControllerRunningIndicator):
  """Indicator for Gyro Diff Estimator."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE)
  def __init__(self, mode):
    super(EstimatorGyroDiffIndicator, self).__init__(mode, 'PQR Diff [rad/s]')

  @indicator.ReturnIfInputInvalid('A-B:  --   B-C:  --   C-A:  -- ',
                                  stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry):

    pqr_diff = []
    imus = telemetry.control_input.imus
    gyro_biases = telemetry.estimator.gyro_biases
    pqr_vec = ctype_util.CastPointer(telemetry.state_est.pqr, c_math_util.Vec3)
    for i in range(labels.kNumWingImus):
      diff = ctypes.pointer(c_math_util.Vec3())
      imus_vec = ctype_util.CastPointer(imus[i].gyro, c_math_util.Vec3)
      gyro_biases_vec = ctype_util.CastPointer(gyro_biases[i], c_math_util.Vec3)
      c_math_util.Vec3Sub(imus_vec, gyro_biases_vec, diff)
      c_math_util.Vec3Sub(pqr_vec, diff, diff)
      pqr_diff.append(c_math_util.Vec3Norm(diff))

    gyro_diff = 'A-B:%5.3f  B-C:%5.3f  C-A:%5.3f' % (
        pqr_diff[labels.kWingImuA], pqr_diff[labels.kWingImuB],
        pqr_diff[labels.kWingImuC])

    return gyro_diff, stoplights.STOPLIGHT_NORMAL


class EstimatorGyroBiasDriftIndicator(control.BaseControllerRunningIndicator):
  """Indicator for Gyro Bias Drift."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE)
  def __init__(self, mode):
    super(EstimatorGyroBiasDriftIndicator, self).__init__(
        mode, 'Gyro Bias Drift')
    self._crosswind_drift_limits = self._GetLimits(
        _MONITOR_PARAMS.est.gyro_bias_crosswind_drift)
    self._hover_gyro_biases = numpy.empty((3, 3), dtype=float)
    self._hover_gyro_biases[:] = numpy.nan

  def _RunningAverage(self, value, count, new_sample):
    return float(value * count + new_sample) / (count + 1)

  @indicator.ReturnIfInputInvalid('X:    --   Y:    --   Z:    -- ',
                                  stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry):

    gyro_bias_crosswind_drift_max = [0.0, 0.0, 0.0]
    gyro_biases = telemetry.estimator.gyro_biases
    is_crosswind = control_common.AnyCrosswindFlightMode(telemetry.flight_mode)

    if is_crosswind:
      for i in range(labels.kNumWingImus):
        gyro_bias_crosswind_drift_max[0] = numpy.max((
            gyro_bias_crosswind_drift_max[0],
            abs(gyro_biases[i].x - self._hover_gyro_biases[i][0])))
        gyro_bias_crosswind_drift_max[1] = numpy.max((
            gyro_bias_crosswind_drift_max[1],
            abs(gyro_biases[i].y - self._hover_gyro_biases[i][1])))
        gyro_bias_crosswind_drift_max[2] = numpy.max((
            gyro_bias_crosswind_drift_max[2],
            abs(gyro_biases[i].z - self._hover_gyro_biases[i][2])))

      # TODO(b/77811299): The limit is currently only applied to the
      # X-axis gyros, as reasonable limits for the Y and Z axes are
      # not yet known.  If we keep this indicator around, limits for
      # the Y and Z axes should be added.
      if numpy.any(numpy.isnan(gyro_bias_crosswind_drift_max)):
        stoplight = stoplights.STOPLIGHT_UNAVAILABLE
      else:
        stoplight = stoplights.SetByLimits(
            gyro_bias_crosswind_drift_max[0], self._crosswind_drift_limits)
      text = 'X: %6.4f  Y: %6.4f  Z: %6.4f' % (
          gyro_bias_crosswind_drift_max[0], gyro_bias_crosswind_drift_max[1],
          gyro_bias_crosswind_drift_max[2])
    else:
      for i in range(labels.kNumWingImus):
        self._hover_gyro_biases[i, 0] = gyro_biases[i].x
        self._hover_gyro_biases[i, 1] = gyro_biases[i].y
        self._hover_gyro_biases[i, 2] = gyro_biases[i].z
      stoplight = stoplights.STOPLIGHT_ANY
      text = 'X:    --   Y:    --   Z:    -- '

    return text, stoplight


class EstimatorGyroBiasIndicator(control.BaseControllerRunningIndicator):
  """Indicator for Gyro Bias Estimator."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE)
  def __init__(self, mode):
    super(EstimatorGyroBiasIndicator, self).__init__(
        mode, 'Gyro Bias[rad/s]')
    self._limits = self._GetLimits(_MONITOR_PARAMS.est.gyro_bias)

  @indicator.ReturnIfInputInvalid('A:    --   B:    --   C:    -- ',
                                  stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry):

    gyro_bias_max = []
    gyro_biases = telemetry.estimator.gyro_biases

    for i in range(labels.kNumWingImus):
      gyro_bias_max.append(max(abs(gyro_biases[i].x), abs(gyro_biases[i].y),
                               abs(gyro_biases[i].z)))

    stoplight = stoplights.SetByLimits(max(gyro_bias_max), self._limits)

    text = 'A: %6.4f  B: %6.4f  C: %6.4f' % (
        gyro_bias_max[0], gyro_bias_max[1], gyro_bias_max[2])

    return text, stoplight


class EstimatorGyroBiasChart(control.BaseControllerRunningListChart):

  @indicator.RegisterModes(common.FULL_COMMS_MODE)
  def __init__(self, mode, **widget_kwargs):
    super(EstimatorGyroBiasChart, self).__init__(
        mode, ['A', 'B', 'C'], 'Gyro Bias[rad/s]',
        precision=4, ylim=[-0.02, 0.02], **widget_kwargs)
    self._limits = self._GetLimits(_MONITOR_PARAMS.est.gyro_bias)

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    gyro_biases = args[0].estimator.gyro_biases
    gyro_bias_max = []
    for i in range(labels.kNumWingImus):
      gyro_bias_max.append(max(abs(gyro_biases[i].x), abs(gyro_biases[i].y),
                               abs(gyro_biases[i].z)))

    stoplight = stoplights.SetByLimits(max(gyro_bias_max), self._limits)

    return self._GetTimestamps(*args), gyro_bias_max, stoplight


class EstimatorGpsDiff(control.BaseControllerRunningIndicator):
  """"Compare position estimates from individual GPS receivers.

  This indicator shows the difference in position estimate between
  each individual GPS receiver, referenced to the body origin,
  compared to the estimator's current position estimate. Note that
  errors in attitude estimation will also show up here, as the
  attitude is required to reference individual sensor measurements to
  the body origin.
  """

  @indicator.RegisterModes(common.FULL_COMMS_MODE)
  def __init__(self, mode):
    super(EstimatorGpsDiff, self).__init__(mode, 'GPS Diff')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    dx = [_Vec3Distance(gps.Xg, telemetry.state_est.Xg)
          if gps.wing_pos_valid else float('nan')
          for gps in telemetry.estimator.gps]

    is_hover = control_common.AnyHoverFlightMode(telemetry.flight_mode)

    message = ' '.join(['    --' if numpy.isnan(x) else '{:6.2f}'.format(x)
                        for x in dx]) + ' m'

    # Agreement between the GPS receivers is only crucial during
    # launch and landing.
    stoplight = (
        stoplights.STOPLIGHT_ERROR if all(numpy.isnan(dx)) else
        stoplights.STOPLIGHT_WARNING if numpy.nanmax(dx) > 1.0 and is_hover else
        stoplights.STOPLIGHT_NORMAL)

    return message, stoplight


class EstimatorGsgDiff(control.BaseControllerRunningIndicator):

  @indicator.RegisterModes(common.FULL_COMMS_MODE)
  def __init__(self, mode):
    super(EstimatorGsgDiff, self).__init__(mode, 'GLAS Diff')
    self._limits = self._GetLimits(_MONITOR_PARAMS.est.glas_pos_diff)

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    delta_norm = _Vec3Distance(
        telemetry.state_est.Xg, telemetry.estimator.glas.Xg)
    valid = telemetry.estimator.glas.wing_pos_valid

    message = 'pos: {:6.2f} m'.format((delta_norm))
    stoplight = stoplights.SetByLimits(delta_norm, self._limits)

    return ((message, stoplight) if valid
            else ('--', stoplights.STOPLIGHT_UNAVAILABLE))


class EstimatorGsgBias(control.BaseControllerRunningIndicator):

  @indicator.RegisterModes(common.FULL_COMMS_MODE)
  def __init__(self, mode):
    super(EstimatorGsgBias, self).__init__(mode, 'GSG Bias')
    self.gsg_bias_azi_limits = self._GetLimits(_MONITOR_PARAMS.est.gsg_bias_azi)
    self.gsg_bias_ele_limits = self._GetLimits(_MONITOR_PARAMS.est.gsg_bias_ele)

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):

    stoplight = stoplights.MostSevereStoplight(
        stoplights.SetByLimits(telemetry.estimator.gsg_bias.azi,
                               self.gsg_bias_azi_limits),
        stoplights.SetByLimits(telemetry.estimator.gsg_bias.ele,
                               self.gsg_bias_ele_limits))

    message = 'azi: {azi: 3.0f} deg   ele: {ele: 3.0f} deg'.format(
        azi=(180.0 / numpy.pi) * telemetry.estimator.gsg_bias.azi,
        ele=(180.0 / numpy.pi) * telemetry.estimator.gsg_bias.ele)

    return message, stoplight


class EstimatorMagnetometerDiffIndicator(
    control.BaseControllerRunningIndicator):

  @indicator.RegisterModes(common.FULL_COMMS_MODE)
  def __init__(self, mode):
    super(EstimatorMagnetometerDiffIndicator, self).__init__(
        mode, 'Mag. Diff. [G]')

  @indicator.ReturnIfInputInvalid('A-B:  --   B-C:  --   C-A:  -- ',
                                  stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, telemetry, *args):
    imus = telemetry.control_input.imus
    mag_vecs = [
        ctype_util.CastPointer(imus[i].mag, c_math_util.Vec3)
        for i in range(labels.kNumWingImus)]
    mag_diff = []
    for i in range(labels.kNumWingImus):
      diff = ctypes.pointer(c_math_util.Vec3())
      c_math_util.Vec3Sub(mag_vecs[i], mag_vecs[(i + 1) % 3], diff)
      mag_diff.append(c_math_util.Vec3Norm(diff))

    if max(mag_diff) > _MONITOR_PARAMS.est.max_mag_diff:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    text = 'A-B:%5.3f  B-C:%5.3f  C-A:%5.3f' % (
        mag_diff[0], mag_diff[1], mag_diff[2])

    return text, stoplight


def _AttitudeDiff(control_telemetry):
  """Compute attitude diffs from control telemetry. Return None if invalid."""
  dcm_g2b = ctype_util.CastPointer(control_telemetry.state_est.dcm_g2b,
                                   c_math_util.Mat3)

  # The computation requires dcm_g2b to be orthogonal. If not, something
  # must be going wrong.
  if not c_math_util.Mat3IsSpecialOrthogonal(dcm_g2b, c_math_util.DBL_TOL):
    return None

  q = ctypes.pointer(c_math_util.Quat())
  c_math_util.DcmToQuat(dcm_g2b, q)

  attitude_diff = []
  for i in range(labels.kNumWingImus):
    dot = c_math_util.QuatDot(
        q, ctype_util.CastPointer(control_telemetry.estimator.q_g2b[i],
                                  c_math_util.Quat))
    # See the comment for the QuatDot function in quaternion.h to see
    # why this calculates the rotation angle between two quaternions.
    attitude_diff.append(c_math_util.Acos(1 - 2 * (1 - dot * dot)))

  return attitude_diff


class EstimatorAttitudeDiffChart(control.BaseControllerRunningListChart):

  @indicator.RegisterModes(common.FULL_COMMS_MODE)
  def __init__(self, mode, **widget_kwargs):
    assert labels.kNumWingImus == 3
    super(EstimatorAttitudeDiffChart, self).__init__(
        mode, ['A', 'B', 'C'], 'Att. Diff. [rad]',
        precision=3, ylim=[0.0, 0.3], **widget_kwargs)

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    attitude_diff = _AttitudeDiff(args[0])
    if not attitude_diff:
      return None, None, stoplights.STOPLIGHT_WARNING

    if max(attitude_diff) > _MONITOR_PARAMS.est.max_attitude_diff:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    return self._GetTimestamps(*args), attitude_diff, stoplight


class EstimatorAttitudeDiffIndicator(control.BaseControllerRunningIndicator):

  @indicator.RegisterModes(common.FULL_COMMS_MODE)
  def __init__(self, mode):
    super(EstimatorAttitudeDiffIndicator, self).__init__(
        mode, 'Att. Diff. [rad]')

  @indicator.ReturnIfInputInvalid('A:    --   B:    --   C:    -- ',
                                  stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    attitude_diff = _AttitudeDiff(args[0])
    if not attitude_diff:
      return 'A:    --   B:    --   C:    -- ', stoplights.STOPLIGHT_WARNING

    if max(attitude_diff) > _MONITOR_PARAMS.est.max_attitude_diff:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    text = 'A:  %5.3f  B:  %5.3f  C:  %5.3f' % (
        attitude_diff[0], attitude_diff[1], attitude_diff[2])

    return text, stoplight
