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

"""Indicators for GPS."""

from makani.analysis.checks.collection import gps_checks
from makani.avionics.common import novatel_types
from makani.avionics.common import pack_avionics_messages
from makani.avionics.common import septentrio_types
from makani.gs.monitor2.apps.layout import indicator
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins import common
from makani.lib.python import c_helpers
from makani.system import labels
import numpy

NOVATEL_SOLUTION_STATUS_HELPER = c_helpers.EnumHelper(
    'NovAtelSolutionStatus', novatel_types)
NOVATEL_SOLUTION_TYPE_HELPER = c_helpers.EnumHelper(
    'NovAtelSolutionType', novatel_types)
SEPTENTRIO_ERROR_HELPER = c_helpers.EnumHelper(
    'SeptentrioPvtError', septentrio_types)
SEPTENTRIO_MODE_HELPER = c_helpers.EnumHelper(
    'SeptentrioPvtMode', septentrio_types)

SEPTENTRIO_MODE_BITMASK = c_helpers.EnumHelper(
    'SeptentrioPvtModeBit', septentrio_types).Value('SolutionMask')

TETHER_GPS_SOLUTION_STATUS_HELPER = c_helpers.EnumHelper(
    'TetherGpsSolutionStatus', pack_avionics_messages)

WING_GPS_RECEIVER_HELPER = c_helpers.EnumHelper('WingGpsReceiver', labels)


def _NumSatsStoplight(num_sol):
  if num_sol >= 5:
    return stoplights.STOPLIGHT_NORMAL
  elif num_sol == 4:
    return stoplights.STOPLIGHT_WARNING
  else:
    return stoplights.STOPLIGHT_ERROR


class BaseGpsIndicator(indicator.SingleAttributeIndicator):
  """The base class for GPS indicators."""

  def __init__(self, message_type, aio_node, name=None):
    super(BaseGpsIndicator, self).__init__((message_type, aio_node), name)

  def _SolutionStatus(self, status, diff_age):
    """Set the GPS stoplight according to its status."""
    raise NotImplementedError

  def _SolutionType(self, allowed_modes, mode):
    """Get the name of the mode and set stoplight accordingly."""
    raise NotImplementedError


class BaseNovAtelIndicator(BaseGpsIndicator):
  """The base class for NovAtel indicators."""

  def __init__(self, aio_node, name=None):
    super(BaseNovAtelIndicator, self).__init__(
        'NovAtelSolution', aio_node, name)

  def _SolutionStatus(self, status, diff_age):
    if (status == novatel_types.kNovAtelSolutionStatusSolComputed
        and diff_age < 3.0):
      return stoplights.STOPLIGHT_NORMAL
    else:
      return stoplights.STOPLIGHT_WARNING

  def _SolutionType(self, allowed_modes, mode_enum):
    if mode_enum in allowed_modes:
      stoplight = stoplights.STOPLIGHT_NORMAL
    else:
      stoplight = stoplights.STOPLIGHT_WARNING
    mode = NOVATEL_SOLUTION_TYPE_HELPER.ShortName(mode_enum)
    return mode, stoplight

  def _PosVelType(self, gps_solution, allowed_pos_types, allowed_vel_types):

    pos_type = gps_solution.best_xyz.pos_type
    vel_type = gps_solution.best_xyz.vel_type

    pos_type_name, pos_stoplight = self._SolutionType(
        allowed_pos_types, pos_type)
    vel_type_name, vel_stoplight = self._SolutionType(
        allowed_vel_types, vel_type)

    stoplight = stoplights.MostSevereStoplight(pos_stoplight, vel_stoplight)

    return ('pos: {:>6}    vel: {:>6}'.format(pos_type_name, vel_type_name),
            stoplight)


class BaseCompassIndicator(indicator.SingleAttributeIndicator):

  def __init__(self, aio_node, name=None):
    super(BaseCompassIndicator, self).__init__(
        ('NovAtelCompass', aio_node), name)

  def _SolutionStatus(self, status):
    # TODO: Check valid bits when it becomes available.
    if status == novatel_types.kNovAtelSolutionStatusSolComputed:
      return stoplights.STOPLIGHT_NORMAL
    else:
      return stoplights.STOPLIGHT_WARNING


class NovAtelNumSatsIndicator(BaseNovAtelIndicator):

  def __init__(self, aio_node, name='Num Satellites'):
    super(NovAtelNumSatsIndicator, self).__init__(aio_node, name)

  def _Filter(self, gps_solution):
    num_sol = gps_solution.best_xyz.num_sol
    num_gg_l1 = gps_solution.best_xyz.num_gg_l1
    num_tracked = gps_solution.best_xyz.num_tracked
    text = 'Sol.: {:2d} GG L1: {:2d} Tracked: {:2d}'.format(
        num_sol, num_gg_l1, num_tracked)
    return text, _NumSatsStoplight(num_sol)


class NovAtelSigmasIndicator(BaseNovAtelIndicator):
  """Indicator about the quality of the NovAtel GPS solutions."""

  def __init__(self, aio_node, name='GPS Sigmas'):
    super(NovAtelSigmasIndicator, self).__init__(aio_node, name)

  def _NovAtelSigmas(self, gps_solution):
    best_xyz = gps_solution.best_xyz
    stoplight = self._SolutionStatus(
        best_xyz.pos_sol_status, best_xyz.diff_age)

    pos_sigma = numpy.linalg.norm([best_xyz.pos_x_sigma,
                                   best_xyz.pos_y_sigma,
                                   best_xyz.pos_z_sigma])
    vel_sigma = numpy.linalg.norm([best_xyz.vel_x_sigma,
                                   best_xyz.vel_y_sigma,
                                   best_xyz.vel_z_sigma])
    text = 'pos: {:6.2f} m  vel: {:6.2f} m/s'.format(pos_sigma, vel_sigma)
    return text, stoplight

  def _Filter(self, gps_solution):
    return self._NovAtelSigmas(gps_solution)


class NovAtelPosSolStatusIndicator(BaseNovAtelIndicator):

  def __init__(self, aio_node, name='Pos. Sol. Status'):
    super(NovAtelPosSolStatusIndicator, self).__init__(aio_node, name)

  def _Filter(self, gps_solution):
    status = gps_solution.best_xyz.pos_sol_status
    diff_age = gps_solution.best_xyz.diff_age
    description = '{}\nMean Corr. Age: {: 5.1f} s'.format(
        NOVATEL_SOLUTION_STATUS_HELPER.ShortName(status), diff_age)
    return description, self._SolutionStatus(status, diff_age)


class NovAtelVelSolStatusIndicator(BaseNovAtelIndicator):

  def __init__(self, aio_node, name='Vel. Sol. Status'):
    super(NovAtelVelSolStatusIndicator, self).__init__(aio_node, name)

  def _Filter(self, gps_solution):
    status = gps_solution.best_xyz.vel_sol_status
    diff_age = gps_solution.best_xyz.diff_age
    description = '{}\nMean Corr. Age: {: 5.1f} s'.format(
        NOVATEL_SOLUTION_STATUS_HELPER.ShortName(status), diff_age)
    return description, self._SolutionStatus(status, diff_age)


class GsNovAtelPosVelTypeIndicator(BaseNovAtelIndicator):

  def __init__(self, aio_node, name='Sol. Type'):
    super(GsNovAtelPosVelTypeIndicator, self).__init__(aio_node, name)

  def _Filter(self, gps_solution):
    return self._PosVelType(gps_solution,
                            [novatel_types.kNovAtelSolutionTypeFixedPos],
                            [novatel_types.kNovAtelSolutionTypeDopplerVelocity])


class WingNovAtelPosVelTypeIndicator(BaseNovAtelIndicator):

  def __init__(self, aio_node, name='Sol. Type'):
    super(WingNovAtelPosVelTypeIndicator, self).__init__(aio_node, name)

  def _Filter(self, gps_solution):
    allowed_types = [
        novatel_types.kNovAtelSolutionTypeL1Float,
        novatel_types.kNovAtelSolutionTypeL1Int,
        novatel_types.kNovAtelSolutionTypeNarrowInt,
    ]
    return self._PosVelType(gps_solution, allowed_types, allowed_types)


class CompassHeadingIndicator(BaseCompassIndicator):

  def __init__(self, aio_node, name='Heading'):
    super(CompassHeadingIndicator, self).__init__(aio_node, name)

  def _Filter(self, compass):
    heading = compass.heading.heading
    if compass.heading_latency * 1e-6 > 0.1:
      # Expected update rate of the NovAtel compass is 20 Hz.
      return '--', stoplights.STOPLIGHT_UNAVAILABLE
    elif (compass.heading.pos_type ==
          novatel_types.kNovAtelSolutionTypeNone or
          compass.heading.pos_sol_status !=
          novatel_types.kNovAtelSolutionStatusSolComputed):
      # Once a compass is installed, this should be a warning.
      return 'No solution', stoplights.STOPLIGHT_UNAVAILABLE
    else:
      return ('{: 5.1f} deg'.format(numpy.rad2deg(heading)),
              stoplights.STOPLIGHT_NORMAL)


class CompassNumSatsIndicator(BaseCompassIndicator):

  def __init__(self, aio_node, name='Num Satellites'):
    super(CompassNumSatsIndicator, self).__init__(aio_node, name)

  def _Filter(self, compass):
    if compass.heading_latency * 1e-6 > 0.1:
      # Expected update rate of the NovAtel compass is 20 Hz.
      return '--', stoplights.STOPLIGHT_UNAVAILABLE
    num_sol = compass.heading.num_sol
    num_obs = compass.heading.num_obs
    num_tracked = compass.heading.num_tracked
    text = 'Sol.: {:2d} Obs.: {:2d} Tracked: {:2d}'.format(
        num_sol, num_obs, num_tracked)
    return text, _NumSatsStoplight(num_sol)


class CompassSigmasIndicator(BaseCompassIndicator):

  def __init__(self, aio_node, name='Sigmas'):
    super(CompassSigmasIndicator, self).__init__(aio_node, name)

  def _Filter(self, compass):
    if compass.heading_latency * 1e-6 > 0.1:
      # Expected update rate of the NovAtel compass is 20 Hz.
      return '--', stoplights.STOPLIGHT_UNAVAILABLE
    heading_sigma = numpy.rad2deg(compass.heading.heading_sigma)
    pitch_sigma = numpy.rad2deg(compass.heading.pitch_sigma)
    text = 'Heading:{: 5.2f} deg  Pitch:{: 5.2f} deg'.format(
        heading_sigma, pitch_sigma)
    return text, self._SolutionStatus(compass.heading.pos_sol_status)


class CompassSolTypeIndicator(BaseCompassIndicator):
  """Indicator that shows the compass solution type."""

  def __init__(self, aio_node, name='Sol. Type'):
    super(CompassSolTypeIndicator, self).__init__(aio_node, name)

  def _SolutionType(self, allowed_modes, mode_enum):
    if mode_enum in allowed_modes:
      stoplight = stoplights.STOPLIGHT_NORMAL
    else:
      stoplight = stoplights.STOPLIGHT_WARNING
    mode = NOVATEL_SOLUTION_TYPE_HELPER.ShortName(mode_enum)
    return mode, stoplight

  def _Filter(self, compass):
    if compass.heading_latency * 1e-6 > 0.1:
      # Expected update rate of the NovAtel compass is 20 Hz.
      return '--', stoplights.STOPLIGHT_UNAVAILABLE

    pos_type = compass.heading.pos_type
    allowed_types = [
        novatel_types.kNovAtelSolutionTypeL1Float,
        novatel_types.kNovAtelSolutionTypeL1Int,
        novatel_types.kNovAtelSolutionTypeNarrowInt,
    ]
    pos_type_name, pos_stoplight = self._SolutionType(
        allowed_types, pos_type)
    return '{:>6}'.format(pos_type_name), pos_stoplight


class BaseSeptentrioIndicator(BaseGpsIndicator):
  """The base class for Septentrio indicators."""

  def __init__(self, aio_node, name=None):
    super(BaseSeptentrioIndicator, self).__init__(
        'SeptentrioSolution', aio_node, name)

  def _SolutionStatus(self, error, mean_corr_age):
    if (error == septentrio_types.kSeptentrioPvtErrorNone and
        mean_corr_age < 300):
      return stoplights.STOPLIGHT_NORMAL
    else:
      return stoplights.STOPLIGHT_WARNING

  def _SolutionType(self, allowed_modes, mode_bits):
    # See page 64 of AsteRx-m Firmware v3.3.0 SBF Reference Guide.
    mode_enum = mode_bits & septentrio_types.kSeptentrioPvtModeBitSolutionMask

    if not allowed_modes or mode_enum in allowed_modes:
      stoplight = stoplights.STOPLIGHT_NORMAL
    else:
      stoplight = stoplights.STOPLIGHT_WARNING

    return SEPTENTRIO_MODE_HELPER.ShortName(mode_enum), stoplight


class SeptentrioNumSatsIndicator(BaseSeptentrioIndicator):

  def __init__(self, aio_node, name='Num Satellites'):
    super(SeptentrioNumSatsIndicator, self).__init__(aio_node, name)

  def _Filter(self, gps_solution):
    # Number of satellites used in solution.
    num_sol = gps_solution.pvt_cartesian.nr_sv
    return 'Sol.: {:3d}'.format(num_sol), _NumSatsStoplight(num_sol)


class SeptentrioSigmasIndicator(BaseSeptentrioIndicator):

  def __init__(self, aio_node, name='GPS Sigmas'):
    super(SeptentrioSigmasIndicator, self).__init__(aio_node, name)

  def _Filter(self, gps_solution):
    stoplight = self._SolutionStatus(
        gps_solution.pvt_cartesian.error,
        gps_solution.pvt_cartesian.mean_corr_age)

    pos_cov = gps_solution.pos_cov_cartesian
    pos_sigma = numpy.sqrt(
        pos_cov.cov_xx + pos_cov.cov_yy + pos_cov.cov_zz)

    vel_cov = gps_solution.vel_cov_cartesian
    vel_sigma = numpy.sqrt(
        vel_cov.cov_xx + vel_cov.cov_yy + vel_cov.cov_zz)

    text = 'Pos: {: 2.2f}[m] Vel: {: 2.2f}[m/s]'.format(pos_sigma, vel_sigma)
    return text, stoplight


class SeptentrioSolStatusIndicator(BaseSeptentrioIndicator):

  def __init__(self, aio_node, name='Pos. Sol. Status'):
    super(SeptentrioSolStatusIndicator, self).__init__(aio_node, name)

  def _Filter(self, gps_solution):
    error = gps_solution.pvt_cartesian.error
    mean_corr_age = gps_solution.pvt_cartesian.mean_corr_age
    description = '{}\nMean Corr. Age: {: 5.1f} s'.format(
        SEPTENTRIO_ERROR_HELPER.ShortName(error), mean_corr_age * 0.01)
    stoplight = self._SolutionStatus(error, mean_corr_age)
    return description, stoplight


class SeptentrioModeIndicator(BaseSeptentrioIndicator):

  def __init__(self, aio_node, name='Mode'):
    super(SeptentrioModeIndicator, self).__init__(aio_node, name)

  def _Filter(self, gps_solution):
    return self._SolutionType([
        septentrio_types.kSeptentrioPvtModeDifferential,
        septentrio_types.kSeptentrioPvtModeFixedLocation,
        septentrio_types.kSeptentrioPvtModeRtkFixed,
        septentrio_types.kSeptentrioPvtModeRtkFloat,
        septentrio_types.kSeptentrioPvtModeMovingBaseRtkFixed,
        septentrio_types.kSeptentrioPvtModeMovingBaseRtkFloat
    ], gps_solution.pvt_cartesian.mode)


class BaseObsCn0Indicator(indicator.SingleAttributeIndicator):

  def __init__(self, message_type, source, cn0_checker_class, name):
    super(BaseObsCn0Indicator, self).__init__((message_type, source), name)
    self._cn0_checker = cn0_checker_class(False, source, name)

  def _Filter(self, gps_observations):
    cn0 = self._GetCTypeFieldByString(
        gps_observations, self._cn0_checker.Cn0Field())
    num_obs = self._GetCTypeFieldByString(
        gps_observations, self._cn0_checker.NumField())
    type_bits = self._GetCTypeFieldByString(
        gps_observations, self._cn0_checker.TypeField())
    avg_cn0, max_cn0, num_tracked = self._cn0_checker.GetAvgAndMaxCn0(
        cn0, num_obs, type_bits)
    if avg_cn0 > 40.0 and num_tracked >= 5:
      stoplight = stoplights.STOPLIGHT_NORMAL
    else:
      stoplight = stoplights.STOPLIGHT_WARNING
    text = 'Avg:{:3.1f} Max:{:3.1f} Tracked:{:2d}'.format(
        avg_cn0, max_cn0, num_tracked)
    return text, stoplight


# TODO: Deprecate it once we are comfortable with NovAtelCn0Indicator.
class NovAtelObsCn0Indicator(BaseObsCn0Indicator):

  def __init__(self, source, name='L1 C/N0'):
    super(NovAtelObsCn0Indicator, self).__init__(
        'NovAtelObservations', source,
        gps_checks.NovAtelCn0Checker, name)


class SeptentrioObsCn0Indicator(BaseObsCn0Indicator):

  def __init__(self, source, name='L1 C/N0'):
    super(SeptentrioObsCn0Indicator, self).__init__(
        'SeptentrioObservations', source,
        gps_checks.SeptentrioCn0Checker, name)


class BaseSolCn0Indicator(indicator.SingleAttributeIndicator):

  def __init__(self, message_type, source, name='L1 C/N0'):
    super(BaseSolCn0Indicator, self).__init__((message_type, source), name)

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, gps_solution):
    avg_cn0 = gps_solution.avg_cn0
    max_cn0 = gps_solution.max_cn0
    if avg_cn0 > 40.0:
      stoplight = stoplights.STOPLIGHT_NORMAL
    else:
      stoplight = stoplights.STOPLIGHT_WARNING
    text = 'avg: {:6.1f}    max: {:6.1f} dB'.format(avg_cn0, max_cn0)
    return text, stoplight


class NovAtelCn0Indicator(BaseSolCn0Indicator):

  def __init__(self, source, name='L1 C/N0'):
    super(NovAtelCn0Indicator, self).__init__('NovAtelSolution', source, name)


class SeptentrioCn0Indicator(BaseSolCn0Indicator):

  def __init__(self, source, name='L1 C/N0'):
    super(SeptentrioCn0Indicator, self).__init__(
        'SeptentrioSolution', source, name)


class TetherUpDownGpsIndicator(indicator.BaseAttributeIndicator):
  """Indicator for GPS status using TetherUp or TetherDown."""

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, gps_status, *args):
    # *args contains "valid" and "timestamp_sec".
    avg_cn0 = gps_status.avg_cn0
    num_sol = gps_status.satellites
    status = gps_status.status
    pos_sigma = gps_status.pos_sigma

    if num_sol < 4:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif (num_sol == 4 or avg_cn0 <= 40.0 or
          status not in self._allowed_solution_status):
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    data = {
        'sol_type': TETHER_GPS_SOLUTION_STATUS_HELPER.ShortName(status),
        'num_sol': num_sol,
        'pos_sigma': pos_sigma,
        'pos_sigma_str': ('%5.2f' % pos_sigma if pos_sigma < 100.0
                          else ('%5.0f' % pos_sigma if pos_sigma < 1e5
                                else ' BAD ')),
        'avg_cn0': avg_cn0,
    }

    text = ('{pos_sigma_str: <5} m {sol_type: <12} '
            '{num_sol:02d} {avg_cn0:2d} dBHz').format(**data)

    return text, stoplight


class TetherDownGpsIndicator(TetherUpDownGpsIndicator):
  """Indicator for GPS status using TetherDown."""

  def __init__(self, wing_gps_receiver, name=None):
    if not name:
      name = 'GPS ' + wing_gps_receiver
    index = WING_GPS_RECEIVER_HELPER.Value(wing_gps_receiver)
    gps_status = 'tether_down.gps_statuses[%d]' % index
    super(TetherDownGpsIndicator, self).__init__(
        [('filtered', 'merge_tether_down', gps_status),
         ('filtered', 'merge_tether_down', 'valid'),
         ('filtered', 'merge_tether_down', 'timestamp_sec')], name)
    self._allowed_solution_status = [
        pack_avionics_messages.kTetherGpsSolutionStatusFixedPos,
        pack_avionics_messages.kTetherGpsSolutionStatusRtkFloat,
        pack_avionics_messages.kTetherGpsSolutionStatusRtkFixed]

  def _IsValidInput(self, *args):
    return (args[1] and args[0] and
            args[0].no_update_count <= common.MAX_NO_UPDATE_COUNT_GPS_STATUS)


class TetherUpGpsIndicator(TetherUpDownGpsIndicator):
  """Indicator for GPS status using TetherUp."""

  def __init__(self, name=None):
    if not name:
      name = 'TetherUp GPS'
    super(TetherUpGpsIndicator, self).__init__(
        [('TetherUp', None, 'gps_status')], name)
    self._allowed_solution_status = [
        pack_avionics_messages.kTetherGpsSolutionStatusSingle,
        pack_avionics_messages.kTetherGpsSolutionStatusFixedPos]

  def _IsValidInput(self, *args):
    return len(args) == 1 and args[0] is not None
