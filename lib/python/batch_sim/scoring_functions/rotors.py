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

"""Scoring functions relating to rotors."""

from makani.avionics.network import aio_labels
from makani.control import system_types
from makani.lib.python.batch_sim import scoring_functions

import numpy as np
import scoring_functions_util as scoring_util


class MaxRotorThrustScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if a maximum rotor thrust limit is met."""

  def __init__(self, good_limit, bad_limit, severity):
    super(MaxRotorThrustScoringFunction, self).__init__(
        'Max Thrust', 'N', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['rotors', 'loads']

  def GetValue(self, output):
    return output['rotor_thrust_max']

  def GetOutput(self, timeseries):
    return {'rotor_thrust_max': np.max(timeseries['rotor_thrust'])}

  def GetTimeSeries(self, params, sim, control):
    thrust = self._SelectTelemetry(sim, control, 'rotor_thrusts')
    return {'rotor_thrust': np.abs(thrust)}


class MaxMotorTorqueScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if a maximum motor torque limit is met."""

  def __init__(self, good_limit, bad_limit, severity):
    super(MaxMotorTorqueScoringFunction, self).__init__(
        'Max Motor Torque', 'N-m', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['loads', 'power']

  def GetValue(self, output):
    return output['motor_torque_max']

  def GetOutput(self, timeseries):
    return {'motor_torque_max': np.max(timeseries['motor_torque_abs'])}

  def GetTimeSeries(self, params, sim, control):
    torque = self._SelectTelemetry(sim, control, 'motor_torques')
    return {'motor_torque_abs': np.abs(torque)}


class MaxRotorSpeedsScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if a maximum rotor speed limit for a specific rotor is met."""

  def __init__(self, good_limit, bad_limit, severity):
    super(MaxRotorSpeedsScoringFunction, self).__init__(
        'Max Rotor Speeds', 'rad/s', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['loads', 'motor', 'rotor']

  def GetValue(self, output):
    return output['rotor_speeds_max']

  def GetOutput(self, timeseries):
    max_speeds = np.max(timeseries['omegas_abs'], axis=0)
    return {'rotor_speeds_max': max_speeds.tolist()}

  def GetTimeSeries(self, params, sim, control):
    omegas = self._SelectTelemetry(sim, control, 'rotor_speeds')
    return {'omegas_abs': np.abs(omegas)}


class ThrustMomentSaturationDurationScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if the rotor thrust or moment command is saturated."""

  def __init__(self, axis, good_limit, bad_limit, severity):
    super(ThrustMomentSaturationDurationScoringFunction, self).__init__(
        'Max thrust_moment[''%s''] Saturation Duration' % axis, 's', good_limit,
        bad_limit, severity)
    self._axis = axis
    self._sources = ['control']

  def GetSystemLabels(self):
    # This scoring function is marked as 'experimental' until the empirical
    # correction to the rotor aerodynamic moment is finalized (see b/117942095).
    return ['rotors', 'controls', 'experimental']

  def GetValue(self, output):
    return output['max_saturation_duration']

  def GetOutput(self, timeseries):
    return {'max_saturation_duration': timeseries['max_saturation_duration']}

  def GetTimeSeries(self, params, sim, control):
    hover_params = params['control_params']['hover']

    thrust_moment, thrust_moment_avail, control_time = self._SelectTelemetry(
        sim, control, ['thrust_moment', 'thrust_moment_avail', 'time'])

    if (not scoring_util.IsSelectionValid(thrust_moment) or
        not scoring_util.IsSelectionValid(control_time)):
      return {'max_saturation_duration': float('nan')}

    if self._axis == 'thrust':
      cmd = thrust_moment['thrust']
      cmd_avail = thrust_moment_avail['thrust']
      min_software_limit = float('-infinity')
      max_software_limit = hover_params['altitude']['max_thrust']
    elif self._axis == 'moment_y':
      cmd = thrust_moment['moment']['y']
      cmd_avail = thrust_moment_avail['moment']['y']
      min_software_limit = hover_params['angles']['min_moment']['y']
      max_software_limit = hover_params['angles']['max_moment']['y']
    elif self._axis == 'moment_z':
      cmd = thrust_moment['moment']['z']
      cmd_avail = thrust_moment_avail['moment']['z']
      min_software_limit = hover_params['angles']['min_moment']['z']
      max_software_limit = hover_params['angles']['max_moment']['z']
    else:
      assert False, 'The axis %s is not supported.' % self._axis

    t_samp = scoring_util.GetTimeSamp(control_time)
    if np.isnan(t_samp):
      return {'max_saturation_duration': np.array([float('nan')])}

    saturation_mask = np.argwhere(np.logical_or(
        np.abs(cmd - cmd_avail) > 1e-2, np.logical_or(
            np.abs(cmd - min_software_limit) < 1e-2,
            np.abs(cmd - max_software_limit) < 1e-2)))

    if saturation_mask.size > 0:
      saturated_intervals = scoring_util.GetIntervals(saturation_mask)
      max_saturation_size = np.max(
          [interval[1] - interval[0] for interval in saturated_intervals])
    else:
      max_saturation_size = 0.0

    return {'max_saturation_duration': max_saturation_size * t_samp}


class RotorStallScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if a rotor exceeds stall (advance ratio) limit."""

  # When the max advance ratio is exceeded in crosswind flight,
  # the rotor is considered stalled.
  # TODO: Add a hover-descend stall limit as well, which is
  # a lower bound of advance ratio (separate from vortex-ring state).

  # Stall margin between stall advance ratio and max advance ratio,
  # (should match what is in rotor_control.py).
  _advance_ratio_stall_margin = {
      system_types.kPropVersionRev1: 0.3,
      system_types.kPropVersionRev1Trimmed: 0.3,
      system_types.kPropVersionRev2: 0.3,
      system_types.kPropVersionRev3NegativeX: 0.1,
      system_types.kPropVersionRev3PositiveX: 0.1,
      system_types.kPropVersionRev4NegativeX: 0.1,
      system_types.kPropVersionRev4PositiveX: 0.1,
  }

  def __init__(self, good_limit, bad_limit, severity):
    super(RotorStallScoringFunction, self).__init__(
        'Rotor Stall Margin', '-', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['experimental', 'rotors', 'power']

  def GetValue(self, output):
    return output['rotor_stall_margin']

  def GetOutput(self, timeseries):
    rotors_stall_margin = timeseries['rotors_stall_margin']
    # Calculate the minimum margin from all rotors and return.
    rotor_stall_margin = 1.0
    for rotor in rotors_stall_margin:
      rotor_stall_margin = min(rotor_stall_margin,
                               np.min(rotors_stall_margin[rotor]))

    return {'rotor_stall_margin': rotor_stall_margin}

  def GetTimeSeries(self, params, sim, control):
    prop_versions = params['system_params']['rotors']['version']
    omegas, v_freestream = self._SelectTelemetry(
        sim, control, ['rotor_speeds', 'rotor_freestream_speeds'])
    rotors_stall_margin = {}
    for i in range(aio_labels.kNumMotors):
      # j_max is the advance ratio max including some margin from stall.
      j_max = (params['control_params']['rotor_control']['simple_models']
               ['J_max'][0, i])
      # j_stall is the stall advance ratio from the rotor tables,
      # when d_thrust/d_omega switches signs at a constant v_axial.
      j_stall = (j_max +
                 self._advance_ratio_stall_margin[prop_versions[0, i]])
      radius = params['system_params']['rotors']['D'][0, i] / 2.0
      # j_flight is advance ratio experienced during the flight.
      j_flight = np.pi * v_freestream[:, i] / (omegas[:, i] * radius)
      # Calculate stall margin [#] for flight for each rotor. Margin < 0
      # indicates rotor was stalled according to rotor aero tables.
      rotors_stall_margin[i] = j_stall - j_flight

    return {
        'rotors_stall_margin': rotors_stall_margin
    }


class VortexRingStateScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests the axial freestream margin [m/s] from the VRS suck-in cliff."""

  # These coefficients describe a linear model of the vortex ring state
  # threshold airspeed, i.e. (signed) airspeed at which the rotor thrust damping
  # changes sign:
  #     vrs_speed = coeffs[0] * abs(omega) + coeffs[1].
  # This is independent of air density.
  #
  # For prop models for which we have not determined a VRS model, the
  # coefficients are set to nan, resulting in a score of nan.
  #
  # TODO: Describe how these coefficients are calculated, and ideally
  # automate the process.
  _VRS_COEFFICIENTS = {
      system_types.kPropVersionRev1: (float('nan'), float('nan')),
      system_types.kPropVersionRev1Trimmed: (float('nan'), float('nan')),
      system_types.kPropVersionRev2: (float('nan'), float('nan')),
      system_types.kPropVersionRev3NegativeX: (-0.0407, 0.126),
      system_types.kPropVersionRev3PositiveX: (-0.0407, 0.126),
      system_types.kPropVersionRev4NegativeX: (-0.0358, -0.0264),
      system_types.kPropVersionRev4PositiveX: (-0.0358, -0.0264),
  }

  # The freestream margin in m/s below which a rotor is determined
  # to be nearing VRS
  _VRS_START = 0.5
  # The freestream margin in  m/s where a rotor is considered fully in VRS
  _VRS_SATURATE = -1.5

  def __init__(self, good_margin, bad_margin, severity):
    # Transition from good to bad over a margin of good_margin m/s to bad_margin
    # m/s from the VRS axial freestream threshold.
    super(VortexRingStateScoringFunction, self).__init__(
        'Vortex Ring State - Number of Rotors', 'num_rotors',
        good_margin, bad_margin, severity)

  def GetSystemLabels(self):
    return ['experimental', 'controls', 'rotors']

  def GetValue(self, output):
    return output['vrs_max_num_rotors']

  def _GetNumberOfRotors(self, timeseries):
    if timeseries is None:
      return None
    vrs_margin = timeseries['vrs_margin']

    if np.any(np.isnan(vrs_margin)):
      return float('nan')
    else:
      # remap margin from [_VRS_START, _VRS_SATURATE] to [0, 1]
      # for each rotor, then sum all rotors
      return np.sum(
          (np.clip(vrs_margin, self._VRS_SATURATE, self._VRS_START)
           - self._VRS_START) / (self._VRS_SATURATE - self._VRS_START),
          axis=1)

  def GetOutput(self, timeseries):
    vrs_num_rotors = self._GetNumberOfRotors(timeseries)

    return {
        'vrs_max_num_rotors': np.max(vrs_num_rotors)
    }

  def GetFailureCount(self, timeseries):
    if timeseries is None:
      return None
    if 'vrs_num_rotors' in timeseries and len(timeseries.keys()) == 1:
      # This case is for the GetFailureCount call in GetIndexOfFirstFailure
      # where the `timeseries` dict has already been through the
      # _GetNumberOfRotors function.
      return super(VortexRingStateScoringFunction, self).GetFailureCount(
          timeseries)
    else:
      return super(VortexRingStateScoringFunction, self).GetFailureCount(
          {'vrs_num_rotors': self._GetNumberOfRotors(timeseries)})

  def GetIndexOfFirstFailure(self, timeseries):
    if timeseries is None:
      return None
    return super(VortexRingStateScoringFunction, self).GetIndexOfFirstFailure(
        {'vrs_num_rotors': self._GetNumberOfRotors(timeseries)})

  def _FilterInitialTransients(self, time, omegas):
    """Retrieves data starting 30 seconds after the first nonzero omega.

    This prevents us from scoring the VRS margin during launch when unrealistic
    transients occur.

    Args:
      time: flight time.
      omegas: motor speeds.

    Returns:
      Indices starting 30 seconds after the first nonzero omega.
    """

    i_nonzero_omega = len(omegas)

    for i in range(aio_labels.kNumMotors):
      i_nonzero_omega = min((np.where(omegas[:, i] > 0.0))[0][0],
                            i_nonzero_omega)

    t_nonzero_omega = time[i_nonzero_omega] + 30.0
    indices = np.argwhere(time >= t_nonzero_omega)
    indices = np.reshape(indices, (indices.size,))

    return indices

  def GetTimeSeries(self, params, sim, control):
    prop_versions = params['system_params']['rotors']['version']

    try:
      omegas, v_freestream, wing_xg, time = self._SelectTelemetry(
          sim, control,
          ['rotor_speeds', 'rotor_freestream_speeds', 'wing_xg', 'time'])
    except ValueError:
      # TODO: Implement function to compute 'v_app_locals' to
      # get VRS scoring function for flights prior to RPX-06.
      return {'vrs_margin': np.array([float('nan')])}

    ind = self._FilterInitialTransients(time, omegas)
    wing_xg = wing_xg[ind]

    if ind.size == 0:
      vrs_margin = np.array([float('nan')])
    else:
      vrs_speeds = np.zeros((len(ind), aio_labels.kNumMotors))
      for i in range(aio_labels.kNumMotors):
        coeffs = self._VRS_COEFFICIENTS[prop_versions[0, i]]
        vrs_speeds[:, i] = (coeffs[0] * omegas[ind, i] + coeffs[1])

      # Margin from the critical axial airspeed.
      vrs_margin = v_freestream[ind, :] - vrs_speeds

    return {'vrs_margin': vrs_margin}
