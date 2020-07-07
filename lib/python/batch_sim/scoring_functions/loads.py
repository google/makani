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

"""Scoring functions relating to loads."""

import json
import os

import makani
from makani.analysis.aero import apparent_wind_util
from makani.analysis.control import geometry
from makani.control import control_types
from makani.lib.python import c_helpers
from makani.lib.python.batch_sim import scoring_functions
import numpy as np
from scipy import interpolate
import scoring_functions_util as scoring_util

_FLIGHT_MODE_HELPER = c_helpers.EnumHelper('FlightMode', control_types)


class YbAccelerationScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests if the body-y acceleration falls outside of acceptable limits."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity):
    super(YbAccelerationScoringFunction, self).__init__(
        'Acceleration', 'm/s^2', bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)

  def GetSystemLabels(self):
    return ['loads']

  def GetValue(self, output):
    return np.array([output['yb_accel_min'],
                     output['yb_accel_max']])

  def GetOutput(self, timeseries):
    return {
        'yb_accel_min': np.min(timeseries['yb_accel']),
        'yb_accel_max': np.max(timeseries['yb_accel'])
    }

  def GetTimeSeries(self, params, sim, control):
    yb_accel = self._SelectTelemetry(sim, control, 'wing_acc')['y']
    return {'yb_accel': yb_accel}


class ZgAccelerationScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests the kite vertical acceleration in ground frame."""

  def __init__(self, good_limit, bad_limit, severity):
    super(ZgAccelerationScoringFunction, self).__init__(
        'Wing Accel Zg', 'm/s^2', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return np.array([output['zg_accel_min'],
                     output['zg_accel_max']])

  def GetOutput(self, timeseries):
    return {
        'zg_accel_min': np.min(timeseries['zg_accel']),
        'zg_accel_max': np.max(timeseries['zg_accel'])
    }

  def GetTimeSeries(self, params, sim, control):
    zg_accel = self._SelectTelemetry(sim, control, 'wing_acc')['z']
    return {'zg_accel': zg_accel}


class MaxServoMoment(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if a maximum hinge moment limit is met on any of the servos."""

  def __init__(self, good_limit, bad_limit, severity):
    super(MaxServoMoment, self).__init__(
        'Max Servo Moment', 'N.m', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['loads']

  def GetValue(self, output):
    return output['servo_moment_max']

  def GetOutput(self, timeseries):
    max_moment = np.max(timeseries['servo_torques_abs'], axis=0)
    return {
        'servo_moment_max': max_moment.tolist()
    }

  def GetTimeSeries(self, params, sim, control):
    servo_torques = self._SelectTelemetry(sim, control,
                                          'servo_shaft_torques')
    servo_torques_abs = np.abs(np.array(servo_torques))

    return {'servo_torques_abs': servo_torques_abs}


class MaxElevatorServoMoment(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests the maximum hinge moment for the elevator servo pair."""

  def __init__(self, good_limit, bad_limit, severity):
    super(MaxElevatorServoMoment, self).__init__(
        'Max Elevator Servo Moment', 'N.m', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['loads']

  def GetValue(self, output):
    return output['elev_hm_max']

  def GetOutput(self, timeseries):
    return {'elev_hm_max': np.max(timeseries['elev_hm'])}

  def GetTimeSeries(self, params, sim, control):
    servo_torques = self._SelectTelemetry(sim, control,
                                          'servo_shaft_torques')

    if scoring_util.IsSelectionValid(servo_torques):
      elev_hm_1 = servo_torques[:, 6]
      elev_hm_2 = servo_torques[:, 7]
    else:
      # Servo_torques are returned as np.array([float('nan')]) for flight logs.
      elev_hm_1 = np.array([float('nan')])
      elev_hm_2 = np.array([float('nan')])

    summed_moment = np.abs(elev_hm_1) + np.abs(elev_hm_2)

    return {'elev_hm': summed_moment}


class MaxRudderServoMoment(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests the maximum hinge moment for the rudder servo pair."""

  def __init__(self, good_limit, bad_limit, severity):
    super(MaxRudderServoMoment, self).__init__(
        'Max Rudder Servo Moment', 'N.m', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['loads']

  def GetValue(self, output):
    return output['rud_hm_max']

  def GetOutput(self, timeseries):
    return {'rud_hm_max': np.max(timeseries['rud_hm'])}

  def GetTimeSeries(self, params, sim, control):
    servo_torques = self._SelectTelemetry(sim, control,
                                          'servo_shaft_torques')
    if scoring_util.IsSelectionValid(servo_torques):
      rud_hm_1 = servo_torques[:, 8]
      rud_hm_2 = servo_torques[:, 9]
    else:
      # Servo_torques are returned as np.array([float('nan')]) for flight logs.
      rud_hm_1 = np.array([float('nan')])
      rud_hm_2 = np.array([float('nan')])

    summed_moment = np.abs(rud_hm_1) + np.abs(rud_hm_2)

    return {'rud_hm': summed_moment}


class MaxRotorInPlaneMoment(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if the limit on maximum rotor in-plane moment is met."""

  def __init__(self, good_limit, bad_limit, severity):
    super(MaxRotorInPlaneMoment, self).__init__(
        'Max Rotor In-plane Moment', 'N.m', good_limit, bad_limit, severity)
    # TODO: Pull stability tables based on rotor rev.
    path = 'database/m600/rotor_rev4_stability_tables.json'
    with open(os.path.join(makani.HOME, path), 'r') as f:
      lookup_tables = json.load(f)
      self._omegas = lookup_tables['omegas']
      self._v_freestreams = lookup_tables['v_freestreams']
      self._my_a_cw = np.array(lookup_tables['My_a_cw'])
      self._mz_a_cw = np.array(lookup_tables['Mz_a_cw'])
      self._my_a_ccw = np.array(lookup_tables['My_a_ccw'])
      self._mz_a_ccw = np.array(lookup_tables['Mz_a_ccw'])

  def GetSystemLabels(self):
    return ['loads', 'experimental']

  def GetValue(self, output):
    return output['rotor_moment_max']

  def GetOutput(self, timeseries):
    rotor_moments = timeseries['rotor_moments']
    m_max = 0.0
    for nr in rotor_moments:
      m_res = np.linalg.norm(rotor_moments[nr], axis=1)
      if m_res.size != 0:
        m_max = max(m_max, m_res.max())
    return {'rotor_moment_max': m_max}

  def GetTimeSeries(self, params, sim, control):
    # TODO: This scoring function takes a while to evaluate, needs
    # some attention to reduce execution time.
    # Table look-up
    v = self._v_freestreams
    o = self._omegas
    my_cw_lookup = interpolate.RectBivariateSpline(v, o, self._my_a_cw.T,
                                                   kx=1, ky=1)
    mz_cw_lookup = interpolate.RectBivariateSpline(v, o, self._mz_a_cw.T,
                                                   kx=1, ky=1)
    my_ccw_lookup = interpolate.RectBivariateSpline(v, o, self._my_a_ccw.T,
                                                    kx=1, ky=1)
    mz_ccw_lookup = interpolate.RectBivariateSpline(v, o, self._mz_a_ccw.T,
                                                    kx=1, ky=1)

    rotors_dir = params['system_params']['rotors']['dir']
    rotors_axis = params['system_params']['rotors']['axis']
    # Note: rotors_inertia is rotational inertia of rotor and motor.
    rotors_inertia = params['system_params']['rotors']['I']

    param_names = ['airspeed', 'alpha', 'beta',
                   'rotor_speeds', 'rotor_gyro_moments', 'body_rates']
    (vapp, alpha, beta, rotor_omega, gyro_moments_xyz,
     body_rates) = self._SelectTelemetry(sim, control, param_names)

    # Transformation: Standard kite body (b) to standard hub fixed (h)
    # (x-forward, z-downward)
    dcm_b2h = np.array(geometry.AngleToDcm(0.0, np.deg2rad(-3.0), 0.0))

    # Transformation: Geometric hub fixed (gh, X-rearward, Z-upward)) to
    # standard hub fixed (h)
    dcm_gh2h = np.array(geometry.AngleToDcm(0.0, np.pi, 0.0))

    # Kite apparent speed components in (h)
    vk = vapp[np.newaxis].T * np.matmul(dcm_b2h, np.transpose(
        np.array([[-np.cos(alpha)*np.cos(beta)],
                  [-np.sin(beta)],
                  [-np.sin(alpha)*np.cos(beta)]]), (2, 0, 1)))[:, :, 0]

    # Rotor apparent speed in spherical coordinates
    va = np.linalg.norm(vk, axis=1)
    a = -np.arctan2(np.hypot(vk[:, 1], vk[:, 2]), vk[:, 0])
    t = -np.arctan2(vk[:, 2], vk[:, 1])

    # Geometric wind aligned (gw) to geometric hub fixed (gh)
    # TODO: Vectorize this function.
    dcm_gw2gh = np.ndarray((len(t), 3, 3))
    for i in range(len(t)):
      dcm_gw2gh[i, :, :] = geometry.AngleToDcm(0.0, 0.0, t[i])

    # Gyroscopic moment components in (h)
    if scoring_util.IsSelectionValid(gyro_moments_xyz):
      gyro_moment_y = gyro_moments_xyz['y']
      gyro_moment_z = gyro_moments_xyz['z']
    else:
      angular_momentum_h = (rotors_inertia[0, :] * rotor_omega
                            * rotors_dir[0, :])
      axis = np.concatenate([rotors_axis['x'], rotors_axis['y'],
                             rotors_axis['z']])
      angular_momentum_b = np.multiply(
          np.transpose(angular_momentum_h[np.newaxis], (1, 0, 2)),
          axis[np.newaxis])
      body_omega = np.concatenate([[body_rates['x']], [body_rates['y']],
                                   [body_rates['z']]])
      gyro_moment_b = np.cross(angular_momentum_b,
                               np.transpose(body_omega[np.newaxis], (2, 1, 0)),
                               axis=1)
      gyro_moment_y = gyro_moment_b[:, 1, :]
      gyro_moment_z = gyro_moment_b[:, 2, :]

    gyro_moment = np.zeros((gyro_moment_y.shape[0], gyro_moment_y.shape[1],
                            3, 1))
    gyro_moment[:, :, 1, 0] = gyro_moment_y
    gyro_moment[:, :, 2, 0] = gyro_moment_z
    m_gyro = np.matmul(dcm_b2h, gyro_moment)[:, :, :, 0]

    v_freestream_in_range = np.logical_and(
        va >= np.min(self._v_freestreams),
        va <= np.max(self._v_freestreams))

    # Loop on 8 rotors
    m_totals = {}
    for nr in range(0, 8):
      rotor_omega_cur = rotor_omega[:, nr]

      omega_in_range = np.logical_and(
          rotor_omega_cur >= np.min(self._omegas),
          rotor_omega_cur <= np.max(self._omegas))

      in_range = np.logical_and(omega_in_range, v_freestream_in_range)

      # Table look-up
      if rotors_dir[0, nr] > 0:
        my_aero_w = (my_cw_lookup(va[in_range], rotor_omega_cur[in_range],
                                  grid=False) * a[in_range])
        mz_aero_w = (mz_cw_lookup(va[in_range], rotor_omega_cur[in_range],
                                  grid=False) * a[in_range])
      else:
        my_aero_w = (my_ccw_lookup(va[in_range], rotor_omega_cur[in_range],
                                   grid=False) * a[in_range])
        mz_aero_w = (mz_ccw_lookup(va[in_range], rotor_omega_cur[in_range],
                                   grid=False) * a[in_range])
      # Aerodynamic moment components in (h)
      m_aero_w = np.zeros((my_aero_w.shape[0], 3, 1))
      m_aero_w[:, 1, 0] = my_aero_w
      m_aero_w[:, 2, 0] = mz_aero_w
      m_aero = np.matmul(dcm_gh2h, np.matmul(dcm_gw2gh[in_range, :, :],
                                             m_aero_w))[:, :, 0]

      # Total resultant in-plane moment
      m_totals[nr] = m_aero + m_gyro[in_range, nr]

    return {
        'rotor_moments': m_totals
    }


class MaxWingBendingFailureIndex(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if the limit on maximum wing bending is met."""

  def __init__(self, good_limit, bad_limit, severity,
               aero_tension_limit=1.0, accel_limit=1.0, domega_limit=1.0):
    super(MaxWingBendingFailureIndex, self).__init__(
        'Max Wing Bending Failure Index', '-', good_limit, bad_limit, severity)
    self._aero_tension_limit = aero_tension_limit
    self._accel_limit = accel_limit
    self._domega_limit = domega_limit

  def GetSystemLabels(self):
    return ['loads']

  def GetValue(self, output):
    return output['wing_bending_failure_index']

  def GetOutput(self, timeseries):
    wing_bending_failure_indices = timeseries['wing_bending_failure_indices']
    return {
        'wing_bending_failure_index': np.max(wing_bending_failure_indices)
    }

  def GetTimeSeries(self, params, sim, control):
    # Exclude Perched, PilotHover, HoverAscend and HoverDescend flight modes.
    flight_mode_exclusion_list = ['kFlightModePilotHover', 'kFlightModePerched',
                                  'kFlightModeHoverAscend',
                                  'kFlightModeHoverDescend']
    flight_modes = []
    for name in _FLIGHT_MODE_HELPER.Names():
      if name not in flight_mode_exclusion_list:
        flight_modes.append(name)

    tension, wing_acc, omega_i, domega_i, time_i = (
        self._SelectTelemetry(sim, control,
                              ['tether_tension', 'wing_acc',
                               'body_rates', 'angular_acc', 'time'],
                              flight_modes=flight_modes))

    # Check if body rates data exist. These may not exist if the relevant
    # flight modes do not exist.
    if not (scoring_util.IsSelectionValid(omega_i) and
            scoring_util.IsSelectionValid(wing_acc)):
      return {'wing_bending_failure_indices': np.array(float('nan'))}

    # Create angular acceleration data for flight logs.
    if scoring_util.IsSelectionValid(domega_i):
      domega = domega_i
    else:
      domega = {'x': np.gradient(omega_i['x'], time_i),
                'y': np.gradient(omega_i['y'], time_i),
                'z': np.gradient(omega_i['z'], time_i)}

    wing_acc_z = wing_acc['z']
    ang_acc_x = domega['x']
    kite_mass = params['system_params']['wing']['m']

    # Scoring function is a linear interaction equation of aerodynamic lift
    # loads (adjusted for roll acceleration contribution) and inertial loads
    # relative to their respective limits. It returns a failure index (ratio
    # of interaction equation output to max allowable value, a value of 1.0
    # being the max allowable value). Source derivation here:
    # https://goo.gl/qUjCgy
    wing_bending_failure_indices = (
        (tension - kite_mass * wing_acc_z) /
        (self._aero_tension_limit
         * (1 - 0.484 * abs(ang_acc_x / self._domega_limit)))
        + wing_acc_z / self._accel_limit)

    return {
        'wing_bending_failure_indices': wing_bending_failure_indices
    }


class MaxTailMomentScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests if the tail moments exceed acceptable limits."""

  def __init__(self, axis, bad_lower_limit, good_lower_limit,
               good_upper_limit, bad_upper_limit, severity):
    super(MaxTailMomentScoringFunction, self).__init__(
        'Max wing-fuselage M%s moment' % (axis), 'kN-m',
        bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)

    self._axis = axis

    # TODO: Add fuselage contribution to tail aero tables.
    path = 'database/m600/m600_tail_aero_tables.json'
    with open(os.path.join(makani.HOME, path), 'r') as f:
      lookup_tables = json.load(f)
      self._alphas = lookup_tables['alphas']
      self._betas = lookup_tables['betas']
      self._d_elev = lookup_tables['d_elev']
      self._d_rudder = lookup_tables['d_rudder']
      self._cl_tail = np.array(lookup_tables['Cl_tail'])
      self._cm_tail = np.array(lookup_tables['Cm_tail'])
      self._cn_tail = np.array(lookup_tables['Cn_tail'])

  def GetSystemLabels(self):
    return ['loads']

  def GetValue(self, output):
    return np.array([output['moment_min'],
                     output['moment_max']])

  def GetOutput(self, timeseries):
    moment_junct = timeseries['wing_fuse_junction_moment']
    return {
        'moment_min': moment_junct.min(),
        'moment_max': moment_junct.max()
    }

  def GetTimeSeries(self, params, sim, control):
    # Exclude Perched, PilotHover, HoverAscend and HoverDescend flight modes.
    flight_mode_exclusion_list = ['kFlightModePilotHover', 'kFlightModePerched',
                                  'kFlightModeHoverAscend',
                                  'kFlightModeHoverDescend']
    flight_modes = []
    for name in _FLIGHT_MODE_HELPER.Names():
      if name not in flight_mode_exclusion_list:
        flight_modes.append(name)

    # Constants.
    rhoair = params['sim_params']['phys_sim']['air_density']
    g_g = params['system_params']['phys']['g_g'][0]
    gravity = np.array([g_g['x'],
                        g_g['y'],
                        g_g['z']])
    sref = params['system_params']['wing']['A']
    bref = params['system_params']['wing']['b']
    cref = params['system_params']['wing']['c']

    # Maintain backwards compatibility with RPX-09 and previous logs.
    wing_params_names = params['system_params']['wing'].dtype.names
    tail_params_names = ['m_tail', 'i_tail', 'tail_cg_pos',
                         'horizontal_tail_pos']
    if all(name in wing_params_names for name in tail_params_names):
      system_params = params['system_params'][0]
      m_tail = system_params['wing']['m_tail']
      i_tail = np.array(system_params['wing']['i_tail']['d'])
      tail_cg_pos_i = system_params['wing']['tail_cg_pos']
      tail_cg_pos = np.array([tail_cg_pos_i['x'],
                              tail_cg_pos_i['y'],
                              tail_cg_pos_i['z']])
      r_empennage_i = system_params['wing']['horizontal_tail_pos']
      r_empennage = np.array([r_empennage_i['x'],
                              r_empennage_i['y'],
                              r_empennage_i['z']])
    else:
      # Tail CG [m] in body coordinates (tail = tub + fuselage + empennage).
      # Source: full kite finite element model m600assy_sn2_fem_r13_s.fem
      tail_cg_pos = np.array([-5.139, 0.003, 0.181])
      # Tail mass [kg] (tail = tub + fuselage + empennage).
      # Source: full kite finite element model m600assy_sn2_fem_r13_s.fem
      m_tail = 189.2
      # Tail matrix of inertia [kg-m^2] about tail CG
      # (tail = tub + fuselage + empennage).
      # Source: full kite finite element model m600assy_sn2_fem_r13_s.fem
      i_tail = np.array([[152.20, -0.5964, 61.80],
                         [-0.5964, 1356.5, 0.7337],
                         [61.80, 0.7337, 1280.2]])
      # Position [m] of fuselage-empennage junction
      r_empennage = np.array([-6.776, 0.045, 0.8165])

    # "_i" is used to denote the as-imported variable.
    (acc_b_i, omega_i, domega_i, airspeed_i, alpha_i, beta_i,
     flaps_i, time_i, dcm_g2b_i) = self._SelectTelemetry(
         sim, control, ['wing_acc', 'body_rates', 'angular_acc', 'airspeed',
                        'alpha', 'beta', 'flaps', 'time', 'dcm_g2b'],
         flight_modes=flight_modes)

    # Check if body rates data exist. These may not exist if the relevant
    # flight modes do not exist.
    if not scoring_util.IsSelectionValid(omega_i):
      return {'wing_fuse_junction_moment': np.array([float('nan')])}

    vapp = np.array(airspeed_i)
    alpha = np.array(alpha_i)
    beta = np.array(beta_i)
    d_elev = flaps_i[:, 6]
    d_rud = flaps_i[:, 7]
    dcm_g2b = np.array(dcm_g2b_i)

    acc_b = np.array([acc_b_i['x'],
                      acc_b_i['y'],
                      acc_b_i['z']])
    # Add gravity vector to inertial acceleration.
    acc_b -= np.matmul(dcm_g2b, gravity).T

    omega = np.array([omega_i['x'],
                      omega_i['y'],
                      omega_i['z']])

    # Create angular acceleration data for flight logs.
    if scoring_util.IsSelectionValid(domega_i):
      domega = np.array([domega_i['x'],
                         domega_i['y'],
                         domega_i['z']])
    else:
      domega = np.array([np.gradient(omega_i['x'], time_i),
                         np.gradient(omega_i['y'], time_i),
                         np.gradient(omega_i['z'], time_i)])

    # Account for motion of empennage relative to kite origin as it affects
    # apparent wind, alpha, beta.
    v_kite = apparent_wind_util.ApparentWindSphToCart(vapp, alpha, beta).T
    v_omega = np.cross(omega, r_empennage, axis=0)
    vapp, alpha, beta = (
        apparent_wind_util.ApparentWindCartToSph((v_kite + v_omega).T))

    # Tail aerodynamic moment lookup tables (tail = empennage only).
    cl_lookup = interpolate.interp2d(
        self._d_rudder, self._betas, self._cl_tail, kind='linear')
    cm_lookup = interpolate.interp2d(
        self._d_elev, self._alphas, self._cm_tail, kind='linear')
    cn_lookup = interpolate.interp2d(
        self._d_rudder, self._betas, self._cn_tail, kind='linear')

    # Initialize arrays filled out in for-loop
    cl = np.zeros(time_i.shape[0], dtype=float)
    cm = np.zeros(time_i.shape[0], dtype=float)
    cn = np.zeros(time_i.shape[0], dtype=float)

    # Tail aerodynamic moment at the wing/fuselage junction looped on all
    # datapoints. For cm reduce alpha by 3 deg for wing downwash at empennage.
    # This is true for higher wing CL's which helps capture the peak bending
    # moments, but is not necessarily valid for all data in the time series.
    for ni in range(0, time_i.shape[0]):
      cl[ni] = cl_lookup(np.rad2deg(d_rud[ni]), np.rad2deg(beta[ni]))
      cm[ni] = cm_lookup(np.rad2deg(d_elev[ni]), np.rad2deg(alpha[ni])-3.0)
      cn[ni] = cn_lookup(np.rad2deg(d_rud[ni]), np.rad2deg(beta[ni]))

    aero_moment_junct = 0.5 * rhoair * vapp**2.0 * sref * np.array(
        [cl * bref,
         cm * cref,
         cn * bref])

    # Inertial moment at the wing/fuselage junction.
    inertial_force_cg = m_tail * (
        acc_b
        + np.cross(domega, tail_cg_pos, axis=0)
        + np.cross(
            omega, np.cross(omega, tail_cg_pos, axis=0), axis=0))

    inertial_moment_cg = (
        np.dot(i_tail, domega)
        + np.cross(omega, np.dot(i_tail, omega), axis=0))

    inertial_moment_junct = inertial_moment_cg + np.cross(
        tail_cg_pos, inertial_force_cg, axis=0)

    # Total moment at the wing/fuselage junction.
    moment_junct = {
        'x': aero_moment_junct[0, :] - inertial_moment_junct[0, :],
        'y': aero_moment_junct[1, :] - inertial_moment_junct[1, :],
        'z': aero_moment_junct[2, :] - inertial_moment_junct[2, :]}

    return {'wing_fuse_junction_moment': moment_junct[self._axis] / 1000.0}
