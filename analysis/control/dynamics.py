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

"""Python dynamics helpers.

Provides a simplified control-oriented model of the wing and tether.
"""
import collections
import copy

from makani.analysis.control import actuator_util
from makani.analysis.control import catenary
from makani.analysis.control import geometry
from makani.analysis.control import type_util
from makani.control import control_types
from makani.control import system_types
from makani.sim.physics import physics
import numpy as np

# Structure storing a force, moment, and position at which that force
# is applied.
ForceMomentPos = collections.namedtuple('ForceMomentPos',
                                        ['force', 'moment', 'pos'])

# Structure for storing forces and moment.
ForceMoment = collections.namedtuple('ForceMoment', ['force', 'moment'])

# Structure representing the inputs to the wing.
#   thrust: Motor thrust [N] (1-by-1 np.matrix).
#   motor_moment: Motor moments [N-m] (3-by-1 np.matrix).
#   flaps: Flaps [rad] (kNumFlaps-by-1 np.matrix).
#   wind_g: Wind speed [m/s] in ground coordinates (3-by-1 np.matrix).
WingInputs = type_util.MakeNamedVectorClass(  # pylint: disable=invalid-name
    'WingInputs', [('thrust', range(0, 1)),
                   ('motor_moment', range(1, 4)),
                   ('flaps', range(4, 4 + system_types.kNumFlaps)),
                   ('wind_g', range(4 + system_types.kNumFlaps,
                                    7 + system_types.kNumFlaps))])


class WingState(type_util.MakeStateClass(
    'WingState', [('omega_b', range(0, 3)),
                  ('dcm_g2b', range(3, 6)),
                  ('wing_vel_g', range(6, 9)),
                  ('wing_pos_g', range(9, 12))])):
  """Class representing the state of the wing.

  Attributes:
    omega_b: Body angular rates.
    dcm_g2b: Ground to body rotation DCM.  Increments in the DCM are represented
        by an Euler vector.
    wing_vel_g: Velocity of the wing in ground coordinates.
    wing_pos_g: Position of the wing in ground coordinates.
  """

  def Increment(self, tangent, step=1.0):
    """Return a state evolved from this state along a tangent direction.

    Args:
      tangent: A WingState.Tangent along which to move.
      step: A scaling of how far to move.

    Returns:
      A new WingState.
    """
    return WingState(omega_b=self.omega_b + step * tangent.domega_b,
                     dcm_g2b=(geometry.AxisToDcm(step * tangent.ddcm_g2b)
                              * self.dcm_g2b),
                     wing_vel_g=self.wing_vel_g + step * tangent.dwing_vel_g,
                     wing_pos_g=self.wing_pos_g + step * tangent.dwing_pos_g)

  def Difference(self, other_state):
    """Inverse operation of Increment with a step size of 1.0."""
    return WingState.Tangent(
        domega_b=other_state.omega_b - self.omega_b,
        ddcm_g2b=geometry.DcmToAxis(other_state.dcm_g2b * self.dcm_g2b.T),
        dwing_vel_g=other_state.wing_vel_g - self.wing_vel_g,
        dwing_pos_g=other_state.wing_pos_g - self.wing_pos_g)

  @type_util.RequireMatrixArguments(None, (3, 1))
  def CalcAerodynamicAngles(self, wind_g):
    """Calculates (v_rel, alpha, beta) from the current wing state.

    Args:
      wind_g: A 3-by-1 matrix storing the wind in g coordinates.

    Returns:
      A tuple (v_rel, alpha, beta).
    """
    return geometry.VelocitiesToAerodynamicAngles(
        self.dcm_g2b, self.wing_vel_g, wind_g)


@type_util.RequireMatrixArguments((3, 1), (3, 2), None, None)
def _CalcBridleKnotPos(tether_force_b, bridle_pos, bridle_y_offset,
                       bridle_radius):
  """Calculate the bridle knot position in body coordinates."""
  if np.linalg.norm(tether_force_b) == 0.0:
    tether_force_b = np.matrix([[0.0], [0.0], [1.0]])

  # Calculate the knot point location.  Here we use a bridle
  # coordinate system with its origin at the bridle pivot, its
  # y-axis pointing toward the starboard bridle point and its z-axis
  # pointed at the knot.
  bridle_coord_y = bridle_pos[:, 1] - bridle_pos[:, 0]
  bridle_coord_y /= np.linalg.norm(bridle_coord_y)

  bridle_coord_z = copy.copy(tether_force_b)
  bridle_coord_z -= bridle_coord_y * (np.transpose(bridle_coord_y)
                                      * tether_force_b)
  bridle_coord_z /= np.linalg.norm(bridle_coord_z)

  bridle_coord_origin = (bridle_pos[:, 1] + bridle_pos[:, 0]) * 0.5
  bridle_coord_origin[1] += bridle_y_offset

  return bridle_coord_origin + bridle_coord_z * bridle_radius


class MotorModel(object):

  # pylint: disable=unused-argument
  def CalcMotorForceMomentPos(self, v_rel, alpha, beta, omega_b,
                              thrust, motor_moment_r):
    raise NotImplementedError()


class PureForceMomentMotorModel(MotorModel):

  def __init__(self, rotor_params, pos_com_b):
    self._dcm_r2b = geometry.AngleToDcm(
        0.0, np.arctan2(rotor_params[0]['axis'][2],
                        rotor_params[0]['axis'][0]), 0.0)
    self._pos_com_b = np.matrix(pos_com_b).T

  # pylint: disable=unused-argument
  @type_util.RequireMatrixArguments(None, None, None, None, (3, 1), (1, 1),
                                    (3, 1))
  def CalcMotorForceMomentPos(self, v_rel, alpha, beta, omega_b,
                              thrust, motor_moment_r):
    # NOTE: This neglects motor reaction torques, and assumes that
    # MixRotors cancels the non-zero torque about the center-of-mass
    # that results from pure thrusting.
    motor_force_r = np.matrix([[thrust[0, 0]], [0.0], [0.0]])
    return ForceMomentPos(
        self._dcm_r2b * motor_force_r, self._dcm_r2b * motor_moment_r,
        self._pos_com_b)


class MotorMixerMotorModel(MotorModel):
  """Model the commanded thrust and moment by calling MixRotors."""

  def __init__(self, rotor_databases, air_density, weights, rotor_params,
               rotor_control_params, hover_flight_mode=False):
    self._dcm_r2b = geometry.AngleToDcm(
        0.0, np.arctan2(rotor_params[0]['axis'][2],
                        rotor_params[0]['axis'][0]), 0.0)
    self._rotor_databases = rotor_databases
    self._air_density = air_density
    self._weights = weights
    self._rotor_params = rotor_params
    self._rotor_control_params = rotor_control_params
    self._hover_flight_mode = hover_flight_mode

  @type_util.RequireMatrixArguments(None, None, (3, 1), (1, 1), (3, 1))
  def CalcRotorSpeeds(self, v_rel, omega_b, thrust, motor_moment_r):
    thrust_moment = {
        'thrust': thrust[0, 0],
        'moment': [motor_moment_r[i, 0] for i in range(3)]
    }

    return actuator_util.MixRotors(
        thrust_moment, self._weights, v_rel, [omega_b[i, 0] for i in range(3)],
        control_types.kStackingStateNormal, self._hover_flight_mode,
        self._air_density, self._rotor_params, self._rotor_control_params)

  @type_util.RequireMatrixArguments(None, None, None, None, (3, 1), (1, 1),
                                    (3, 1))
  def CalcMotorForceMomentPos(self, v_rel, alpha, beta, omega_b,
                              thrust, motor_moment_r):
    rotor_speeds = self.CalcRotorSpeeds(v_rel, omega_b, thrust, motor_moment_r)
    total_force = np.matrix(np.zeros((3, 1)))
    total_moment = np.matrix(np.zeros((3, 1)))

    v_rel_b = geometry.AerodynamicAnglesToRelativeVelocity(v_rel, alpha, beta)
    for i in range(rotor_speeds.shape[0]):
      rotor_speed = rotor_speeds[i, 0]
      if self._rotor_params[i]['dir'] == system_types.kPositiveX:
        direction = 1.0
      else:
        direction = -1.0
      rotor_velocity = direction * rotor_speed

      rotor_pos_b = np.matrix(self._rotor_params[i]['pos']).T
      v_freestream = np.dot(
          self._dcm_r2b[:, 0].T, v_rel_b + np.cross(omega_b.T, rotor_pos_b.T).T)
      v_freestream *= (1.0 - self._rotor_params[i]['local_pressure_coeff'])**0.5

      rotor_thrust = self._rotor_databases[i].CalcThrust(
          rotor_speed, v_freestream[0, 0], self._air_density)
      rotor_torque = direction * self._rotor_databases[i].CalcTorque(
          rotor_speed, v_freestream[0, 0], self._air_density)

      motor_force_b = self._dcm_r2b * np.matrix([[rotor_thrust], [0.0], [0.0]])

      lever_arm_moment_b = np.cross(
          rotor_pos_b.T, motor_force_b.T).T
      aero_moment_b = self._dcm_r2b * np.matrix([[rotor_torque], [0.0], [0.0]])
      gyro_moment_b = np.cross(
          self._rotor_params[i]['I'] * rotor_velocity * self._dcm_r2b[:, 0].T,
          omega_b.T).T

      total_force += motor_force_b
      total_moment += lever_arm_moment_b + aero_moment_b + gyro_moment_b

    return ForceMomentPos(
        total_force, total_moment, np.matrix(np.zeros((3, 1))))


class TetherForceModel(object):

  # pylint: disable=unused-argument
  def CalcBodyForce(self, dcm_g2b, wing_pos_g, wing_vel_g, wind_g):
    raise NotImplementedError()


class ConstantTetherForceModel(TetherForceModel):
  """Simple model of tether force as constant in ground coordinates."""

  def __init__(self, force_g):
    self.SetForce(force_g)

  # pylint: disable=unused-argument
  @type_util.RequireMatrixArguments(None, (3, 3), (3, 1), (3, 1), (3, 1))
  def CalcBodyForce(self, dcm_g2b, wing_pos_g, wing_vel_g, wind_g):
    """Calculate the tether force in body coordinates.

    Args:
      dcm_g2b: DCM rotating ground to body coordinates (3-by-3 np.matrix).
      wing_pos_g: Wing positon [m] in ground coordinates (unused
          3-by-1 np.matrix).
      wing_vel_g: Wing velocity [m/s] in ground coordinates (unused
          3-by-1 np.matrix).
      wind_g: Wind velocity [m/s] in ground coordinates (unused 3-by-1
          np.matrix).

    Returns:
      A 3-by-1 np.matrix storing the tether force in body coordinates.
    """
    return dcm_g2b * self._force_g

  @type_util.RequireMatrixArguments(None, (3, 1))
  def SetForce(self, force_g):
    """Update the force vector.

    Args:
      force_g: New tether force in ground coordinates.
    """
    self._force_g = copy.copy(force_g)


class SimpleSpringTetherForceModel(TetherForceModel):
  """Model of tether force as a simple spring, including bridle interactions."""

  def __init__(self, spring_const, system_params):
    tether_params = system_params['tether']
    wing_params = system_params['wing']

    self._spring_const = spring_const
    self._tether_length = tether_params['length']
    self._tether_drag_area = (0.25 * tether_params['section_drag_coeff']
                              * tether_params['length']
                              * tether_params['outer_diameter'])
    self._air_density = system_params['phys']['rho']
    self._bridle_pos = np.matrix(wing_params['bridle_pos']).T
    self._bridle_y_offset = wing_params['bridle_y_offset']
    self._bridle_radius = wing_params['bridle_rad']

  # pylint: disable=unused-argument
  @type_util.RequireMatrixArguments(None, (3, 3), (3, 1), (3, 1), (3, 1))
  def CalcBodyForce(self, dcm_g2b, wing_pos_g, wing_vel_g, wind_g):
    """Calculate the tether force in body coordinates.

    Args:
      dcm_g2b: DCM rotating ground to body coordinates (3-by-3 np.matrix).
      wing_pos_g: Wing positon [m] in ground coordinates (3-by-1 np.matrix).
      wing_vel_g: Wing velocity [m/s] in ground coordinates (3-by-1 np.matrix).
      wind_g: Wind velocity [m/s] in ground coordinates (3-by-1 np.matrix).

    Returns:
      A 3-by-1 np.matrix storing the tether force in body coordinates.
    """
    # This intentionally ignores the small offset from the GSG
    # position for simplicity.
    bridle_knot_b = _CalcBridleKnotPos(dcm_g2b * -wing_pos_g,
                                       self._bridle_pos,
                                       self._bridle_y_offset,
                                       self._bridle_radius)
    bridle_knot_g = wing_pos_g + dcm_g2b.T * bridle_knot_b
    tension = self._spring_const * (np.linalg.norm(bridle_knot_g)
                                    - self._tether_length)
    spring_force_g = -tension * bridle_knot_g / np.linalg.norm(bridle_knot_g)

    airspeed = np.linalg.norm(wing_vel_g - wind_g)
    drag = 0.5 * self._air_density * airspeed**2.0 * self._tether_drag_area
    drag_force_g = drag * (wind_g - wing_vel_g) / max(airspeed, 0.1)
    return dcm_g2b * (spring_force_g + drag_force_g)


class CatenaryTetherForceModel(TetherForceModel):
  """Model of tether force using catenary tension and rigid-rod drag."""

  def __init__(self, tether_params, gsg_pos_g, bridle_radius, g, air_density):
    """Create a catenary tether force model.

    Args:
      tether_params: TetherParams dictionary.
      gsg_pos_g: Position [m] of the GSG in the g-frame.
      bridle_radius: Bridle radius [m] of the kite.
      g: Gravitational acceleration [m/s^2].
      air_density: Air density [kg/m^3].
    """
    self._gsg_pos_g = np.matrix(np.reshape(gsg_pos_g, (3, 1)))
    self._length = tether_params['length'] + bridle_radius
    self._weight = tether_params['length'] * tether_params['linear_density'] * g
    self._section_drag_coeff = tether_params['section_drag_coeff']
    self._outer_diameter = tether_params['outer_diameter']
    self._air_density = air_density

  # pylint: disable=unused-argument
  @type_util.RequireMatrixArguments(None, (3, 3), (3, 1), (3, 1), (3, 1))
  def CalcBodyForce(self, dcm_g2b, wing_pos_g, wing_vel_g, wind_g):
    """Calculate the tether force in body coordinates.

    Args:
      dcm_g2b: DCM rotating ground to body coordinates (3-by-3 np.matrix).
      wing_pos_g: Wing positon [m] in ground coordinates (3-by-1 np.matrix).
      wing_vel_g: Wing velocity [m/s] in ground coordinates (3-by-1 np.matrix).
      wind_g: Wind velocity [m/s] in ground coordinates (unused 3-by-1
          np.matrix).

    Returns:
      A 3-by-1 np.matrix storing the tether force in body coordinates.
    """
    # Calculate catenary tension.
    horizontal_distance = (wing_pos_g[0, 0]**2.0 + wing_pos_g[1, 0]**2.0)**0.5
    vertical_distance = self._gsg_pos_g[2, 0] - wing_pos_g[2, 0]
    (h, v) = catenary.DimensionlessTensionsFromPoint(
        horizontal_distance / self._length,
        vertical_distance / self._length)
    azi = np.arctan2(wing_pos_g[1, 0], wing_pos_g[0, 0])
    tension_g = self._weight * np.matrix(
        [[-h * np.cos(azi)], [-h * np.sin(azi)], [v]])

    # Calculate drag reaction force on the wing. This is calculated by modeling
    # the tether as a rigid rod that is pinned at the GSG and rotating at fixed
    # angular velocity.
    #
    # Let
    #     CD  = cross-sectional drag coefficient
    #     s   = diameter of the rod
    #     L   = length of rod
    #     V   = velocity of the free end of the rod
    #     rho = air density
    # The drag dD along a segment of the rod with length dx at distance x from
    # the fixed end is
    #     dD(x) = 1/2 * rho * v(x)^2 * CD * s * dx.
    # Therefore,
    #     dD/dx = 1/2 * rho * v(x)^2 * CD * s.
    # The velocity of the segment is v(x) = x/L * V, so
    #     dD/dx = 1/2 * rho * x^2 / L^2 * V^2 * CD * s
    # From this, we obtain the differential moment about the fixed end:
    #     dM/dx = x * dD/dx = 1/2 * rho * x^3 / L^2 * V^2 * CD * s.
    # Integrating from x=0 to x=L yields the total moment due to drag,
    #     M = 1/8 * rho * L^2 * V^2 * CD * s.
    # Force at the fixed end induces no moment, so the drag moment must be
    # entirely balanced by a reaction force at the free end (i.e. the kite).
    # The magnitude of this force, R, is
    #     R = M / L = 1/8 * rho * L * V^2 * CD * s.
    #
    # Here, we treat the rod as extending from the GSG to the body frame origin,
    # and we use the wing velocity normal to the rod to determine V.
    gsg_to_wing_g = wing_pos_g - self._gsg_pos_g
    gsg_to_wing_dir_g = gsg_to_wing_g / np.linalg.norm(gsg_to_wing_g)

    normal_vel_g = (wing_vel_g
                    - float(wing_vel_g.T * gsg_to_wing_dir_g)
                    * gsg_to_wing_dir_g)
    normal_vel_mag = np.linalg.norm(normal_vel_g)
    drag_direction_g = -normal_vel_g / normal_vel_mag
    drag_g = (1.0 / 8.0 * self._air_density * np.linalg.norm(gsg_to_wing_g)
              * normal_vel_mag**2.0 * self._section_drag_coeff
              * self._outer_diameter * drag_direction_g)

    return dcm_g2b * (tension_g + drag_g)


class SwigAeroModel(object):
  """Swig import of the simulator aerodynamics model."""

  def __init__(self):
    self._aero = physics.Aero(physics.GetAeroSimParams())

  @type_util.RequireMatrixArguments(None, None, None, None,
                                    (system_types.kNumFlaps, 1), (3, 1),
                                    None)
  def CalcFMCoeff(self, alpha, beta, reynolds_number, flaps, omega_hat,
                  thrust_coeff):
    """Calculates force and moment coefficients from the Swig database."""
    omega_hat_vec3 = physics.Vec3()
    omega_hat_vec3.x = omega_hat[0, 0]
    omega_hat_vec3.y = omega_hat[1, 0]
    omega_hat_vec3.z = omega_hat[2, 0]
    flaps_vec = physics.VecWrapper(system_types.kNumFlaps)
    for i in range(system_types.kNumFlaps):
      flaps_vec.SetValue(i, flaps[i, 0])
    force_moment = physics.ForceMoment()
    self._aero.CalcForceMomentCoeff(alpha, beta, omega_hat_vec3.this,
                                    flaps_vec.GetVec(), reynolds_number,
                                    force_moment.this, thrust_coeff)
    force_moment_coeff = (np.matrix([[force_moment.force.x],
                                     [force_moment.force.y],
                                     [force_moment.force.z]]),
                          np.matrix([[force_moment.moment.x],
                                     [force_moment.moment.y],
                                     [force_moment.moment.z]]))
    return force_moment_coeff


class Wing(object):
  """Simplified model of the wing for control design.

  The Wing class stores parameters defined by the environment (air
  density, gravitational constant), a stateless tether force model,
  a stateless aerodynamic model, and a nominal orientation.

  It provides functions for calculating the ODEs that govern a 6-DOF
  rigid body model.
  """

  def __init__(self, system_params, sim_params, aero_model, motor_model,
               tether_force_model):
    """Constructs a Wing model.

    Args:
      system_params: A system parameters structure from mconfig.
      sim_params: A simulator parameters structure from mconfig.
      aero_model: A Python class implementing a function CalcFMCoeff.  See
          SwigAeroModel in this module as an example.
      motor_model: A MotorModel.
      tether_force_model: A TetherForceModel.
    """
    self._wing_area = system_params['wing']['A']
    self._wing_span = system_params['wing']['b']
    self._wing_chord = system_params['wing']['c']
    self._wing_mass = system_params['wing']['m']
    self._wing_inertia_matrix = np.matrix(system_params['wing']['I']['d'])
    self._pos_com_b = np.matrix(system_params['wing']['center_of_mass_pos']).T

    # Bridle parameters.
    self._bridle_pos = np.matrix(system_params['wing']['bridle_pos']).T
    self._bridle_y_offset = system_params['wing']['bridle_y_offset']
    self._bridle_radius = system_params['wing']['bridle_rad']

    # Physics parameters.
    self._g_g = np.matrix([[0.0], [0.0], [system_params['phys']['g']]])
    self._air_density = system_params['phys']['rho']
    self._dynamic_viscosity = sim_params['phys_sim']['dynamic_viscosity']

    self._aero_model = aero_model
    self._motor_model = motor_model
    self._tether_force_model = tether_force_model

  @type_util.RequireMatrixArguments(None, (3, 3))
  def _CalcGravityForceMomentPos(self, dcm_g2b):
    return ForceMomentPos(dcm_g2b * (self._wing_mass * self._g_g),
                          np.matrix(np.zeros((3, 1))), self._pos_com_b)

  @type_util.RequireMatrixArguments(None, None, None, None, (3, 1), (1, 1),
                                    (3, 1))
  def _CalcMotorForceMomentPos(self, v_rel, alpha, beta, omega_b,
                               thrust, motor_moment):
    """Calculates the motor forces and moments."""
    return self._motor_model.CalcMotorForceMomentPos(
        v_rel, alpha, beta, omega_b, thrust, motor_moment)

  @type_util.RequireMatrixArguments(None, (3, 3), (3, 1), (3, 1), (3, 1))
  def _CalcTetherForceMomentPos(self, dcm_g2b, wing_pos_g, wing_vel_g, wind_g):
    tether_force_b = self._tether_force_model.CalcBodyForce(dcm_g2b, wing_pos_g,
                                                            wing_vel_g, wind_g)
    return ForceMomentPos(tether_force_b, np.matrix(np.zeros((3, 1))),
                          _CalcBridleKnotPos(tether_force_b, self._bridle_pos,
                                             self._bridle_y_offset,
                                             self._bridle_radius))

  def CalcTetherForceG(self, state, inputs):
    return state.dcm_g2b.T * self._tether_force_model.CalcBodyForce(
        state.dcm_g2b, state.wing_pos_g, state.wing_vel_g, inputs.wind_g)

  def CalcTetherTensionRollPitch(self, state, inputs):
    tether_force_b = self._tether_force_model.CalcBodyForce(
        state.dcm_g2b, state.wing_pos_g, state.wing_vel_g, inputs.wind_g)
    return geometry.TetherForceCartToSph(tether_force_b)

  @type_util.RequireMatrixArguments(None, None, None, None, (3, 1),
                                    (system_types.kNumFlaps, 1), None)
  def CalcAeroForceMomentPos(self, v_rel, alpha, beta, omega_b, flaps,
                             thrust_coeff):
    """Calculate the aerodynamic force and moments on the wing.

    Args:
      v_rel: Airspeed [m/s].
      alpha: Angle-of-attack [rad].
      beta: Angle-of-sideslip [rad].
      omega_b: Wing body-rates [rad/s] (3-by-1 np.matrix).
      flaps: Flap deflections (kNumFlaps-by-1 np.matrix).
      thrust_coeff: Thrust coefficient [#] using wind turbine convention.

    Returns:
      (ForceMomentPos in body coordinates, force coeffs., moment coeffs.)
    """
    reynolds_number = ((v_rel * self._wing_chord * self._air_density)
                       / self._dynamic_viscosity)
    dynamic_pressure = 0.5 * self._air_density * v_rel**2.0

    length_scale = np.matrix([[self._wing_span],
                              [self._wing_chord],
                              [self._wing_span]])
    omega_hat = np.multiply(omega_b, length_scale) / (2.0 * v_rel)
    (cf, cm) = self._aero_model.CalcFMCoeff(alpha, beta, reynolds_number,
                                            flaps, omega_hat, thrust_coeff)

    return (ForceMomentPos(dynamic_pressure * self._wing_area * cf,
                           (dynamic_pressure * self._wing_area
                            * np.multiply(length_scale, cm)),
                           np.matrix(np.zeros((3, 1)))),
            cf, cm)

  def _BodyForceMomentPosToComForceMoment(self, force_moment_pos_list):
    force = np.matrix(np.zeros((3, 1)))
    moment = np.matrix(np.zeros((3, 1)))
    for force_moment_pos in force_moment_pos_list:
      force += force_moment_pos.force
      moment += force_moment_pos.moment
      moment += np.cross(force_moment_pos.pos - self._pos_com_b,
                         force_moment_pos.force, axis=0)
    return ForceMoment(force, moment)

  def CalcDeriv(self, state, inputs):
    """Calculates the derivative of the wing state vector.

    Args:
      state: A WingState.
      inputs: A WingInputs.

    Returns:
      A WingState.Tangent containing the derivative of the state.
    """
    euler_moment = np.cross(self._wing_inertia_matrix * state.omega_b,
                            state.omega_b, axis=0)

    v_rel, alpha, beta = state.CalcAerodynamicAngles(inputs.wind_g)

    # Fixing total thrust coefficient to 0.0 for this application.
    # NOTE: By accounting for the rotor wake effect on the tail,
    # we found that the synthesized gains yield worse flight quality than when
    # the effect is ignored (see b/110491871 for details).
    thrust_coeff = 0.0
    aero_force_moment_pos, _, _ = self.CalcAeroForceMomentPos(
        v_rel, alpha, beta, state.omega_b, inputs.flaps, thrust_coeff)

    force_moment_com = self._BodyForceMomentPosToComForceMoment([
        self._CalcGravityForceMomentPos(state.dcm_g2b),
        self._CalcMotorForceMomentPos(
            v_rel, alpha, beta, state.omega_b, inputs.thrust,
            inputs.motor_moment),
        self._CalcTetherForceMomentPos(state.dcm_g2b, state.wing_pos_g,
                                       state.wing_vel_g, inputs.wind_g),
        aero_force_moment_pos,
        ForceMomentPos(np.matrix(np.zeros((3, 1))), euler_moment,
                       np.matrix(np.zeros((3, 1))))
    ])

    # Calculate center-of-mass acceleration.
    accel_com_g = (state.dcm_g2b.T * force_moment_com.force) / self._wing_mass

    # Calculate body angular acceleration.
    omega_b_dot = np.matrix(np.linalg.solve(self._wing_inertia_matrix,
                                            force_moment_com.moment))

    wing_accel_g = accel_com_g - state.dcm_g2b.T * (
        np.cross(state.omega_b,
                 np.cross(state.omega_b, self._pos_com_b, axis=0), axis=0)
        + np.cross(omega_b_dot, self._pos_com_b, axis=0))

    return WingState.Tangent(domega_b=omega_b_dot, ddcm_g2b=state.omega_b,
                             dwing_vel_g=wing_accel_g,
                             dwing_pos_g=state.wing_vel_g)

  def CalcDVbCom(self, state, state_dot):
    """Calculates the rate of change of Vb for unit tests."""
    return (state.dcm_g2b * state_dot.dwing_vel_g
            - np.cross(state.omega_b, state.dcm_g2b * state.wing_vel_g, axis=0)
            + np.cross(state_dot.domega_b, self._pos_com_b, axis=0))

  def CalcEnergy(self, state):
    """Calculates energy of the rigid body model for unit tests."""
    wing_com_pos_g = state.wing_pos_g + state.dcm_g2b.T * self._pos_com_b
    wing_com_vel_g = (state.wing_vel_g
                      + (state.dcm_g2b.T
                         * np.cross(state.omega_b, self._pos_com_b, axis=0)))
    return ((0.5 * np.transpose(state.omega_b)
             * self._wing_inertia_matrix * state.omega_b)
            + (0.5 * self._wing_mass * np.transpose(wing_com_vel_g)
               * wing_com_vel_g)
            - self._wing_mass * np.transpose(self._g_g) * wing_com_pos_g)[0, 0]


def CalcLinearization(f, state, inputs, state_step_sizes, input_step_sizes):
  """Calculate the system matrices for the Wing model.

  Produces a linearized model:

    f(x + dx, u + du) ~ f(x) + A * dx + B * du

  where f is an arbitrary function, x is the wing state and u are
  the wing inputs.

  Args:
    f: A function mapping an n-by-1 np.matrix and an m-by-1 np.matrix to
        a n-by-1 np.matrix.
    state: An instance of a state class from type_util.
    inputs: An instance of a named vector from type_util.
    state_step_sizes: A vector of step sizes for the state.
    input_step_sizes: A vector of step sizes for the inputs.

  Returns:
    A tuple (A, B) where A and B are both of type np.matrix.
  """
  num_states = state.Tangent.GetDim()
  num_inputs = inputs.GetDim()
  num_outputs = f(state, inputs).shape[0]

  dfdx = np.matrix(np.zeros((num_outputs, num_states)))
  dfdu = np.matrix(np.zeros((num_outputs, num_inputs)))
  for i in range(num_states):
    h = state_step_sizes[i, 0]
    e = state.Tangent.FromVector(np.matrix([
        [1.0 if j == i else 0.0] for j in range(num_states)]))

    dfdx[:, i] = (f(state.Increment(e, step=h), inputs)
                  - f(state.Increment(e, step=-h), inputs)) / (2.0 * h)

  for i in range(num_inputs):
    h = input_step_sizes[i, 0]
    e = np.matrix([[1.0 if j == i else 0.0] for j in range(num_inputs)])
    dfdu[:, i] = (
        f(state, inputs.FromVector(inputs.ToVector() + h * e))
        - f(state, inputs.FromVector(inputs.ToVector() - h * e))) / (2.0 * h)

  return (dfdx, dfdu)
