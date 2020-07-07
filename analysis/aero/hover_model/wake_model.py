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

"""Wake model functions."""

import numpy as np
from scipy import interpolate


def SumWakeVelocityIncrementsAtPoint(point_b, apparent_wind_b, params):
  """Sums velocity increments of all rotors.

  Sums the wake velocity increments from all the rotors.  Overlapping
  velocity increments are summed in quadrature according to momentum
  theory.  For example, in the extreme case where twice the thrust is
  obtained from the same disk area, the following relation holds:

       thrust_1 + thrust_2 = thrust_3
    => (vel_1^2 + vel_2^2) = vel_3^2

  Args:
    point_b: Point [m] in the body frame at which to evaluate the wake
        velocity.  This may be a single point or a (..., 3) ndarray.
    apparent_wind_b: Apparent wind vector [m/s] in the body frame.
        This may be a single vector or a (..., 3) ndarray.
    params: Parameters describing the rotor system including the rotor
        pitch [rad], position in body coordinates [m], thrust [N],
        radius [m], and the air density [kg/m^3].

  Returns:
    Total velocity increment in body coordinates represented as an
    (..., 3) ndarray.
  """
  if len(np.shape(point_b)) == 1:
    shape = np.shape(apparent_wind_b)[:-1]
    point_b = np.tile(point_b, shape + (1,))
  elif len(np.shape(apparent_wind_b)) == 1:
    shape = np.shape(point_b)[:-1]
    apparent_wind_b = np.tile(apparent_wind_b, shape + (1,))
  else:
    assert point_b.shape == apparent_wind_b.shape
    shape = np.shape(point_b)[:-1]

  dcm_b2r = np.array(
      [[np.cos(params['rotor_pitch']), 0.0, -np.sin(params['rotor_pitch'])],
       [0.0, 1.0, 0.0],
       [np.sin(params['rotor_pitch']), 0.0, np.cos(params['rotor_pitch'])]])

  # Rotate to rotor frame: apparent_wind_r = dcm_b2r * apparent_wind_b.
  apparent_wind_r = np.tensordot(apparent_wind_b, dcm_b2r.T, axes=1)
  total_vel_incr_r = np.zeros(shape + (3,))
  for rotor in params['rotors']:
    # Transform (rotate and translate) to rotor frame:
    #   point_r = dcm_b2r * (point_b - rotor_pos_b)
    rotor_pos_b = np.tile(rotor['pos'], shape + (1,))
    point_r = np.tensordot(point_b - rotor_pos_b, dcm_b2r.T, axes=1)
    wake_path_r = _CalcWakePath(rotor['thrust'], apparent_wind_r,
                                rotor['radius'], params['phys']['rho'])
    wake_vel_incr_r = _CalcWakeVelocityIncrementAtPoint(
        point_r, wake_path_r, rotor['thrust'], apparent_wind_b, rotor['radius'],
        params['phys']['rho'])
    total_vel_incr_r += np.sign(wake_vel_incr_r) * wake_vel_incr_r**2.0

  total_vel_incr_r = (np.sign(total_vel_incr_r)
                      * np.sqrt(abs(total_vel_incr_r)))

  # Rotate to body frame: total_vel_incr_b = dcm_b2r' * total_vel_incr_r.
  return np.tensordot(total_vel_incr_r, dcm_b2r, axes=1)


def _CalcWakePath(thrust, apparent_wind_r, rotor_radius, air_density,
                  num_steps=100, wake_length=15.0):
  """Calculates the path of the wake's core.

  Integrates the combined apparent wind and wake velocity increment to
  determine the path of the wake's core.

  Args:
    thrust: Thrust [N] of the rotor.
    apparent_wind_r: Apparent wind vector [m/s] in the rotor frame.
        This may be a single vector or a (..., 3) ndarray.
    rotor_radius: Radius of the rotor [m].
    air_density: Density of air [kg/m^3].
    num_steps: Number of points used to integrate the wake path.
    wake_length: Length [m] to integrate the wake path over.  This
        should be longer than the distance between the rotors and the
        elevator.

  Returns:
    Path of the wake [m] in rotor coordinates represented as a
    (...,) ndarray of interpolants for the y and z axes.
  """
  wake_path_r = np.zeros(np.shape(apparent_wind_r) + (num_steps,))

  x_jet = np.linspace(0.0, wake_length, num_steps)
  dx = x_jet[1] - x_jet[0]
  for i in range(1, len(x_jet)):
    wake_vel_incr = _CalcWakeVelocityIncrement((x_jet[i-1] + x_jet[i]) / 2.0,
                                               0.0, thrust, apparent_wind_r,
                                               rotor_radius, air_density)
    wake_vel_r = np.zeros(wake_vel_incr.shape + (3,))
    wake_vel_r[..., 0] = -wake_vel_incr

    total_vel_r = apparent_wind_r + wake_vel_r
    total_vel_x_r = total_vel_r[..., 0][..., np.newaxis]

    # Use Euler integration of the direction of the combined apparent
    # wind and wake velocity increment to determine the position of
    # the wake core.
    wake_path_r[..., -(i+1)] = (wake_path_r[..., -i]
                                - dx * total_vel_r / total_vel_x_r)

  shape = apparent_wind_r.shape[:-1]
  interpolants = np.empty(shape, dtype=object)
  for i in np.ndindex(*shape):
    interpolants[i] = interpolate.interp1d(
        wake_path_r[i + (0, slice(None))],
        wake_path_r[i + (slice(1, 3), slice(None))])

  return interpolants


def _CalcWakeVelocityIncrementAtPoint(point_r, wake_path_r, thrust,
                                      apparent_wind_r, rotor_radius,
                                      air_density):
  """Calculates the velocity increment from a single rotor at a point.

  Given a point and the path of the wake's core in rotor coordinates,
  this calculates the velocity increment from a single rotor.

  Args:
    point_r: Point [m] in the rotor frame.  This may be a single point
        or a (..., 3) ndarray.
    wake_path_r: Path of the wake [m] in rotor coordinates represented
        as a (...,) ndarray of interpolants for the y and z axes.
    thrust: Thrust [N] of the rotor.
    apparent_wind_r: Apparent wind vector [m/s] in rotor coordinates.
        This may be a single vector or a (..., 3) ndarray.
    rotor_radius: Radius of the rotor [m].
    air_density: Density of air [kg/m^3].

  Returns:
    Wake velocity increment vector [m/s] in rotor coordinates as a
    (..., 3) ndarray.
  """
  # Convert point_r to an ndarray, and create array of x values the
  # same shape as wake_path_r.
  point_r = np.array(point_r)
  if len(np.shape(point_r)) == 1:
    point_x_r = np.tile(point_r[0], wake_path_r.shape)
  else:
    point_x_r = point_r[..., 0]

  # Find distance to the wake core.
  wake_core_y_r = np.zeros(wake_path_r.shape)
  wake_core_z_r = np.zeros(wake_path_r.shape)
  for i in np.ndindex(*wake_path_r.shape):
    wake_core_y_r[i], wake_core_z_r[i] = wake_path_r[i](point_x_r[i])
  dist = np.sqrt((point_r[..., 1] - wake_core_y_r)**2.0
                 + (point_r[..., 2] - wake_core_z_r)**2.0)

  # Calculate the velocity increment.
  wake_vel_incr = _CalcWakeVelocityIncrement(-point_r[..., 0], dist, thrust,
                                             apparent_wind_r, rotor_radius,
                                             air_density)

  # Convert wake_vel_incr to a vector in the rotor frame.  The minus
  # sign accounts for the rotor frame axis being negative on the wake
  # side.
  return -np.dot(wake_vel_incr[..., np.newaxis], np.array([[1.0, 0.0, 0.0]]))


def _CalcWakeVelocityIncrement(x_jet, r_jet, thrust, apparent_wind_r,
                               rotor_radius, air_density):
  """Calculates the wake velocity increment.

  Calculates the wake of a rotor producing a specified thrust in a
  given apparent wind as a function of the axial and radial
  coordinates.  This wake model is based on the model described in
  Drela, ASWING 5.86 Technical Description -- Steady Formulation.

  TODO: Add wake swirl. This was found to be important for pylon forces.

  Args:
    x_jet: Axial coordinate [m], positive on wake side.
    r_jet: Radial coordinate [m].
    thrust: Thrust [N] produced by the rotor.
    apparent_wind_r: Local apparent wind [m/s] at the rotor in the
        rotor frame, represented as an ndarray where the last
        dimension is the vector.
    rotor_radius: Radius of the rotor [m].
    air_density: Density of air [kg/m^3].

  Returns:
    The velocity increment [m/s] at the specified axial and radial
    coordinates.
  """
  assert np.all(x_jet > 0.0)

  # Grab x velocities from apparent_wind_r, converting it to an
  # ndarray if it is not already one.
  freestream_velocity = -np.array(apparent_wind_r)[..., 0]

  rotor_area = np.pi * rotor_radius**2.0

  # Compute the velocity increment from the quadratic thrust relation
  # (Eq. 120).
  velocity_increment = (np.sqrt(freestream_velocity**2.0 +
                                2.0 * thrust / (air_density * rotor_area))
                        - freestream_velocity)

  # There is an initial contraction of the wake due to mass continuity
  # (Eq. 125).
  contracted_rotor_radius = rotor_radius * np.sqrt(
      (freestream_velocity + velocity_increment / 2.0)
      / (freestream_velocity + velocity_increment))

  # Empirical shear layer spreading rate [m/m] (Eq. 128).
  shear_spreading_rate = 0.11

  # Inner boundary spreading rate [m/m] (Eq. 126).
  inner_spreading_rate = (shear_spreading_rate
                          * (abs(velocity_increment)
                             / (freestream_velocity
                                + 1.0 * velocity_increment)))

  # Outer boundary spreading rate [m/m] (Eq. 127).
  outer_spreading_rate = (shear_spreading_rate
                          * (abs(velocity_increment)
                             / (freestream_velocity
                                + 0.5 * velocity_increment)))

  # Mixing distance [m] along the axial coordinate at which the inner
  # mixing boundaries merge at the center-line (Eq. 129).
  x_mix = contracted_rotor_radius / inner_spreading_rate

  # Outer mixing boundary [m] (Eq. 130).
  b_jet = contracted_rotor_radius + outer_spreading_rate * x_jet

  # Inner mixing boundary [m] (Eq. 131).
  c_jet = ((contracted_rotor_radius - inner_spreading_rate * x_jet)
           * (x_jet < x_mix))

  # Calculate the velocity increment along the center-line.
  # (Eq. 134).  This equation is derived from the requirement that the
  # momentum excess must be equal to the rotor thrust.
  k1 = (c_jet**2.0 + (9.0 / 10.0) * c_jet * (b_jet - c_jet)
        + (9.0 / 35.0) * (b_jet - c_jet)**2.0)
  k2 = (c_jet**2.0 + (243.0 / 385.0) * c_jet * (b_jet - c_jet)
        + (243.0 / 1820.0) * (b_jet - c_jet)**2.0)
  center_velocity_increment = (
      np.sqrt((0.25 * k1**2.0 / k2**2.0 * freestream_velocity**2.0)
              + (thrust / (air_density * np.pi * k2)))
      - 0.5 * (k1 / k2) * freestream_velocity)

  # Calculate the velocity increment [m/s] as a function of the radial
  # coordinate using Schlichting's asymptotic wake profile (Eq. 132).
  # Note that the absolute value here prevents NaNs from interfering
  # with the function branch.
  return (center_velocity_increment
          * (np.logical_and(b_jet > r_jet, r_jet >= c_jet)
             * (1.0 - abs((r_jet - c_jet) / (b_jet - c_jet))**1.5)**2.0
             + (c_jet > r_jet)))
