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

"""Apparent wind utilities."""

from makani.analysis.control import geometry
import numpy as np


def CalcDcmWToB(alpha, beta):
  """Calculate the wind-to-body DCM given alpha and beta.

  Args:
    alpha: Angle of attack [rad]
    beta:  Angle of sideslip [rad]

  Returns:
    dcm_w2b

  Compare to CalcDcmWToB in sim/physics/aero_frame.cc.
  """
  return geometry.AngleToDcm(-beta, alpha, 0.0, 'ZYX')


def ApparentWindCartToSph(apparent_wind):
  """Converts Cartesian apparent wind vector to spherical coordinates.

  Args:
    apparent_wind: Apparent wind vector [m/s] represented as a
        (..., 3) ndarray.

  Returns:
    Airspeed [m/s], angle-of-attack [rad], and sideslip angle [rad] as
    (...,) ndarrays.
  """
  airspeed = np.linalg.norm(apparent_wind, axis=-1)
  alpha = np.arctan2(-apparent_wind[..., 2], -apparent_wind[..., 0])
  beta = np.arctan2(-apparent_wind[..., 1],
                    np.sqrt(apparent_wind[..., 0]**2.0
                            + apparent_wind[..., 2]**2.0))
  return airspeed, alpha, beta


def ApparentWindSphToCart(airspeed, alpha, beta):
  """Converts apparent wind in spherical coordinates to Cartesian coordinates.

  Args:
    airspeed: Airspeed [m/s] represented as a (...,) ndarray.
    alpha: Angle-of-attack [rad] represented as a (...,) ndarray.
    beta: Sideslip angle [rad] represented as a (...,) ndarray.

  Returns:
    Cartesian apparent wind vector [m/s] represented as a (..., 3) ndarray.
  """
  x = -airspeed * np.cos(alpha) * np.cos(beta)
  y = -airspeed * np.sin(beta)
  z = -airspeed * np.sin(alpha) * np.cos(beta)
  return np.array([x.T, y.T, z.T]).T
