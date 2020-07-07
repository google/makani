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

"""Basic geometry functions and wrappers for common/c_math/geometry.c."""
import ctypes

from makani.analysis.control import type_util
from makani.lib.python import c_helpers
from makani.sim import sim_types as m
import numpy as np


_rotation_order_helper = c_helpers.EnumHelper('RotationOrder', m,
                                              prefix='kRotationOrder')


@type_util.RequireMatrixArguments((3, 3))
def _MatrixToMat3(arg):
  mat3 = m.Mat3()
  for i in range(3):
    for j in range(3):
      mat3.d[i][j] = arg[i, j]

  return mat3


@type_util.RequireMatrixArguments((4, 1))
def _MatrixToQuat(arg):
  quat = m.Quat()
  quat.q0 = arg[0, 0]
  quat.q1 = arg[1, 0]
  quat.q2 = arg[2, 0]
  quat.q3 = arg[3, 0]
  return quat


@type_util.RequireMatrixArguments((3, 1))
def _MatrixToVec3(arg):
  vec3 = m.Vec3()
  vec3.x = arg[0, 0]
  vec3.y = arg[1, 0]
  vec3.z = arg[2, 0]
  return vec3


@type_util.RequireMatrixArguments((4, 1))
def QuatToDcm(q):
  """Thin wrapper around the C function QuatToDcm.

  Args:
    q: A 4-by-1 numpy.matrix representing [q0; q1; q2; q3] (see
        common/c_math/quaternion.h).

  Returns:
    A 3-by-3 numpy.matrix containing the equivalent DCM.
  """
  quat = m.Quat(q[0, 0], q[1, 0], q[2, 0], q[3, 0])
  mat3 = m.Mat3()
  m.QuatToDcm(ctypes.pointer(quat), ctypes.pointer(mat3))
  return np.matrix([[mat3.d[i][j] for j in range(3)]
                    for i in range(3)])


@type_util.RequireMatrixArguments((3, 3))
def DcmToQuat(dcm):
  """Thin wrapper around the C function DcmToQuat.

  Args:
    dcm: A 3-by-3 numpy.matrix representing a DCM.

  Returns:
    A 4-by-1 numpy.matrix representing [q0; q1; q2; q3] (see
    common/c_math/quaternion.h).
  """
  mat3 = _MatrixToMat3(dcm)
  quat = m.Quat()
  m.DcmToQuat(ctypes.pointer(mat3), ctypes.pointer(quat))
  return np.matrix([[quat.q0], [quat.q1], [quat.q2], [quat.q3]])


@type_util.RequireMatrixArguments((4, 1), None)
def QuatToAngle(q, order='ZYX'):
  """Thin wrapper around the C function QuatToAngle.

  Args:
    q: a 4-by-1 numpy.matrix representing [q0; q1; q2; q3] (see
        common/c_math/quaternion.h).
    order: Order of the rotations about different axes.

  Returns:
    A tuple (r1, r2, r3) of angles [rad].  Note that for the default rotation
    order this has the interpretation of (yaw, pitch, roll).
  """
  quat = m.Quat(q[0, 0], q[1, 0], q[2, 0], q[3, 0])

  order_value = _rotation_order_helper.Value(order.capitalize())

  r1 = ctypes.c_double()
  r2 = ctypes.c_double()
  r3 = ctypes.c_double()

  m.QuatToAngle(ctypes.pointer(quat), order_value, ctypes.pointer(r1),
                ctypes.pointer(r2), ctypes.pointer(r3))

  return (r1.value, r2.value, r3.value)


@type_util.RequireMatrixArguments((3, 1))
def AxisToDcm(axis):
  """Convert an Euler vector representation to a DCM.

  Args:
    axis: A 3-by-1 numpy.matrix representing an Euler vector.

  Returns:
    A 3-by-3 numpy.matrix representing a DCM.
  """
  vec3 = _MatrixToVec3(axis)
  quat = m.Quat()
  m.AxisToQuat(ctypes.pointer(vec3), ctypes.pointer(quat))
  return QuatToDcm(np.matrix(
      [[quat.q0], [quat.q1], [quat.q2], [quat.q3]]))


@type_util.RequireMatrixArguments((3, 3))
def DcmToAxis(dcm):
  """Thin wrapper around the C function DcmToAxis.

  Args:
    dcm: A 3-by-3 numpy.matrix representing a DCM.

  Returns:
    A 3-by-1 numpy.matrix representing a rotation axis.
  """
  quat = _MatrixToQuat(DcmToQuat(dcm))
  axis = m.Vec3()
  m.QuatToAxis(ctypes.pointer(quat), ctypes.pointer(axis))
  return np.matrix([[axis.x], [axis.y], [axis.z]])


@type_util.RequireMatrixArguments((3, 1))
def CrossMatrix(v):
  """Return a cross product matrix.

  Args:
    v: 3-by-1 numpy.matrix.

  Raises:
    MatrixShapeOrTypeException: If v is not a numpy.matrix or has a bad
        shape.

  Returns:
    A 3-by-3 anti-symmetric matrix A such that numpy.cross(v, u) = A * u for
    all 3-by-1 matrices u.
  """
  return np.matrix([[0.0, -v[2], v[1]],
                    [v[2], 0.0, -v[0]],
                    [-v[1], v[0], 0.0]])


def AngleToDcm(r1, r2, r3, order='ZYX'):
  """Convert Euler angles to a direct-cosine matrix.

  Args:
    r1: First angle [rad] (yaw in the default rotation order).
    r2: Second angle [rad] (pitch in the default rotation order).
    r3: Third angle [rad] (roll in the default rotation order).
    order: Optional argument specifying the rotation order and being one of:
        ['ZYZ', 'ZXY', 'ZXZ', 'YXZ', 'YXY', 'YZX', 'YZY', 'XYZ', 'XZY', 'XYX',
         'XZX', 'ZYX'].
        The default is 'ZYX'.

  Returns:
    An numpy.matrix containing a rotation matrix.
  """
  order_value = _rotation_order_helper.Value(order.capitalize())
  dcm = m.Mat3()
  m.AngleToDcm(r1, r2, r3, order_value, ctypes.pointer(dcm))
  return np.matrix([[dcm.d[i][j] for j in range(3)] for i in range(3)])


@type_util.RequireMatrixArguments((3, 3), None)
def DcmToAngle(dcm, order='ZYX'):
  """Convert a direct-cosine matrix to Euler angles.

  Args:
    dcm: 3-by-3 numpy.matrix containing a matrix from SO(3).
    order: Optional argument specifying the rotation order and being one of:
        ['ZYZ', 'ZXY', 'ZXZ', 'YXZ', 'YXY', 'YZX', 'YZY', 'XYZ', 'XZY', 'XYX',
         'XZX', 'ZYX'].
        The default is 'ZYX'.

  Raises:
    MatrixShapeOrTypeException: If dcm is not a numpy.matrix or has a
        bad shape.

  Returns:
    A tuple (r1, r2, r3) of angles [rad].  Note that for the default rotation
    order this has the interpretation of (yaw, pitch, roll).
  """
  c_dcm = _MatrixToMat3(dcm)

  order_value = _rotation_order_helper.Value(order.capitalize())
  r1 = ctypes.c_double()
  r2 = ctypes.c_double()
  r3 = ctypes.c_double()

  m.DcmToAngle(ctypes.pointer(c_dcm), order_value, ctypes.pointer(r1),
               ctypes.pointer(r2), ctypes.pointer(r3))

  return (r1.value, r2.value, r3.value)


@type_util.RequireMatrixArguments((3, 1), (3, 1))
def GetAngleDerivative(eulers_zyx, pqr):
  """Get the time derivative of Euler angles given body angular rates.

  Args:
    eulers_zyx: A 3-by-1 numpy.matrix containing the [roll, pitch, yaw] angles.
    pqr: A 3-by-1 numpy.matrix containing body angular rates.

  Raises:
    MatrixShapeOrTypeException: If an argument is not a numpy.matrix or
        has a bad shape.

  Returns:
    A 3-by-1 numpy.matrix containing d/dt [roll, pitch, yaw].
  """
  # See equation (1.3-22a) on page 27 of Stevens and Lewis. Aircraft
  # Control and Simulation. 2nd Ed.  Wiley, 2003
  h = np.matrix([
      [1.0,
       np.tan(eulers_zyx[1]) * np.sin(eulers_zyx[0]),
       np.tan(eulers_zyx[1]) * np.cos(eulers_zyx[0])],
      [0.0,
       np.cos(eulers_zyx[0]),
       -np.sin(eulers_zyx[0])],
      [0.0,
       np.sin(eulers_zyx[0]) / np.cos(eulers_zyx[1]),
       np.cos(eulers_zyx[0]) / np.cos(eulers_zyx[1])]])
  return h * pqr


@type_util.RequireMatrixArguments((3, 3), (3, 1), (3, 1))
def VelocitiesToAerodynamicAngles(dcm_g2b, v_g, wind_g):
  """Convert inertial and wind velocities into aerodynamic angles.

  Args:
    dcm_g2b: Direct-cosine matrix rotating ground coordinates to the body frame.
    v_g: Velocity of the body resolved in the ground frame.
    wind_g: Wind velocity in the ground frame.

  Raises:
    MatrixShapeOrTypeException: If an argument is not a numpy.matrix or
        has a bad shape.

  Returns:
    A tuple (v_rel, alpha, beta) containing the apparent wind speed
    (always non-negative), the angle-of-attack [rad] and side-slip angle [rad].
  """
  v_rel_b = dcm_g2b * (v_g - wind_g)  # Apparent wind in body coordinates.
  v_rel = np.linalg.norm(v_rel_b)
  alpha = np.arctan2(v_rel_b[2], v_rel_b[0])
  beta = np.arcsin(v_rel_b[1] / v_rel)
  return (v_rel, alpha[0, 0], beta[0, 0])


@type_util.RequireMatrixArguments(None, None, None)
def AerodynamicAnglesToRelativeVelocity(v_rel, alpha, beta):
  """Convert aerodynamic angles to relative velocity in body coordinates.

  Args:
    v_rel: Airspeed [m/s].
    alpha: Angle-of-attack [rad].
    beta: Angle-of-sideslip [rad].

  Returns:
    A 3-by-1 np.matrix storing the relative velocity in body coordinates.
  """
  return v_rel * np.matrix([
      [np.cos(alpha) * np.cos(beta)],
      [np.sin(beta)],
      [np.sin(alpha) * np.cos(beta)],
  ])


@type_util.RequireMatrixArguments((3, 1))
def TetherForceCartToSph(tether_force_b):
  """Converts a tether force from Cartesian to spherical coordinates.

  Args:
    tether_force_b: Tether force vector in body coordinates.

  Raises:
    MatrixShapeOrTypeException: If an argument is not a numpy.matrix or
        has a bad shape.

  Returns:
    Tuple (tension, tether_roll, tether_pitch) describing the tether
    force vector in spherical coordinates.  Tether pitch is the angle
    [rad] (positive about positive y-axis) between the bridle plane
    and the body z-axis.  Tether roll is the angle [rad] between the
    tether and the rotated z-axis (positive about rotated x-axis).
  """
  return (np.linalg.norm(tether_force_b),
          np.arctan2(-tether_force_b[1, 0],
                     np.hypot(tether_force_b[0, 0], tether_force_b[2, 0])),
          np.arctan2(tether_force_b[0, 0], tether_force_b[2, 0]))
