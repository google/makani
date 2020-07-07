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

"""Model of the ground station."""

from makani.analysis.control import geometry

import numpy as np


def HomogeneousTransform(euler_angles, axis_order, translate=None):
  """Apply a homogeneous transformation."""
  # We only use the DCM part of the homogeneous transform matrix, but
  # the entire matrix is used for testing and debugging purposes.
  result = np.identity(4)
  if euler_angles is not None:
    result[:3, :3] = geometry.AngleToDcm(*euler_angles, order=axis_order)
    if translate is not None:
      result[0:3, 3] = translate[:]
  return result


class GroundStation(object):
  """Geometric model of the ground station."""

  def __init__(self, platform_pos_ned, drum_center_p, platform_azimuth,
               winch_azimuth_p, detwist_ele, drum_rotation, gsg_position_drum,
               detwist_angle, gsg_yoke, gsg_termination):
    # The platform center in NED coordinate system.
    self._platform_pos_ned = np.array(platform_pos_ned)
    self._drum_center_p = np.array(drum_center_p)
    self._gsg_position_drum = gsg_position_drum
    self._detwist_ele = detwist_ele
    self._SetPlatformAzimuth(platform_azimuth)
    self._SetDrumRotation(winch_azimuth_p, drum_rotation)
    self._SetDetwistRotation(detwist_angle)
    self._SetGsgAngles(gsg_yoke, gsg_termination)

  def _SetPlatformAzimuth(self, platform_azimuth):
    self._platform_azimuth = platform_azimuth
    self._ned_to_platform = HomogeneousTransform(
        [platform_azimuth, 0.0, 0.0], 'ZYX', self._platform_pos_ned)

  def _SetDrumRotation(self, winch_azi, drum_rotation):
    # The drum azimuth in platform coordinate system.
    self._winch_azimuth_p = winch_azi
    # The drum position.
    self._drum_rotation = drum_rotation

    self._platform_to_drum = HomogeneousTransform(
        [winch_azi, 0.0, self._drum_rotation], 'ZYX', self._drum_center_p)

  def _SetDetwistRotation(self, detwist_angle):
    self._detwist_angle = detwist_angle
    self._drum_to_gsg0 = HomogeneousTransform(
        [0.0, np.pi * 0.5 - self._detwist_ele, detwist_angle + np.pi], 'XYZ',
        self._gsg_position_drum)

  def _SetGsgAngles(self, gsg_yoke, gsg_termination):
    self._gsg_yoke = gsg_yoke
    self._gsg_termination = gsg_termination
    # The detwist angle
    # This transforms from gsg0 coordinate frame to gsg2, as defined in
    # go/makani-gs02-coordinates.
    # Rotation around the yoke needs to be applied before the
    # termination rotation.
    self._gsg0_to_gsg2 = HomogeneousTransform(
        [self._gsg_yoke, self._gsg_termination, 0.0], 'XYZ')

  def _DcmFromHomogeneousTransform(self, matrix):
    return matrix[:3, :3]

  def GetTetherAngle(self):
    """Compute the tether angle."""
    # TODO: Reuse the estimator value.
    dcm_ned_to_platform = self._DcmFromHomogeneousTransform(
        self._ned_to_platform)
    dcm_platform_to_drum = self._DcmFromHomogeneousTransform(
        self._platform_to_drum)
    dcm_drum_to_gsg0 = self._DcmFromHomogeneousTransform(
        self._drum_to_gsg0)
    dcm_gsg0_to_gsg2 = self._DcmFromHomogeneousTransform(
        self._gsg0_to_gsg2)

    dcm_ned_to_gsg2 = dcm_ned_to_platform
    for m in [dcm_platform_to_drum, dcm_drum_to_gsg0, dcm_gsg0_to_gsg2]:
      dcm_ned_to_gsg2 = np.dot(m, dcm_ned_to_gsg2)
    tether_dir_gsg2 = np.array([0.0, 0.0, -1.0])
    tether_dir_ned = np.dot(dcm_ned_to_gsg2.T, tether_dir_gsg2)
    tether_ele_ned = np.arctan2(
        -tether_dir_ned[2], np.linalg.norm(tether_dir_ned[:2]))
    return tether_ele_ned


if __name__ == '__main__':
  # This is for testing purpose only.
  gs = GroundStation(
      platform_pos_ned=[0, 0, -20.0],
      drum_center_p=[0.0, 0.0, -5.0],
      platform_azimuth=np.pi * 0.3,
      winch_azimuth_p=np.pi * 0.5,
      detwist_ele=22.5 / 180 * np.pi,
      drum_rotation=np.pi * 0.5,
      gsg_position_drum=[-2.1, 0.0, 1.348],
      detwist_angle=0.5 * np.pi,
      gsg_yoke=0.5 * np.pi,
      gsg_termination=0.5 * np.pi)

  print gs.GetTetherAngle()
