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

"""Propeller parameters.

This file unifies parameters associated with each propeller version.
"""

from makani.config import mconfig
from makani.control import system_types


@mconfig.Config
def MakeParams():
  """Makes parameters for propellers."""
  propellers = [None] * system_types.kNumPropVersions

  # The propellers follow the sign convention where kPositiveX means
  # that the propeller is rotating in a positive direction (i.e. right
  # hand rule) about the propeller axis, which is predominately in the
  # same direction as the body x-axis.
  propellers[system_types.kPropVersionRev1] = {
      # Propeller direction.
      'dir': system_types.kNegativeX,

      # Propeller moment-of-inertia [kg-m^2].
      'I': 0.9,

      # Propeller diameter [m].
      'D': 2.06,

      # Database filename.
      'database': {'name': 'm600/rotor_rev1_corrected.json'},

      # 3D database filename.
      # TODO: Include 3D database for correct rotor type.
      'database3d': {'name': 'm600/rotor_rev4_3d.json'},
  }

  propellers[system_types.kPropVersionRev1Trimmed] = {
      # Propeller direction.
      'dir': system_types.kNegativeX,

      # Propeller moment-of-inertia [kg-m^2].
      'I': 0.8,

      # Propeller diameter [m].
      'D': 1.81,

      # Database filename.
      'database': {'name': 'm600/rotor_rev1_trimmed_corrected.json'},

      # 3D database filename.
      # TODO: Include 3D database for correct rotor type.
      'database3d': {'name': 'm600/rotor_rev4_3d.json'},
  }

  propellers[system_types.kPropVersionRev2] = {
      # Propeller direction.
      'dir': system_types.kPositiveX,

      # Propeller moment-of-inertia [kg-m^2].
      'I': 1.0,

      # Propeller diameter [m].
      'D': 2.2,

      # Database filename.
      'database': {'name': 'm600/rotor_rev2_corrected.json'},

      # 3D database filename.
      # TODO: Include 3D database for correct rotor type.
      'database3d': {'name': 'm600/rotor_rev4_3d.json'},
  }

  propellers[system_types.kPropVersionRev3PositiveX] = {
      # Propeller direction.
      'dir': system_types.kPositiveX,

      # Propeller moment-of-inertia [kg-m^2].
      'I': 1.2,

      # Propeller diameter [m].
      'D': 2.32,

      # Database filename.
      'database': {'name': 'm600/rotor_rev3_corrected.json'},

      # 3D database filename.
      # TODO: Include 3D database for correct rotor type.
      'database3d': {'name': 'm600/rotor_rev4_3d.json'},
  }

  propellers[system_types.kPropVersionRev3NegativeX] = {
      # Propeller direction.
      'dir': system_types.kNegativeX,

      # Propeller moment-of-inertia [kg-m^2].
      'I': 1.2,

      # Propeller diameter [m].
      'D': 2.32,

      # Database filename.
      'database': {'name': 'm600/rotor_rev3_corrected.json'},

      # 3D database filename.
      # TODO: Include 3D database for correct rotor type.
      'database3d': {'name': 'm600/rotor_rev4_3d.json'},
  }

  propellers[system_types.kPropVersionRev4PositiveX] = {
      # Propeller direction.
      'dir': system_types.kPositiveX,

      # Propeller moment-of-inertia [kg-m^2]. See b/35872470.
      'I': 1.23,

      # Propeller diameter [m].
      'D': 2.3,

      # Database filename.
      'database': {'name': 'm600/rotor_rev4.json'},

      # 3D database filename.
      'database3d': {'name': 'm600/rotor_rev4_3d.json'},
  }

  propellers[system_types.kPropVersionRev4NegativeX] = {
      # Propeller direction.
      'dir': system_types.kNegativeX,

      # Propeller moment-of-inertia [kg-m^2].  See b/35872470.
      'I': 1.23,

      # Propeller diameter [m].
      'D': 2.3,

      # Database filename.
      'database': {'name': 'm600/rotor_rev4.json'},

      # 3D database filename.
      'database3d': {'name': 'm600/rotor_rev4_3d.json'},
  }

  assert all([p is not None for p in propellers])

  return propellers
