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

"""Wing IMU mount sim parameters."""
from makani.config import mconfig


@mconfig.Config
def MakeParams():
  return {
      # Vibration frequency [Hz] and damping ratio [#] for shaping
      # white noise. The resulting signal is used to model the dynamic response
      # of the wing IMU mount.
      # Values derived from the flight dataset:
      # YM600-02/wing_recorder_logs/20160816-190236-flight05.h5.
      # The frequencies had to be adjusted due to the distortion from
      # a Tustin transformation in the filter implementation, and
      # were closer to: [20.0, 34.0, 20.0].
      'frequencies': [23.0, 75.5, 22.5],
      'damping_ratios': [0.03, 0.2, 0.04],

      # Scale [m/s^2] for the accelerometer vibration.
      'acc_scale': [4.0, 5.0, 8.7],

      # Scale [rad/s] for the gyroscope vibrations.
      'gyro_scale': [0.035, 0.1, 0.055]
  }
