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

"""GPS parameters."""

from makani.avionics.common import gps_receiver
from makani.config import mconfig
from makani.control import sensor_types
from makani.lib.python import dict_util
import numpy as np


@mconfig.Config
def MakeParams():
  pos_sigma_scales = [-1.0 for _ in range(sensor_types.kNumGpsSolutionTypes)]
  pos_sigma_scales[sensor_types.kGpsSolutionTypeNone] = 1000.0
  pos_sigma_scales[sensor_types.kGpsSolutionTypeRtkInt] = 0.02
  pos_sigma_scales[sensor_types.kGpsSolutionTypeRtkFloat] = 0.2
  pos_sigma_scales[sensor_types.kGpsSolutionTypeDifferential] = 3.0

  vel_sigma_scales = [-1.0 for _ in range(sensor_types.kNumGpsSolutionTypes)]
  vel_sigma_scales[sensor_types.kGpsSolutionTypeNone] = 1000.0
  vel_sigma_scales[sensor_types.kGpsSolutionTypeRtkInt] = 0.08
  vel_sigma_scales[sensor_types.kGpsSolutionTypeRtkFloat] = 0.2
  vel_sigma_scales[sensor_types.kGpsSolutionTypeDifferential] = 0.66

  # Each dropout or dropin is modeled as the next event in an inhomogeneous
  # Poisson process. The arrival time of this event follows an exponential
  # distribution.
  #
  # If \lambda is the rate parameter of the process, then 1/\lambda is both:
  #  - The time in which one arrival is expected from the Poisson process.
  #  - The mean of the corresponding exponential distribution.
  #
  # Below, we choose rate parameters with heuristic conditions of the form
  # "The event has probability p of occurring within t seconds." This is done
  # using the quantile function of the exponential distribution,
  #     t = Q(p) = -1/\lambda ln(1 - p).

  # Attitude dependent dropout rate [Hz/#].
  #
  # The total rate depends on antenna direction as
  #     antenna_dir_dropout_rate * ((antenna_dir_g.z + 1) / 2)^2.
  # Condition: When ignoring other effects, a GPS with its antenna pointed
  # straight downwards for 3 seconds has a 50% chance of experiencing a dropout.
  antenna_dir_dropout_rate = -1.0 / 3.0 * np.log(0.5)

  # Acceleration dependent dropout rate [Hz/(m/s^2)].
  #
  # The total rate depends on antenna acceleration as
  #     acc_dropout_rate * ||antenna_acc||.
  # Condition: When ignoring other effects, a GPS under an acceleration of
  # 5 m/s^2 for 10 seconds has a 10% chance of experiencing a dropout.
  acc_dropout_rate = -1.0 / 50.0 * np.log(0.9)

  # Dropin rate coefficients, [Hz] and [#], respectively.
  #
  # The total rate is based on the dropout rate,
  #     c_0 - c_1 * dropout_rate.
  #
  # If antenna_dir_dropout_rate is nonzero, coefficients are chosen so that
  # under no acceleration:
  #  - With a straight-downward antenna, dropin has a 1% chance of occurring
  #    within 3 seconds.
  #  - With a straight-upward antenna, dropin has a 99% chance of occurring
  #    within 3 seconds.
  # If antenna_dir_dropout_rate is zero, then only the first condition is used,
  # and c1 is zero.
  assert antenna_dir_dropout_rate >= 0.0
  c0 = -1.0 / 3.0 * np.log(0.01)
  if antenna_dir_dropout_rate > 0.0:
    dropin_rate_coeffs = [
        c0, 1.0 / antenna_dir_dropout_rate * (c0 + 1.0 / 3.0 * np.log(0.99))]
  else:
    dropin_rate_coeffs = [c0, 0.0]

  base_params = {
      # Normal sample time and "slow noise" sample time [s].
      'ts_rtcm_update_noise': 0.1,

      'antenna_dir_dropout_rate': antenna_dir_dropout_rate,
      'acc_dropout_rate': acc_dropout_rate,
      'dropin_rate_coeffs': dropin_rate_coeffs,

      'sigma_ratio': 8.3666,
      'pos_rtcm_update_noise_scale': 2.0,
      'vel_rtcm_update_noise_scale': 8.0,
      'sigma_per_dropout_rate': [150.0, 120.0, 180.0],

      # Estimated scale of position [m] and velocity [m/s] sigma values for
      # solution types for which we have data.
      'pos_sigma_scales': pos_sigma_scales,
      'vel_sigma_scales': vel_sigma_scales,
  }

  params = [None for _ in range(gps_receiver.GpsReceiverType.max_value + 1)]

  params[gps_receiver.GpsReceiverType.NONE.value] = (
      # Unused; required to use GpsReceiverType as an index.
      dict_util.MergeNestedDicts(base_params, {
          'ts': 0.0,
          'pos_delay': 0.0,
          'vel_delay': 0.0,
      }))

  params[gps_receiver.GpsReceiverType.NOV_ATEL.value] = (
      dict_util.MergeNestedDicts(base_params, {
          'ts': 0.05,

          # Update delay [s] for position and velocity.
          #
          # These are based on examining the pps_latency field in
          # FlightComputerSensor and the timestamp field in the
          # NovAtelLogBestXyz.
          'pos_delay': 0.05,
          'vel_delay': 0.075,
      }))

  params[gps_receiver.GpsReceiverType.SEPTENTRIO.value] = (
      dict_util.MergeNestedDicts(base_params, {
          'ts': 0.1,

          # Update delay [s] for position and velocity.
          #
          # These are based on examining the pps_latency field in
          # FlightComputerSensor and the timestamp field in
          # SeptentrioBlockPvtCartesian.
          'pos_delay': 0.1,
          'vel_delay': 0.1,
      }))

  return params
