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

"""Simulated wing IMU sensor parameters."""

import copy

from makani.analysis.control import geometry
from makani.avionics.network import aio_labels
from makani.config import mconfig
from makani.system import labels
from makani.system import labels_util
import numpy as np
import scipy.linalg


@mconfig.Config(deps={
    'common_params': 'common.common_params',
    'imus': 'm600.wing_imus'
})
def MakeParams(params):
  wing_imus_sim = [{
      # The IMU data will actually be transmitted at a faster rate
      # than the system time-step.  The effect of dropped packets or
      # network latency can be simulated using the delay parameter
      # below.
      'ts': params['common_params']['ts'],
  } for _ in range(labels.kNumWingImus)]

  # Time delay [s] applied to all IMU channels.
  #
  # These delay values combine:
  # - 36.5/2400 seconds of group delay from a 73 tap, linear-phase
  #   filter in the IMU.
  # - 1/600 seconds of delay from the network assuming the IMU messages are
  #   transmitted at a rate of 600 Hz.
  #
  # Each IMU is assigned a delay in the range [0.016875, 0.018542] due to
  # time offsets in sampling.
  wing_imus_sim[0]['delay'] = 0.018542
  wing_imus_sim[1]['delay'] = 0.016875
  wing_imus_sim[2]['delay'] = 0.017708

  # Unit quaternion representing the rotation converting the
  # expected measurement coordinates to the actual measurement
  # coordinates.
  #
  # The typical axis-to-frame misalignment for each sensor triad is
  # 0.0175 [rad] (1 degree).  Installation in the FCUs will lead to
  # additional misalignment.  Here we pick a random 2 [deg] rotation
  # for the body to mass-balance-tube rotation, followed by three
  # random 1.5 degree rotations to represent misalignment of the
  # sensors mounting (both internal and external to the IMU).
  #
  # This does not capture triad-to-triad misalignment.
  #
  # TODO: Automate testing with correlated and uncorrelated
  # misalignments.
  q_individual_misalignment = [
      np.matrix([[0.9998], [0.0026], [0.0075], [0.0145]]),
      np.matrix([[0.9998], [0.0117], [0.0118], [0.0019]]),
      np.matrix([[0.9998], [0.0112], [0.0110], [0.0056]])
  ]

  # Matrix representing the common misalignment.
  dcm_parent2c = geometry.QuatToDcm(
      np.matrix([[0.999], [0.0014], [0.010], [0.0071]]))

  for i in range(labels.kNumWingImus):
    # The nominal DCM rotates from a common parent reference frame (c, ideally
    # body coordinates) to the IMUs expected coordinates (m).
    # The actual DCM includes individual IMU misalignment (m2a) and common frame
    # misalignment (parent2c).
    dcm_m2a = geometry.QuatToDcm(q_individual_misalignment[i])

    # The DCM's given in the IMU parameter file are not close enough
    # to SO(3) for DcmToQuat, so we use a polar decomposition to
    # project them.
    (dcm_nominal, _) = scipy.linalg.polar(
        params['imus'][i]['dcm_parent2m']['d'])

    dcm_parent2a = dcm_m2a * dcm_nominal * dcm_parent2c
    dcm_m2a = dcm_parent2a * np.transpose(dcm_nominal)
    q = geometry.DcmToQuat(dcm_m2a)
    wing_imus_sim[i]['q_m2actual'] = [q[j, 0] for j in range(4)]

  # Set common scale and limits for all IMUs.  Biases and noise levels
  # are set below.
  # TODO: Update scale factors for variation between
  # channels and IMUs.
  acc_sensor_comm = {
      'scale': 1.0,
      'bound_low': -18.0 * 9.80665,
      'bound_high': 18.0 * 9.80665,
      # TODO: Add a real value.
      'quantization': 0.0
  }

  for i in range(labels.kNumWingImus):
    wing_imus_sim[i]['acc_sensor'] = [copy.deepcopy(acc_sensor_comm)
                                      for _ in range(3)]

  gyro_sensor_comm = {
      'scale': 1.0,
      'bound_low': np.deg2rad(-450.0),
      'bound_high': np.deg2rad(450.0),
      # TODO: Add a real value.
      'quantization': 0.0
  }

  for i in range(labels.kNumWingImus):
    wing_imus_sim[i]['gyro_sensor'] = [copy.deepcopy(gyro_sensor_comm)
                                       for _ in range(3)]

  mag_sensor_comm = {
      'scale': 1.0,
      'bound_low': -2.5,
      'bound_high': 2.5,
      # TODO: Add a real value.
      'quantization': 0.0
  }

  for i in range(labels.kNumWingImus):
    wing_imus_sim[i]['mag_sensor'] = [copy.deepcopy(mag_sensor_comm)
                                      for _ in range(3)]

  # Bias random walk parameters.  These parameters were chosen to
  # match the Allan variance curves on the ADIS16488 data sheet.  The
  # curves were confirmed by comparison to a 3 hour recording made on
  # 2016-02-26.
  #
  # The bias is the sum of the constant term (defined in the SensorModelParams
  # structure) and two varying processes b_rw(t) and b_mp(t) defined by:
  #
  #   b_rw(t + dt) = b_rw(t) + sqrt(dt) * random_walk_scale * w_rw(t)
  #
  #   b_mp(t + dt) = exp(-dt * 2.0 * pi * markov_process_cutoff_freq) * b_mp(t)
  #                  + sqrt(dt) * markov_process_scale * w_mp(t)
  #
  # where w_rw(t) and w_mp(t) are uncorrelated, i.i.d. unit
  # variance, zero mean Gaussian processes defined at sample times.
  acc_bias_common = {
      'random_walk_scale': 0.000077,
      'markov_process_scale': 0.0002,
      'markov_process_cutoff_freq': 0.0016,
  }
  gyro_bias_common = {
      'random_walk_scale': 0.000004,
      'markov_process_scale': 0.0000175,
      'markov_process_cutoff_freq': 0.008
  }
  for i in range(labels.kNumWingImus):
    wing_imus_sim[i]['acc_bias'] = copy.deepcopy(acc_bias_common)
    wing_imus_sim[i]['gyro_bias'] = copy.deepcopy(gyro_bias_common)

  # The bias values are set below to be uniformly, randomly
  # distributed over an interval that is 1.5 times the bias
  # repeatability / initial bias error numbers from the ADIS16488A
  # data-sheet.

  # Typical accelerometer bias repeatability is +/- 0.157 [m/s^2].
  wing_imus_sim[0]['acc_sensor'][0]['bias'] = 0.1376
  wing_imus_sim[0]['acc_sensor'][1]['bias'] = 0.2164
  wing_imus_sim[0]['acc_sensor'][2]['bias'] = 0.0734

  wing_imus_sim[1]['acc_sensor'][0]['bias'] = -0.2187
  wing_imus_sim[1]['acc_sensor'][1]['bias'] = 0.1644
  wing_imus_sim[1]['acc_sensor'][2]['bias'] = 0.2044

  wing_imus_sim[2]['acc_sensor'][0]['bias'] = 0.0842
  wing_imus_sim[2]['acc_sensor'][1]['bias'] = 0.1214
  wing_imus_sim[2]['acc_sensor'][2]['bias'] = 0.1145

  # Typical gyroscope bias repeatability is +/- 0.0035 [rad/s].
  wing_imus_sim[0]['gyro_sensor'][0]['bias'] = -0.0011
  wing_imus_sim[0]['gyro_sensor'][1]['bias'] = 0.0016
  wing_imus_sim[0]['gyro_sensor'][2]['bias'] = -0.0035

  wing_imus_sim[1]['gyro_sensor'][0]['bias'] = 0.0022
  wing_imus_sim[1]['gyro_sensor'][1]['bias'] = -0.0049
  wing_imus_sim[1]['gyro_sensor'][2]['bias'] = -0.0023

  wing_imus_sim[2]['gyro_sensor'][0]['bias'] = -0.0048
  wing_imus_sim[2]['gyro_sensor'][1]['bias'] = -0.0042
  wing_imus_sim[2]['gyro_sensor'][2]['bias'] = 0.0034

  # Typical magnetometer initial bias error is +/- 0.015 [gauss].
  wing_imus_sim[0]['mag_sensor'][0]['bias'] = 0.0088
  wing_imus_sim[0]['mag_sensor'][1]['bias'] = -0.0082
  wing_imus_sim[0]['mag_sensor'][2]['bias'] = 0.0203

  wing_imus_sim[1]['mag_sensor'][0]['bias'] = -0.0209
  wing_imus_sim[1]['mag_sensor'][1]['bias'] = -0.0028
  wing_imus_sim[1]['mag_sensor'][2]['bias'] = -0.0053

  wing_imus_sim[2]['mag_sensor'][0]['bias'] = 0.0119
  wing_imus_sim[2]['mag_sensor'][1]['bias'] = 0.0133
  wing_imus_sim[2]['mag_sensor'][2]['bias'] = -0.0141

  # Magnetometer noise levels.
  for imu in range(labels.kNumWingImus):
    for channel in range(3):
      wing_imus_sim[imu]['mag_sensor'][channel]['noise_level'] = 0.0005

    wing_imus_sim[imu]['mag_noise'] = {
        # Glitches are present when the NovAtel GPS receiver is mounted on
        # the sensor interface board.  These are set to non-zero values
        # below.
        'glitch_period': 0.0,
        'glitch_duration': 0.0,
        'glitch_magnitudes': [0.0, 0.0, 0.0],
        # In addition to white noise spectrum, the magnetometers display low
        # amplitude 20 Hz, 40 Hz, and 50 Hz noise harmonics.  Here we set
        # conservatively high amplitudes for these harmonics and the noise
        # level.
        'harmonics_amplitudes': [0.001, 0.0006, 0.001],
        'harmonics_frequencies': [20.0, 40.0, 50.0],
    }

  # As of 2015-03-20 the presence of the NovAtel GPS receiver causes
  # the x and z axes of the corresponding IMU to see an increase in
  # noise and a periodic, single sample glitch every 1.05 seconds.
  for gps_label in range(labels_util.kNumWingGpsReceivers):
    if (labels_util.WingGpsReceiverLabelToGpsReceiverType(gps_label)
        == labels_util.kGpsReceiverTypeNovAtel):

      # NOTE: We don't currently model the wingtip IMU's.
      if gps_label not in [labels_util.kWingGpsReceiverCrosswind,
                           labels_util.kWingGpsReceiverHover]:
        continue

      fc_node = labels_util.WingGpsReceiverLabelToAioNode(gps_label)
      fc_label = aio_labels.FlightComputerAioNodeToFlightComputerLabel(fc_node)

      wing_imus_sim[fc_label]['mag_sensor'][0]['noise_level'] = 0.0015
      wing_imus_sim[fc_label]['mag_sensor'][2]['noise_level'] = 0.0015

      # Period [s] between glitches.
      wing_imus_sim[fc_label]['mag_noise']['glitch_period'] = 1.05
      wing_imus_sim[fc_label]['mag_noise']['glitch_duration'] = 0.02

      # Amplitude [Gauss] of the glitches.
      wing_imus_sim[fc_label]['mag_noise']['glitch_magnitudes'] = (
          [-0.005, -0.0025, -0.006])

  acc_sensor_noise_level_comm = [0.04, 0.04, 0.04]
  gyro_sensor_noise_level_comm = [7e-4, 7e-4, 7e-4]
  for i in range(labels.kNumWingImus):
    for j in range(3):
      wing_imus_sim[i]['acc_sensor'][j]['noise_level'] = (
          acc_sensor_noise_level_comm[j])
      wing_imus_sim[i]['gyro_sensor'][j]['noise_level'] = (
          gyro_sensor_noise_level_comm[j])
      wing_imus_sim[i]['stat_sensor'] = {
          'bias': 0.0,
          'scale': 1.0,
          'noise_level': 3.0,
          'bound_low': -1.0e9,
          'bound_high': 1.0e9,
          # TODO: Add a real value.
          'quantization': 0.0
      }

  return wing_imus_sim
