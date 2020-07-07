#!/usr/bin/python
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

"""Produces a wind database for simulation from a flight log."""

import contextlib
import ctypes
import os
import sys

import gflags
import h5py
from makani.config import mconfig
from makani.control import system_types
from makani.lib.python import h5log_loader
from makani.sim.physics import wind_frame
import numpy as np
from scipy import signal

gflags.DEFINE_string('output_file', None,
                     'Output wind database.')

gflags.DEFINE_enum('log_type', 'wing', ('wing', 'cc', 'weather'),
                   'Type of log (wing, command center, or weather station)'
                   ' being read.')

gflags.DEFINE_enum('wind_source', 'est', ('est', 'ws'),
                   'Source of wind data:'
                   'the controller estimate or raw weather station.')

gflags.DEFINE_float('cutoff_freq', None,
                    'If specified, wind data will be low-pass filtered with '
                    'this cutoff frequency [Hz].')

FLAGS = gflags.FLAGS


def ExtractLogData(log_file_names, log_type, wind_source):
  """Extracts data from log files.

  Args:
    log_file_names: Names of HDF5 log files.
    log_type: 'wing', 'cc', or 'weather'.
    wind_source: 'est' or 'ws'

  Returns:
     (wind_vel_ws, duration):
       wind_vel_ws: [:, 3] array of wind velocity measurements [m/s] in the wind
           sensor frame.
       duration: Length of time [s] spanned by wind_vel_ws.

  Raises:
    ValueError: If log_type is invalid.
  """

  log = h5log_loader.H5LogLoader()
  log.Open(log_file_names)
  if log_type == 'weather':
    telemetry_path = 'PlatformSensorsA/GroundStationWeather'
  elif log_type == 'wing':
    telemetry_path = 'ControllerA/ControlDebug'
  elif log_type == 'cc':
    telemetry_path = 'ControllerA/ControlTelemetry'
  else:
    raise ValueError('Invalid value of --log_type: %s.' % FLAGS.log_type)

  t = log.capture_time[telemetry_path]

  if wind_source == 'ws':
    if log_type == 'weather':
      wind_vel = log[telemetry_path + '/wind/wind_velocity'].astype(np.float64)
    elif log_type == 'wing' or log_type == 'cc':
      # Each wind_ws point is stored as a Vec3. Use a view to reinterpret it as
      # an array.
      wind_vel = log[telemetry_path + '/control_input/wind_ws'].view(
          np.dtype(('>f8', 3)))

  elif wind_source == 'est':
    if log_type == 'weather':
      raise ValueError('Cannot use estimator wind_g from weather log.')
    elif log_type == 'wing' or log_type == 'cc':
      # Each wind_g point is stored as a Vec3. Use a view to reinterpret it as
      # an array.
      wind_vel = log[telemetry_path + '/state_est/wind_g/vector'].view(
          np.dtype(('>f8', 3)))

  else:
    raise ValueError('Invalid value of --wind_source: %s.' % wind_source)

  log.Close()
  return wind_vel, t[-1] - t[0]


def CalcWindVelMwAndMeanWind(wind_vel, wind_source):
  """Converts wind to the mean wind frame, and provides mean speed/direction.

  Args:
    wind_vel: Wind velocity [m/s] in the wind sensor frame or the ground frame,
    depending on wind_source (ws and est, respectively).
    wind_source: 'est' or 'ws'
  Returns:
    (wind_vel_mw, mean_wind_speed, mean_wind_dir), defined as:
      wind_vel_mw: Wind velocity [m/s] in the mean wind frame.
      mean_wind_speed: Mean wind speed [m/s] in the horizontal plane.
      mean_wind_dir: Direction [rad] of the mean wind.
  Raises:
    ValueError: The specified wind source was invalid.
  """

  # Transform from weather station to ground coordinates, if needed
  if wind_source == 'ws':
    system_params = mconfig.MakeParams('m600.system_params')
    assert (system_params['gs_model'] ==
            system_types.kGroundStationModelTopHat), (
                'This script assumes a top hat configuration.')
    dcm_ws2g = np.array(system_params['wind_sensor']['dcm_parent2ws']['d']).T
    wind_vel_g = np.dot(dcm_ws2g, wind_vel.T).T
  elif wind_source == 'est':
    wind_vel_g = wind_vel
  else:
    raise ValueError('Invalid value of --wind_source: %s.' % wind_source)

  # Calculate mean wind xy-speed and direction.
  mean_wind_g = np.mean(wind_vel_g, axis=0)
  mean_wind_speed = np.hypot(mean_wind_g[0], mean_wind_g[1])
  mean_wind_dir = np.arctan2(-mean_wind_g[1], -mean_wind_g[0])

  # Rotate wind velocity to the mean wind frame.
  dcm_mw2g = wind_frame.Mat3()
  wind_frame.CalcDcmMwToG(mean_wind_dir, ctypes.byref(dcm_mw2g))
  dcm_mw2g = np.array([row[:] for row in dcm_mw2g.d])
  wind_vel_mw = np.dot(dcm_mw2g.T, wind_vel_g.T).T

  return wind_vel_mw, mean_wind_speed, mean_wind_dir


def main(argv):
  def PrintUsageAndExit():
    print 'Usage: %s <log_1.h5> [log_2.h5] [log_3.h5] ...\n%s' % (
        os.path.basename(argv[0]), FLAGS)
    sys.exit(1)

  # Parse flags and validate inputs.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\n' % e
    PrintUsageAndExit()

  if len(argv) < 2:
    PrintUsageAndExit()

  log_files = argv[1:]

  wind_vel, duration = ExtractLogData(log_files, FLAGS.log_type,
                                      FLAGS.wind_source)
  wind_vel_mw, mean_wind_speed, mean_wind_dir = CalcWindVelMwAndMeanWind(
      wind_vel, FLAGS.wind_source)

  nt = wind_vel_mw.shape[0]

  # Filter measured wind if requested.
  if FLAGS.cutoff_freq is not None:
    nyquist_freq = float(nt) / duration / 2.0
    nyquist_frac = FLAGS.cutoff_freq / nyquist_freq
    assert 0.0 < FLAGS.cutoff_freq <= 1.0

    b, a = signal.butter(1, nyquist_frac, 'lowpass')
    for i in range(3):
      wind_vel_mw[:, i] = signal.lfilter(b, a, wind_vel_mw[:, i])

  # Each timepoint in the wind database defines a wind field in the mean wind
  # yz-plane (vw-plane) that advects downwind. This field can be inhomogeneous
  # in general.
  #
  # However, we only have one reading at the wind sensor to work with.
  # Consequently, we make this field constant at each timepoint in the simplest
  # way possible - by using a 2x2 grid that fills a "large" region.
  ny = 2
  nz = 2
  width = 5000.0
  height = 1000.0

  with contextlib.closing(h5py.File(FLAGS.output_file, 'w')) as f:
    f.create_dataset('num_t', data=np.array([nt]), dtype='<i4')
    f.create_dataset('num_y', data=np.array([ny]), dtype='<i4')
    f.create_dataset('num_z', data=np.array([nz]), dtype='<i4')

    f.create_dataset('mean_wind_speed', data=np.array([mean_wind_speed]))
    f.create_dataset('mean_wind_direction', data=np.array([mean_wind_dir]))

    f.create_dataset('duration', data=np.array([duration]))

    f.create_dataset('width', data=np.array([width]))
    f.create_dataset('height', data=np.array([height]))

    # The velocity field is uniform over y and z for each point in time.
    # Each dataset corresponds to an array over (t, y, z) in row-major
    # order, so each data point should be repeated ny*nz times.
    f.create_dataset('u', data=np.repeat(wind_vel_mw[:, 0], ny * nz),
                     compression='gzip', compression_opts=5)
    f.create_dataset('v', data=np.repeat(wind_vel_mw[:, 1], ny * nz),
                     compression='gzip', compression_opts=5)
    f.create_dataset('w', data=np.repeat(wind_vel_mw[:, 2], ny * nz),
                     compression='gzip', compression_opts=5)


if __name__ == '__main__':
  gflags.MarkFlagAsRequired('output_file')
  main(sys.argv)
