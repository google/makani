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


"""Motor angle calibration utility."""

import signal
import socket
import sys
import time

import gflags
from makani.avionics.common import aio
from makani.avionics.common import motor_util
from makani.avionics.common import pack_avionics_messages
from makani.avionics.motor import motor_client
from makani.avionics.motor.firmware import config_params
from makani.avionics.motor.firmware import params as motor_param_values
from makani.avionics.network import aio_labels
from makani.lib.python import c_helpers
import numpy as np
from PySide import QtCore
from PySide import QtGui
import scipy.optimize

# Constants.
EXPECTED_PACKETS_PER_POLE = 100
EXPECTED_NOISE_PACKETS = 63

motor_label_helper = c_helpers.EnumHelper('MotorLabel', aio_labels,
                                          prefix='kMotor')
angle_cal_helper = c_helpers.EnumHelper('MotorAngleCalMode',
                                        pack_avionics_messages)
motor_types = {name[0].lower(): name[1]
               for name in config_params.MotorType().iteritems()}


# Helper functions.
def GetMotorNames():
  """Returns a list of motor short names."""
  motor_names = [motor.lower() for motor in motor_label_helper.ShortNames()]
  motor_names.extend(['dyno_%s' % motor.lower()
                      for motor in motor_label_helper.ShortNames()])
  return motor_names


def GetMotorPairs():
  """Returns a mapping of stacked motor pairs."""
  motor_pairs = {}
  num_blocks = aio_labels.kNumMotors / 2

  for i_block in range(num_blocks):
    bottom = motor_label_helper.ShortName(
        motor_util.GetBottomMotorIndex(i_block)).lower()
    top = motor_label_helper.ShortName(
        motor_util.GetTopMotorIndex(i_block)).lower()
    motor_pairs[bottom] = top
    motor_pairs[top] = bottom

  return motor_pairs


# gflags.
gflags.DEFINE_boolean('save_data', False,
                      'Save raw data to angle_noise.csv and angle_cal.csv')
gflags.DEFINE_enum('motor_type', None, motor_types.keys(), 'Motor type.')
gflags.DEFINE_enum('target', None, GetMotorNames(), 'Target motor.')
gflags.DEFINE_enum('vbus_config', None, ['single', 'stacked'],
                   'Voltage bus configuration.')

gflags.MarkFlagAsRequired('motor_type')
gflags.MarkFlagAsRequired('target')
gflags.MarkFlagAsRequired('vbus_config')
FLAGS = gflags.FLAGS


class MainWindow(QtGui.QMainWindow):
  """The future main GUI window for motor data plotting.

  This will handle the setup and rendering of position sensor data during
  the calibration of a motor. It is included here to accomodate the use of
  QThreads and in preparation for adding GUI elements.
  """

  def __init__(
      self, num_pole_pairs, sensor_ratio, save_data, target, vbus_config):
    """Creates a new GUI window instance."""
    super(MainWindow, self).__init__()
    self._num_pole_pairs = num_pole_pairs
    self._sensor_ratio = sensor_ratio
    self._save_data = save_data
    self._target = target
    self._vbus_config = vbus_config
    self._motor_cmd_client = motor_client.MotorCommandClient()
    self._motor_runner = None
    self._data_listener = None
    self._HandleRunRequest()

  def _CalibrationStarted(self):
    """Update stdout after a calibration run begins."""
    self._PrintInfo(
        'Calibration started. Target: {:s}.'.format(self._target.upper()))

  def _CalibrationDone(self):
    """Updates stdout after a calibration run ends."""
    self._PrintInfo('Calibration done.')

  def _HandleRunRequest(self):
    """Handles user request to run a calibration."""
    run_time = self._num_pole_pairs * 2
    self._PrintInfo('Calibration run time: %d seconds.' % run_time)

    if self._target.lower().startswith('dyno_'):
      source = 'kAioNodeDynoMotor' + self._target[len('dyno_'):].capitalize()
    else:
      source = 'kAioNodeMotor' + self._target.capitalize()

    # Setup data collection and processing threads.
    self._data_listener = AngleDataListener(
        source, self._num_pole_pairs, self._sensor_ratio, self._save_data)
    self._motor_runner = MotorRunner(
        self._motor_cmd_client, self._target, self._vbus_config, run_time)

    self._data_listener.analysis_done.connect(self._CloseThreads)
    self._motor_runner.started.connect(self._CalibrationStarted)
    self._motor_runner.finished.connect(self._data_listener.RunAnalysis)
    self._motor_runner.finished.connect(self._CalibrationDone)

    # Start all threads.
    self._data_listener.start()
    self._motor_runner.start()

  def _PrintInfo(self, status):
    """Prints an info message to stdout."""
    print '[INFO] %s' % status

  def _PrintError(self, error):
    """Prints an error message to stdout."""
    print '[ERROR] %s' % error

  def _CloseThread(self, thread):
    """This method attempts to close a QThread."""
    thread.should_exit = True
    time.sleep(1)

    if thread.isRunning():
      self._PrintError(
          'Could not terminate {:s}'.format(thread.__class__))
      self.close()

  def _CloseThreads(self):
    """This method attempts to close all Qthreads before exiting the app."""
    self._PrintInfo('Closing all threads ...')

    if self._motor_runner:
      self._CloseThread(self._motor_runner)

    if self._data_listener:
      self._CloseThread(self._data_listener)

    self.close()


class AngleDataListener(QtCore.QThread):
  """A thread that listens for motor calibration.

  Attributes:
    analysis_done: A signal used to communicate to other threads that the
      AngleDataListener thread is done listening for and analyzing angle data.
    should_exit: A boolean indicating if thread should stop executing.
  """
  # These are class members that via some magic create
  # instance members of the same name.  Magic here:
  # http://qt-project.org/wiki/Signals_and_Slots_in_PySide
  analysis_done = QtCore.Signal()

  _NUM_RETRIES = 3

  def __init__(
      self, source, num_pole_pairs, sensor_ratio, save_data, parent=None):
    """Initializes a listener for position sensor data.

    Args:
      source: AIO node name for motor to receive packets from.
      num_pole_pairs: Number of pole pairs for given motor.
      sensor_ratio: Ratio of position sensor angle to motor electrical angle.
      save_data: A boolean indicating whether or not to save current run data.
      parent: An optional parent argument for QtCore.QThread.
    """
    QtCore.QThread.__init__(self, parent)

    self.should_exit = False
    self._num_pole_pairs = num_pole_pairs
    self._sensor_ratio = sensor_ratio
    self._save_data = save_data
    self._aio_client = aio.AioClient(['kMessageTypeMotorCalibration'],
                                     allowed_sources=[source], timeout=0.6)
    self._noise_packets = []
    self._angle_packets = []

  def _WriteSamplesToFile(self, file_name, samples):
    """Writes raw sensor data to a .csv file."""
    with open(file_name, 'w') as f:
      f.write('index,angle,a1,b1,a2,b2\n')
      for data in samples:
        f.write('{},{},{},{},{},{}\n'.format(
            data.index, data.angle, data.a1, data.b1, data.a2, data.b2))

  def _ErrorFunc(self, params, ydata, xdata):
    """Calculates the error for a given set of offsets."""
    angle_offset = params[0]
    sin_scale = params[1]
    sin_offset = params[2]
    cos_offset = params[3]

    sin_val = sin_scale * (xdata[:, 0] - sin_offset)
    cos_val = xdata[:, 1] - cos_offset

    angle = self._WrapAngle(
        self._sensor_ratio * np.arctan2(sin_val, cos_val) - angle_offset)

    error = self._WrapAngle(ydata - angle)

    return error

  def _WrapAngle(self, angle_data):
    """Wraps angle data."""
    angle_data[angle_data > np.pi] -= 2 * np.pi
    angle_data[angle_data < -np.pi] += 2 * np.pi
    return angle_data

  def _CalibrateData(self):
    """Calculates calibration scales and offsets."""

    angle = [data.angle for data in self._angle_packets]
    sin_val_1 = [data.a1 for data in self._angle_packets]
    cos_val_1 = [data.b1 for data in self._angle_packets]
    sin_val_2 = [data.a2 for data in self._angle_packets]
    cos_val_2 = [data.b2 for data in self._angle_packets]

    xdata_1 = np.array([sin_val_1, cos_val_1]).T
    xdata_2 = np.array([sin_val_2, cos_val_2]).T
    ydata = np.array(angle).T

    params_1, _ = scipy.optimize.leastsq(
        lambda p: self._ErrorFunc(p, ydata, xdata_1), [0, 1, 0, 0])
    params_2, _ = scipy.optimize.leastsq(
        lambda p: self._ErrorFunc(p, ydata, xdata_2), [0, 1, 0, 0])

    self.sensor1_fit = {'angle_offset': params_1[0],
                        'sin_scale': params_1[1],
                        'sin_offset': params_1[2],
                        'cos_offset': params_1[3]}

    self.sensor2_fit = {'angle_offset': params_2[0],
                        'sin_scale': params_2[1],
                        'sin_offset': params_2[2],
                        'cos_offset': params_2[3]}

  def _CheckPackets(self, num_received_packets, num_expected_packets,
                    packets_type):
    if num_received_packets == num_expected_packets:
      print '[PASS] Expected and Received: %d %s packets.' % (
          num_received_packets, packets_type)
      return True
    else:
      print '[FAIL] Expected: %d %s packets. Received: %d.' % (
          num_expected_packets, packets_type, num_received_packets)
      return False

  def RunAnalysis(self):
    """Obtains fit parameters from the position sensor raw data."""

    print 'Analyzing data ...'
    expected_angle_packets = self._num_pole_pairs * EXPECTED_PACKETS_PER_POLE

    angle_packets_check = self._CheckPackets(
        len(self._angle_packets), expected_angle_packets, 'angle')
    noise_packets_check = self._CheckPackets(
        len(self._noise_packets), EXPECTED_NOISE_PACKETS, 'noise')

    # Check that we received the right number of angle and noise packets.
    if angle_packets_check and noise_packets_check:
      self._CalibrateData()
      print self.sensor1_fit  # Second sensor calibration ignored for now.

      if self._save_data:
        self._WriteSamplesToFile('/tmp/angle_noise.csv', self._noise_packets)
        self._WriteSamplesToFile('/tmp/angle_cal.csv', self._angle_packets)

    self.analysis_done.emit()

  def run(self):
    """Event loop of the current thread. thread terminates on exit."""

    while not self.should_exit:
      try:
        (_, _, msg) = self._aio_client.Recv()

        if msg.mode == angle_cal_helper.Value('kMotorAngleCalModeNoise'):
          self._noise_packets.append(msg)
        elif msg.mode == angle_cal_helper.Value('kMotorAngleCalModeAngle'):
          self._angle_packets.append(msg)

      except socket.timeout:
        pass

    self._aio_client.Close()


class MotorRunner(QtCore.QThread):
  """A thread that executes commands to a motor for calibration.

  Attributes:
    should_exit: A boolean indicating if thread should stop executing.
  """

  def __init__(self, motor_cmd_client, target, vbus_config, run_time,
               parent=None):
    """Initializes a MotorRunner.

    Args:
      motor_cmd_client: An instance of the motor_client object used to send
        commands to the selected motor.
      target: Common name for the given motor target.
      vbus_config: Voltage bus configuration: 'single' or 'stacked'.
      run_time: Time in seconds to run the motor calibration.
      parent: An optional parent argument for QtCore.QThread.
    """

    QtCore.QThread.__init__(self, parent)

    self.should_exit = False
    self._motor_cmd_client = motor_cmd_client
    self._vbus_config = vbus_config
    self._run_time = run_time

    if self._vbus_config == 'single':
      self._targets = [target]
    elif self._vbus_config == 'stacked':
      try:
        motor_pairs = GetMotorPairs()
        self._targets = motor_pairs.keys()
        self._targets.remove(motor_pairs[target])
      except KeyError:
        print '[ERROR] Cannot select stacked configuration with a dyno motor.'
        sys.exit()
    else:
      print '[ERROR] %s is not a valid bus configuration.' % self._vbus_config
      sys.exit()

  def run(self):
    """Event loop of the current thread.

    Runs a calibration using an instance of the motor client.
    """
    self._motor_cmd_client.onecmd('clear_targets')
    time.sleep(0.1)

    if len(self._targets) == 1 and self._targets[0].lower().startswith('dyno_'):
      target = self._targets[0]
      self._motor_cmd_client.onecmd(
          'set_targets_dyno %s' % target[len('dyno_'):])
    else:
      self._motor_cmd_client.onecmd(
          'set_targets {}'.format(' '.join(self._targets)))

    print 'INFO: Clearing errors ...'
    time.sleep(1)
    self._motor_cmd_client.onecmd('clear_errors')
    time.sleep(1)

    self._motor_cmd_client.onecmd('arm')
    time.sleep(0.1)

    if self._vbus_config == 'stacked':
      self._motor_cmd_client.onecmd('set_omega 30')

    self._motor_cmd_client.onecmd('run %ds' % self._run_time)
    time.sleep(self._run_time + 0.1)
    self._motor_cmd_client.onecmd('stop')
    time.sleep(0.5)
    self._motor_cmd_client.onecmd('disarm')
    time.sleep(0.5)


def main(argv):
  # Get commandline arguments.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], FLAGS)
    sys.exit(1)

  motor_type = motor_types[FLAGS.motor_type.lower()]
  motor_params = motor_param_values.GetMotorParamsByType(motor_type).contents
  num_pole_pairs = int(motor_params.num_pole_pairs_elec)
  sensor_ratio = int(num_pole_pairs / motor_params.num_pole_pairs_sens)

  # Handle SIGINT, i.e. quit on Ctrl+C.
  signal.signal(signal.SIGINT, signal.SIG_DFL)

  # Start the application.
  app = QtGui.QApplication(argv)
  # In future `unused_win` will be used to show the UI i.e. `unused_win.show()`.
  unused_win = MainWindow(num_pole_pairs, sensor_ratio, FLAGS.save_data,
                          FLAGS.target, FLAGS.vbus_config)
  sys.exit(app.exec_())

if __name__ == '__main__':
  main(sys.argv)
