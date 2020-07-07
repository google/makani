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

"""This module abstracts the ctypes EstimatorReplay interface for python."""

import copy
import ctypes
import re
import sys

import gflags
from makani.analysis.control.replay import estimator_replay as replay
from makani.control import control_types
import numpy as np
import scipy.io as sio


def LoadMessages(filename,
                 path='/messages/kAioNodeControllerA/kMessageTypeControlDebug',
                 flight=None):
  """Load ControlTelemetry messages for reprocessing the estimator.

  Args:
    filename: A string describing the full path to the HDF5 kite log file.
    path: A string describing the HDF5 message path.
    flight: A string describing the flight name (e.g., rpx02) or None.

  Returns:
    A ctypes array of ControlTelemetry messages.
  """
  num_messages = replay.H5GetNumMessages(filename, path)
  messages = (replay.ControlTelemetry * num_messages)()
  replay.H5GetControlTelemetryMessages(filename, path, flight, num_messages,
                                       messages)
  return messages


class Estimator(object):
  """Python interface to EstimatorReplay."""

  def __init__(self, name=None):
    self.name = name
    self._params = copy.deepcopy(replay.GetControlParams().contents.estimator)
    self._fault_subsystems = set()
    self._outputs = []
    self.Reset()

  def _SetParamByPath(self, params, path, value):
    p = path.split('.', 1)
    if len(p) == 1 and hasattr(params, p[0]):
      setattr(params, p[0], value)
    elif len(p) > 1 and hasattr(params, p[0]):
      self._SetParamByPath(getattr(params, p[0]), p[1], value)
    else:
      raise ValueError('Invalid parameter path: ' + path)

  def UpdateParam(self, path, value):
    """Update a parameter in EstimatorParams.

    Args:
      path: A string describing the dot path to the parameter.
      value: A numerical value to assign.
    """
    self._SetParamByPath(self._params, path, value)

  def UpdateParams(self, params):
    """Update multiple parameters in EstimatorParams.

    Args:
      params: A dict mapping the parameter dot path to the assignment value.
    """
    for k, v in params.iteritems():
      self.UpdateParam(k, v)

  def SaturateIndices(self, messages, first_index=0, last_index=None):
    """Ensure message indices are within the bounds of the messages array.

    Args:
      messages: A ctypes array of ControlTelemetry messages.
      first_index: An integer describing the first index in the messages array.
      last_index: An integer describing the last index in the messages array.

    Returns:
      Tuple (first_index, last_index), where first_index and last_index are on
      interval [0, num_messages - 1].
    """
    num_messages = len(messages)
    if last_index is None:
      last_index = num_messages - 1
    first_index = max(0, min(first_index, num_messages - 1))
    last_index = max(0, min(last_index, num_messages - 1))
    return first_index, last_index

  def Reset(self):
    """Reinitialize the estimator."""
    self._outputs = []
    self.ResetState(replay.FlightMode(), replay.EstimatorState())
    replay.EstimatorReplayInit(ctypes.byref(self._params),
                               ctypes.byref(self._flight_mode),
                               ctypes.byref(self._state))

  def ResetState(self, flight_mode, state):
    """Reset the estimator state.

    Args:
      flight_mode: A ctypes FlightMode enum value.
      state: A ctypes EstimatorState structure.
    """
    self._flight_mode = copy.deepcopy(flight_mode)
    self._state = copy.deepcopy(state)

  def ResetMessages(self, original, first_index, last_index, messages):
    """Reset the fault state of messages on interval [first_index, last_index].

    Args:
      original: A ctypes array of unmodified ControlTelemetry messages.
      first_index: An integer describing the first index in the messages array.
      last_index: An integer describing the last index in the messages array.
      messages: A ctypes array of modified ControlTelemetry messages. This
          function updates interval [first_index, last_index].
    """
    first_index, last_index = self.SaturateIndices(original, first_index,
                                                   last_index)

    # Reset all faults in all subsystems back to their original value.
    subsystems = range(control_types.kNumSubsystems)
    labels = self._GetSubsystemLabelsArray(subsystems)
    replay.ClearControlTelemetryFaults(first_index, last_index, original,
                                       len(labels), labels, 0xFFFFFFFF,
                                       messages)

    # Set all possible faults in selected subsystems.
    set_fault_mask = 0xFFFFFFFF
    labels = self._GetSubsystemLabelsArray(self._fault_subsystems)
    replay.SetControlTelemetryFaults(first_index, last_index, len(labels),
                                     labels, set_fault_mask, messages)

  def _SubsystemsParameterToSet(self, subsystems):
    """Convert subsystems parameters to a set."""
    if not isinstance(subsystems, (set, list)):
      subsystems = [subsystems]
    return set(subsystems)

  def _GetSubsystemLabelsArray(self, subsystems):
    """Translate a list of subsystems to a ctypes SubsystemLabel array."""
    subsystems = self._SubsystemsParameterToSet(subsystems)
    labels = (replay.SubsystemLabel * len(subsystems))()
    for i, subsys in enumerate(subsystems):
      labels[i] = subsys
    return labels

  def ClearFaults(self, subsystems):
    """Clear faults for a list of subsystems."""
    subsystems = self._SubsystemsParameterToSet(subsystems)
    self._fault_subsystems -= subsystems

  def ClearAllFaults(self):
    """Clear all faults."""
    self.ClearFaults(range(control_types.kNumSubsystems))

  def SetFaults(self, subsystems):
    """Set faults for a list of subsystems."""
    subsystems = self._SubsystemsParameterToSet(subsystems)
    self._fault_subsystems |= subsystems

  def SetAllFaults(self):
    """Set faults in all controller subsystems."""
    self.SetFaults(range(control_types.kNumSubsystems))

  def ClearImuAccelGyroFaults(self):
    """Clear IMU accelerometer and gyro subsystem faults."""
    subsystems = [control_types.kSubsysImuAAcc,
                  control_types.kSubsysImuAGyro,
                  control_types.kSubsysImuBAcc,
                  control_types.kSubsysImuBGyro,
                  control_types.kSubsysImuCAcc,
                  control_types.kSubsysImuCGyro]
    self.ClearFaults(subsystems)

  def SetGpsCrosswindFaults(self):
    """Set faults in the GPS subsystem for the crosswind antenna."""
    subsystems = [control_types.kSubsysWingGpsCrosswindPos,
                  control_types.kSubsysWingGpsCrosswindVel]
    self.SetFaults(subsystems)

  def SetGpsHoverFaults(self):
    """Set faults in the GPS subsystem for the hover antenna."""
    subsystems = [control_types.kSubsysWingGpsHoverPos,
                  control_types.kSubsysWingGpsHoverVel]
    self.SetFaults(subsystems)

  def SetGpsPortFaults(self):
    """Set faults in the GPS subsystem for the port wingtip antenna."""
    subsystems = [control_types.kSubsysWingGpsPortPos,
                  control_types.kSubsysWingGpsPortVel]
    self.SetFaults(subsystems)

  def SetGpsStarboardFaults(self):
    """Set faults in the GPS subsystem for the starboard wingtip antenna."""
    subsystems = [control_types.kSubsysWingGpsStarPos,
                  control_types.kSubsysWingGpsStarVel]
    self.SetFaults(subsystems)

  def SetGpsFaults(self):
    """Set faults in all wing GPS subsystems."""
    self.SetGpsCrosswindFaults()
    self.SetGpsHoverFaults()
    self.SetGpsPortFaults()
    self.SetGpsStarboardFaults()

  def SetGsGpsFaults(self):
    """Set faults in the ground station GPS subsystem."""
    subsystems = [control_types.kSubsysGsCompass,
                  control_types.kSubsysGsGpsPos,
                  control_types.kSubsysGsGpsVel]
    self.SetFaults(subsystems)

  def SetGsgFaults(self):
    """Set faults in the ground side gimble subsystems."""
    subsystems = [control_types.kSubsysGsgAAzi,
                  control_types.kSubsysGsgAEle,
                  control_types.kSubsysGsgBAzi,
                  control_types.kSubsysGsgBEle]
    self.SetFaults(subsystems)

  def SetGlasFaults(self):
    """Set faults in the ground line angle sensing subsystems."""
    self.SetGsgFaults()
    self.SetLevelwindFaults()
    self.SetLoadcellFaults()
    self.SetPerchAziFaults()

  def SetLevelwindFaults(self):
    """Set faults in the levelwind subsystems."""
    subsystems = [control_types.kSubsysLevelwindEleA,
                  control_types.kSubsysLevelwindEleB]
    self.SetFaults(subsystems)

  def SetLoadcellFaults(self):
    """Set faults in the loadcell subsystems."""
    subsystems = [control_types.kSubsysLoadcellSensorPort0,
                  control_types.kSubsysLoadcellSensorPort1,
                  control_types.kSubsysLoadcellSensorStarboard0,
                  control_types.kSubsysLoadcellSensorStarboard1]
    self.SetFaults(subsystems)

  def SetMagFaults(self):
    """Set faults in the magnetometer subsystems."""
    subsystems = [control_types.kSubsysImuAMag,
                  control_types.kSubsysImuBMag,
                  control_types.kSubsysImuCMag]
    self.SetFaults(subsystems)

  def SetPerchAziFaults(self):
    """Set faults in the perch azimuth subsystems."""
    subsystems = [control_types.kSubsysPerchAziA,
                  control_types.kSubsysPerchAziB]
    self.SetFaults(subsystems)

  def SetPitotFaults(self):
    """Set faults in the pitot tube subsystems."""
    subsystems = [control_types.kSubsysPitotSensorHighSpeedAlpha,
                  control_types.kSubsysPitotSensorHighSpeedBeta,
                  control_types.kSubsysPitotSensorHighSpeedDynamic,
                  control_types.kSubsysPitotSensorHighSpeedStatic,
                  control_types.kSubsysPitotSensorLowSpeedAlpha,
                  control_types.kSubsysPitotSensorLowSpeedBeta,
                  control_types.kSubsysPitotSensorLowSpeedDynamic,
                  control_types.kSubsysPitotSensorLowSpeedStatic]
    self.SetFaults(subsystems)

  def SetWeatherFaults(self):
    """Set faults in the weather subsystems."""
    subsystems = [control_types.kSubsysWeather]
    self.SetFaults(subsystems)

  def SetWindFaults(self):
    """Set faults in the wind subsystems."""
    subsystems = [control_types.kSubsysWindSensor]
    self.SetFaults(subsystems)

  def Iterate(self, messages, first_index, last_index, states, estimates):
    """Iterate the state estimate from first_index to last_index.

    Args:
      messages: A ctypes array of ControlTelemetry messages to process.
      first_index: An integer describing the first index to process.
      last_index: An integer describing the last index to process.
      states: A ctypes array of EstimatorStates output states, equal in length
          to the messages array.
      estimates: A ctypes array of StateEstimate output estimates, equal in
          length to the messages array.
    """
    assert first_index <= last_index
    first_index, last_index = self.SaturateIndices(messages, first_index,
                                                   last_index)
    replay.EstimatorReplayIterateArray(ctypes.byref(self._params),
                                       first_index, last_index, messages,
                                       ctypes.byref(self._flight_mode),
                                       ctypes.byref(self._state), states,
                                       estimates)

  def IterateSegment(self, flight_mode_z1, state_z1, first_index, last_index,
                     messages, modified_messages, states, estimates):
    self.ResetState(flight_mode_z1, state_z1)
    self.ResetMessages(messages, first_index, last_index, modified_messages)
    self.Iterate(modified_messages, first_index, last_index, states, estimates)

  def ComputeOutputs(self, messages, first_index, last_index, states,
                     estimates):
    """Compute estimator outputs from first_index to last_index.

    Note that this function also stores the outputs in an array for each
    interval. Use property 'output' to access this array.

    Args:
      messages: A ctypes array of ControlTelemetry messages to process.
      first_index: An integer describing the first index to process.
      last_index: An integer describing the last index to process.
      states: A ctypes array of EstimatorStates output states, equal in length
              to the messages array.
      estimates: A ctypes array of StateEstimate output estimates, equal in
                 length to the messages array.

    Returns:
      An EstimatorOutput object.
    """
    output = EstimatorOutput(self.initializing, messages, states, estimates,
                             first_index, last_index)
    self._outputs.append(output)
    return output

  def ComputeErrorMetrics(self, references):
    return [ErrorMetrics(o, r) for o, r in zip(self._outputs, references)]

  @property
  def params(self):
    return self._params

  @property
  def flight_mode(self):
    return copy.deepcopy(self._flight_mode)

  @property
  def state(self):
    return copy.deepcopy(self._state)

  @property
  def initializing(self):
    return replay.GetEstimatorTelemetry().contents.initializing

  @property
  def outputs(self):
    return self._outputs

  @property
  def output_interval(self):
    return max([o.segment_time for o in self._outputs])


class EstimatorMetrics(object):
  """Base class to store estimator outputs over a given interval."""

  def __init__(self, messages, first_index, last_index):
    self._first_index = first_index
    self._last_index = last_index
    self._indices = range(first_index, last_index + 1)
    self._num_messages = last_index - first_index + 1
    self._valid = np.zeros(self._num_messages, dtype=bool)
    self._time = np.array([messages[i].time for i in self._indices])
    self._position = np.zeros((self._num_messages, 3))
    self._velocity = np.zeros((self._num_messages, 3))
    self._attitude = np.zeros((self._num_messages, 3, 3))
    self._gyro_bias = np.zeros((self._num_messages, 3))
    self._flight_modes = np.unique([messages[i].flight_mode
                                    for i in self._indices])

  def SetValid(self, valid):
    if isinstance(valid, bool):
      self._valid = valid * np.ones(self._num_messages, dtype=bool)
    else:
      self._valid = valid

  def SetPosition(self, position):
    self._position = position

  def SetVelocity(self, velocity):
    self._velocity = velocity

  def SetAttitude(self, attitude):
    self._attitude = attitude

  def SetGyroBias(self, gyro_bias):
    self._gyro_bias = gyro_bias

  @property
  def first_index(self):
    return self._first_index

  @property
  def last_index(self):
    return self._last_index

  @property
  def indices(self):
    return self._indices

  @property
  def num_messages(self):
    return self._num_messages

  @property
  def segment_time(self):
    return self._num_messages * replay.GetSystemParams().contents.ts

  @property
  def valid(self):
    return self._valid

  @property
  def position(self):
    return self._position

  @property
  def velocity(self):
    return self._velocity

  @property
  def attitude(self):
    return self._attitude

  @property
  def gyro_bias(self):
    return self._gyro_bias

  @property
  def time(self):
    return self._time

  @property
  def flight_modes(self):
    return self._flight_modes


class EstimatorOutput(EstimatorMetrics):
  """Store the estimator outputs."""

  def __init__(self, initializing, messages, states, estimates, first_index,
               last_index):
    super(EstimatorOutput, self).__init__(messages, first_index, last_index)
    self.SetPosition(self.ExtractPositionEstimate(estimates))
    self.SetVelocity(self.ExtractVelocityEstimate(estimates))
    self.SetAttitude(self.ExtractAttitudeEstimate(estimates))
    self.SetGyroBias(self.ExtractGyroBiasEstimate(states))
    self.SetValid(not initializing)

  def ExtractPositionEstimate(self, estimates):
    """Extract the estimator position estimates."""
    position = np.zeros((self.num_messages, 3))
    for i in xrange(self.num_messages):
      m = self.first_index + i
      position[i, 0] = estimates[m].Xg.x
      position[i, 1] = estimates[m].Xg.y
      position[i, 2] = estimates[m].Xg.z
    return position

  def ExtractVelocityEstimate(self, estimates):
    """Extract the estimator velocity estimates."""
    velocity = np.zeros((self.num_messages, 3))
    for i in xrange(self.num_messages):
      m = self.first_index + i
      velocity[i, 0] = estimates[m].Vg.x
      velocity[i, 1] = estimates[m].Vg.y
      velocity[i, 2] = estimates[m].Vg.z
    return velocity

  def ExtractAttitudeEstimate(self, estimates):
    """Extract the estimator attitude estimates."""
    attitude = np.zeros((self.num_messages, 3, 3))
    for i in xrange(self.num_messages):
      m = self.first_index + i
      for j in range(3):
        for k in range(3):
          attitude[i, j, k] = estimates[m].dcm_g2b.d[j][k]
    return attitude

  def ExtractGyroBiasEstimate(self, states):
    """Extract the estimator gyro bias estimates."""
    gyro_bias = np.zeros((self.num_messages, 3))
    for i in xrange(self.num_messages):
      m = self.first_index + i
      imu = states[m].nav.last_used_imu
      gyro_bias[i, 0] = states[m].nav.attitude[imu].filter.gyro_bias.x
      gyro_bias[i, 1] = states[m].nav.attitude[imu].filter.gyro_bias.y
      gyro_bias[i, 2] = states[m].nav.attitude[imu].filter.gyro_bias.z
    return gyro_bias


class ErrorMetrics(object):
  """Compute error between two EstimatorMetrics objects."""

  def __init__(self, a, b):
    """Instantiate an ErrorMetrics object.

    Args:
      a: An EstimatorMetrics object.
      b: An EstimatorMetrics object.
    """
    assert a.first_index == b.first_index
    assert a.last_index == b.last_index

    # Compute error over valid trajectory indices.
    ii = np.where(a.valid)[0]
    ii = ii[b.valid[ii]]

    # Compute position error.
    self._position_error = np.linalg.norm(a.position[ii] - b.position[ii],
                                          axis=1)

    # Compute velocity error.
    self._velocity_error = np.linalg.norm(a.velocity[ii] - b.velocity[ii],
                                          axis=1)

    # Compute attitude error as the norm of the small angle rotation vector.
    attitude_error = np.zeros((len(a.attitude), 3))
    for i in ii:
      dcm_a = np.matrix(a.attitude[i])
      dcm_b = np.matrix(b.attitude[i])
      delta = dcm_b.transpose() * dcm_a
      attitude_error[i, 0] = -delta[1, 2]
      attitude_error[i, 1] = delta[0, 2]
      attitude_error[i, 2] = -delta[0, 1]
    self._attitude_error = np.linalg.norm(attitude_error[ii], axis=1)

    # Compute gyro bias error.
    self._gyro_bias_error = np.linalg.norm(a.gyro_bias[ii] - b.gyro_bias[ii],
                                           axis=1)

    # Store time and flight modes. These quantities should be common.
    self._time = a.time[ii]
    self._flight_modes = a.flight_modes

  @property
  def flight_modes(self):
    return self._flight_modes

  @property
  def time(self):
    return self._time

  @property
  def position_error(self):
    return self._position_error

  @property
  def position_mae(self):
    return np.sum(np.abs(self._position_error)) / len(self._position_error)

  @property
  def position_maxe(self):
    return np.max(self._position_error)

  @property
  def position_rmse(self):
    return np.std(self._position_error)

  @property
  def velocity_error(self):
    return self._velocity_error

  @property
  def velocity_mae(self):
    return np.sum(np.abs(self._velocity_error)) / len(self._velocity_error)

  @property
  def velocity_maxe(self):
    return np.max(self._velocity_error)

  @property
  def velocity_rmse(self):
    return np.std(self._velocity_error)

  @property
  def attitude_error(self):
    return self._attitude_error

  @property
  def attitude_mae(self):
    return np.sum(np.abs(self._attitude_error)) / len(self._attitude_error)

  @property
  def attitude_maxe(self):
    return np.max(self._attitude_error)

  @property
  def attitude_rmse(self):
    return np.std(self._attitude_error)

  @property
  def gyro_bias_error(self):
    return self._gyro_bias_error

  @property
  def gyro_bias_mae(self):
    return np.sum(np.abs(self._gyro_bias_error)) / len(self._gyro_bias_error)

  @property
  def gyro_bias_maxe(self):
    return np.max(self._gyro_bias_error)

  @property
  def gyro_bias_rmse(self):
    return np.std(self._gyro_bias_error)


def ComputeCdf(error_metrics, attribute):
  x = np.sort(np.array([getattr(e, attribute) for e in error_metrics
                        if len(e.time)]))
  y = np.linspace(0.0, 1.0, len(x))
  return x, y


def ComputeEstimatorErrorCdfs(ref_estimator, test_estimators, t0=-float('inf'),
                              t1=float('inf')):
  """Compute error CDFs by comparing each test estimator against the reference.

  Args:
    ref_estimator: An Estimator object.
    test_estimators: A list of Estimator objects.
    t0: A float describing the minimum time to consider.
    t1: A float describing the maximum time to consider.

  Returns:
    A dict that maps the test estimator name to its error metric CDFs.
  """
  output = {}
  for est in test_estimators:
    error_metrics = est.ComputeErrorMetrics(ref_estimator.outputs)
    error_metrics = [o for o in error_metrics
                     if o.time.size > 0 and t0 <= np.min(o.time)
                     and np.max(o.time) <= t1]

    pos_maxe, prob = ComputeCdf(error_metrics, 'position_maxe')
    pos_mae, _ = ComputeCdf(error_metrics, 'position_mae')
    pos_rmse, _ = ComputeCdf(error_metrics, 'position_rmse')

    vel_maxe, _ = ComputeCdf(error_metrics, 'velocity_maxe')
    vel_mae, _ = ComputeCdf(error_metrics, 'velocity_mae')
    vel_rmse, _ = ComputeCdf(error_metrics, 'velocity_rmse')

    att_maxe, _ = ComputeCdf(error_metrics, 'attitude_maxe')
    att_mae, _ = ComputeCdf(error_metrics, 'attitude_mae')
    att_rmse, _ = ComputeCdf(error_metrics, 'attitude_rmse')

    bg_maxe, _ = ComputeCdf(error_metrics, 'gyro_bias_maxe')
    bg_mae, _ = ComputeCdf(error_metrics, 'gyro_bias_mae')
    bg_rmse, _ = ComputeCdf(error_metrics, 'gyro_bias_rmse')

    output[est.name] = {
        'name': est.name,
        'prob': prob,

        'pos_maxe': pos_maxe,
        'pos_mae': pos_mae,
        'pos_rmse': pos_rmse,

        'vel_maxe': vel_maxe,
        'vel_mae': vel_mae,
        'vel_rmse': vel_rmse,

        'att_maxe': att_maxe,
        'att_mae': att_mae,
        'att_rmse': att_rmse,

        'bg_maxe': bg_maxe,
        'bg_mae': bg_mae,
        'bg_rmse': bg_rmse,
    }
  return output


def SaveEstimatorErrorCdfsToMatFile(output, filename):
  # Replace invalid variable name characters with an underscore.
  mat = {re.sub(r'[^(A-Za-z0-9_)]', r'_', k): v for k, v in output.iteritems()}
  sio.savemat(filename, mat)


def ProcessEstimatorSegments(messages, increment, seg_length, ref_estimator,
                             test_estimators):
  """Periodically process test estimator segments from the reference estimator.

  This function helps understand the relative performance between two or more
  estimator configurations. It iterates the reference estimator forward in steps
  of 'increment' messages. At each increment, it iterates all estimators for
  'seg_length' messages from the current reference estimator state. Each
  estimator then stores its output trajectory within its own object structure.

  Args:
    messages: A ctypes array of ControlTelemetry messages.
    increment: An integer number of messages to iterate between each segment.
    seg_length: An integer number of messages to iterate for each segment.
    ref_estimator: An Estimator object.
    test_estimators: A list of Estimator objects.
  """
  assert increment > 0
  assert seg_length > 0

  # Allocate memory.
  num_messages = len(messages)
  states = (replay.EstimatorState * num_messages)()
  estimates = (replay.StateEstimate * num_messages)()
  modified_messages = copy.deepcopy(messages)
  num_segments = (num_messages + increment - 1) / increment

  # Set initial state and clear previous outputs.
  ref_estimator.Reset()
  for est in test_estimators:
    est.Reset()

  first_index_z1 = 0
  flight_mode_z1 = ref_estimator.flight_mode
  state_z1 = ref_estimator.state

  # Iterate for each increment.
  for segment in range(num_segments):
    first_index = segment * increment
    last_index = min(segment * increment + seg_length, num_messages) - 1

    # Advance reference estimator to the segment start.
    if first_index_z1 < first_index - 1:
      ref_estimator.IterateSegment(flight_mode_z1, state_z1, first_index_z1,
                                   first_index - 1, messages, modified_messages,
                                   states, estimates)
      first_index_z1 = first_index
      flight_mode_z1 = ref_estimator.flight_mode
      state_z1 = ref_estimator.state

    # Iterate reference estimator over the current segment.
    ref_estimator.IterateSegment(flight_mode_z1, state_z1, first_index,
                                 last_index, messages, modified_messages,
                                 states, estimates)
    ref_estimator.ComputeOutputs(modified_messages, first_index, last_index,
                                 states, estimates)

    # Iterate test configurations over the current segment.
    for est in test_estimators:
      est.IterateSegment(flight_mode_z1, state_z1, first_index, last_index,
                         messages, modified_messages, states, estimates)
      est.ComputeOutputs(modified_messages, first_index, last_index, states,
                         estimates)


def CreatePureInertialScenario(ref_estimator, name='Pure inertial'):
  est = copy.deepcopy(ref_estimator)
  est.name = name
  est.SetAllFaults()
  est.ClearImuAccelGyroFaults()
  return est


def CreateGpsDropoutScenario(ref_estimator, name='Full GPS dropout'):
  est = copy.deepcopy(ref_estimator)
  est.name = name
  est.SetGpsFaults()
  return est


def main(argv):
  """Implement a simple demo for computing error CDFs."""
  # Input/output flags.
  gflags.DEFINE_string('input_file', None, 'Full path to wing HDF5 log file.')
  gflags.MarkFlagAsRequired('input_file')
  gflags.DEFINE_string('output_file', None, 'Full path to output MAT file.')
  gflags.MarkFlagAsRequired('output_file')

  # Segment processing flags.
  gflags.DEFINE_integer('increment', 100,
                        'Integer number of messages between segments.')
  gflags.DEFINE_integer('seg_length', 1000,
                        'Integer number of messages in each segment.')

  # Evaluate segments over a specific time interval.
  gflags.DEFINE_float('start_time', -float('inf'),
                      'Start time to evaluate segment errors.')
  gflags.DEFINE_float('end_time', float('inf'),
                      'End time to evaluate segment errors.')

  # Override default parameters.
  gflags.DEFINE_list('params', [],
                     'A comma-separated list of param=value tokens, where '
                     'each param describes the dot path to a parameter in '
                     'EstimatorParams.')
  gflags.RegisterValidator('params',
                           lambda l: all(len(s.split('=')) == 2 for s in l),
                           message='Invalid key=value parameter syntax.')

  # Scenarios to process.
  gflags.DEFINE_bool('scenario_pure_inertial', False,
                     'Process pure inertial scenario.')
  gflags.DEFINE_bool('scenario_gps_dropout', False,
                     'Process GPS dropout scenario.')

  # Common faults to introduce.
  gflags.DEFINE_bool('fault_weather', False,
                     'Fault weather subsystems to avoid an assert when '
                     'reprocessing historical data.')
  gflags.DEFINE_bool('fault_glas', False, 'Fault GLAS subsystems.')

  # Specify flight for special handling.
  gflags.DEFINE_string('flight', None,
                       'Fix known issues associated with the given flight.')

  try:
    argv = gflags.FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], gflags.FLAGS)
    sys.exit(1)
  flags = gflags.FLAGS

  ref_estimator = Estimator('Reference')
  if flags.fault_glas:
    ref_estimator.SetGlasFaults()
  if flags.fault_weather:
    ref_estimator.SetWeatherFaults()
  for param_value in flags.params:
    param, value = param_value.split('=', 1)
    ref_estimator.UpdateParam(param, float(value))

  test_estimators = []
  if flags.scenario_pure_inertial:
    test_estimators.append(CreatePureInertialScenario(ref_estimator))
  if flags.scenario_gps_dropout:
    test_estimators.append(CreateGpsDropoutScenario(ref_estimator))

  messages = LoadMessages(flags.input_file, flight=flags.flight)
  ProcessEstimatorSegments(messages, flags.increment, flags.seg_length,
                           ref_estimator, test_estimators)
  output = ComputeEstimatorErrorCdfs(ref_estimator, test_estimators,
                                     t0=flags.start_time, t1=flags.end_time)
  SaveEstimatorErrorCdfsToMatFile(output, flags.output_file)


if __name__ == '__main__':
  main(sys.argv)
