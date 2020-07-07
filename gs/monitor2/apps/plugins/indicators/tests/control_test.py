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

"""Test for web monitor indicators."""

import ctypes
import time
import unittest

from makani.avionics.common import pack_avionics_messages
from makani.avionics.linux.swig import aio_helper
from makani.avionics.linux.swig import aio_util
from makani.avionics.network import aio_labels
from makani.avionics.network import aio_node
from makani.avionics.network import message_type as aio_message_type
from makani.control import control_types
from makani.control import system_params
from makani.gs.monitor import monitor_params
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import control
from makani.gs.monitor2.apps.receiver import test_util
from makani.gs.monitor2.high_frequency_filters import filter_handlers
from makani.lib.python import c_helpers
from makani.lib.python import struct_tree
import mock

AIO_NODE_HELPER = c_helpers.EnumHelper('AioNode', aio_node)
MESSAGE_TYPE_HELPER = c_helpers.EnumHelper('MessageType', aio_message_type)
MONITOR_PARAMS = monitor_params.GetMonitorParams().contents


class TestIndicators(unittest.TestCase):

  @classmethod
  def setUp(cls):
    aio_util.InitFilters()

  def _SynthesizeControlTelemetry(self, sequence=0):
    return test_util.SynthesizeMessages(
        ['ControlTelemetry', 'ControlSlowTelemetry'], sequence)

  def _MockMessage(self, message, seq_num, timestamp, source_name,
                   message_type, pack_func, packed_size,
                   validate_callback=None):
    """Mock a message in the CVT.

    Args:
      message: The message to insert into the CVT.
      seq_num: Desired sequence number of the message.
      timestamp: Desired timestamp of the message.
      source_name: Name of the aio node, e.g. 'kAioNodeControllerA'.
      message_type: Name of the message type, e.g. 'kAioNodeControlTelemetry'.
      pack_func: Function to pack the message. Something like
          PackControlTelemetry or PackGroundStationWeatherMessage.
      packed_size: Number of bytes of the packed message.
      validate_callback: Optional function that takes two messages, the input
          and the reconstructed message, and evaluates to True if they are
          satisfactorily equal.
          E.g. lambda msg1, msg2: msg1.x == msg2.x and msg1.y == msg2.y.
    """
    source = AIO_NODE_HELPER.Value(source_name)
    message_enum = MESSAGE_TYPE_HELPER.Value(message_type)
    # Pack the message, get its raw bytestring, and use CvtPut to insert the
    # message into the CVT.
    packed_msg_ptr = (ctypes.c_ubyte * packed_size)()
    pack_func(ctypes.byref(message), 1, packed_msg_ptr)
    raw_message = str(buffer(packed_msg_ptr, 0, packed_size))
    aio_util.CvtPut(source, message_enum, seq_num, timestamp, raw_message)
    if validate_callback is not None:
      # Retrieve the message and validate it using the given validate callback.
      result = aio_util.CvtPeek(source, message_enum)
      assert len(result) == 3
      buf = result[0]
      reconstructed_message = aio_helper.UnpackMessage(buf, message_type)
      assert validate_callback(message, reconstructed_message)

  def testWingPosChart(self):
    self._messages = self._SynthesizeControlTelemetry()
    # Convert data from ctypes object to Python dicts.
    data = self._messages.Data(convert_to_basic_types=False)

    control_telemetry = data['ControlTelemetry']['ControllerA']
    control_telemetry.flight_mode = control_types.kFlightModePerched
    indicator = control.WingPosChart(common.FULL_COMMS_MODE)

    with mock.patch('makani.control.common.IsControlSystemRunning',
                    lambda x: True):
      xg = control_telemetry.state_est.Xg
      xg.x, xg.y, xg.z = 1.5, 2.5, 0.0
      _, _, stoplight, _ = indicator.Filter(self._messages)
      self.assertEqual(stoplight, stoplights.STOPLIGHT_NORMAL)

  def testFilterFrequency(self):
    """Test that the frequency in filtered_data is accurate."""
    window = filter_handlers.FILTER_FREQUENCY_WINDOW_SIZE

    # Mock some interval times and check that the average frequency is accurate.
    # The times are alternating .001 and .003.
    times = [.001 + (i%2)*.002 for i in xrange(window)]
    # We expect an average interval of .002, and 1.0/.002 is 500 Hz. We let the
    # error margin be large enough to distinguish between 1000 and 333 Hz.
    expected = 500
    error_margin = 250
    aio_util.RunAllFilters()
    for i in xrange(window):
      time.sleep(times[i])
      aio_util.RunAllFilters()
    filtered_data = aio_helper.GetFilteredData()
    freq = filtered_data.filter_frequency.frequency
    self.assertGreaterEqual(freq, expected - error_margin)
    self.assertLessEqual(freq, expected + error_margin)
    self.assertTrue(filtered_data.filter_frequency.valid)

  def testWindGustFilteredData(self):
    """Test that the wind gust filter behaves as expected.

    Validate that the filter only samples values at a frequency of 20Hz, and
    that the tke values returned are correct. This test runs in two phases.
    """
    message_type = 'kMessageTypeGroundStationWeather'
    message_size = pack_avionics_messages.PACK_GROUNDSTATIONWEATHERMESSAGE_SIZE
    source_name = 'kAioNodePlatformSensorsA'
    messages = {}
    # Allow a 1.5 Hz margin of error.
    error_margin = 1.5
    first_test_iterations = 50
    # Only half of the samples will be taken because the frequency at which we
    # insert values into the CVT and call RunAllFilters() is twice the rate
    # that the filter handler samples at.
    first_test_valid_iterations = first_test_iterations / 2
    second_test_iterations = 50
    gs_obj = pack_avionics_messages.GroundStationWeatherMessage()

    # Call aio_util.RunAllFilters() at a very fast frequency, and check that
    # the filter is sampling at ~20Hz. Also begin mocking velocities.
    message_frequency = 40
    delay = 1.0 / message_frequency
    desired_frequency = 20
    # Set up arbitrary velocity values to feed into the CVT.
    first_u_velocities = [10.0 + i % 2 for i in xrange(first_test_iterations)]
    first_v_velocities = [4.0 + 3 * (i % 2)
                          for i in xrange(first_test_iterations)]
    # The filter handler only sees every other sample.
    seen_u_velocities = [first_u_velocities[i]
                         for i in xrange(first_test_iterations) if i % 2 == 1]
    seen_v_velocities = [first_v_velocities[i]
                         for i in xrange(first_test_iterations) if i % 2 == 1]
    aio_util.RunAllFilters()
    for i in xrange(first_test_iterations):
      gs_obj.wind.wind_velocity[0] = first_u_velocities[i]
      gs_obj.wind.wind_velocity[1] = first_v_velocities[i]
      self._MockMessage(gs_obj, i, 12345, source_name, message_type,
                        pack_avionics_messages.PackGroundStationWeatherMessage,
                        message_size)
      time.sleep(delay)
      aio_util.RunAllFilters()
    # Get filtered data and assert the sampling frequency was correct.
    filtered_data = aio_helper.GetFilteredData()
    frequency = filtered_data.wind_gust.frequency
    self.assertGreaterEqual(frequency, desired_frequency - error_margin)
    self.assertLessEqual(frequency, desired_frequency + error_margin)

    # Call aio_util.RunAllFilters() at a slower frequency, and check that
    # the frequency matches that slower frequency. Also mock some velocities
    # and verify that the tke is computed correctly for those velocities.
    message_frequency = 16
    delay = 1.0 / message_frequency
    desired_frequency = 16
    second_u_velocities = [10.0 + i % 2 for i in xrange(second_test_iterations)]
    second_v_velocities = [4.0 + 3 * (i % 2)
                           for i in xrange(second_test_iterations)]

    aio_util.RunAllFilters()
    for i in xrange(second_test_iterations):
      gs_obj.wind.wind_velocity[0] = second_u_velocities[i]
      gs_obj.wind.wind_velocity[1] = second_v_velocities[i]
      self._MockMessage(gs_obj, first_test_iterations + i, 12345, source_name,
                        message_type,
                        pack_avionics_messages.PackGroundStationWeatherMessage,
                        message_size)
      time.sleep(delay)
      aio_util.RunAllFilters()
    filtered_data = aio_helper.GetFilteredData()
    messages['filtered'] = filtered_data
    frequency = filtered_data.wind_gust.frequency
    self.assertGreaterEqual(frequency, desired_frequency - error_margin)
    self.assertLessEqual(frequency, desired_frequency + error_margin)

    # Check that the tke is correct. This takes into account the fact that
    # several wind gust samples should have been skipped in the first phase.
    tke = filtered_data.wind_gust.tke
    seen_iterations = first_test_valid_iterations + second_test_iterations
    u_velocities = seen_u_velocities + second_u_velocities
    v_velocities = seen_v_velocities + second_v_velocities
    u_avg = sum(u_velocities) / seen_iterations
    v_avg = sum(v_velocities) / seen_iterations
    u_squared_avg = sum([x ** 2 for x in u_velocities]) / seen_iterations
    v_squared_avg = sum([x ** 2 for x in v_velocities]) / seen_iterations
    expected_tke = .5 * ((u_squared_avg - u_avg ** 2)
                         + (v_squared_avg - v_avg ** 2))
    self.assertTrue(filtered_data.wind_gust.valid)
    self.assertAlmostEqual(tke, expected_tke)

  def testFlapsIndicatorSparseCommsCoverage(self):
    filtered = aio_helper.GetFilteredData()
    filtered.merge_tether_down.valid = True
    for i in range(aio_labels.kNumServos):
      state = filtered.merge_tether_down.tether_down.servo_statuses[i]
      state.no_update_count = 0
      state.angle = float(i)

    indicator = control.FlapsIndicator(common.SPARSE_COMMS_MODE)
    messages = struct_tree.StructTree({'filtered': filtered})
    self.assertEqual(stoplights.STOPLIGHT_NORMAL,
                     indicator.Filter(messages)[1])

  def _GetSynthesizedControlTelemetry(self):
    messages = self._SynthesizeControlTelemetry()
    data = messages.Data(convert_to_basic_types=False)
    control_telemetry = data['ControlTelemetry']['ControllerA']
    filtered = aio_helper.GetFilteredData()
    filtered.merge_tether_down.valid = True
    tether_control_telemetry = (
        filtered.merge_tether_down.tether_down.control_telemetry)
    tether_control_telemetry.no_update_count = 0
    data['filtered'] = filtered
    messages = struct_tree.StructTree(data, readonly=False)
    return control_telemetry, tether_control_telemetry, messages

  def testTensionChart(self):
    def SetTension(value, tether_control_telemetry, control_telemetry):
      control_telemetry.state_est.tether_force_b.sph.tension = value
      tether_control_telemetry.tension = value

    def TestResult(tension, tether_control_telemetry, control_telemetry,
                   messages, expected_stoplight, expected_warning):
      SetTension(tension, tether_control_telemetry, control_telemetry)
      for mode in [common.SPARSE_COMMS_MODE, common.FULL_COMMS_MODE]:
        indicator = control.TensionChart(mode)
        _, tensions, stoplight, warning = indicator.Filter(messages)
        self.assertEqual(stoplight, expected_stoplight)
        self.assertEqual(warning, expected_warning)
        self.assertAlmostEqual(tensions[-1], tension * 0.001, places=3)

    control_telemetry, tether_control_telemetry, messages = (
        self._GetSynthesizedControlTelemetry())
    control_telemetry.state_est.tether_force_b.valid = True

    with mock.patch('makani.control.common.IsControlSystemRunning',
                    lambda x: True):

      with mock.patch('makani.control.common.AnyHoverFlightMode',
                      lambda x: False):

        with mock.patch('makani.control.common.AnyCrosswindFlightMode',
                        lambda x: False):
          TestResult(MONITOR_PARAMS.tether.tension.high,
                     tether_control_telemetry, control_telemetry,
                     messages, stoplights.STOPLIGHT_NORMAL, '')
          TestResult(MONITOR_PARAMS.tether.tension.very_high,
                     tether_control_telemetry, control_telemetry,
                     messages, stoplights.STOPLIGHT_WARNING, '')
          TestResult(MONITOR_PARAMS.tether.tension.very_high + 1.0,
                     tether_control_telemetry, control_telemetry,
                     messages, stoplights.STOPLIGHT_ERROR, '')

        with mock.patch('makani.control.common.AnyCrosswindFlightMode',
                        lambda x: True):
          TestResult(MONITOR_PARAMS.tether.tension_crosswind.low,
                     tether_control_telemetry, control_telemetry,
                     messages, stoplights.STOPLIGHT_NORMAL, '')
          TestResult(MONITOR_PARAMS.tether.tension_crosswind.high,
                     tether_control_telemetry, control_telemetry,
                     messages, stoplights.STOPLIGHT_NORMAL, '')
          TestResult(MONITOR_PARAMS.tether.tension_crosswind.very_low,
                     tether_control_telemetry, control_telemetry,
                     messages, stoplights.STOPLIGHT_WARNING, '')
          TestResult(MONITOR_PARAMS.tether.tension_crosswind.very_high,
                     tether_control_telemetry, control_telemetry,
                     messages, stoplights.STOPLIGHT_WARNING, '')
          TestResult(MONITOR_PARAMS.tether.tension_crosswind.very_low - 1.0,
                     tether_control_telemetry, control_telemetry,
                     messages, stoplights.STOPLIGHT_ERROR, '')
          TestResult(MONITOR_PARAMS.tether.tension_crosswind.very_high + 1.0,
                     tether_control_telemetry, control_telemetry,
                     messages, stoplights.STOPLIGHT_ERROR, '')

  def testTetherReleaseReadinessIndicator(self):
    """Test entering the 'continue' and 'any_warning_or_error' branch."""
    loadcells = ['LoadcellPortA', 'LoadcellPortB']
    indicator = control.TetherReleaseReadinessIndicator(loadcells)
    # Create a message snapshot that fail silently to gracefully handle
    # missing messages.
    messages = test_util.SynthesizeMessages(
        ['ControlTelemetry', 'ControlSlowTelemetry', 'Loadcell'], 0,
        fail_silently=True)

    # Set up the port message so that it contains an empty StructTree in one
    # port and an error in the other port.
    read_message = messages.Data(convert_to_basic_types=False)
    error_code = pack_avionics_messages.kLoadcellErrorBatteryDisconnected
    del read_message['Loadcell']['LoadcellPortA']
    read_message['Loadcell']['LoadcellPortB'].status.warning = 0
    read_message['Loadcell']['LoadcellPortB'].status.error = error_code
    read_message['ControlSlowTelemetry']['ControllerA'].flight_plan = (
        system_params.kFlightPlanTurnKey)

    # Call Filter, check that stoplight value is error,
    # and check the string values detect a Low Battery Error in PortB.
    result = indicator.Filter(messages)
    result_string, result_stoplight = result
    result_lines = result_string.split('\n')

    self.assertEqual(stoplights.STOPLIGHT_ERROR, result_stoplight)
    self.assertEqual('PortB:', result_lines[0])
    self.assertEqual('[ERROR]', result_lines[1])
    self.assertEqual('BatteryDisconnected', result_lines[2])

    # Change flight plan to a low altitude plan.
    read_message['ControlSlowTelemetry']['ControllerA'].flight_plan = (
        system_params.kFlightPlanLaunchPerch)
    result_string, result_stoplight = indicator.Filter(messages)
    self.assertEqual(result_string, 'No Key')
    self.assertEqual(result_stoplight, stoplights.STOPLIGHT_NORMAL)

    # Disable BatterDisconnected error (key is plugged in).
    read_message['Loadcell']['LoadcellPortB'].status.error = 0
    result_string, result_stoplight = indicator.Filter(messages)
    self.assertEqual(result_string, 'Keyed: PortB')
    self.assertEqual(result_stoplight, stoplights.STOPLIGHT_ERROR)

  def testTetherReleaseIndicator_Invalid_And_Disarmed(self):
    indicator = control.TetherReleaseIndicator()
    messages = self._SynthesizeControlTelemetry()
    read_message = messages.Data(convert_to_basic_types=False)
    read_message['ControlSlowTelemetry']['ControllerA'].flight_plan = (
        system_params.kFlightPlanTurnKey)
    attributes = indicator._GetAllAttributes(messages)
    skeleton = attributes[0]

    # Test invalid input.
    # Set up all messages for invalid input.
    for s in skeleton:
      s.no_update_count = 33

    # Call Filter, check stoplight value, and check the string value
    # to make sure the invalid input string was returned.
    result = indicator.Filter(messages)
    result_string, result_stoplight = result
    self.assertEqual(stoplights.STOPLIGHT_UNAVAILABLE, result_stoplight)
    self.assertEqual('--\n\n\n', result_string)

    # Test that all statuses are disarmed.
    # Set up all messages for disarmed state.
    for s in skeleton:
      s.no_update_count = 1

    # Call Filter, check stoplight value, and check the string values
    # to make sure each node state is "Disarmed".
    result_string, result_stoplight = indicator.Filter(messages)
    result_array = result_string.split('\n')

    self.assertEqual(stoplights.STOPLIGHT_WARNING, result_stoplight)
    self.assertEqual('PortA: Disarmed, safety on', result_array[0])
    self.assertEqual('PortB: Disarmed, safety on', result_array[1])
    self.assertEqual('StarboardA: Disarmed, safety on', result_array[2])
    self.assertEqual('StarboardB: Disarmed, safety on', result_array[3])

  def testTetherReleaseIndicator_Invalid_Released_Error_ArmedNoInterlock(self):
    """Test Invalid, Released, Error, and Armed No Interlock."""
    indicator = control.TetherReleaseIndicator()
    messages = self._SynthesizeControlTelemetry()
    read_message = messages.Data(convert_to_basic_types=False)
    read_message['ControlSlowTelemetry']['ControllerA'].flight_plan = (
        system_params.kFlightPlanTurnKey)
    attributes = indicator._GetAllAttributes(messages)

    skeleton = attributes[0]
    # Set PortA to greater than MAX_NO_UPDATE_COUNT_TETHER_RELEASE.
    skeleton[0].no_update_count = 33
    # Set PortB to released.
    skeleton[1].released = 1
    skeleton[1].no_update_count = 1
    # Set StarboardA state to "ERROR".
    skeleton[2].state = pack_avionics_messages.kActuatorStateError
    skeleton[2].no_update_count = 1
    # Set StarboardB state to "ARMED" and interlock off.
    skeleton[3].state = pack_avionics_messages.kActuatorStateArmed
    skeleton[3].interlock_switched = 0
    skeleton[3].no_update_count = 1

    result_string, result_stoplight = indicator.Filter(messages)
    result_array = result_string.split('\n')

    # We expect STOPLIGHT_ERROR because at least one of the releases
    # is in state RELEASED.
    self.assertEqual(stoplights.STOPLIGHT_ERROR, result_stoplight)
    self.assertEqual('PortA: --', result_array[0])
    self.assertEqual('PortB: Released', result_array[1])
    self.assertEqual('StarboardA: Error', result_array[2])
    self.assertEqual('StarboardB: Armed, safety on', result_array[3])

  def testTetherReleaseIndicator_Invalid_Error_NotArmedInterLock_ArmedRdy(self):
    """Test Invalid, Error, Not armed interlock, and Armed."""
    indicator = control.TetherReleaseIndicator()
    messages = self._SynthesizeControlTelemetry()
    read_message = messages.Data(convert_to_basic_types=False)
    read_message['ControlSlowTelemetry']['ControllerA'].flight_plan = (
        system_params.kFlightPlanTurnKey)
    attributes = indicator._GetAllAttributes(messages)

    # Set node_status flags to cause "Error" for PortB
    node_status = attributes[2]
    node_status[aio_node.kAioNodeLoadcellPortB].flags = (
        pack_avionics_messages.kTetherNodeFlagAnyError)

    skeleton = attributes[0]

    # Set PortA equal to MAX_NO_UPDATE_COUNT_TETHER_RELEASE.
    skeleton[0].no_update_count = 32
    skeleton[1].no_update_count = 1
    # Set StarboardA state to init and interlock_switched on.
    skeleton[2].state = pack_avionics_messages.kActuatorStateInit
    skeleton[2].interlock_switched = 1
    skeleton[2].no_update_count = 1
    # Set StarboardB interlock_switch on and state to armed.
    skeleton[3].interlock_switched = 1
    skeleton[3].state = pack_avionics_messages.kActuatorStateArmed
    skeleton[3].no_update_count = 1

    result_string, result_stoplight = indicator.Filter(messages)
    result_array = result_string.split('\n')

    self.assertEqual(stoplights.STOPLIGHT_ERROR, result_stoplight)
    self.assertEqual('PortA: --', result_array[0])
    self.assertEqual('PortB: Error', result_array[1])
    self.assertEqual('StarboardA: Disarmed, safety off', result_array[2])
    self.assertEqual('StarboardB: Armed, safety off', result_array[3])

if __name__ == '__main__':
  unittest.main()
