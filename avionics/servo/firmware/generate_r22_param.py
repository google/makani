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


"""Code generation module for Copley R22 Parameters."""

import csv
import struct
import sys
import textwrap

import gflags
import numpy as np
import yaml

FLAGS = gflags.FLAGS
gflags.DEFINE_string('ccx_file', None,
                     'Full path to Copley CCX configuration file.',
                     short_name='c')
gflags.DEFINE_string('dict_file', None,
                     'Full path to parameter dictionary file.',
                     short_name='d')
gflags.DEFINE_string('yaml_output', None,
                     'Full path to YAML parameter dictionary output file.',
                     short_name='y')
gflags.DEFINE_string('source_file', None,
                     'Full path to output source file.',
                     short_name='s')
gflags.DEFINE_string('header_file', None,
                     'Full path to output header file.',
                     short_name='h')


def _ServoConfigurationAileron1():
  """Override configuration for aileron 1 servos."""

  config = {}
  config['NegativeSoftwareLimit'] = [_DegreesToCounts(-45.0)]
  config['PositiveSoftwareLimit'] = [_DegreesToCounts(45.0)]
  config['DesiredState'] = [30]                 # 30 = CANopen mode.
  # Profile velocity is in units of [0.1 output shaft counts / s]
  config['ProfileVelocity'] = [
      _RadiansPerSecondToProfileVelocity(1.0)]
  # Velocity loop limit is in units of [0.1 drive shaft counts / s]
  config['VelocityLoopVelocityLimit'] = [_RadiansPerSecondToLoopVelocity(1.15)]
  # Profile acceleration is in units of [10 output shaft counts / s^2]
  config['ProfileAcceleration'] = [
      _RadiansPerSecondSquaredToProfileAcceleration(7.0)]
  config['ProfileDeceleration'] = [
      _RadiansPerSecondSquaredToProfileAcceleration(7.0)]
  config['PositionPp'] = [600]                  # Proportional gain.
  config['PositionVff'] = [1000]                # Velocity feed-forward.
  config['PositionAff'] = [7000]                # Acceleration feed-forward.
  config['PositionFollowingWarningLimit'] = [_DegreesToCountsRelative(5.0)]
  config['UserContinuousCurrentLimit'] = [1400]  # [0.01 Amps/count]
  config['UserPeakCurrentTimeLimit'] = [2000]   # [ms]
  config['NodeIdConfiguration'] = [0x0004]      # Bit rate 1Mbps, node ID 4.
  config['MotorWiring'] = [1]                   # U/V swapped.
  config['MotorHallWiring'] = [4]               # W/V/U hall order.
  return config


def _ServoConfigurationAileron2():
  """Override configuration for aileron 2 servos."""

  return _ServoConfigurationAileron1()


def _ServoConfigurationAileron4():
  """Override configuration for aileron 4 servos."""

  return _ServoConfigurationAileron1()


def _ServoConfigurationElevator():
  """Override configuration for elevator servos."""

  return _ServoConfigurationAileron1()


def _ServoConfigurationRudder():
  """Override configuration for rudder servos."""

  return _ServoConfigurationAileron1()


def _ServoConfigurationDetwist():
  """Override configuration for tether detwist servo."""

  return _ServoConfigurationAileron1()


def GetPatchConfiguration():
  """Get dictionary of custom patch configurations."""

  return {'aileron1': _ServoConfigurationAileron1(),
          'aileron2': _ServoConfigurationAileron2(),
          'aileron4': _ServoConfigurationAileron4(),
          'detwist': _ServoConfigurationDetwist(),
          'elevator': _ServoConfigurationElevator(),
          'rudder': _ServoConfigurationRudder()}


_OUTPUT_ENCODER_COUNTS_PER_REV = 2**14


def _RadiansToCounts(angle):
  """Convert radians to encoder counts."""

  counts = (angle + np.pi) / (2.0*np.pi) * _OUTPUT_ENCODER_COUNTS_PER_REV
  return int(np.clip(counts, 0, _OUTPUT_ENCODER_COUNTS_PER_REV - 1))


def _DegreesToCounts(angle):
  """Convert degrees to output shaft encoder counts."""
  return _RadiansToCounts(np.deg2rad(angle))


def _DegreesToCountsRelative(angle):
  """Convert degrees to output shaft encoder count delta."""
  return int(angle / 360.0 * _OUTPUT_ENCODER_COUNTS_PER_REV)


def _RadiansPerSecondSquaredToProfileAcceleration(accel):
  """Convert radians per second squared to profile acceleration counts."""
  counts = accel / (2.0 * np.pi) * _OUTPUT_ENCODER_COUNTS_PER_REV * 0.1
  # Clip at 10 rev/sec/sec.
  return int(np.clip(counts, 0, _OUTPUT_ENCODER_COUNTS_PER_REV))


def _RadiansPerSecondToProfileVelocity(velocity):
  """Convert radians per second to profile velocity counts."""

  counts = velocity / (2.0 * np.pi) * _OUTPUT_ENCODER_COUNTS_PER_REV * 10
  # Clip at 1 rev/sec.
  return int(np.clip(counts, 0, _OUTPUT_ENCODER_COUNTS_PER_REV * 10))


def _RadiansPerSecondToLoopVelocity(velocity):
  """Convert radians per second to velocity loop velocity counts."""

  encoder_counts = 160 * 36
  counts = velocity / (2.0 * np.pi) * encoder_counts * 10
  # Clip at 1 rev/sec.
  return int(np.clip(counts, 0, encoder_counts * 10))


def _GetName(name):
  """Translate CCX parameter name to camel case name."""

  name = name.replace('\'', '').replace('.', '').replace('/', '')
  name = ''.join([n.capitalize() for n in name.split(' ')])
  return name


class CopleyConfiguration(object):
  """Handle Copley parameter configuration parameters."""

  def _AddTableEntry(self, serial_id, device_net, macro, can, bank, typ):
    """Add a new parameter to the parameter table."""
    entry = {'serial_id': serial_id, 'device_net': device_net,
             'macro': macro, 'can': can, 'bank': bank, 'typ': typ,
             'index': len(self.table)}
    if can == 'None':
      can = None
    # Reject duplicate entries.
    if serial_id in self.by_serial_id:
      raise ValueError('Serial ID %d already exists in the table.' % serial_id)
    if can in self.by_can:
      raise ValueError('CAN ID %d already exists in the table.' % can)
    self.table.append(entry)
    if serial_id is not None:
      self.by_serial_id[serial_id] = entry
    if can is not None:
      # These parameters need to be mapped to legacy values.
      param_can_remap = {0x2380: 0x60F6, 0x2381: 0x60F9, 0x2382: 0x60FB,
                         0x2383: 0x6410, 0x2384: 0x6510}
      if isinstance(can, int):
        entry['can_index'] = can
        entry['can_sub'] = 0
      elif ':' in can:
        entry['can_index'] = int(can.split(':')[0], 16)
        entry['can_sub'] = int(can.split(':')[1], 10)
      if entry['can_index'] in param_can_remap:
        entry['can_index'] = param_can_remap[entry['can_index']]
        can = '0x%04X:%d' % (entry['can_index'], entry['can_sub'])
      self.by_can[can] = entry
    else:
      # Index 0 is reserved as not used.
      entry['can_index'] = 0
      entry['can_sub'] = 0

  def _AddDesc(self, entry, desc):
    # Remove entry if already indexed by name and store new name.
    if 'name' in entry and entry['name'] in self.by_name:
      del self.by_name[entry['name']]
    entry['desc'] = desc
    entry['name'] = _GetName(desc)
    self.by_name[entry['name']] = entry

  def __init__(self):
    self.table = []
    self.by_name = {}
    self.by_serial_id = {}
    self.by_can = {}

  def _AddExtraDescriptions(self):
    """Specify R22 dictionary parameters not found in the CCX file."""
    # ASCII command ID and description.
    serial_params = [
        (0x03, 'Winding A Current'),
        (0x04, 'Winding B Current'),
        (0x05, 'Current Offset A'),
        (0x06, 'Current Offset B'),
        (0x07, 'Stator Current X Axis'),
        (0x08, 'Stator Current Y Axis'),
        (0x09, 'Current Loop Output X Axis'),
        (0x0A, 'Current Loop Output Y Axis'),
        (0x0B, 'Actual Current D'),
        (0x0C, 'Actual Current Q'),
        (0x0D, 'Commanded Current D'),
        (0x0E, 'Commanded Current Q'),
        (0x0F, 'Current Error D'),
        (0x10, 'Current Error Q'),
        (0x11, 'Current Integral D'),
        (0x12, 'Current Integral Q'),
        (0x13, 'Current Loop Output D'),
        (0x14, 'Current Loop Output Q'),
        (0x15, 'Commanded Motor Current'),
        (0x17, 'Actual Position'),
        (0x18, 'Actual Velocity'),
        (0x1B, 'Sine Feedback Voltage'),
        (0x1C, 'Cosine Feedback Voltage'),
        (0x1D, 'Analog Reference Voltage'),
        (0x1E, 'High Voltage'),
        (0x20, 'Drive Temperature'),
        (0x25, 'Limited Motor Current Command'),
        (0x29, 'Velocity Loop Limited Velocity'),
        (0x2A, 'Velocity Loop Error'),
        (0x2B, 'Velocity Loop Integral Sum'),
        (0x2C, 'Commanded Velocity'),
        (0x2D, 'Commanded Position'),
        (0x2E, 'Velocity Loop Acceleration Feed Forward'),
        (0x32, 'Actual Motor Position'),
        (0x35, 'Position Loop Error'),
        (0x38, 'Actual Motor Current'),
        (0x3B, 'Instantaneous Commanded Velocity'),
        (0x3C, 'Instantaneous Commanded Acceleration'),
        (0x3D, 'Trajectory Destination Position'),
        (0x47, 'Motor Temperature Sensor Type'),
        (0x4E, 'Motor Wiring'),
        (0x52, 'Motor Hall Wiring'),
        (0x5E, 'Load Encoder Velocity'),
        (0x68, 'Captured Index Position'),
        (0x69, 'Unfiltered Velocity'),
        (0x6D, 'Position Capture Status Register'),
        (0x81, 'Drive Serial Number'),
        (0x85, 'PWM Period'),
        (0x8B, 'Drive Rated Minimum Voltage'),
        (0x8C, 'Drive Rated Maximum Temperature'),
        (0x90, 'Baud Rate'),
        (0x91, 'Maximum Data Words Per Command'),
        (0x94, 'Firmware Version Number'),
        (0x96, 'Analog Reference Calibration Offset'),
        (0x97, 'Over Temperature Cutout Hysteresis'),
        (0x9C, 'Over Voltage Cutout Hysteresis'),
        (0x9D, 'PWM Dead Time Continuous Current'),
        (0x9E, 'PWM Off Time Minimum'),
        (0x9F, 'PWM Dead Time Zero Current'),
        (0xA0, 'Event Status'),
        (0xA1, 'Latched Event Status'),
        (0xA2, 'Hall Input State'),
        (0xA4, 'Latched Fault Status'),
        (0xA6, 'Input Pin States'),
        (0xAA, 'Raw Input State'),
        (0xAB, 'Output States'),
        (0xAC, 'Sticky Event Status'),
        (0xB0, 'Motor Phase Angle'),
        (0xB4, 'Encoder Phase Angle'),
        (0xB5, 'Homing Adjustment'),
        (0xB7, 'System Time'),
        (0xC0, 'Network Node ID'),
        (0xC9, 'Trajectory Status'),
        (0xCA, 'Target Position'),
        (0xCB, 'Profile Velocity'),
        (0xCC, 'Profile Acceleration'),
        (0xCD, 'Profile Deceleration'),
        (0xDC, 'Regen Turn On Voltage'),
        (0xDD, 'Regen Turn Off Voltage'),
        (0xDE, 'Regen Peak Current Rating'),
        (0xDF, 'Regen Continuous Current Rating'),
        (0xE0, 'Regen Time At Peak Current'),
        (0xE2, 'Regen Resistor Status'),
        (0x100, 'CANopen Limit Status Mask'),
        (0x101, 'Network Address Switch Value'),
        (0x102, 'Network Status'),
        (0x108, 'Trigger CANopen PDO 254'),
        (0x10A, 'Captured Home Position'),
        (0x10B, 'Firmware Version Number Extended'),
        (0x110, 'Position Capture Timestamp'),
        (0x111, 'Position Capture Position'),
        (0x112, 'Position Encoder Position'),
        (0x113, 'CANopen Emergency Inhibit Time'),
        (0x116, 'CANopen Quick Stop Option Code'),
        (0x117, 'CANopen Shutdown Option Code'),
        (0x118, 'CANopen Disable Option Code'),
        (0x119, 'CANopen Halt Option Code'),
        (0x120, 'Number of Axis'),
        (0x122, 'Internal Maximum Regen Current'),
        (0x126, 'FPGA Firmware Version'),
        (0x128, 'Gain Scheduling Key Parameter'),
        (0x129, 'Reserved Drive Hardware Options'),
        (0x12C, 'Secondary Firmware Version'),
        (0x12E, 'Motor Encoder Status'),
        (0x12F, 'Load Encoder Status'),
        (0x130, 'RMS Current Calculation Period'),
        (0x131, 'RMS Current Measurement'),
        (0x132, 'User Current Limit Running Sum'),
        (0x133, 'Amp Current Limit Running Sum'),
        (0x134, 'Analog Output Configuration'),
        (0x135, 'Analog Output Value'),
        (0x136, 'Second Analog Reference Value'),
        (0x137, 'Second Analog Reference Offset'),
        (0x138, 'Second Analog Reference Calibration Offset'),
        (0x139, 'Drive Safety Circuit Status'),
        (0x13A, 'Analog Motor Temperature Sensor Voltage'),
        (0x13B, 'Analog Motor Temperature Sensor Limit'),
        (0x154, 'Servo Loop Configuration'),
        (0x155, 'Position Loop Integral Gain'),
        (0x156, 'Position Loop Derivative Gain'),
        (0x157, 'Velocity Loop Command Feed Forward'),
        (0x158, 'Position Loop Integral Drain'),
        (0x159, 'CANopen Abort Option Code'),
        (0x15A, 'IO Options'),
        (0x15B, 'Motor Brake Enable Delay'),
        (0x180, 'UV Configuration'),
        (0x181, 'UV Mode U Input'),
        (0x182, 'UV Mode V Input'),
        (0x18B, 'Trajectory Options'),
        (0x18C, 'IO Extension Configuration'),
        (0x18D, 'IO Extension Transmit Data'),
        (0x18E, 'IO Extension Receive Data'),
        (0x18F, 'Encoder Sine Offset'),
        (0x190, 'Encoder Cosine Offset'),
        (0x191, 'Encoder Cosine Scaling Factor'),
        (0x192, 'Motor Encoder Calibration Settings'),
        (0x193, 'Load Encoder Calibration Settings'),
        (0x194, 'PWM Input Duty Cycle'),
        (0x195, 'Trajectory Abort Jerk Limit'),
        (0x196, 'Analog Encoder Magnitude'),
        (0x197, 'Cross Coupling Kp Gain'),
        (0x198, 'Cross Coupling Ki Gain'),
        (0x199, 'Cross Coupling Kd Gain'),
        (0x19B, 'Minimum PWM Deadtime Current'),
        (0x19C, 'Passive Load Encoder High Speed Capture'),
        (0x19D, 'Open Motor Wiring Check Current'),
        (0x19E, 'Position Error Timeout')]
    for param in serial_params:
      self._AddDesc(self.by_serial_id[param[0]], param[1])

    # CANopen only parameters.
    # See http://www.copleycontrols.com/Motion/pdf/CANopenProgrammersManual.pdf
    can_params = []

    # SDOs and PDOs.
    can_params += [
        (0x1200, 'H', 'Servo SDO Parameters',
         [('I', 'SDO Recieve ID'), ('I', 'SDO Transmit ID')])]
    can_params += [
        (0x1B00, 'B', 'Fixed Transmit PDO Mapping',
         [('I', 'Fixed Tx PDO Mapping %d' % i) for i in range(5)])]
    can_params += [
        (0x1400 + i, 'B', 'Rx PDO %d Parameters' % i,
         [('I', 'Rx PDO %d ID' % i), ('B', 'Rx PDO %d Type' % i)])
        for i in range(8)]
    can_params += [
        (0x1600 + i, 'B', 'Rx PDO %d Num Mapped' % i,
         [('I', 'Rx PDO %d Mapping %d' % (i, j)) for j in range(8)])
        for i in range(8)]
    # Missing Rx PDOs at 1700-1702.
    can_params += [
        (0x1800 + i, 'B', 'Tx PDO %d Parameters' % i,
         [('I', 'Tx PDO %d ID' % i), ('B', 'Tx PDO %d Type' % i)])
        for i in range(8)]
    can_params += [
        (0x1A00 + i, 'B', 'Tx PDO %d Num Mapped' % i,
         [('I', 'Tx PDO %d Mapping %d' % (i, j)) for j in range(8)])
        for i in range(8)]

    # Network Management.
    can_params += [
        (0x1005, 'I', 'COB ID Sync Message'),
        (0x1006, 'I', 'Communication Cycle Period'),
        (0x100C, 'H', 'Guard Time'),
        (0x100D, 'B', 'Life Time Factor'),
        (0x1013, 'I', 'High Resolution Time Stamp'),
        (0x1017, 'H', 'Producer Heartbeat Time'),
        (0x1014, 'I', 'Emergency Object ID'),
        (0x1015, 'H', 'Emergency Object ID Inhibit Time')]

    # Device Control and Status.
    can_params += [
        (0x6040, 'H', 'Control Word'),
        (0x6041, 'H', 'Status Word'),
        (0x6060, 'b', 'Mode of Operation'),
        (0x6061, 'b', 'Mode of Operation Display')]

    # Error Management.
    can_params += [
        (0x1003, 'B', 'Error History',
         [('I', 'Error History Entry %d' % i) for i in range(8)])]

    # Control Loop Configuration.
    can_params += [
        (0x6086, 'h', 'Motion Profile Type'),
        (0x60B0, 'i', 'Target Position Offset'),
        (0x60B1, 'i', 'Target Velocity Offset'),
        (0x60B2, 'i', 'Target Torque Offset'),
        (0x60FF, 'i', 'Target Velocity'),
        (0x6071, 'h', 'Target Torque'),
        (0x6076, 'h', 'Motor Rated Torque'),
        (0x6077, 'h', 'Actual Torque'),
        (0x6087, 'i', 'Torque Slope'),
        (0x6088, 'h', 'Torque Profile Type'),
        ]

    for param in can_params:
      can = '0x%04X:0' % param[0]
      self._AddTableEntry(None, None, None, can, 'R', param[1])
      self._AddDesc(self.by_can[can], param[2])
      if len(param) == 4:
        for i, subentry in enumerate(param[3]):
          can = '0x%04X:%d' % (param[0], i + 1)
          self._AddTableEntry(None, None, None, can, 'R', subentry[0])
          self._AddDesc(self.by_can[can], subentry[1])

    # Filter entries without descriptions from the table.
    old_table = self.table
    self.table = []
    for entry in old_table:
      if 'desc' in entry:
        entry['index'] = len(self.table)
        self.table.append(entry)

  def LoadConfiguration(self, ccx_file):
    """Load Copley Motion Explorer (CME) CCX configuration file."""

    # These parameters are not supported by amp.
    ignore = []
    ignore.append(0x95)   # Host config state.
    ignore.append(0x94C)  # Basic host config.

    # These parameters do not affect operation and prevent meeting our
    # control loop timing requirements during verification.
    ignore.append(0x41)  # Manufacture string.
    ignore.append(0x42)  # Model number string.
    ignore.append(0x80)  # Amp model number string.
    ignore.append(0x92)  # Amp name string.
    ignore.append(0xE1)  # Regen resistor model number string.

    # These parameters require special handling.
    param_hex = [0x70, 0x71, 0x72, 0x73, 0x73, 0x74, 0x75, 0x76, 0x77]

    # Parse configuration file.
    with open(ccx_file, 'r') as csvfile:
      csvreader = csv.reader(csvfile)
      for row in csvreader:
        if len(row) < 4:
          continue
        pid, _, desc, value = row
        pid = int(pid, 16)
        values = value.split(':')
        if pid in ignore:
          continue
        if pid not in self.by_serial_id:
          print 'Parameter id 0x{:02X} not found in dictionary!'.format(pid)
          continue

        # Update parameter dictionary given number of parameter values.
        entry = self.by_serial_id[pid]
        if entry['typ'] == 'c':
          values = value + '\0'
          while len(values) % 2 != 0:
            values += '\0'
        elif pid in param_hex:
          values = [int(v, 16) for v in values]
        else:
          values = [int(v, 10) for v in values]
        if len(entry['typ']) == 1:
          entry['typ'] *= len(values)
        self._AddDesc(entry, desc)
        entry['config_value'] = values

    self._AddExtraDescriptions()

  def LoadCopleyDictionary(self, dict_file):
    """Load Copley's R22 Parameter Dictionary document.

    This function interprets the pdftotext -layout output of Copley's
    R22_Parameter_Dictionary.pdf (January 2014, Revision 00, P/N 16-01091) to
    understand the R22 parameter identification codes (ASCII, Device Net,
    Macro, and CAN) and associated variable type. Use GenerateYamlDictionary()
    to output the relevant information in YAML format.

    Use the following command line to create a compatible input file:

    pdftotext -layout R22_Parameter_Dictionary.pdf r22_param.txt.

    Args:
      dict_file: Full path to text version of Copley's R22 Parameter
        Dictionary file.

    Returns:
      None.
    """

    # These parameters require special handling.
    # Input pin config for general purpose input 16-23.
    self._AddTableEntry(0x160, 0x161, 0x560, '0x2192:17', 'RF', 'H')
    self._AddTableEntry(0x161, 0x162, 0x561, '0x2192:18', 'RF', 'H')
    self._AddTableEntry(0x162, 0x163, 0x562, '0x2192:19', 'RF', 'H')
    self._AddTableEntry(0x163, 0x164, 0x563, '0x2192:20', 'RF', 'H')
    self._AddTableEntry(0x164, 0x165, 0x564, '0x2192:21', 'RF', 'H')
    self._AddTableEntry(0x165, 0x166, 0x565, '0x2192:22', 'RF', 'H')
    self._AddTableEntry(0x166, 0x167, 0x566, '0x2192:23', 'RF', 'H')
    self._AddTableEntry(0x167, 0x168, 0x567, '0x2192:24', 'RF', 'H')

    # Debounce time for general purpose input 16-23.
    self._AddTableEntry(0x170, 0x171, 0x570, '0x2195:17', 'RF', 'H')
    self._AddTableEntry(0x171, 0x172, 0x571, '0x2195:18', 'RF', 'H')
    self._AddTableEntry(0x172, 0x173, 0x572, '0x2195:19', 'RF', 'H')
    self._AddTableEntry(0x173, 0x174, 0x573, '0x2195:20', 'RF', 'H')
    self._AddTableEntry(0x174, 0x175, 0x574, '0x2195:21', 'RF', 'H')
    self._AddTableEntry(0x175, 0x176, 0x575, '0x2195:22', 'RF', 'H')
    self._AddTableEntry(0x176, 0x177, 0x576, '0x2195:23', 'RF', 'H')
    self._AddTableEntry(0x177, 0x178, 0x577, '0x2195:24', 'RF', 'H')

    # Output configuration 0-7.
    self._AddTableEntry(0x70, 0x71, 0x470, '0x2193:1', 'RF', 'HII')
    self._AddTableEntry(0x71, 0x72, 0x471, '0x2193:2', 'RF', 'HII')
    self._AddTableEntry(0x72, 0x73, 0x472, '0x2193:3', 'RF', 'HII')
    self._AddTableEntry(0x73, 0x74, 0x473, '0x2193:4', 'RF', 'HII')
    self._AddTableEntry(0x74, 0x75, 0x474, '0x2193:5', 'RF', 'HII')
    self._AddTableEntry(0x75, 0x76, 0x475, '0x2193:6', 'RF', 'HII')
    self._AddTableEntry(0x76, 0x77, 0x476, '0x2193:7', 'RF', 'HII')
    self._AddTableEntry(0x77, 0x78, 0x477, '0x2193:8', 'RF', 'HII')

    # Velocity loop output filter.
    self._AddTableEntry(0x5F, 0x60, 0x45F, '0x2106', 'RF', 'h'*9)

    # Velocity loop command filter.
    self._AddTableEntry(0x6B, 0x6C, 0x46B, '0x2108', 'RF', 'h'*9)

    # Analog input filter.
    self._AddTableEntry(0x12D, 0x12E, 0x52D, '0x2109', 'RF', 'h'*9)

    # Map Copley's document types to Python types.
    type_map = {'U8': 'B', 'U16': 'H', 'U32': 'I',
                'INT8': 'b', 'INT16': 'h', 'INT32': 'i',
                'String': 'c'}

    with open(dict_file, 'r') as f:
      for line in f:

        # Handle pdftotext conversion errors.
        line = line.replace('0x ', '0x')
        line = line.replace(': ', ':')
        line = line.replace('INT 8', 'INT8')
        line = line.replace('INT 16', 'INT16')
        line = line.replace('INT 32', 'INT32')

        # Expect columns ASCII, DvcNet, MACRO, CAN/ECAT IDX:SUB, Bank, Type.
        fields = line.split()
        if len(fields) < 6:
          continue

        # ASCII column contains parameter id in hex format.
        try:
          pid = int(fields[0], 16)
        except ValueError:
          continue

        # Device Net column contains parameter id in hex format.
        try:
          device_net = int(fields[1], 16)
        except ValueError:
          continue

        # Macro column contains parameter id in hex format.
        try:
          macro = int(fields[2], 16)
        except ValueError:
          continue

        # CAN column contains IDX:SUB notation. Treat as text.
        can = fields[3]

        # Bank column specifies (R)AM and/or (F)lash. Treat as text.
        bank = fields[4].strip('*')
        if bank not in {'R', 'F', 'RF'}:
          continue

        # Type column may contain number of elements in array instead of type.
        typ = fields[5].strip('*')
        if typ in type_map:
          typ = type_map[typ]
        elif pid in self.typ:
          typ = self.typ[pid]
        else:
          continue

        # Output.
        self._AddTableEntry(pid, device_net, macro, can, bank, typ)

  def GenerateYamlDictionary(self):
    """Generate YAML parameter dictionary."""

    src = ''
    src += '\n'
    src += (
        '# AUTOMATICALLY GENERATED BY avionics/servo/firmware/'
        'generate_r22_param.py.\n')
    for entry in self.table:
      src += '\n'
      src += '---\n'
      src += 'id: 0x{:02X}\n'.format(entry['serial_id'])
      src += 'device_net: 0x{:02X}\n'.format(entry['device_net'])
      src += 'macro: 0x{:02X}\n'.format(entry['macro'])
      src += 'can: {}\n'.format(entry['can'])
      src += 'bank: {}\n'.format(entry['bank'])
      src += 'type: {}\n'.format(entry['typ'])
    return src

  def LoadYamlDictionary(self, yaml_output):
    """Load YAML parameter dictionary generated by GenerateYamlDictionary()."""

    with open(yaml_output, 'r') as f:
      for param in yaml.full_load_all(f):
        pid = param.pop('id')
        device_net = param.pop('device_net')
        macro = param.pop('macro')
        can = param.pop('can')
        bank = param.pop('bank')
        typ = param.pop('type')
        self._AddTableEntry(pid, device_net, macro, can, bank, typ)


def _PackList(fmt, lst):
  """Pack variable list into byte stream."""

  # Most significant byte first.
  try:
    data = struct.pack('>' + fmt, *lst)
  except struct.error:
    data = struct.pack('>' + fmt.lower(), *lst)
  return data


def _SerializeByIndex(copley, config_by_index):
  """Serialize configuration using map with index as key."""

  param_addr = {}
  param_length = {}
  param_data = ''
  # Skip these parameters in the configuration data.
  config_skip_params = [
      'UserPeakCurrentLimit',  # Set dynamically.
      'TargetPosition',  # Set dynamically.
      'TrajectoryProfileMode',  # Set dynamically.
      'AmpFamily',  # Not in CAN.
      'MacroAmplifiersEncoderCaptureConfig',  # Not in CAN.
      'VelocityLoopOutputFilter',  # Not in our R22.
      'VelocityLoopCommandFilter',  # Not in our R22.
      'AnalogInputFilter',  # Not in our R22.
      'VoltageSense',  # Not in our R22.
      'RegistrationOffsetForPulseAndDirection',  # Invalid in our mode.
      'MaximumPwmPulseWidthPwmPositionMode',  # Invalid in our mode.
      'MinimumPwmPulseWidthPwmPositionMode']  # Invalid in our mode.
  for entry in copley.table:
    index = entry['index']
    name = entry['name']
    if index in config_by_index and name not in config_skip_params:
      var_data = _PackList(entry['typ'], config_by_index[index])
      param_length[index] = len(var_data)
      param_addr[index] = len(param_data)
      param_data += var_data
    # Ensure all length values make it into the dict.  Dynamic string values
    # are not currently supported.
    elif entry['typ'] != 'c':
      param_length[index] = len(_PackList(entry['typ'], [0]*len(entry['typ'])))
  param_list = struct.unpack('B'*len(param_data), param_data)
  return (param_list, param_addr, param_length)


def _SerializeByName(copley, config_by_name):
  """Serialize configuration using map with parameter name as key."""

  config_by_index = {}
  for name, value in config_by_name.iteritems():
    index = copley.by_name[name]['index']
    config_by_index[index] = value
  return _SerializeByIndex(copley, config_by_index)


def _GetEnumName(name, prefix='kR22Param'):
  """Get enumeration-formatted name."""

  return prefix + _GetName(name)


def _GetDefineName(name):
  """Get define-formatted name."""

  name = name.replace('\'', '').replace('.', '').replace('/', '')
  name = 'R22_PARAM_' + name.replace(' ', '_').upper()
  return name


def _GetVariableName(name):
  """Get variable-formatted name."""

  name = name.replace('\'', '').replace('.', '').replace('/', '')
  name = '_'.join([n.lower() for n in name.split(' ')])
  return name


def GenerateHeader(copley, patch_dict):
  """Generate C header file."""

  hdr = textwrap.dedent('''
      // AUTOMATICALLY GENERATED BY avionics/servo/firmware/generate_r22_param.py

      #ifndef AVIONICS_SERVO_FIRMWARE_R22_PARAM_H_
      #define AVIONICS_SERVO_FIRMWARE_R22_PARAM_H_

      ''')
  hdr += '#define R22_PARAM_COUNT {}\n'.format(len(copley.table))
  hdr += '\n'
  hdr += '#include <stdbool.h>\n'
  hdr += '#include <stdint.h>\n'
  hdr += '\n'
  hdr += '#include "avionics/servo/firmware/r22_types.h"\n'
  hdr += '\n'
  hdr += 'typedef enum {\n'
  # Special case for None, since it isn't technically a R22 param.
  hdr += '  %s = %d,\n' % (_GetEnumName('None'), -1)
  for entry in copley.table:
    hdr += '  %s = %d,\n' % (_GetEnumName(entry['desc']), entry['index'])
  hdr += '} R22Parameter;\n'
  hdr += '\n'
  hdr += 'typedef enum {\n'
  for entry in copley.table:
    if entry['can_index']:
      hdr += '  %s = 0x%X,\n' % (_GetEnumName(entry['desc'], 'kR22CanIndex'),
                                 entry['can_index'])
  hdr += '} R22CanIndex;\n'
  hdr += '\n'
  hdr += 'typedef enum {\n'
  for entry in copley.table:
    if entry['can_index']:
      hdr += '  %s = 0x%X,\n' % (_GetEnumName(entry['desc'], 'kR22CanSub'),
                                 entry['can_sub'])
  hdr += '} R22CanSub;\n'
  hdr += '\n'
  hdr += textwrap.dedent('''
      typedef struct {
        R22Parameter param;
        uint16_t serial_id;
        int32_t addr_length;
        uint8_t memory_bank;
        int16_t config_offset;
        uint16_t can_index;
        uint8_t can_sub;
      } R22ParamInfo;

      ''')
  for p_name, _ in sorted(patch_dict.iteritems()):
    hdr += 'void R22ParamApplyPatch{}(void);\n'.format(p_name.capitalize())
  hdr += textwrap.dedent('''
      R22Parameter R22ParamGetNextConfigParam(R22Parameter param,
                                              R22Memory mem);
      bool R22ParamGetValuePtr(R22Parameter param, const uint8_t **value_ptr);
      bool R22ParamSetValue(R22Parameter param, uint32_t value);
      bool R22ParamCompare(R22Parameter param, int32_t data_length,
                           const uint8_t *data);
      const R22ParamInfo *R22ParamGetInfo(R22Parameter param);
      ''')
  hdr += _ReadValue(copley)[1]
  hdr += 'const char *R22ParamName(R22Parameter param);\n'
  hdr += '\n'
  hdr += '#endif  // AVIONICS_SERVO_FIRMWARE_R22_PARAM_H_\n'
  return hdr


def GenerateSource(copley, patch_dict):
  """Generate C source file."""

  src = textwrap.dedent('''
      // AUTOMATICALLY GENERATED BY avionics/servo/firmware/generate_r22_param.py

      #include "avionics/servo/firmware/r22_param.h"

      #include <assert.h>
      #include <stdbool.h>
      #include <stdint.h>
      #include <string.h>

      #include "avionics/common/endian.h"
      #include "common/macros.h"
      ''')
  src += _DefineGlobalVariables(copley, patch_dict)
  src += _GetNextConfigParam()
  src += _GetInfo()
  src += _GetValuePtr()
  src += _SetValue()
  src += _Compare()
  src += _ReadValue(copley)[0]
  src += _ParamName(copley)
  return src


def _DefineGlobalVariables(copley, patch_dict):
  """Generate global variables section of C source file."""

  config_default_value = {}
  for entry in copley.table:
    if 'config_value' in entry:
      config_default_value[entry['index']] = entry['config_value']
  (def_data, def_addr, def_length) = _SerializeByIndex(
      copley, config_default_value)
  def_str = ', '.join(['0x%02X' % v for v in def_data])
  src = '\n'
  src += 'static uint8_t g_config[{}] = {{\n'.format(len(def_data))
  src += '  ' + '\n  '.join(textwrap.wrap(def_str, 76)) + '};\n'
  src += '\n'

  patch_src = ''
  for patch_name, patch_values in sorted(patch_dict.iteritems()):
    (data, addr, length) = _SerializeByName(copley, patch_values)
    patch_name = patch_name.capitalize()
    patch_hex = ', '.join(['0x%02X' % v for v in data])
    src += 'static const uint8_t kPatch{}[{}] = {{\n'.format(
        patch_name, len(data))
    src += '  ' + '\n  '.join(textwrap.wrap(patch_hex, 76)) + '};\n'
    src += '\n'
    patch_src += '\nvoid R22ParamApplyPatch{}(void) {{\n'.format(patch_name)
    for name, _ in patch_values.iteritems():
      def_index = copley.by_name[name]['index']
      patch_src += '  memcpy(&g_config[{}], &kPatch{}[{}], {});\n'.format(
          def_addr[def_index], patch_name, addr[def_index],
          def_length[def_index])
    patch_src += '}\n'

  config_list = []
  for entry in copley.table:
    index = entry['index']
    enum = _GetEnumName(entry['desc'])
    addr = -1
    if index in def_addr:
      addr = def_addr[index]
    length = def_length[index]
    mem = entry['bank']
    can_index = entry['can_index']
    can_sub = entry['can_sub']
    serial = entry['serial_id']
    if serial is None:
      # Serial ID 0 is allocated, but these parameters are CAN-only.
      serial = 0
    if 'R' in mem and 'F' in mem:
      mem_str = 'kR22MemoryRam | kR22MemoryFlash'
    elif 'F' in mem:
      mem_str = 'kR22MemoryFlash'
    else:
      mem_str = 'kR22MemoryRam'
    config_str = '{{{}, 0x{:X}, {}, {}, {}, 0x{:X}, {}}}'.format(
        enum, serial, length, mem_str, addr, can_index, can_sub)
    config_list.append(config_str)
  src += 'static const R22ParamInfo kConfig[R22_PARAM_COUNT] = {\n'
  src += '  ' + ',\n  '.join(config_list) + '\n};\n'
  src += '\n'
  src += patch_src
  return src


def _GetNextConfigParam():
  """Generate R22ParamGetNextConfigParam() C function source."""

  return textwrap.dedent('''
      R22Parameter R22ParamGetNextConfigParam(R22Parameter param,
                                              R22Memory mem) {
        int32_t index = (int32_t)param;
        assert((mem & (kR22MemoryRam | kR22MemoryFlash)) != 0);
        assert(-1 <= index && index < R22_PARAM_COUNT);

        if ((mem & (kR22MemoryRam | kR22MemoryFlash)) == 0) {
          mem |= (kR22MemoryRam | kR22MemoryFlash);
        }
        do {
          ++index;
          index %= R22_PARAM_COUNT;
        } while ((kConfig[index].memory_bank & mem) == 0 ||
                 kConfig[index].config_offset < 0);
        return (R22Parameter)index;
      }
      ''')


def _GetInfo():
  """Generate R22ParamGetInfo() C function source."""

  return textwrap.dedent('''
      const R22ParamInfo *R22ParamGetInfo(R22Parameter param) {
        int32_t index = (int32_t)param;
        if (index < 0 || index >= R22_PARAM_COUNT) {
          return NULL;
        }
        return &kConfig[index];
      }
      ''')


def _GetValuePtr():
  """Generate R22ParamGetValuePtr() C function source."""

  return textwrap.dedent('''
      bool R22ParamGetValuePtr(R22Parameter param,
                               const uint8_t **value_ptr) {
        const R22ParamInfo *info = R22ParamGetInfo(param);

        if (!info) {
          return false;
        }
        if (info->config_offset < 0) {
          assert(false);
          return false;
        }

        *value_ptr = &g_config[info->config_offset];
        return true;
      }
      ''')


def _SetValue():
  """Generate R22ParamSetValue() C function source."""

  return textwrap.dedent('''
      bool R22ParamSetValue(R22Parameter param,
                            uint32_t value) {
        const R22ParamInfo *info = R22ParamGetInfo(param);

        // An invalid parameter ID was specified.
        if (!info) {
          return false;
        }
        // Only set the parameter value if the register is stored in RAM.
        if (!info->memory_bank & kR22MemoryRam) {
          return false;
        }
        // The config_offset is invalid in the const parameter data.
        if (info->config_offset >= ARRAYSIZE(g_config)) {
          assert(false);
          return false;
        }
        // Only set the parameter value if the register is stored in the config
        // array.
        if (info->config_offset < 0) {
          return false;
        }

        assert(info->config_offset + info->addr_length < ARRAYSIZE(g_config));
        uint8_t *addr = &g_config[info->config_offset];
        switch (info->addr_length) {
          case 1:
            WriteUint8Be(value, addr);
            break;
          case 2:
            WriteUint16Be(value, addr);
            break;
          case 4:
            WriteUint32Be(value, addr);
            break;
          default:
            assert(false);
            return false;
        }
        return true;
      }
      ''')


def _Compare():
  """Generate R22ParamCompare() C function source."""

  return textwrap.dedent('''
      bool R22ParamCompare(R22Parameter param, int32_t data_length,
                           const uint8_t *data) {
        int16_t index = (int16_t)param;

        assert(0 <= index && index < R22_PARAM_COUNT);
        assert(data_length > 0);
        assert(data != NULL);

        bool match = (0 <= index && index < R22_PARAM_COUNT);
        if (match) {
          int32_t addr_length = kConfig[index].addr_length;
          int16_t offset = kConfig[index].config_offset;
          assert(offset >= 0);
          const uint8_t *addr = &g_config[offset];
          int32_t i = 0;
          for (i = 0; i < addr_length && i < data_length && match; ++i) {
            match = (addr[i] == data[i]);
          }
          for ( ; i < addr_length && match; ++i) {
            match = (addr[i] == 0);
          }
          for ( ; i < data_length && match; ++i) {
            match = (data[i] == 0);
          }
        }
        return match;
      }
      ''')


def _ReadValue(copley):
  """Generate individual read functions for each parameter."""

  type_map = {'c': 'char',
              'b': 'int8', 'B': 'uint8',
              'h': 'int16', 'H': 'uint16',
              'i': 'int32', 'I': 'uint32'}
  src = ''
  hdr = ''
  for entry in copley.table:
    if entry['typ']:
      typ = entry['typ']
      name = entry['name']
      if 'config_value' in entry:
        length = len(entry['config_value'])
      else:
        length = 1
      if length == 1:
        c_type = type_map[typ[0]]
        proto = '{}_t R22ParamRead{}(const uint8_t *addr)'.format(
            c_type, name)
        src += '\n'
        src += '{} {{\n'.format(proto)
        src += '  assert(addr != NULL);\n'
        src += '\n'
        src += '  {}_t value;\n'.format(c_type)
        src += '  Read{}Be(addr, &value);\n'.format(
            c_type.capitalize())
        src += '  return value;\n'
        src += '}\n'
        hdr += '{};\n'.format(proto)
  return (src, hdr)


def _ParamName(copley):
  """Generate enumeration of parameter names."""

  src = '\n'
  src += 'const char *R22ParamName(R22Parameter param) {\n'
  src += '  switch (param) {\n'
  for entry in copley.table:
    src += '    case {}:\n'.format(_GetEnumName(entry['desc']))
    src += '      return "{}";\n'.format(_GetName(entry['name']))
  src += '    default:\n'
  src += '      return "<Unknown>";\n'
  src += '  }\n'
  src += '}\n'
  return src


def main(argv):
  """Entry point."""

  gflags.RegisterValidator('ccx_file',
                           lambda f: f.endswith('.ccx'),
                           message='Expected a .ccx file.')
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], FLAGS)
    sys.exit(1)

  copley = CopleyConfiguration()
  if FLAGS.dict_file:
    if FLAGS.dict_file.endswith('.yaml'):
      copley.LoadYamlDictionary(FLAGS.dict_file)
    else:
      copley.LoadCopleyDictionary(FLAGS.dict_file)
  if FLAGS.ccx_file:
    copley.LoadConfiguration(FLAGS.ccx_file)
  if FLAGS.yaml_output:
    with open(FLAGS.yaml_output, 'w') as f:
      f.write(copley.GenerateYamlDictionary())

  patch_dict = GetPatchConfiguration()
  if FLAGS.header_file:
    with open(FLAGS.header_file, 'w') as f:
      f.write(GenerateHeader(copley, patch_dict))
  if FLAGS.source_file:
    with open(FLAGS.source_file, 'w') as f:
      f.write(GenerateSource(copley, patch_dict))


if __name__ == '__main__':
  main(sys.argv)
