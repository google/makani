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


"""Command line tool for joystick AIO node."""

import sys

import gflags
from makani.avionics.common import aio
from makani.avionics.common import pack_avionics_messages
from makani.lib.python import c_helpers
from makani.system import labels

switch_position_helper = c_helpers.EnumHelper('JoystickSwitchPositionLabel',
                                              labels,
                                              prefix='kJoystickSwitchPosition')


AXES = ['roll', 'pitch', 'yaw', 'throttle']
SWITCHES = ['tri', 'momentary']

# Values determined empirically.
_AXIS_CALIBRATION_THRESHOLD = 0xf000  # [counts]
_SWITCH_CALIBRATION_THRESHOLD = 0x14000  # [counts]

gflags.DEFINE_bool('sample', False,
                   'Output a single sample from the joystick.')
gflags.DEFINE_bool('sample_raw', False,
                   'Output a single raw sample from the joystick.')
gflags.DEFINE_bool('dump', False,
                   'Continuously output samples from the joystick.')
gflags.DEFINE_bool('dump_raw', False,
                   'Continuously output raw samples from the joystick.')
gflags.DEFINE_bool('calibrate', False,
                   'Run calibration procedure.')

FLAGS = gflags.FLAGS


def _IsPositiveAxis(axis):
  return axis == 'throttle'


def _EnableRawSamples(aio_client):
  message = pack_avionics_messages.JoystickCommandMessage()
  message.enable_raw = True
  aio_client.Send(message, 'kMessageTypeJoystickCommand', 'kAioNodeControllerA')


def _DisableRawSamples(aio_client):
  message = pack_avionics_messages.JoystickCommandMessage()
  message.enable_raw = False
  aio_client.Send(message, 'kMessageTypeJoystickCommand', 'kAioNodeControllerA')


def _GetRawSample(aio_client):
  try:
    (_, _, msg) = aio_client.Recv()
    return msg.channel

  except KeyboardInterrupt:
    _DisableRawSamples(aio_client)
    aio_client.Close()
    sys.exit()


def _CalibrateAxis(aio_client):
  axis_data = [{'min': 0xffffffff, 'max': 0x0000000} for _ in xrange(7)]
  _EnableRawSamples(aio_client)
  while True:
    raw_data = _GetRawSample(aio_client)
    for chan in xrange(7):
      value = raw_data[chan]

      if axis_data[chan]['max'] < value:
        axis_data[chan]['max'] = value
      if axis_data[chan]['min'] > value:
        axis_data[chan]['min'] = value

      diff = axis_data[chan]['max'] - axis_data[chan]['min']
      center = axis_data[chan]['min'] + diff / 2

      # Calibration finished when the range of values is above
      # _AXIS_CALIBRATION_THRESHOLD and the joystick returns to near center.
      if (diff > _AXIS_CALIBRATION_THRESHOLD
          and abs(center - value) < _AXIS_CALIBRATION_THRESHOLD / 8):
        _DisableRawSamples(aio_client)
        return (chan, axis_data[chan])


def _CalibrateSwitch(aio_client):
  axis_data = [{'min': 0xffffffff, 'max': 0x0000000} for _ in xrange(7)]
  _EnableRawSamples(aio_client)
  while True:
    raw_data = _GetRawSample(aio_client)
    for chan in xrange(7):
      value = raw_data[chan]

      if axis_data[chan]['max'] < value:
        axis_data[chan]['max'] = value
      if axis_data[chan]['min'] > value:
        axis_data[chan]['min'] = value

      diff = axis_data[chan]['max'] - axis_data[chan]['min']

      # Calibration finished when the range of values is above
      # _SWITCH_CALIBRATION_THRESHOLD and the switch returns to its
      # down position.
      if (diff > _SWITCH_CALIBRATION_THRESHOLD
          and value - axis_data[chan]['min'] < 0x100):
        _DisableRawSamples(aio_client)
        return (chan, axis_data[chan])


def _DecodeSwitch(switch):
  if switch == switch_position_helper.Value('Up'):
    return 'up'
  elif switch == switch_position_helper.Value('Middle'):
    return 'mid'
  elif switch == switch_position_helper.Value('Down'):
    return 'down'
  else:
    return 'unknown'


def _DecodeArmed(value):
  if value == pack_avionics_messages.kJoystickArmed:
    return 'armed'
  else:
    return 'disarmed'


def _PrintSample(aio_client):
  try:
    (_, _, msg) = aio_client.Recv()
    print('roll: %f, pitch: %f, yaw: %f, throttle: %f' %
          (msg.roll, msg.pitch, msg.yaw, msg.throttle))
    print('tri_switch: %s, momentary_switch: %s, armed_switch: %s' %
          (_DecodeSwitch(msg.tri_switch),
           _DecodeSwitch(msg.momentary_switch),
           _DecodeArmed(msg.armed_switch)))

  except KeyboardInterrupt:
    aio_client.Close()
    sys.exit()


def _PrintRawSample(aio_client):
  try:
    (_, _, msg) = aio_client.Recv()

    for chan in xrange(7):
      sys.stdout.write('%d: %08x  ' % (chan, msg.channel[chan]))
    sys.stdout.write('\n')

  except KeyboardInterrupt:
    _DisableRawSamples(aio_client)
    aio_client.Close()
    sys.exit()


def _Sample():
  aio_client = aio.AioClient(['kMessageTypeJoystickStatus'])
  _PrintSample(aio_client)


def _SampleRaw():
  aio_client = aio.AioClient(['kMessageTypeJoystickRawStatus',
                              'kMessageTypeJoystickCommand'])
  _EnableRawSamples(aio_client)
  _PrintRawSample(aio_client)
  _DisableRawSamples(aio_client)


def _Dump():
  aio_client = aio.AioClient(['kMessageTypeJoystickStatus'])
  while True:
    _PrintSample(aio_client)


def _DumpRaw():
  aio_client = aio.AioClient(['kMessageTypeJoystickRawStatus',
                              'kMessageTypeJoystickCommand'])
  _EnableRawSamples(aio_client)
  while True:
    _PrintRawSample(aio_client)


def _GenerateAxisCalibration(cal):
  string = """\
    '{name}.index': {index:d},
    '{name}.offset': {offset:e},
    '{name}.gain': {gain:e},
    '{name}.min': {min:f},
    '{name}.max': {max:f},
""".format(**cal)
  return string


def _GenerateSwitchCalibration(cal):
  string = """\
    '{name}_switch.index': {index:d},
    '{name}_switch.min': 0x{min:08X},
    '{name}_switch.max': 0x{max:08X},
""".format(**cal)
  return string


def _Calibrate():
  """Run the calibration procedure.

  This functions walks the operator through each of the axes,
  finding their limits and zeros.
  """

  aio_client = aio.AioClient(['kMessageTypeJoystickRawStatus',
                              'kMessageTypeJoystickCommand'])
  axis_data = {}
  _ = raw_input('Center all axes and press enter.')
  for axis in AXES:
    print 'Move %s axis through its limits.' % (axis)
    axis_data[axis] = _CalibrateAxis(aio_client)

  _ = raw_input('Move axes to their zero points and press enter.')
  _EnableRawSamples(aio_client)
  zero_points = _GetRawSample(aio_client)
  _DisableRawSamples(aio_client)

  switch_data = {}
  _ = raw_input('Make sure all switches are in their down state '
                'and press enter.')
  for switch in SWITCHES:
    print 'Move %s switch to its up state and back down.' % (switch)
    switch_data[switch] = _CalibrateSwitch(aio_client)

  print '# Joystick calibration parameters.'
  print 'joystick_calib_params = {'
  for axis in AXES:
    index = axis_data[axis][0]
    min_value = axis_data[axis][1]['min']
    max_value = axis_data[axis][1]['max']
    offset = -zero_points[index]

    # TODO: Deal with axis sign calibration.  Some axes are inverted
    #                (i.e. top is negative and bottom is positive.)
    gain = 1.0 / (max_value - min_value)
    if _IsPositiveAxis(axis):
      axis_min = 0.0
      axis_max = 1.0
    else:
      gain *= 2.0
      axis_min = -1.0
      axis_max = 1.0

    cal = _GenerateAxisCalibration({
        'name': axis,
        'index': index,
        'offset': offset,
        'gain': gain,
        'min': axis_min,
        'max': axis_max,
    })
    print cal,

  for switch in SWITCHES:
    index = switch_data[switch][0]
    min_value = switch_data[switch][1]['min']
    max_value = switch_data[switch][1]['max']

    cal = _GenerateSwitchCalibration({
        'name': switch,
        'index': index,
        'min': min_value,
        'max': max_value,
    })
    print cal,

  print '}'


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], FLAGS)
    sys.exit(1)

  action_taken = False

  if FLAGS.sample:
    _Sample()
    action_taken = True

  if FLAGS.sample_raw:
    _SampleRaw()
    action_taken = True

  if FLAGS.dump:
    _Dump()
    action_taken = True

  if FLAGS.dump_raw:
    _DumpRaw()
    action_taken = True

  if FLAGS.calibrate:
    _Calibrate()
    action_taken = True

  if not action_taken:
    print('No action specified.\n'
          'Usage: {} ARGS\n{}'.format(sys.argv[0], FLAGS))
    sys.exit(1)


if __name__ == '__main__':
  main(sys.argv)
