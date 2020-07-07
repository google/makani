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

"""Simple tool to decode AIO messages."""

from __future__ import absolute_import
from __future__ import print_function
import collections
import curses
import datetime
import errno
import math
import pprint
import socket
import sys
import time

import gflags
from makani.avionics.common import aio
from makani.avionics.common import aio_version
from makani.avionics.network import aio_node
from makani.avionics.network import message_type
from makani.lib.python import c_helpers
from makani.lib.python import ctype_util

aio_node_helper = c_helpers.EnumHelper('AioNode', aio_node)
message_type_helper = c_helpers.EnumHelper('MessageType', message_type)

FLAGS = gflags.FLAGS

gflags.DEFINE_integer('max_age', 0, 'Age out messages after <max_age> seconds.'
                      ' (curses mode)')
gflags.DEFINE_bool('body', True, 'Show message body.')
gflags.DEFINE_bool('curses', True, 'Use curses view.')
gflags.DEFINE_bool('enumerate', False, 'Show the received packet number.')
gflags.DEFINE_bool('header', True, 'Show message header.')
gflags.DEFINE_bool('pretty', True, 'Pretty-print output.')
gflags.DEFINE_bool('prune', False, 'Prune common structure off the message '
                   'before printing.')
gflags.DEFINE_bool('sequence', False, 'Show the header sequence number.')
gflags.DEFINE_bool('timestamp', False, 'Show the header timestamp.')
gflags.DEFINE_bool('stats', False, 'Compute message statistics.')
gflags.DEFINE_integer('count', None, 'Stop after receiving <count> packets.')
gflags.DEFINE_list('filter', [], 'List of parameters to filter data on.')
gflags.DEFINE_bool('invalid', False, 'Show invalid messages.')
gflags.DEFINE_list('sources', [], 'List of source nodes to display.  By '
                   'default all sources will be displayed.  Valid sources are:'
                   '  %s.' % ', '.join(aio_node_helper.ShortNames()))
gflags.DEFINE_bool('time', True, 'Show message receive time.')
gflags.DEFINE_list('types', None, 'List of message types to display.  By '
                   'default all types will be displayed.  Valid types are:  %s.'
                   % ', '.join(message_type_helper.ShortNames()))


class MessageStats(object):
  """Compute and store message summary statistics."""

  def __init__(self):
    # All delta values in us.
    self._delta_send_min = 2 ** 31
    self._delta_send_max = 0
    self._delta_send_sum = 0
    self._delta_send_squared_sum = 0
    self._delta_recv_min = 2 ** 31
    self._delta_recv_max = 0
    self._delta_recv_sum = 0
    self._delta_recv_squared_sum = 0
    self._delta_count = 0
    self._recv_count = 0
    self._sequence_count = 0
    self._first_message = True

  def ProcessMessageHeader(self, header):
    """Update intermediate counters with message header values."""
    recv_time = time.time()
    self._recv_count += 1
    if not self._first_message:
      if header.timestamp < self._last_send_time_us:
        self._last_send_time_us -= 2 ** 32
      if header.sequence < self._last_sequence:
        self._last_sequence -= 2 ** 16
      self._sequence_count += header.sequence - self._last_sequence
      if header.sequence == (self._last_sequence + 1):
        self._delta_count += 1
        delta_recv = int((recv_time - self._last_recv_time) * 1e6)
        delta_send = header.timestamp - self._last_send_time_us
        if self._delta_send_min > delta_send:
          self._delta_send_min = delta_send
        if self._delta_send_max < delta_send:
          self._delta_send_max = delta_send
        self._delta_send_sum += delta_send
        self._delta_send_squared_sum += delta_send ** 2
        if self._delta_recv_min > delta_recv:
          self._delta_recv_min = delta_recv
        if self._delta_recv_max < delta_recv:
          self._delta_recv_max = delta_recv
        self._delta_recv_sum += delta_recv
        self._delta_recv_squared_sum += delta_recv ** 2
    else:
      self._sequence_count = 1

    self._first_message = False
    self._last_sequence = header.sequence
    self._last_send_time_us = header.timestamp
    self._last_recv_time = recv_time

  def GetStats(self):
    """Get current message stats (all deltas in milliseconds)."""
    def _Sigma(x_sum, x_squared_sum, n):
      if n == 0:
        return float('nan')
      return math.sqrt(float(x_squared_sum) / n - (float(x_sum) / n) ** 2)
    def _Mean(x_sum, n):
      if n == 0:
        return float('nan')
      return float(x_sum) / n
    return {'delta_send_min': self._delta_send_min / 1e3,
            'delta_send_max': self._delta_send_max / 1e3,
            'delta_send_mean':
                _Mean(self._delta_send_sum, self._delta_count) / 1e3,
            'delta_send_sigma':
                _Sigma(self._delta_send_sum, self._delta_send_squared_sum,
                       self._delta_count) / 1e3,
            'delta_recv_min': self._delta_recv_min / 1e3,
            'delta_recv_max': self._delta_recv_max / 1e3,
            'delta_recv_mean':
                _Mean(self._delta_recv_sum, self._delta_count) / 1e3,
            'delta_recv_sigma':
                _Sigma(self._delta_recv_sum, self._delta_recv_squared_sum,
                       self._delta_count) / 1e3,
            'recv_rate': float(self._recv_count) / self._sequence_count * 100}


def _GenerateFilter(params):
  """Generate a filter object using dot separated object paths.

  A list of dot separated paths (foo, foo.bar, foo.baz) is merged into a
  dict which can be used as a filter on a received object.  Objects are parsed
  until a falsy value (such as {}) is reached which includes everything inside
  that part of the object.  Missing values in a dict at a given level are
  excluded.  Currently arrays are not filtered and array indices should simply
  be excluded in a path.

  Args:
    params: List of object paths to filter on.

  Returns:
    A dict which can be matched up with fields in a received object.
  """
  filt = {}
  for param in params:
    path = filt
    for token in param.split('.'):
      if token not in path:
        path[token] = {}
      path = path[token]
  return filt


def _ApplyFilter(filt, obj):
  """Apply the filter from _GenerateFilter to incoming message object."""
  if not filt:
    return obj
  if isinstance(obj, list):
    # Lists are not filtered for now.
    return [_ApplyFilter(filt, o) for o in obj]
  elif isinstance(obj, dict):
    return {key: _ApplyFilter(filt[key], obj[key])
            for key in obj if key in filt}
  else:
    # Other types are not filtered.
    return obj


def _PruneMessage(obj):
  """Remove any common structure in the message object before printing."""
  if isinstance(obj, list) and len(obj) == 1:
    return _PruneMessage(obj[0])
  elif isinstance(obj, dict) and len(obj) == 1:
    for v in obj.values():
      return _PruneMessage(v)
  else:
    return obj


stats = collections.defaultdict(MessageStats)


def _FormatMessage(source, header, message, recv_count):
  """Format a message for output to screen."""
  output = {'local_time': datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3],
            'source_ip': source, 'recv_count': recv_count,
            'sequence': header.sequence,
            'timestamp': '%08.6f' % (header.timestamp / 1e6)}
  try:
    output['type'] = message_type_helper.ShortName(header.type)
  except c_helpers.EnumError:
    output['type'] = 'UnknownMessage:%d' % header.type
  try:
    output['source_node'] = aio_node_helper.ShortName(header.source)
  except c_helpers.EnumError:
    output['source_node'] = 'UnknownNode:%d' % header.source
  if FLAGS.stats:
    stats[(header.type, header.source)].ProcessMessageHeader(header)
  # Only decode the message if we need to.
  if FLAGS.body:
    msg_obj = ctype_util.CTypeToPython(message)
    msg_obj = _ApplyFilter(msg_filter, msg_obj)
    if FLAGS.prune:
      msg_obj = _PruneMessage(msg_obj)
    if FLAGS.pretty:
      msg_str = '\n' + pprint.pformat(msg_obj)
      output['msg_content'] = '\n  '.join(msg_str.split('\n'))
    else:
      if output['type'] == 'Stdio':
        output['msg_content'] = str(msg_obj)
      else:
        output['msg_content'] = repr(msg_obj)
    output['msg_content'] = output['msg_content'].rstrip('\n')
  output['warning'] = ''
  if header.version != aio_version.AIO_VERSION:
    output['warning'] = ('[WARNING: AIO version was {:04X}, expected {:04X}]'
                         .format(header.version, aio_version.AIO_VERSION))
  format_parts = []
  if FLAGS.time:
    format_parts.append('[{local_time}]')
  if FLAGS.enumerate:
    format_parts.append('[#{recv_count}]')
  if FLAGS.header:
    format_parts += ['{warning}', '{type}', '{source_node}', '({source_ip})']
  if FLAGS.sequence or FLAGS.timestamp:
    seq_parts = []
    if FLAGS.sequence:
      seq_parts.append('{sequence}')
    if FLAGS.timestamp:
      seq_parts.append('{timestamp}')
    format_parts.append('[%s]' % ' @ '.join(seq_parts))
  if FLAGS.stats:
    output.update(stats[(header.type, header.source)].GetStats())
    if math.isnan(output['delta_send_mean']):
      format_parts.append('[{recv_rate:1.2f}% rx]')
    else:
      format_parts.append(
          '[{recv_rate:1.2f}% rx (delta msec tx/rx) '
          'avg:{delta_send_mean:1.2f}/{delta_recv_mean:1.2f} '
          'min:{delta_send_min:1.2f}/{delta_recv_min:1.2f} '
          'max:{delta_send_max:1.2f}/{delta_recv_max:1.2f} '
          'sigma:{delta_send_sigma:1.2f}/{delta_recv_sigma:1.2f}]')

  format_string = ' '.join(format_parts)
  if FLAGS.body:
    if format_string:
      format_string += ': '
    format_string += '{msg_content}'
  return format_string.format(**output)


def CursesMain():
  recv_count = 0
  message_buf = {}
  message_timeout = {}
  last_refresh = 0
  try:
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    screen_size = stdscr.getmaxyx()
    pad = curses.newpad(1000, screen_size[1])
    pad.keypad(1)
    pad.nodelay(1)
    display_row = 0
    redraw = False
    while not FLAGS.count or recv_count < FLAGS.count:
      # User input.
      while True:
        user_input = pad.getch()
        if user_input == curses.ERR:
          break
        redraw = True
        if user_input == curses.KEY_PPAGE:
          display_row -= screen_size[0]
        elif user_input == curses.KEY_NPAGE:
          display_row += screen_size[0]
        elif user_input == curses.KEY_UP:
          display_row -= 1
        elif user_input == curses.KEY_DOWN:
          display_row += 1
        if display_row < 0:
          display_row = 0
        elif display_row > (pad.getmaxyx()[0] - screen_size[0]):
          display_row = pad.getmaxyx()[0] - screen_size[0]

      # Receive and format messages.
      try:
        source, header, message = client.Recv(accept_invalid=FLAGS.invalid)
        redraw = True
        recv_count += 1
        key = (source, header.source, header.type)
        message_buf[key] = _FormatMessage(source, header, message, recv_count)
        if FLAGS.max_age:
          message_timeout[key] = time.time() + FLAGS.max_age
      except socket.timeout:
        pass
      except socket.error as e:
        if e.errno != errno.EINTR:
          raise e

      # Update display.
      # Limit refresh interval for performance.  Use a number relatively prime
      # to 1000 to avoid biasing time display.
      current_time = time.time()
      if redraw and (current_time - last_refresh) > 0.073:
        # Age out messages.
        expire_list = []
        for key, timeout in message_timeout.items():
          if timeout < current_time:
            expire_list.append(key)
        for key in expire_list:
          del message_buf[key]
          del message_timeout[key]
        redraw = False
        last_refresh = current_time
        pad.erase()
        row = 0
        for k in sorted(message_buf):
          try:
            pad.addstr(row, 0, message_buf[k])
          except curses.error:
            pad.addstr(pad.getmaxyx()[0] - 1, 0, '... (output truncated)  ')
            row = pad.getmaxyx()[0] - 1
            break
          row = pad.getyx()[0] + 1
          if row > display_row + screen_size[0]:
            break
        if row <= display_row:
          display_row = max(0, row - 1)
        screen_size = stdscr.getmaxyx()
        pad.refresh(display_row, 0, 0, 0, screen_size[0] - 1,
                    min(screen_size[1] - 1, pad.getmaxyx()[1]))
  except BaseException as e:  # pylint: disable=broad-except
    curses.nocbreak()
    pad.keypad(0)
    curses.echo()
    curses.endwin()
    if isinstance(e, KeyboardInterrupt):
      sys.exit(0)
    else:
      raise e


def ConsoleMain():
  recv_count = 0
  try:
    while not FLAGS.count or recv_count < FLAGS.count:
      try:
        source, header, message = client.Recv(accept_invalid=FLAGS.invalid)
        recv_count += 1
        print(_FormatMessage(source, header, message, recv_count))
      except socket.timeout:
        pass
  except BaseException as e:  # pylint: disable=broad-except
    sys.stdout.write('\n')
    if isinstance(e, KeyboardInterrupt):
      sys.exit(0)
    else:
      raise e


if __name__ == '__main__':
  try:
    argv = FLAGS(sys.argv)
  except gflags.FlagsError as e:
    print('%s\nUsage: %s ARGS\n%s'
          % (e, sys.argv[0], FLAGS))
    sys.exit(1)
  types = FLAGS.types
  if not types:
    types = message_type_helper.ShortNames()
  client = aio.AioClient(['kMessageType' + t for t in types], timeout=0.05,
                         allowed_sources=['kAioNode' + t
                                          for t in FLAGS.sources])
  msg_filter = _GenerateFilter(FLAGS.filter)
  if FLAGS.curses:
    CursesMain()
  else:
    ConsoleMain()
