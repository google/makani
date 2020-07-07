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

"""Generate network dashboard from json data on stdin."""

import ctypes
import json
import os
import sys

from django.conf import settings
from django.template import Context
from django.template import loader
from django.utils.safestring import mark_safe
import gflags
from makani.avionics.common import pack_avionics_messages
from makani.control import pack_control_telemetry
from makani.control import pack_ground_telemetry

gflags.DEFINE_string('output_dir', None, 'Directory to write html files.')
gflags.MarkFlagAsRequired('output_dir')
FLAGS = gflags.FLAGS

TRUNK_LINKS = {
    'eop_b': {
        'down': {
            'title': 'EOP B down',
            'gauge_title': 'EOP \\u2b07',
            'source': u'switches.cs_b.18',
            'target': 10e6,
            'yellow': 20e6,
            'red': 30e6,
        },
        'up': {
            'title': 'EOP B up',
            'gauge_title': 'EOP \\u2b06',
            'source': u'switches.cs_gs_b.18',
            'target': 5e6,
            'yellow': 7.5e6,
            'red': 10e6,
        },
    },
    'wifi_a': {
        'down': {
            'title': 'WiFi A down',
            'gauge_title': 'WiFi \\u2b07',
            'source': u'switches.cs_a.22',
            'target': 7e6,
            'yellow': 14e6,
            'red': 21e6,
        },
        'up': {
            'title': 'WiFi A up',
            'gauge_title': 'WiFi \\u2b06',
            'source': u'switches.cs_gs_a.22',
            'target': 1e6,
            'yellow': 2e6,
            'red': 5e6,
        },
    },
    'pof_a': {
        'down': {
            'title': 'POF A down',
            'gauge_title': 'POF \\u2b07',
            'source': u'switches.cs_a.20',
            'target': 30e6,
            'yellow': 35e6,
            'red': 40e6,
        },
        'up': {
            'title': 'POF A up',
            'gauge_title': 'POF \\u2b06',
            'source': u'switches.cs_gs_a.20',
            'target': 5e6,
            'yellow': 7.5e6,
            'red': 10e6,
        },
    },
    'joystick': {
        'down': {
            'title': 'Joystick radio down',
            'gauge_title': 'JS \\u2b07',
            'source': u'switches.cs_b.23',
            'target': .2e6,
            'yellow': .25e6,
            'red': .3e6,
        },
        'up': {
            'title': 'Joystick radio up',
            'gauge_title': 'JS \\u2b06',
            'source': u'switches.joystick_a.3',
            'target': .1e6,
            'yellow': .15e6,
            'red': .2e6,
        },
    },
    'operator': {
        'in': {
            'title': 'Operator in',
            'gauge_title': 'OP \\u2b05',
            'source': u'switches.remote_command.5',
            'target': 50e6,
            'yellow': 75e6,
            'red': 100e6,
        },
    },
    'recorder_q7_platform': {
        'in': {
            'title': 'Platform recorder in',
            'gauge_title': 'REC \\u2b05',
            'source': u'switches.recorder_platform.4',
            'target': 30e6,
            'yellow': 75e6,
            'red': 100e6,
        },
    },
    'recorder_q7_wing': {
        'in': {
            'title': 'Wing recorder in',
            'gauge_title': 'REC \\u2b05',
            'source': u'switches.recorder_wing.4',
            'target': 30e6,
            'yellow': 75e6,
            'red': 100e6,
        },
    },
    'controller_a': {
        'in': {
            'title': 'Controller A in',
            'gauge_title': 'C \\u2b05',
            'source': u'switches.fc_a.4',
            'target': 30e6,
            'yellow': 50e6,
            'red': 80e6,
        },
    },
    'controller_b': {
        'in': {
            'title': 'Controller B in',
            'gauge_title': 'C \\u2b05',
            'source': u'switches.fc_b.4',
            'target': 30e6,
            'yellow': 50e6,
            'red': 80e6,
        },
    },
    'controller_c': {
        'in': {
            'title': 'Controller C in',
            'gauge_title': 'C \\u2b05',
            'source': u'switches.fc_c.4',
            'target': 30e6,
            'yellow': 50e6,
            'red': 80e6,
        },
    },
}

# Python ctype name to C type name
TYPE_DICT = {
    'c_byte': 'int8_t',
    'c_ubyte': 'uint8_t',
    'c_short': 'int16_t',
    'c_ushort': 'uint16_t',
    'c_int': 'int32_t',
    'c_uint': 'uint32_t',
    'c_float': 'float',
}


def _LoadClass(name):
  if hasattr(pack_control_telemetry, name):
    return getattr(pack_control_telemetry, name)
  elif hasattr(pack_control_telemetry, name + 'Message'):
    return getattr(pack_control_telemetry, name + 'Message')
  elif hasattr(pack_ground_telemetry, name + 'Message'):
    return getattr(pack_ground_telemetry, name + 'Message')
  else:
    return getattr(pack_avionics_messages, name + 'Message')


def _ClassInfo(c, name='', short_name='', extent=1):
  """Returns a python object describing the format of the ctypes class."""
  type_name = c.__name__
  if type_name in TYPE_DICT:
    type_name = TYPE_DICT[type_name]
  info = {
      'name': name,
      'short_name': short_name,
      'type': type_name,
      'size': ctypes.sizeof(c) * extent,
      'fields': [],
  }

  # pylint: disable=protected-access
  if hasattr(c, '_fields_'):
    for f_name, f_class in c._fields_:
      f_short_name = f_name
      extent = 1
      if issubclass(f_class, ctypes.Array):
        f_name = '{name}[{extent}]'.format(name=f_name,
                                           extent=f_class._length_)
        extent = f_class._length_
        f_class = f_class._type_
      info['fields'].append(_ClassInfo(f_class, f_name, f_short_name, extent))
  # pylint: enable=protected-access
  return info


def _ImportStats(stream):
  """Load bandwidth statistics from json stream."""

  stat_db = {}
  data = json.load(stream)

  for stat in data:
    source = stat[u'source']
    if source not in stat_db:
      stat_db[source] = []
    stat_db[source].append(stat)

  return stat_db


def _ProcessLinks(stat_db):
  """Process link information and generate data for HTML templates."""

  links = []
  msgs = set()
  for link_type, link_data in TRUNK_LINKS.iteritems():
    for direction, dir_data in link_data.iteritems():
      div = link_type + '_' + direction
      title = dir_data['title']
      link = {
          'div': div,
          'title': title,
          'gauge_title': dir_data['gauge_title'],
          'target': dir_data['target'] / 1000000.0,
          'yellow': dir_data['yellow'] / 1000000.0,
          'red': dir_data['red'] / 1000000.0,
          'charts': [],
      }

      total_bps = 0
      stats = []
      if dir_data['source'] in stat_db:
        for stat in stat_db[dir_data['source']]:
          bps = stat[u'bytes_per_sec'] * 8
          total_bps += bps
          stats.append((stat[u'message'], bps))
          msgs.add(str(stat[u'message']))

      link['bps'] = total_bps / 1000000.0

      stats = sorted(stats, key=lambda stat: stat[1], reverse=True)
      stats.insert(0, ('Message Type', 'bits per sec'))

      data_str = mark_safe(json.dumps(stats))

      link['charts'].append({
          'type': 'pie',
          'tab': 'tab_' + div,
          'div': div + '_pie',
          'title': title,
          'data': data_str,
      })
      link['charts'].append({
          'type': 'bar',
          'tab': 'tab_' + div,
          'div': div + '_bar',
          'title': title,
          'data': data_str,
      })
      links.append(link)

  return (links, msgs)


def _GenerateHTML(template_name, data):
  t = loader.get_template(template_name)
  c = Context(data)
  return t.render(c)


def _GenerateDashboard(links):
  return _GenerateHTML('network_dashboard.html', {'links': links})


def _GenerateMessage(msg):
  return _GenerateHTML('network_message.html', {'msg': msg})


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], FLAGS)
    sys.exit(1)

  settings.configure(
      TEMPLATE_DIRS=(os.path.dirname(os.path.realpath(__file__)),),
      TEMPLATE_DEBUG=True,
  )

  stat_db = _ImportStats(sys.stdin)
  links, msgs = _ProcessLinks(stat_db)
  for name in msgs:
    c = _LoadClass(name)
    info = _ClassInfo(c)
    file_name = os.path.join(FLAGS.output_dir, name + '.html')
    with open(file_name, 'w') as f:
      f.write(_GenerateMessage(info))
  with open(os.path.join(FLAGS.output_dir, 'index.html'), 'w') as f:
    f.write(_GenerateDashboard(links))


if __name__ == '__main__':
  main(sys.argv)
