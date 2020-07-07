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


"""Quick-and-dirty plotting tool to display network activity.

This reads an activity file written by validate_pcap.cc
and displays both network usage and missing packets.
"""

import imp
import os
import sys

import gflags
import matplotlib.patches as patches
import matplotlib.pyplot as pyplot

gflags.DEFINE_string('file', None, 'Name of input python file.')
gflags.DEFINE_bool('sender_timestamp', False,
                   'Use sender AIO timestamps instead of capture time.')
gflags.RegisterValidator('file',
                         lambda f: f and os.path.basename(f),
                         '--file must have a valid file name.')


def draw(plot, data, x_key, y_key, len_key, edge_color, data_color, y_offset):
  """Draw the data to the plot.

  Draws a rectangle for each dict in data.

  Args:
    plot: a matplotlib subplot.
    data: an array of points as
      { x_key: x_coordinate, y_key: y_coordinate, len_key: length_of_rect }.
    x_key: a string key into the dicts in data.
    y_key: a string key into the dicts in data.
    len_key: a string key into the dicts in data.
    edge_color: the color used for the circumference of the rectangle drawn.
    data_color: the color used for the interior of the rectangle drawn.
    y_offset: an extra offset to add to the y coordinate of the rectangle.

  Returns:
    The bounding box of the boxes drawn.
  """
  def handle_minmax(cur_min, cur_max, cur, size):
    return (min(cur_min, cur), max(cur_max, cur + size))

  height = 0.4
  min_x = max_x = data[0][x_key]
  min_y = max_y = data[0][y_key]
  for p in data:
    y_scale = 1
    x = p[x_key]
    if x < max_x:
      # When we see packets together whose ts_us and length collectively
      # indicate a full network pipe, we draw them at half height until the
      # congestion clears.
      y_scale = 0.5
      x = max_x
    y = p[y_key] + y_offset
    length = p[len_key] * 8 / 100
    min_x, max_x = handle_minmax(min_x, max_x, x, length)
    min_y, max_y = handle_minmax(min_y, max_y, y, 1)
    plot.add_patch(
        patches.Rectangle(
            (x, y),
            length, height * y_scale,
            facecolor=data_color, edgecolor=edge_color
        )
    )
    plot.add_patch(
        patches.Rectangle(
            (x, -2 + y_offset),
            length, height * y_scale,
            facecolor=data_color, edgecolor=edge_color
        )
    )
  return (min_x, max_x, min_y, max_y)


def main(argv):
  flags = gflags.FLAGS
  try:
    argv = flags(argv)
  except gflags.FlagsError, e:
    print '%s\nUsage: %s ARGS\n%s' % (e, sys.argv[0], flags)
    sys.exit(1)

  with open(flags.file, 'r') as input_file:
    data = imp.load_source('data', '', input_file)
  packets = data.packets
  gaps = data.gaps
  if not (packets or gaps):
    print 'File contains no data.'
    return

  fig = pyplot.figure()
  plot = fig.add_subplot(111, aspect='auto')

  if flags.sender_timestamp:
    x_key = 'timestamp'
  else:
    x_key = 'ts_us'
  y_key = 'message_type'
  len_key = 'len'

  packet_range = gap_range = None
  if packets:
    packet_range = draw(plot, packets, x_key, y_key, len_key, 'black',
                        'blue', 0)
  if gaps:
    gap_range = draw(plot, gaps, x_key, y_key, len_key, 'red', 'pink', 0.5)
  else:
    gap_range = packet_range

  if not packet_range:
    packet_range = gap_range
  plot_xrange = (min(packet_range[0], gap_range[0]),
                 max(packet_range[1], gap_range[1]))
  plot_yrange = (-2, max(packet_range[3], gap_range[3]))

  pyplot.xlabel(x_key)
  pyplot.ylabel(y_key)
  pyplot.yticks(range(0, plot_yrange[1] + 1, 5))
  plot.set_xbound(lower=plot_xrange[0], upper=plot_xrange[1])
  plot.set_ybound(lower=plot_yrange[0], upper=plot_yrange[1])
  pyplot.tight_layout()
  pyplot.show()

if __name__ == '__main__':
  main(sys.argv)
