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

"""Extract flight path from h5 log and produce Google Earth KML file.
"""

import sys
import textwrap

import gflags
import h5py
from makani.common.c_math import coord_trans
from makani.control import control_types
from makani.control import ground_frame
from makani.lib.python import c_helpers
import numpy as np

_FLIGHT_MODE_HELPER = c_helpers.EnumHelper('FlightMode', control_types)

gflags.DEFINE_string('output_file', None, 'Name of output KML file.')
gflags.DEFINE_boolean('quiet', False, 'Suppress progress messages.')
gflags.MarkFlagAsRequired('output_file')
FLAGS = gflags.FLAGS


def GetGsPosLlhVec3(control_telemetry):
  """Get ground station position in latitude, longitude, and height (LLH).

  Args:
    control_telemetry: ControlDebug or ControlTelemetry message
                       structure from h5 log.

  Returns:
    Ground station position in LLH as Vec3 object.
  """

  gs_pos_ecef = control_telemetry['control_input']['gs_gps']['pos']
  gs_pos_ecef_vec3 = coord_trans.Vec3(x=gs_pos_ecef['x'][-1],
                                      y=gs_pos_ecef['y'][-1],
                                      z=gs_pos_ecef['z'][-1])
  gs_pos_llh_vec3 = coord_trans.Vec3()
  coord_trans.EcefToLlh(gs_pos_ecef_vec3, gs_pos_llh_vec3)
  return gs_pos_llh_vec3


def PrintKmlHeader(f, description=None):
  """Write KML header."""
  f.write(textwrap.dedent("""\
    <?xml version="1.0" encoding="UTF-8"?>
    <kml xmlns="http://www.opengis.net/kml/2.2">
    <Document>
    <name>Paths</name>
    """))
  if description is not None:
    f.write('<description>' + description + '</description>\n')

  def PrintStyle(f, name, line_color=None, line_width=None, color=None):
    """Write a KML <Style> tag."""
    f.write('<Style id="%s">' % name)
    if line_color is not None or line_width is not None:
      f.write('<LineStyle>')
      if line_color is not None:
        f.write('<color>%s</color>' % line_color)
      if line_width is not None:
        f.write('<width>%d</width>' % line_width)
      f.write('</LineStyle>')
    if color is not None:
      f.write('<PolyStyle><color>%s</color></PolyStyle>' % color)
    f.write('</Style>\n')

  # Infuriatingly, KML uses the color order alpha-blue-green-red
  # instead of the otherwise-standard alpha-red-green-blue. So HTML
  # color codes need to be permuted.
  hover_modes = ['HoverAscend', 'HoverPayOut', 'HoverFullLength', 'HoverAccel',
                 'HoverTransOut', 'HoverReelIn', 'HoverDescend',
                 'HoverTransformGsDown', 'HoverTransformGsUp']
  for (flight_mode_name, color) in (
      [('Perched', 'ff5733ff'),
       ('PilotHover', 'ff58eeff')] +
      [(hover_mode, 'ff00c3ff') for hover_mode in hover_modes] +
      [('TransIn', 'ff5733ff'),
       ('CrosswindNormal', 'ffb98029'),
       ('CrosswindPrepTransOut', 'ff85a016'),
       ('OffTether', 'ffff007f')]):
    PrintStyle(f, name=flight_mode_name, line_color=color, line_width=4)


def PrintKmlFooter(f):
  """Write KML footer."""
  f.write('</Document>\n</kml>\n')


def PrintCoordsLlh(f, wing_pos_g, g_heading, gs_pos_llh_vec3):
  """Write a list of LLH coordinates.

  Args:
    f: file handle.
    wing_pos_g: wing position in ground coordinates.
    g_heading: Ground frame heading w.r.t. true north.
    gs_pos_llh_vec3: Vec3 object containing ground station LLH.
  """
  wing_pos_g_vec3 = ground_frame.Vec3()
  wing_pos_ned_vec3 = ground_frame.Vec3()
  wing_pos_llh_vec3 = coord_trans.Vec3()

  for wing_pos in wing_pos_g:
    wing_pos_g_vec3.x = wing_pos['x']
    wing_pos_g_vec3.y = wing_pos['y']
    wing_pos_g_vec3.z = wing_pos['z']

    ground_frame.GToNed(wing_pos_g_vec3, g_heading, wing_pos_ned_vec3)

    coord_trans.NedToLlh(coord_trans.Vec3(x=wing_pos_ned_vec3.x,
                                          y=wing_pos_ned_vec3.y,
                                          z=wing_pos_ned_vec3.z),
                         gs_pos_llh_vec3, wing_pos_llh_vec3)

    # Signed Longitude, Signed Latitude, Altitude [m].
    # NOTE: Order is not quite what you would expect!
    f.write('{longitude:12.10f},{latitude:12.10f},{altitude:12.10f}\n'.format(
        latitude=wing_pos_llh_vec3.x,
        longitude=wing_pos_llh_vec3.y,
        altitude=wing_pos_llh_vec3.z))


def PrintCoordSegment(f, wing_pos_g, g_heading, gs_pos_llh_vec3, name=None,
                      description=None, style_url=None):
  """Write a segment of LLH coordinates to a <placemark> tag in a KML file.

  Args:
    f: file handle.
    wing_pos_g: wing position in ground coordinates.
    g_heading: ground frame heading w.r.t. true north.
    gs_pos_llh_vec3: Vec3 object containing ground station LLH.
    name (string, optional): Name of this segment.
    description (string, optional): Description of this segment.
    style_url (string, optional): Name of style to apply.
  """
  f.write('<Placemark>\n')
  if name is not None:
    f.write('<name>' + name + '</name>\n')
  if description is not None:
    f.write('<description>' + description + '</description>\n')
  if style_url is not None:
    f.write('<styleUrl>' + style_url + '</styleUrl>\n')
  f.write(textwrap.dedent("""\
    <LineString>
    <altitudeMode>absolute</altitudeMode>
    <coordinates>
    """))
  PrintCoordsLlh(f, wing_pos_g, g_heading, gs_pos_llh_vec3)
  f.write(textwrap.dedent("""\
    </coordinates>
    </LineString>
    </Placemark>
    """))


def main(argv):
  def PrintUsage(argv, error=None):
    if error:
      print '\nError: %s\n' % error
    print 'Usage: %s --output_file my_file.kml log.h5 [log2.h5 ...]\n%s' % (
        argv[0], FLAGS)

  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    PrintUsage(argv, e)
    sys.exit(1)

  if len(argv) <= 1:
    PrintUsage(argv, 'Must specify at least one logfile.')
    sys.exit(1)

  log_filenames = argv[1:]

  with open(FLAGS.output_file, 'w') as f:
    PrintKmlHeader(f)
    for log_filename in log_filenames:
      if not FLAGS.quiet:
        print 'Processing', log_filename
      log = h5py.File(log_filename, 'r')

      node_name = 'kAioNodeControllerA'
      if 'kMessageTypeControlDebug' in log['messages'][node_name]:
        message_name = 'kMessageTypeControlDebug'
      elif 'kMessageTypeControlTelemetry' in log['messages'][node_name]:
        message_name = 'kMessageTypeControlTelemetry'
      else:
        print 'Could not find control telemetry in {filename}'.format(
            filename=log_filename)
        sys.exit(1)

      control_telemetry = log['messages'][node_name][message_name]['message']

      g_heading = (
          log['parameters']['system_params']['ground_frame']['heading'])
      gs_pos_llh_vec3 = GetGsPosLlhVec3(control_telemetry)

      # Find indicies at which the flight mode changes.
      fm = control_telemetry['flight_mode']
      fm_change_logical = fm[:-1] != fm[1:]
      fm_change_logical[-1] = True
      fm_change_indicies = np.where(fm_change_logical)[0]

      # Process each segment between flight mode changes.
      i_prev = 0
      for i in fm_change_indicies:
        fm = control_telemetry['flight_mode'][i-1]
        fm_name = _FLIGHT_MODE_HELPER.ShortName(int(fm))
        PrintCoordSegment(f, control_telemetry['state_est']['Xg'][i_prev:i],
                          g_heading, gs_pos_llh_vec3,
                          name=fm_name,
                          style_url='#' + fm_name)
        i_prev = i

    PrintKmlFooter(f)

if __name__ == '__main__':
  main(sys.argv)
