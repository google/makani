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

"""Tools for parsing and analyzing AVL files."""

import collections
import importlib
import logging
import sys
import warnings

import numpy


class AvlReader(object):
  """Parses and analyzes AVL files.

  Attributes:
    filename: Filename of the AVL file to be processed.
    avl: Ordered dict that represents the parsed structure of the AVL file.
    properties: Dict that represents properties of the aircraft,
        surfaces, and surface sections.  Its structure mimics that of
        the AVL file itself.
  """

  def __init__(self, filename):
    """Initializes the class by parsing the AVL file."""
    self.filename = filename

    with open(filename, 'r') as f:
      self.avl = self.Parse(f.read())

    self.properties = self.Analyze(self.avl)

  def Analyze(self, avl):
    """Analyze properties of the AVL geometry.

    Args:
      avl: Ordered dict representing a parsed AVL file.

    Returns:
      Dict that represents properties of the aircraft, surfaces, and
      surface sections.  Its structure mimics that of the AVL file
      itself.
    """
    properties = dict()
    properties['surfaces'] = []
    for avl_surface in avl['surfaces']:
      transform = self._GetSurfaceTransformation(avl_surface)

      sections = []
      for avl_section in avl_surface['sections']:
        sections.append(self._CalcSectionProperties(avl_section, transform))

      panels = []
      for section1, section2 in zip(sections[0:-1], sections[1:]):
        panels.append(self._CalcPanelProperties(section1, section2))

      surface = self._CalcSurfaceProperties(sections, panels)
      surface['name'] = avl_surface['name']
      surface['sections'] = sections
      surface['panels'] = panels
      properties['surfaces'].append(surface)

    return properties

  def _CalcSectionProperties(self, avl_section, transform=lambda x: x):
    """Calculates the properties of sections, i.e. stations along the span."""

    # Apply the scaling and offset parameters, if any, from the AVL
    # file.
    chord = avl_section['Chord'] * transform([0.0, 1.0, 0.0])[1]
    leading_edge_avl = transform([avl_section['Xle'],
                                  avl_section['Yle'],
                                  avl_section['Zle']])
    return {
        'chord': chord,
        'incidence': numpy.pi / 180.0 * avl_section['Ainc'],
        'leading_edge_b': numpy.array([-leading_edge_avl[0],
                                       leading_edge_avl[1],
                                       -leading_edge_avl[2]]),
        'quarter_chord_b': numpy.array([-leading_edge_avl[0] - chord / 4.0,
                                        leading_edge_avl[1],
                                        -leading_edge_avl[2]])
    }

  def _CalcPanelProperties(self, section1, section2):
    """Calculates properties of the areas between sections."""
    span = numpy.sqrt(
        (section2['leading_edge_b'][1] - section1['leading_edge_b'][1])**2.0 +
        (section2['leading_edge_b'][2] - section1['leading_edge_b'][2])**2.0)
    area = (section1['chord'] + section2['chord']) * span / 2.0
    taper_ratio = section2['chord'] / section1['chord']
    c = ((2.0 * section1['chord'] + section2['chord']) /
         (section1['chord'] + section2['chord']) / 3.0)
    mean_incidence = (c * section1['incidence'] +
                      (1.0 - c) * section2['incidence'])
    aerodynamic_center_b = (c * section1['quarter_chord_b'] +
                            (1.0 - c) * section2['quarter_chord_b'])

    return {
        'aerodynamic_center_b': aerodynamic_center_b,
        'area': area,
        'mean_aerodynamic_chord': (2.0 / 3.0 * section1['chord'] *
                                   (1.0 + taper_ratio + taper_ratio**2.0) /
                                   (1.0 + taper_ratio)),
        'mean_incidence': mean_incidence,
        'taper_ratio': taper_ratio,
        'span': span,
        'standard_mean_chord': area / span
    }

  def _CalcSurfaceProperties(self, sections, panels):
    """Calculates properties of full surfaces."""
    area = 0.0
    aerodynamic_center_b = numpy.array([0.0, 0.0, 0.0])
    mean_aerodynamic_chord = 0.0
    mean_incidence = 0.0

    for panel in panels:
      area += panel['area']
      aerodynamic_center_b += panel['area'] * panel['aerodynamic_center_b']
      mean_aerodynamic_chord += panel['area'] * panel['mean_aerodynamic_chord']
      mean_incidence += panel['area'] * panel['mean_incidence']

    aerodynamic_center_b /= area
    mean_aerodynamic_chord /= area
    mean_incidence /= area

    # Set the span vector from the leading edge of the first section
    # to the leading edge of the last section.  Ignore the x
    # component.  Choose the direction such that the span is along the
    # surface coordinate y axis.
    span_b = sections[0]['leading_edge_b'] - sections[-1]['leading_edge_b']
    span_b[0] = 0.0
    if abs(span_b[1]) > abs(span_b[2]):
      if span_b[1] < 0.0:
        span_b *= -1.0
    else:
      if span_b[2] < 0.0:
        span_b *= -1.0
    span = numpy.linalg.norm(span_b)

    # Surface coordinates are defined such that they are aligned with
    # body coordinates for horizontal surfaces and are rotated about
    # body x such that surface z is aligned with the *negative* body y
    # for vertical surfaces.  The negative is required to match the
    # convention in AVL.
    surface_x_b = [1.0, 0.0, 0.0]
    surface_y_b = span_b / span
    surface_z_b = numpy.cross(surface_x_b, surface_y_b)

    return {
        'aerodynamic_center_b': aerodynamic_center_b,
        'area': area,
        'aspect_ratio': span * span / area,
        'dcm_b2surface': numpy.array([surface_x_b, surface_y_b, surface_z_b]),
        'mean_aerodynamic_chord': mean_aerodynamic_chord,
        'mean_incidence': mean_incidence,
        'span': span,
        'standard_mean_chord': area / span
    }

  def _GetSurfaceTransformation(self, surface):
    """Returns surface scaling and offset transformation function."""
    if all([k in surface for k in ['Xscale', 'Yscale', 'Zscale']]):
      scale = [surface['Xscale'], surface['Yscale'], surface['Zscale']]
    else:
      scale = [1.0, 1.0, 1.0]

    if all([k in surface for k in ['dX', 'dY', 'dZ']]):
      offset = [surface['dX'], surface['dY'], surface['dZ']]
    else:
      offset = [0.0, 0.0, 0.0]

    return lambda coord: [x * m + b for x, m, b in zip(coord, scale, offset)]

  def PlotGeometry(self):
    """Plots 3-D line drawing of surfaces."""

    # b/120081442: Next lines removed the module initialization load of the
    # matplotlib module which was causing a bazel pip-installed package issue on
    # batch sim workers.
    pyplot = importlib.import_module('matplotlib.pyplot')
    mplot_3d = importlib.import_module('mpl_toolkits.mplot3d')

    # Importing Axes3D has the side effect of enabling 3D projections, but
    # it is not directly used, so we remove it here.
    del mplot_3d.Axes3D

    axes = pyplot.figure().add_subplot(1, 1, 1, projection='3d')
    axes.w_xaxis.set_pane_color((0.8, 0.8, 0.8, 1.0))
    axes.w_yaxis.set_pane_color((0.8, 0.8, 0.8, 1.0))
    axes.w_zaxis.set_pane_color((0.8, 0.8, 0.8, 1.0))
    axes.w_xaxis.gridlines.set_color(('blue'))
    axes.w_yaxis.gridlines.set_color(('blue'))
    axes.w_zaxis.gridlines.set_color(('blue'))

    # The _axinfo update requires additional specification of linestyle and
    # linewidth on our linux distributions in order to function properly.
    axes.w_xaxis._axinfo.update(  # pylint: disable=protected-access
        {'grid': {'color': (0.7, 0.7, 0.7, 1.0), 'linestyle': '-',
                  'linewidth': 0.8}})
    axes.w_yaxis._axinfo.update(  # pylint: disable=protected-access
        {'grid': {'color': (0.7, 0.7, 0.7, 1.0), 'linestyle': '-',
                  'linewidth': 0.8}})
    axes.w_zaxis._axinfo.update(  # pylint: disable=protected-access
        {'grid': {'color': (0.7, 0.7, 0.7, 1.0), 'linestyle': '-',
                  'linewidth': 0.8}})
    axes.set_aspect('equal')
    axes.set_xlabel('x')
    axes.set_ylabel('y')
    axes.set_zlabel('z')
    half_span = self.avl['Bref'] / 2.0
    axes.set_xlim((-half_span * 0.5, half_span * 1.5))
    axes.set_ylim((-half_span, half_span))
    axes.set_zlim((-half_span, half_span))

    color_order = ['black', 'brown', 'red', 'orange', 'yellow', 'green', 'blue',
                   'violet', 'gray']
    legend_plots = []
    legend_labels = []
    for i, surface in enumerate(self.avl['surfaces']):
      transform = self._GetSurfaceTransformation(surface)
      leading_edge_xs = []
      leading_edge_ys = []
      leading_edge_zs = []
      trailing_edge_xs = []
      trailing_edge_ys = []
      trailing_edge_zs = []
      for section in surface['sections']:
        coord = transform([section['Xle'], section['Yle'], section['Zle']])
        leading_edge_xs.append(coord[0])
        leading_edge_ys.append(coord[1])
        leading_edge_zs.append(coord[2])
        coord = transform([section['Xle'] + section['Chord'],
                           section['Yle'],
                           section['Zle']])
        trailing_edge_xs.append(coord[0])
        trailing_edge_ys.append(coord[1])
        trailing_edge_zs.append(coord[2])
      xs = leading_edge_xs + list(reversed(trailing_edge_xs))
      ys = leading_edge_ys + list(reversed(trailing_edge_ys))
      zs = leading_edge_zs + list(reversed(trailing_edge_zs))
      surface_line, = axes.plot(xs + [xs[0]], ys + [ys[0]], zs + [zs[0]],
                                color=color_order[i])
      legend_plots.append(surface_line)
      legend_labels.append(surface['name'])

      # Plot symmetric surfaces.
      if self.avl['iYsym']:
        axes.plot(xs + [xs[0]], -numpy.array(ys + [ys[0]]), zs + [zs[0]], '--',
                  color=color_order[i])
      elif 'Ydupl' in surface:
        y_scale = surface['Yscale'] if 'Yscale' in surface else 1.0
        axes.plot(xs + [xs[0]],
                  -numpy.array(ys + [ys[0]]) + 2.0 * surface['Ydupl'] * y_scale,
                  zs + [zs[0]], '--',
                  color=color_order[i])

    axes.legend(legend_plots, legend_labels, loc='lower left',
                prop={'size': 10})
    pyplot.show()

  def Parse(self, avl_file):
    """Parses AVL file.

    Args:
      avl_file: String of the read AVL file.

    Returns:
      Dictionary representing the information stored in the AVL file.
    """

    # Make iterator over lines in file.  Automatically, remove comments
    # and blank lines.  Terminate the file with an END keyword (this
    # isn't mentioned in the AVL documentation, but at least one of the
    # example files uses this convention and it makes the parsing more
    # natural.
    lines = iter([l.split('!', 1)[0].strip()
                  for l in avl_file.splitlines()
                  if l.strip() and l[0] not in '#!'] + ['END'])

    # Parse the AVL header for information on the case name, reference
    # areas, etc.
    avl, line = self._ParseHeader(lines)

    # Loop through the rest of the file, which should only be composed
    # of surfaces and bodies.
    while True:
      tokens = line.split()
      keyword = tokens[0][0:4]
      if keyword == 'SURFACE'[0:4]:
        surface, line = self._ParseSurface(lines)
        avl.setdefault('surfaces', []).append(surface)

      elif keyword == 'BODY':
        body, line = self._ParseBody(lines)
        avl.setdefault('body', []).append(body)

      else:
        if keyword != 'END':
          logging.error('Encountered unexpected keyword: %s', tokens[0])
        break

    return avl

  def _ParseHeader(self, lines):
    """Parses header information."""
    header = collections.OrderedDict()

    header['case'] = lines.next()

    tokens = lines.next().split()
    header['Mach'] = float(tokens[0])

    tokens = lines.next().split()
    header['iYsym'] = int(tokens[0])
    header['iZsym'] = int(tokens[1])
    header['Zsym'] = float(tokens[2])

    tokens = lines.next().split()
    header['Sref'] = float(tokens[0])
    header['Cref'] = float(tokens[1])
    header['Bref'] = float(tokens[2])

    tokens = lines.next().split()
    header['Xref'] = float(tokens[0])
    header['Yref'] = float(tokens[1])
    header['Zref'] = float(tokens[2])

    line = lines.next()
    try:
      # CDp is optional.
      header['CDp'] = float(line.split()[0])
      line = lines.next()
    except (IndexError, ValueError):
      pass

    return header, line

  def _ParseAirfoil(self, lines):
    """Parses airfoil camber line definition."""
    airfoil = [[]]
    while True:
      line = lines.next()
      tokens = line.split()
      try:
        airfoil.append([float(tokens[0]), float(tokens[1])])
      except (IndexError, ValueError):
        break
    return airfoil, line

  def _ParseFilename(self, lines):
    """Parses filename of airfoil definition."""
    line = lines.next()
    # The file name may either be quoted or not.
    if line[0] == '"':
      filename = line.split()[0][1:-1]
    else:
      filename = line
    return filename

  def _ParseSection(self, lines):
    """Parses information describing cross-section of surface along span."""
    section = collections.OrderedDict()

    tokens = lines.next().split()
    section['Xle'] = float(tokens[0])
    section['Yle'] = float(tokens[1])
    section['Zle'] = float(tokens[2])
    section['Chord'] = float(tokens[3])
    section['Ainc'] = float(tokens[4])
    try:
      # Nspan and Sspace are optional.
      section['Nspan'] = int(tokens[5])
      section['Sspace'] = float(tokens[6])
    except (IndexError, ValueError):
      pass

    next_line = None
    first_keyword = True
    while True:
      line = next_line if next_line else lines.next()
      next_line = None
      tokens = line.split()
      keyword = tokens[0][0:4]

      # Issue warnings if there is a suspicious ordering of the camber
      # line keywords.  According to the AVL documentation, the camber
      # line keywords must immediately follow the data line of the
      # SECTION keyword, and also later camber line keywords overwrite
      # earlier ones.
      if keyword in ['NACA', 'AIRFOIL'[0:4], 'AFILE'[0:4]]:
        if not first_keyword:
          logging.warning('%s did not immediately follow the data line of the '
                          'SECTION keyword.', tokens[0])
        if any([k in section for k in ['naca', 'airfoil', 'afile']]):
          logging.warning('Another camber line definition exists.  This will '
                          'overwrite it.')

      if keyword == 'NACA':
        # Parse NACA camber line.
        section['naca'] = int(lines.next().split()[0])
        assert 0 <= section['naca'] and section['naca'] <= 9999

      elif keyword == 'AIRFOIL'[0:4]:
        # Parse airfoil coordinates.
        try:
          # x/c range is optional.
          section['x1'] = float(tokens[1])
          section['x2'] = float(tokens[2])
        except (IndexError, ValueError):
          pass
        section['airfoil'], next_line = self._ParseAirfoil(lines)

      elif keyword == 'AFILE'[0:4]:
        # Parse airfoil filename.
        try:
          # x/c range is optional.
          section['x1'] = float(tokens[1])
          section['x2'] = float(tokens[2])
        except (IndexError, ValueError):
          pass
        section['afile'] = self._ParseFilename(lines)

      elif keyword == 'DESIGN'[0:4]:
        # Parse design variable.
        tokens = lines.next().split()
        design = collections.OrderedDict()
        design['DName'] = tokens[0]
        try:
          design['Wdes'] = float(tokens[1])
        except (IndexError, ValueError):
          # Although it is not listed as an optional value in the AVL
          # documentation, some of the example AVL files do not have a
          # value for Wdes.
          logging.warning('Wdes value is missing for %s.', design['DName'])
        section.setdefault('designs', []).append(design)

      elif keyword == 'CONTROL'[0:4]:
        # Parse control variable.
        tokens = lines.next().split()
        control = collections.OrderedDict()
        control['name'] = tokens[0]
        control['gain'] = float(tokens[1])
        control['Xhinge'] = float(tokens[2])
        control['XYZhvec'] = [float(tokens[3]),
                              float(tokens[4]),
                              float(tokens[5])]
        try:
          control['SgnDup'] = float(tokens[6])
        except (IndexError, ValueError):
          # Although it is not listed as an optional value in the AVL
          # documentation, some of the example AVL files do not have a
          # value for SgnDup.
          logging.warning('SgnDup value is missing for %s.', control['name'])
        section.setdefault('controls', []).append(control)

      elif keyword == 'CLAF':
        # Parse dCL/da scaling factor.
        section['CLaf'] = float(lines.next().split()[0])

      elif keyword == 'CDCL':
        # Parse CD(CL) function parameters.
        tokens = lines.next().split()
        section['CL1'] = float(tokens[0])
        section['CD1'] = float(tokens[1])
        section['CL2'] = float(tokens[2])
        section['CD2'] = float(tokens[3])
        section['CL3'] = float(tokens[4])
        section['CD3'] = float(tokens[5])

      else:
        break

      first_keyword = False

    return section, line

  def _ParseSurface(self, lines):
    """Parses definition of a lifting surface."""
    surface = collections.OrderedDict()

    surface['name'] = lines.next()

    tokens = lines.next().split()
    surface['Nchord'] = int(tokens[0])
    surface['Cspace'] = float(tokens[1])
    try:
      # Nspan and Sspace are optional.
      surface['Nspan'] = int(tokens[2])
      surface['Sspace'] = float(tokens[3])
    except (IndexError, ValueError):
      pass

    next_line = None
    while True:
      line = next_line if next_line else lines.next()
      next_line = None
      keyword = line.split()[0][0:4]

      if keyword in ['COMPONENT'[0:4], 'INDEX'[0:4]]:
        # Parse component grouping.
        surface['Lcomp'] = int(lines.next().split()[0])

      elif keyword == 'YDUPLICATE'[0:4]:
        # Parse duplicated surface y-plane.
        surface['Ydupl'] = float(lines.next().split()[0])

      elif keyword == 'SCALE'[0:4]:
        # Parse surface scaling.
        tokens = lines.next().split()
        surface['Xscale'] = float(tokens[0])
        surface['Yscale'] = float(tokens[1])
        surface['Zscale'] = float(tokens[2])

      elif keyword == 'TRANSLATE'[0:4]:
        # Parse surface translation.
        tokens = lines.next().split()
        surface['dX'] = float(tokens[0])
        surface['dY'] = float(tokens[1])
        surface['dZ'] = float(tokens[2])

      elif keyword == 'ANGLE'[0:4]:
        # Parse surface incidence angle.
        surface['dAinc'] = float(lines.next().split()[0])

      elif keyword == 'NOWAKE'[0:4]:
        surface['nowake'] = True

      elif keyword == 'NOALBE'[0:4]:
        surface['noalbe'] = True

      elif keyword == 'NOLOAD'[0:4]:
        surface['noload'] = True

      elif keyword == 'SECTION'[0:4]:
        # Parse airfoil section camber line along span.
        section, next_line = self._ParseSection(lines)
        surface.setdefault('sections', []).append(section)
      else:
        break

    return surface, line

  def _ParseBody(self, lines):
    """Parses description of non-lifting bodies shape."""
    body = collections.OrderedDict()

    body['name'] = lines.next()
    tokens = lines.next().split()
    body['Nbody'] = int(tokens[0])
    body['Bspace'] = float(tokens[1])

    while True:
      line = lines.next()
      keyword = line.split()[0][0:4]
      if keyword == 'YDUPLICATE'[0:4]:
        body['Ydupl'] = float(lines.next().split()[0])

      elif keyword == 'SCALE'[0:4]:
        # Parse body scaling.
        tokens = lines.next().split()
        body['Xscale'] = float(tokens[0])
        body['Yscale'] = float(tokens[1])
        body['Zscale'] = float(tokens[2])

      elif keyword == 'TRANSLATE'[0:4]:
        # Parse body translation.
        tokens = lines.next().split()
        body['dX'] = float(tokens[0])
        body['dY'] = float(tokens[1])
        body['dZ'] = float(tokens[2])

      elif keyword == 'BFILE'[0:4]:
        # Parse body shape filename.
        body['bfile'] = self._ParseFilename(lines)

      else:
        break

    return body, line


def main(argv):
  # Internal matplotlib functions currently trigger the following
  # warnings.
  warnings.filterwarnings('ignore', 'elementwise comparison failed; returning '
                          'scalar instead, but in the future will perform '
                          'elementwise comparison')
  warnings.filterwarnings('ignore', 'comparison to `None` will result in an '
                          'elementwise object comparison in the future.')
  logging.basicConfig(stream=sys.stdout,
                      format='%(asctime)s %(levelname)-8s %(message)s',
                      level=logging.INFO)
  avl = AvlReader(argv[1])
  avl.PlotGeometry()
  logging.shutdown()


if __name__ == '__main__':
  main(sys.argv)
