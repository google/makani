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

"""Utility functions for batch simulations."""

import collections
import logging
import math
import textwrap

import numpy


def CalcTotalVariation(time_series):
  return numpy.sum(numpy.abs(numpy.diff(time_series)))


def GetTelemetryIndices(telem, t0, t1=float('inf')):
  """Get the indices of a given time interval in a telemetry dataset.

  In one use case, this function is used to extract the telemetry data
  corresponding to the timestamps during a particular flight mode. If a flight
  mode contains only one sample, then this function returns the telemetry data
  for the timestamp the closest to this sample.

  Args:
    telem: A simulator or controller telemetry dictionary.
    t0: A float defining the lower boundary of a time interval.
    t1: A float defining the upper boundary of a time interval.

  Returns:
    A slice describing the index set corresponding to the requested time
    interval, or None if the index set is empty.
  """
  if t0 != t1:
    indices = numpy.argwhere(numpy.logical_and(telem['time'] >= t0,
                                               telem['time'] <= t1))
  else:
    indices = (numpy.abs(telem['time'] - t0)).argmin()
  indices = numpy.reshape(indices, (indices.size,))

  if indices.size == 0:
    return None
  else:
    assert (numpy.diff(indices) == 1).all()
    return slice(indices[0], indices[-1] + 1)


def CollateOutputs(outputs, ind_func=lambda x: x):
  """Converts a list structures into a structure of NumPy arrays.

  For example, the output of the function call

    CollateOutputs([{'foo': 0.0, 'bar': 22.0},
                    {'foo': 1.0, 'bar': 22.0}
                    {'foo': 2.0, 'bar': 22.0}])

  would be equivalent to

    {'foo': numpy.array([0.0, 1.0, 2.0]),
     'bar': numpy.array([22.0, 22.0, 22.0])}.

  Args:
    outputs: Iterable of outputs from the batch sim workers.
    ind_func: Index function used to access (sub-)elements of `outputs`.

  Returns:
    See above.
  """
  element = ind_func(outputs[0])
  # pylint: disable=cell-var-from-loop
  if isinstance(element, dict):
    return {k: CollateOutputs(outputs, lambda x: ind_func(x)[k])
            for k in element}
  elif isinstance(element, list):
    return [CollateOutputs(outputs, lambda x: ind_func(x)[i])
            for i in range(len(element))]
  else:
    return numpy.array([ind_func(o) for o in outputs])


def Tail(filename, n_lines):
  """Return the last several lines of a file.

  If the file does not exist, an empty string is returned.

  Args:
    filename: Name of the file to read.
    n_lines: Number of lines to return.

  Returns:
    String containing the file data.
  """
  lines = []
  try:
    with open(filename, 'r') as f:
      lines = f.readlines()
  except IOError:
    pass
  return ''.join(lines[-n_lines:])


def EscapeHtml(text):
  """Escape symbols so that text may be safely embedded in HTML."""
  return text.replace('&', '&amp;').replace('>', '&gt;').replace('<', '&lt;')


def SetUpLogging(quiet=False):
  """Configure logging for batch simulations."""

  # Show INFO severity or higher, or WARNING if quiet is enabled, and add a
  # timestamp to the log messages.
  if not quiet:
    logging.root.setLevel(logging.INFO)
  else:
    logging.root.setLevel(logging.WARNING)
  handler = logging.StreamHandler()
  handler.setFormatter(logging.Formatter(
      fmt='%(asctime)s.%(msecs).03d %(levelname)s: %(message)s',
      datefmt='%Y-%m-%d %H:%M:%S'))
  logging.root.addHandler(handler)

  # These loggers are a little too chatty at INFO level.
  logging.getLogger('googleapiclient.discovery').setLevel(logging.WARNING)
  logging.getLogger('oauth2client.discovery').setLevel(logging.WARNING)


HtmlScoreTableValue = collections.namedtuple('HtmlScoreTableValue',
                                             ['name', 'severity', 'limits',
                                              'value', 'score'])


# TODO: Move all coloring logic into Javascript. Currently, each cell
# is assigned a default color here, and then cells with the "data-scores" field
# may be dynamically updated via Javascript.
def _GetHtmlScoreTableCell(score, all_scores=None, link=None):
  """Generate HTML table cell from a score.

  Args:
    score: Numerical value to display and color code the cell with.
    all_scores: List of all scores corresponding to this cell. Used for dynamic
        updates of cells based on score selection.
    link: Optional URL to link numerical value to.

  Returns:
    String containing a table cell HTML element.
  """
  lower_bound = 0.0
  upper_bound = 1.0
  colors = ['#74add1', '#abd9e9', '#e0f3f8', '#ffffbf', '#fee090',
            '#fdae61', '#f46d43', '#d73027', '#a50026']

  if math.isnan(score):
    color = '#888888'
  elif math.isinf(score):
    color = '#aaaaaa'
  else:
    coeff = (score - lower_bound) / (upper_bound - lower_bound)
    idx = int(round(coeff * len(colors)))
    color = colors[min(max(0, idx), len(colors) - 1)]

  all_scores_str = ''
  if all_scores is not None:
    # The data-scores field is interpreted as JSON, which regrettably does not
    # support NaN. Instead, we pass NaN through a string with suitable escaping
    # of quotes.
    parts = ['&quot;NaN&quot;' if numpy.isnan(s) else str(s)
             for s in all_scores]
    all_scores_str = (' data-scores="[%s]"' % ', '.join(parts))

  text = ['<td class="value" style="background-color: %s"%s>'
          % (color, all_scores_str)]
  if link is not None:
    text += ['<a href="%s">' % link]
  text += ['%.0f' % numpy.min([999.0, 100.0 * numpy.max([0.0, score])])]
  if link is not None:
    text += ['</a>']
  text += ['</td>']
  return '\n'.join(text)


def GetHtmlScoreTableLegend():
  """Get a legend for the color code used by GetHtmlScoreTable.

  Returns:
    An HTML string containing the color-code description.
  """
  scores = numpy.linspace(0, 1, 10.0)
  text = ['<table>',
          '<tr>',
          '<td><center>Good</center></td>',
          '<td colspan="%d"></td>' % (scores.shape[0] - 2),
          '<td><center>Bad</center></td>',
          '</tr>',
          '<tr>']
  text += [_GetHtmlScoreTableCell(score) for score in scores]
  text += ['</tr>']
  return '\n'.join(text)


# Generate an HTML table with color-coded cells based on numerical values.
def GetHtmlScoreTable(scores_and_links, title, x_ticks, x_label,
                      y_ticks, y_label):
  """Construct an HTML table from data.

  Args:
    scores_and_links: List of lists of
            (default_score, all_scores, link)
        triples. Each default_score determines a cell's default text and
        color-coding, while all_scores is used for dynamic updating in the
        report. Each link is None or a URL for that text.
    title: Title of the table.
    x_ticks: Values to label the x-axis.
    x_label: Label for the x-axis.
    y_ticks: Values to label the y-axis.
    y_label: Label for the y-axis.

  Returns:
    HTML string with formatted data.
  """
  num_rows = len(y_ticks)
  num_cols = len(x_ticks)
  # Start table and write title.
  text = ['<table class="value_table">',
          '<tr><td></td><td></td>',
          '<td style="text-align: center" colspan="%d">%s</td>' % (num_cols,
                                                                   title),
          '</tr>']

  # Print all rows.
  for i in range(num_rows - 1, -1, -1):
    text += ['<tr>']
    if i == num_rows - 1:
      text += ['<td rowspan="%d"><div class="y_label">%s</div></td>\n'
               % (num_rows + 1, y_label)]
    try:
      text += ['<td class="label">%.2f</td>' % y_ticks[i]]
    except TypeError:
      text += ['<td class="label">' + str(y_ticks[i]) + '</td>']
    text += [
        _GetHtmlScoreTableCell(scores_and_links[i][j][0],
                               all_scores=scores_and_links[i][j][1],
                               link=scores_and_links[i][j][2])
        for j in range(num_cols)
    ]
    text += ['</tr>']

  # Add x_ticks.
  text += ['<tr>',
           '<td></td>']
  text += ['<td>%.2f</td>' % t for t in x_ticks]
  text += ['</tr>']

  # Add x_label and close table.
  text += ['<tr>',
           '<td></td>',
           '<td></td>',
           '<td colspan="%d">' % num_cols,
           '<div class="x_label">%s</div>' % x_label,
           '</td>',
           '</tr>',
           '</table>']

  return '\n'.join(text)


def GetHtmlScoreTableResultPage(
    results, header_info, override_info, commands, event_html=None):
  """Return an HTML string describing various scoring information.

  Args:
    results: A list of HtmlScoreTableValues to be displayed.
    header_info: A list of tuples containing simulation summary data.
    override_info: A list of tuples of override names, value, and value limits.
    commands: A list of tuples of commands to run this sim.
    event_html: HTML to show the event sequence.

  Returns:
    A string containing a complete HTML document.
  """
  output_str = [textwrap.dedent("""
      <html>
      <head>
      <link rel="stylesheet" type="text/css" href="style.css">""")]
  events_body = []
  heading = """
    <link rel="stylesheet" href="bokeh-0.13.0.min.css" type="text/css" />
    <script type="text/javascript" src="bokeh-0.13.0.min.js"></script>
    <script type="text/javascript">
        Bokeh.set_log_level("info");
    </script>
  """
  output_str += [textwrap.dedent(heading)]
  if event_html:
    event_lines = event_html.split('\n')
    in_body = False
    for ln in event_lines:
      if ln.strip().startswith('<body>'):
        in_body = True
      elif ln.strip().startswith('</body>'):
        in_body = False
      elif in_body:
        events_body.append(ln)
  output_str += [textwrap.dedent("""
      </head>
      <body>""")]
  output_str += events_body
  output_str += [textwrap.dedent("""
      <table><tr><td valign="top" width="30%">
        <h2>Simulation Summary</h2>
        <table border="0" class="simheader">""")]

  # Summary
  for header_label, header_data in header_info:
    output_str += [textwrap.dedent("""
        <tr>
        <th class="header_label">%s</th>
          <td class="col_space"></td>
          <td>%s</td>""" % (header_label, header_data))]
  # Write no overrides to html page for baseline sim, override_info is empty
  # for simulation with no overrides.
  if override_info:
    output_str += [textwrap.dedent("""</table>
          <h2>Configuration</h2>
          <table border="0" class="perturbation_info">
            <tr class="perturbation_row_info">
              <td><h3>Parameters</h3></td>
              <td><h3 style="text-align:center">Range Minimum</h3></td>
              <td><h3 style="text-align:center">Sim Value</h3></td>
              <td><h3 style="text-align:center">Range Maximum</h3></td></tr>
              """)]

    for override_name, sim_value, override_min, override_max in override_info:
      if all([isinstance(o, float) for o in [override_min,
                                             sim_value,
                                             override_max]]):
        output_str += [textwrap.dedent("""
            <tr>
              <th class="pertub_label">%s</th>
              <td style="text-align:center">%10.4f</td>
              <td style="text-align:center">%10.4f</td>
              <td style="text-align:center">%10.4f</td>""" % (override_name,
                                                              override_min,
                                                              sim_value,
                                                              override_max))]
      else:
        output_str += [textwrap.dedent("""
              <tr>
                <th class="pertub_label">%s</th>
                <td style="text-align:center">%s</td>
                <td style="text-align:center">%s</td>
                <td style="text-align:center">%s</td>""" % (override_name,
                                                            override_min,
                                                            sim_value,
                                                            override_max))]

  for command in commands:
    output_str += [textwrap.dedent("""
        </table>
        <table class="simheader">
          <h2>Command Line</h2h
          <tr>
            <td class="col_space"></td><td>%s</td>""" % (command[1]))]

  output_str += [textwrap.dedent("""
        </table>
        </td><td valign="top">
      <table border="0" style="text-align: left">""")]

  for result in results:
    sub_text = """<p style=font-weight:lighter;margin:0;">
        Limits: %s  |  Severity: %s</p>""" %(result.limits, result.severity)
    output_str += [textwrap.dedent("""
        <tr>
        <th>%s %s </th>""" % (result.name, sub_text))]
    if result.score is not None:
      if isinstance(result.value, float):
        output_str += [textwrap.dedent("""
          %s
          <td>%10.4f</td>""" % (_GetHtmlScoreTableCell(result.score),
                                result.value))]
      elif isinstance(result.value, numpy.ndarray):
        output_str += [textwrap.dedent("""
          %s
          <td>%s</td>""" % (_GetHtmlScoreTableCell(result.score),
                            (numpy.array2string(result.value, precision=4,
                                                separator=',  '))))]
      elif isinstance(result.value, list):
        output_str += [textwrap.dedent("""
          %s
          <td>%s</td>""" % (_GetHtmlScoreTableCell(result.score),
                            (numpy.array2string(numpy.array(result.value),
                                                precision=4, separator=', '))))]
      else:  # generic handling
        if isinstance(result.value, float):
          output_str += [textwrap.dedent("""
            %s
            <td>%s</td>""" % (_GetHtmlScoreTableCell(result.score),
                              result.value))]
    else:
      output_str += [textwrap.dedent("""
        <td width="30"></td>
        <td>%s</td>""" % result.value)]

  output_str += [textwrap.dedent("""
      </table>
      </td></tr></table>""")]

  output_str += [textwrap.dedent("""
      </body>
      </html>""")]

  return '\n'.join(output_str)
