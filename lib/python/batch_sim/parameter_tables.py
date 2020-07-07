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

"""Utilities for writing batch simulations like hover disturbances."""

import copy
import csv
import datetime
import json
import os
import re
import shutil
import tempfile
import textwrap

import gflags
import makani
from makani.config import mconfig
from makani.config import overrides_util
from makani.control import system_types
from makani.lib.python import build_info
from makani.lib.python import dict_util
from makani.lib.python import gsutil
from makani.lib.python.batch_sim import client as client_base
import makani.lib.python.batch_sim.batch_sim_util as batch_sim_util
from makani.lib.python.batch_sim.scoring_functions import plot
import matplotlib
import numpy

gflags.DEFINE_integer('parameter_seed', 12345,
                      'Seed for use in parameter table generation.')

gflags.DEFINE_boolean('randomize_parameter_seed', False,
                      'Select a random value for parameter table seed.')

gflags.DEFINE_boolean('upload_overview_data', False,
                      'Upload overview_data.json to batch sim folder on GCS.')

FLAGS = gflags.FLAGS


class _Random(object):
  """Container for a PRNG that can be seeded using --parameter_seed."""

  def __init__(self):
    self.seed = None
    self._prng = None

  def Generator(self):
    if self.seed is None:
      if FLAGS.randomize_parameter_seed:
        self.seed = numpy.random.randint(numpy.iinfo(numpy.uint32).max)
      else:
        self.seed = FLAGS.parameter_seed
      self._prng = numpy.random.RandomState(self.seed)

    return self._prng


_random = _Random()


class OverridesTable(object):
  """Abstract class for a table of simulations with parameter overrides.

  Attributes:
    name: The title of this table.
    x_label: The x-axis label.
    x_values: Numerical labels for the columns of this table.
    y_label: The y-axis label.
    y_values: Numerical labels for the rows of this table.
    base_overrides: Overrides to apply to all table entries.
    base_params: Parameter structure with base overrides.
  """

  def __init__(self, name, x_label, x_values, y_label, y_values,
               base_overrides=None):
    if base_overrides is None:
      base_overrides = {}
    self.name = name
    self.base_overrides = copy.deepcopy(base_overrides)
    self.base_params = mconfig.MakeParams(
        'common.all_params', overrides=copy.deepcopy(base_overrides),
        override_method='derived')
    self.x_label = x_label
    self.x_values = tuple(x_values)
    self.y_label = y_label
    self.y_values = tuple(y_values)

  def GetDimensions(self):
    """Return the dimensions of this table."""
    return (len(self.x_values), len(self.y_values))

  def GetOverrides(self, x_idx, y_idx):
    """Abstract method for getting the overrides for this simulation."""
    raise NotImplementedError

  def GetRangeValues(self, x_idx, y_idx):
    """Abstract method for getting the parameter ranges for this simulation."""
    raise NotImplementedError


class OverridesTableSimClient(client_base.BatchSimClient):
  """Abstract client for generating a table of tables."""

  def __init__(self, output_dir, tables, scoring_functions,
               columns=3, title=None, **kwargs):
    """Constructor for a disturbance sim.

    Args:
      output_dir: Directory in which outputs are to be written.
      tables: A list of OverridesTables.
      scoring_functions: A list of ScoringFunctions.
      columns: Number of columns to use in displaying tables.
      title: Title for the generated HTML.
      **kwargs: See client_base.BatchSimClient.
    """
    super(OverridesTableSimClient, self).__init__(**kwargs)
    self._output_dir = output_dir
    self._tables = tables
    self._scoring_functions = scoring_functions
    self._title = title
    self._columns = columns

    # This array should be populated by _GetConfigs with each
    # configuration corresponding to the return value of
    # _GetSimParameters.
    self._linear_indices = [None for _ in range(self._GetNumTables())]
    self._linear_index_to_table_index = []
    self._overrides = []
    for table_idx in range(self._GetNumTables()):
      table_dim = self._tables[table_idx].GetDimensions()
      self._linear_indices[table_idx] = [[None for _ in range(table_dim[1])]
                                         for _ in range(table_dim[0])]
      for x_idx, y_idx in numpy.ndindex(table_dim):
        self._linear_indices[table_idx][x_idx][y_idx] = len(self._overrides)
        self._linear_index_to_table_index.append(table_idx)
        self._overrides += [
            self._tables[table_idx].GetOverrides(x_idx, y_idx)
        ]

  def _GetNumTables(self):
    """Method returning the number of tables.

    Returns:
      The number of parameter tables to be swept.
    """
    return len(self._tables)

  def _GetLinearIndex(self, idx):
    """Convert a 3-D index into a linear index."""
    return self._linear_indices[idx[0]][idx[1]][idx[2]]

  def _GetConfig(self, idx):
    table_idx = self._linear_index_to_table_index[idx]
    return mconfig.SimpleOverride(
        overrides_util.PreprocessOverrides(self._overrides[idx]),
        copy.deepcopy(self._tables[table_idx].base_params))

  def _GenerateConfigs(self):
    for idx in range(len(self._overrides)):
      yield self._GetConfig(idx)

  @client_base.JsonReducer
  def _ReduceWorkerOutput(self, outputs):
    matplotlib.use('Agg')
    self._GenerateHtml(self._output_dir, outputs)
    self._GenerateJson(self._output_dir, outputs)

  def _GenerateHtml(self, output_dir, outputs):
    # Create directory to place output files.
    if not os.path.exists(output_dir):
      os.makedirs(output_dir)

    # File to write each results page.
    def _GetOverridesPageFilename(i):
      return 'overrides_%d.html' % i

    def _GetOverridesFilename(i):
      return 'overrides_%d.json' % i

    def _GetStatsFilename(table_idx):
      return 'table_%d.csv' % table_idx

    def _GetScoreCheckboxes():
      """Returns an HTML title bar for selecting which scores to show."""
      score_fn_names = [score_fn.GetName()
                        for score_fn in self._scoring_functions]
      lines = ['<div id="score_container" style="text-align: center">'
               '<div style="display: inline-block; text-align: left">'
               '<h3>Scores</h3>',
               '<button onclick="$(\'#score_display\').toggle();"'
               '>Hide/show</button>',
               '<div id="score_display">',
               '<form id="active_scores" style="margin-top: 1rem;">']
      for i, name in enumerate(score_fn_names):
        elt_name = 'score' + str(i)
        line = ''.join([
            '<input type="checkbox" name="{0}" value="{0}" checked>'.format(
                elt_name),
            name,
            '</input><br>'])
        lines.append(line)
      lines += ['</form>',
                '<button id="select_all_scores">Select all</button>',
                '<button id="clear_all_scores">Clear all</button>',
                '</div>',  # score_display
                '</div>',
                '</div>']

      return '\n'.join(lines)

    # Convert a worker's output into an HTML table cell.
    def _GetScoreData(idx, outputs):
      """Get score data and link for a given index.

      Args:
        idx: Multi-index of this sim (see _GetLinearIndex).
        outputs: Array of outputs from the workers.

      Returns:
        (default_score, all_scores, link) tuple.
      """
      i = self._GetLinearIndex(idx)
      output = outputs[i]

      all_scores = []
      default_score = float('nan')
      if output['sim_success']:
        all_scores = [score_fn.GetScore(output[score_fn.GetName()])
                      for score_fn in self._scoring_functions]
        default_score = numpy.max(all_scores)

      return (default_score, all_scores, _GetOverridesPageFilename(i))

    def _WriteScoreTables(filename):
      """Write score data tables into HTML string.

      Args:
        filename: name of file where HTML string will be written.
      """
      with open(filename, 'w') as f:
        f.write(textwrap.dedent("""
            <html>
            <head>
            <link rel="stylesheet" type="text/css" href="style.css">
            <style>
              #score_container {
                position: fixed;
                top: 1rem;
                left: 2rem;
                background-color: rgba(255, 255, 255, 1);
                border-style: solid;
                z-index: 10;
                padding: 0rem 1rem 1rem 1rem;
                overflow: auto;
                max-height: 90%;
                box-shadow: 4px 4px 3px rgba(120, 120, 120, 1);
              }
            </style>
            </head>
            <script src="jquery.js"></script>
            <script src="scoring_function_util.js"></script>
            <script>
              $(document).ready(function() {
                var scoreCheckboxes = $("#active_scores").find(":checkbox");

                UpdateTableCellsFromCheckboxes(scoreCheckboxes);

                scoreCheckboxes.change(function() {
                  UpdateTableCellsFromCheckboxes(scoreCheckboxes);
                });

                $("#select_all_scores").click(function() {
                  scoreCheckboxes.prop("checked", true);
                  UpdateTableCellsFromCheckboxes(scoreCheckboxes);
                });

                $("#clear_all_scores").click(function() {
                  scoreCheckboxes.removeAttr("checked");
                  UpdateTableCellsFromCheckboxes(scoreCheckboxes);
                });
              });
            </script>

            <body>
            <center>
            <h1>"""))
        if self._title is not None:
          f.write(self._title + ' ')

        f.write(datetime.datetime.now().strftime('%Y-%m-%d %H:%M %Z'))

        f.write(textwrap.dedent("""\
            </h1>
            %s
            %s
            <table>""" % (_GetScoreCheckboxes(),
                          batch_sim_util.GetHtmlScoreTableLegend())))
        for table_idx in range(self._GetNumTables()):
          table_dim = self._tables[table_idx].GetDimensions()
          if (table_idx % self._columns) == 0:
            f.write('<tr>\n')
          f.write('<td align="right">\n')

          # Note that we transpose the array here.
          table_values = [
              [_GetScoreData([table_idx, x_idx, y_idx], outputs)
               for x_idx in range(table_dim[0])]
              for y_idx in range(table_dim[1])
          ]

          f.write(batch_sim_util.GetHtmlScoreTable(
              table_values,
              '<a href="%s">%s</a>' % (_GetStatsFilename(table_idx),
                                       self._tables[table_idx].name),
              self._tables[table_idx].x_values,
              self._tables[table_idx].x_label,
              self._tables[table_idx].y_values,
              self._tables[table_idx].y_label))

          f.write('</td>\n')
          if (table_idx % self._columns) == self._columns - 1:
            f.write('</tr>\n')
          f.write(textwrap.dedent("""
              </center></body>
              </html>"""))

    def _GcsLink(filename):
      url = ('https://storage.cloud.google.com/makani/batch_sim/'
             + self._sim_name + '/h5_logs/' + filename)
      return '<a href="%s">%s</a>' % (url, filename)

    def _WriteStats(table_idx, outputs):
      """Writes out batch sim output values into a unique csv file.

      Args:
        table_idx: batch sim number of a given Override table.
        outputs: contains all HTML outputs for the batch sim at table_idx.
      """
      filename = os.path.join(output_dir, _GetStatsFilename(table_idx))
      table = self._tables[table_idx]
      with open(filename, 'w') as f:
        writer = csv.writer(f, delimiter=' ', quotechar='"',
                            quoting=csv.QUOTE_MINIMAL)
        writer.writerow([table.name])
        labels = [
            score_fn.GetName() for score_fn in self._scoring_functions
        ] + [
            override_name
            for override_name, _ in table.GetOverridesDesc(0, 0)
        ]
        writer.writerow(labels)
        for x_idx, y_idx in numpy.ndindex(table.GetDimensions()):
          idx = self._GetLinearIndex((table_idx, x_idx, y_idx))
          if not outputs[idx]['sim_success']:
            continue
          row = []
          for score_fn in self._scoring_functions:
            score_fn_output = outputs[idx][score_fn.GetName()]
            row.append(score_fn.GetValue(score_fn_output))
          for _, value in table.GetOverridesDesc(x_idx, y_idx):
            row.append(value)
          writer.writerow(row)

    # For each output, write an overrides page.
    assert len(outputs) == numpy.sum([
        numpy.prod(self._tables[i].GetDimensions())
        for i in range(self._GetNumTables())
    ])

    for table_idx in range(self._GetNumTables()):
      _WriteStats(table_idx, outputs)
      table_dims = self._tables[table_idx].GetDimensions()
      for x_idx, y_idx in numpy.ndindex(table_dims):
        idx = self._GetLinearIndex((table_idx, x_idx, y_idx))
        with open(os.path.join(output_dir,
                               _GetOverridesFilename(idx)), 'w') as f:
          f.write(json.dumps(self._GetConfig(idx)))

        with open(os.path.join(output_dir,
                               _GetOverridesPageFilename(idx)), 'w') as f:
          header_info = [('Table', self._tables[table_idx].name),
                         ('Overrides',
                          '<a href="%s">%s</a>' % (_GetOverridesFilename(idx),
                                                   _GetOverridesFilename(idx))),
                         ('Full log', _GcsLink('%d.h5' % idx)),
                         ('Sparse log (10 Hz)', _GcsLink('%d_sparse.h5' % idx)),
                         ('Note', 'Log files are overwritten whenever sim name '
                          '%s is reused.' % self._sim_name)]

          overrides = self._tables[table_idx].GetOverridesDesc(x_idx, y_idx)
          overrides_lim = (self._tables[table_idx]
                           .GetOverridesLimits(x_idx, y_idx))

          results = []
          if outputs[idx]['sim_success']:
            for score_fn in self._scoring_functions:
              score_fn_output = outputs[idx][score_fn.GetName()]
              results += [batch_sim_util.HtmlScoreTableValue(
                  name=score_fn.GetName(),
                  severity=score_fn.GetSeverity(),
                  limits=score_fn.Limits(),
                  value=score_fn.GetValue(score_fn_output),
                  score=score_fn.GetScore(score_fn_output)
              )]
          else:
            err = batch_sim_util.EscapeHtml(
                outputs[idx]['sim_error_message'])
            results += [batch_sim_util.HtmlScoreTableValue(
                name='Error Message',
                severity=None,
                limits=None,
                value='<pre>%s</pre>' % err,
                score=None
            )]

          # Need to zip the two overrides tuples together
          overrides_info = []
          for (description, limits) in zip(overrides, overrides_lim):
            overrides_info += [(description + limits)]

          command_info = [
              ('Command line',
               'run_sim -o \'%s\'' % json.dumps(dict_util.UpdateNestedDict(
                   self._tables[table_idx].base_overrides,
                   self._overrides[idx])))
          ]

          if 'events' in outputs[idx]:
            with tempfile.NamedTemporaryFile(
                suffix='.html', delete=False) as temp_fp:
              plot.PlotEvents(outputs[idx]['events'], temp_fp.name)
            with open(temp_fp.name) as fp:
              event_html = fp.read()
            os.remove(temp_fp.name)
          else:
            event_html = ''

          f.write(batch_sim_util.GetHtmlScoreTableResultPage(
              results, header_info, overrides_info, command_info, event_html))

    _WriteScoreTables(output_dir + '/old_report.html')

    # Copy CSS and JS files to the output directory.
    shutil.copyfile(os.path.join(
        makani.HOME, 'lib/python/batch_sim/overrides_table_style.css'),
                    os.path.join(output_dir, 'style.css'))
    shutil.copy(os.path.join(
        makani.HOME, 'lib/python/batch_sim/scoring_function_util.js'),
                output_dir)
    shutil.copy(os.path.join(makani.HOME, 'lib/python/batch_sim/jquery.js'),
                output_dir)
    os.chmod(os.path.join(output_dir, 'jquery.js'), 0770)

  def _GenerateJson(self, output_dir, outputs):
    """Writes JSON data for new-style reports."""

    table_data = []
    for table_idx in range(self._GetNumTables()):
      table_dims = self._tables[table_idx].GetDimensions()

      if isinstance(self._tables[table_idx].y_values[0], (float, int)):
        yticks = [('%.2f' % x).rstrip('0').rstrip('.')
                  for x in self._tables[table_idx].y_values]
      elif isinstance(self._tables[table_idx].y_values[0], (list, dict)):
        yticks = [str(x) for x in self._tables[table_idx].y_values]
      else:
        assert False, 'yticks must be number, list, or dict.'

      table_entry = {
          'title': self._tables[table_idx].name,
          'num_rows': table_dims[1],
          'num_cols': table_dims[0],
          'xlabel': self._tables[table_idx].x_label,
          'xticks': [('%.2f' % x).rstrip('0').rstrip('.')
                     for x in self._tables[table_idx].x_values],
          'ylabel': self._tables[table_idx].y_label,
          'yticks': yticks,
          'job_data': [],
      }
      table_data.append(table_entry)

      for x_idx, y_idx in numpy.ndindex(table_dims):
        idx = self._GetLinearIndex((table_idx, x_idx, y_idx))
        job_entry = {
            'job_id': idx,
            'table_pos': [x_idx, y_idx],
            'scores': [],
            'sim_success': True,
        }
        table_entry['job_data'].append(job_entry)

        if outputs[idx]['sim_success']:
          for score_fn in self._scoring_functions:
            score_fn_output = outputs[idx][score_fn.GetName()]
            job_entry['scores'] += [score_fn.GetScore(score_fn_output)]
        else:
          job_entry['sim_success'] = False

    overview_data = {
        'title': (self._title + ' ' +
                  datetime.datetime.now().strftime('%Y-%m-%d %H:%M %Z')),
        'sim_name': self._sim_name,
        'metrics': [{'name': f.GetName(),
                     'system_labels': f.GetSystemLabels(),
                     'severity': f.GetSeverity()}
                    for f in self._scoring_functions],
        'num_major_cols': self._columns,
        'color_map': ['#74add1', '#e0f3f8', '#fee090',
                      '#fdae61', '#f46d43', '#a50026'],
        'commit': build_info.GetGitSha(),
        'parameter_seed': _random.seed,
        'table_data': table_data,
    }

    # d3 will fail to parse a JSON file that contains NaNs or Infinity's --
    # they aren't officially supported by JSON. Replace them with nulls instead.
    json_str = re.sub(r'\bNaN\b', 'null', json.dumps(overview_data, indent=2))
    json_str = re.sub(r'\bInfinity\b', 'null', json_str)

    with open(output_dir + '/overview_data.json', 'w') as f:
      f.write(json_str)

    if FLAGS.upload_overview_data:
      gsutil_api = gsutil.GsutilApi()
      gsutil_api.Copy(output_dir + '/overview_data.json',
                      os.path.join('gs://makani/', self._gcs_base_dir,
                                   'overview_data.json'), True)

    copy_specs = [('d3.v4.min.js', 'd3.v4.min.js'),
                  ('c3.min.js', 'c3.min.js'),
                  ('c3.css', 'c3.css'),
                  ('bokeh-0.13.0.min.css', 'bokeh-0.13.0.min.css'),
                  ('bokeh-0.13.0.min.js', 'bokeh-0.13.0.min.js'),
                  ('frontend/sweeps_report.css', 'sweeps_report.css'),
                  ('frontend/sweeps_report.html', 'index.html'),
                  ('frontend/sweeps_report.js', 'sweeps_report.js'),
                  ('frontend/shared_library.js', 'shared_library.js')]
    for source, dest_basename in copy_specs:
      dest = os.path.join(output_dir, dest_basename)
      shutil.copy(os.path.join(makani.HOME, 'lib/python/batch_sim', source),
                  dest)
      if dest.endswith('.js'):  # Make JavaScript executable.
        os.chmod(dest, 0770)
      else:  # Allow files to be overwritable
        os.chmod(dest, 0660)


class ParameterRange(object):
  """Abstract class representing a parameter sweep.

  Classes derived from ParameterRange represent lists of labeled, partial
  overrides for simulations.  See WindSpeedParameterRange for an example.

  Attributes:
    label: The label associated with this parameter range.
    values: Numerical labels for each individual value.
    values_range: The values that define the range of the distribution.
    distribution: [dict] probability distribution for random values.
      For normal distribution the expected keys are:
        mean - Mean of the parameter values distribution.
        sigma - Standard deviation of parameter values distribution.
        bound or lower/upper_bound - bounds to truncate parameter distribution,
                                     specified as multiple of sigma, or
                                     absolute.
        type - 'normal'.
      For uniform distribution the expected keys are:
        lower_bound: minimum value of the parameter.
        upper_bound: maximum value of the parameter.
        type - 'uniform'.

      Any other distribution type will raise an error.
  """

  def __init__(self, label, values, distribution=None):
    self.label = label
    self.values = values
    self.values_range = [numpy.min(values, axis=0), numpy.max(values, axis=0)]
    self.distribution = {'type': None} if distribution is None else distribution

  def _ClipValue(self, value):
    """Clip the returned value from random generator to specified bounds."""

    # This is preferred over calling the generator again for a
    # new random number, as this ensures repeatable seed for batches
    # irrespective of choice of parameter values.

    if ('lower_bound' in self.distribution and
        'upper_bound' in self.distribution):
      lower_bound = self.distribution['lower_bound']
      upper_bound = self.distribution['upper_bound']
    else:
      lower_bound = (self.distribution['mean'] -
                     self.distribution['bound'] * self.distribution['sigma'])
      upper_bound = (self.distribution['mean'] +
                     self.distribution['bound'] * self.distribution['sigma'])
    self.values_range = [lower_bound, upper_bound]
    return numpy.clip(value, a_min=lower_bound, a_max=upper_bound)

  def GetRandomValue(self):
    """Randomly select a value from the parameters ditribution."""

    if self.distribution['type'] == 'uniform':
      value = _random.Generator().uniform(self.distribution['lower_bound'],
                                          self.distribution['upper_bound'])
      self.values_range = [self.distribution['lower_bound'],
                           self.distribution['upper_bound']]
    elif self.distribution['type'] == 'normal':
      value = _random.Generator().normal(self.distribution['mean'],
                                         self.distribution['sigma'])
      value = self._ClipValue(value)
    elif self.distribution['type'] == 'lognormal':
      value = (_random.Generator().lognormal(self.distribution['mean'],
                                             self.distribution['sigma'])
               + self.distribution['loc'])
      value = self._ClipValue(value)
    elif self.distribution['type'] == 'vonmises':
      if self.distribution['units'].startswith('rad'):
        mu = self.distribution['mean']
        kappa = 1.0 / numpy.square(self.distribution['sigma'])
        value = _random.Generator().vonmises(mu, kappa)
      elif self.distribution['units'].startswith('deg'):
        mu = numpy.deg2rad(self.distribution['mean'])
        kappa = 1.0 / numpy.square(numpy.deg2rad(self.distribution['sigma']))
        value = numpy.rad2deg(_random.Generator().vonmises(mu, kappa))
      else:
        raise NotImplementedError
      # TODO: Implement smart circular clipping option.
    elif self.distribution['type'] == 'multimodal':
      # Use the ParameterRange class recursively to combine any combination
      # of distributions.

      # Args:
      #   distributions: List of the different distributions to combine, each a
      #     dictionary with all the parameters needed to be a fully defined
      #     ParameterRange distribution.
      #   weights: List of the same length, with the relative probability
      #     weights.

      select_value = _random.Generator().uniform(
          0.0, sum(self.distribution['weights']))
      selected_mode = (numpy.searchsorted(
          numpy.cumsum(self.distribution['weights']),
          select_value))
      mode = ParameterRange(
          None, None, self.distribution['distributions'][selected_mode])
      value = mode.GetRandomValue()
      # Instead of clipping final output, rely on the clipping specified in
      # each mode.
    else:
      # Add other distribution implementations here as needed.
      raise NotImplementedError
    return value

  def GetRandomOverride(self):
    """Randomly select a set of overrides."""
    value = self.GetRandomValue()
    return self.GetOverrides(value), value

  def GetDisplayValue(self, value):
    """Get a human readable representation of a value."""
    return value

  def GetRangeValues(self):
    """Get the parameter range values."""
    return self.values_range

  def GetOverrides(self, value):
    """Abstract method which returns a set of overrides.

    Args:
      value: Scalar value to apply as an override.

    Returns:
      A set of overrides to be included in a simulation.
    """
    raise NotImplementedError


class NoneParameterRange(ParameterRange):
  """An empty parameter range."""

  def __init__(self, label):
    """Constructor.

    Args:
      label: Label for this range.
    """
    super(NoneParameterRange, self).__init__(label, [0.0])

  def GetOverrides(self, unused_idx):
    return {}


class WaveParameterSelector(object):
  """Select appropriate sea conditions given the wind."""

  def __init__(self, wave_variation_params):
    # Dictionary defining the correlation_fit parameters needed to calculate the
    # expected value as a function of other sim parameters.
    self._wave_variation_params = wave_variation_params

  def _GetWaveOverrides(
      self, wave_variation_overrides, wind_speed, wind_direction):
    """Calcute the expected value for each parameter and add the variation."""

    wave_overrides = {}
    wave_overrides['wave_heading_ned'] = (
        (wind_direction
         + numpy.deg2rad(wave_variation_overrides['wave_wind_alignment']))
        % (2 * numpy.pi))

    hs = self._wave_variation_params['significant_height']['correlation_fit']
    wave_overrides['significant_height'] = (
        hs['coefficient'] * numpy.square(wind_speed) +
        hs['intercept'] + wave_variation_overrides['significant_height'])
    wave_overrides['significant_height'] = numpy.clip(
        wave_overrides['significant_height'],
        a_min=hs['lower_bound'], a_max=hs['upper_bound'])
    assert wave_overrides['significant_height'] > 0.0, (
        'Wave height must be greater than zero. '
        'Adjust the correlation_fit lower_bound.')

    tp = self._wave_variation_params['peak_period']['correlation_fit']
    wave_overrides['peak_period'] = (
        tp['coefficient'] * wave_overrides['significant_height'] +
        tp['intercept'] + wave_variation_overrides['peak_period'])
    wave_overrides['peak_period'] = numpy.clip(
        wave_overrides['peak_period'],
        a_min=tp['lower_bound'], a_max=tp['upper_bound'])
    assert wave_overrides['peak_period'] > 0.0, (
        'Wave period must be greater than zero. '
        'Adjust the correlation_fit lower_bound.')

    return {'sim': {'sea_sim': wave_overrides}}

  def AddWaveOverrides(self, overrides, base_overrides):
    """Pull out the wave variation overrides and add in the wave overrides."""
    merged_overrides = dict_util.MergeNestedDicts(overrides, base_overrides)
    wave_overrides = self._GetWaveOverrides(
        merged_overrides['sim']['sea_sim']['waves'],
        merged_overrides['sim']['phys_sim']['wind_speed'],
        merged_overrides['sim']['phys_sim']['wind_direction'])
    overrides = dict_util.MergeNestedDicts(overrides, wave_overrides)
    del overrides['sim']['sea_sim']['waves']
    return overrides


class CustomParameterRange(ParameterRange):
  """A parameter range for a custom defined variable."""

  def __init__(self, name, path, value, distribution):
    """Constructor.

    Args:
      name: String of name and/or description of variable.
      path: List of keys to get to parameter in overrides.
      value: Replacement value for parameter.
      distribution: Parameter for specifying probability distribution.
    """
    self.path = path

    super(CustomParameterRange, self).__init__(
        '%s :' % name, value, distribution)

  def GetOverrides(self, value):

    # Makes a copy to enable popping off last value.
    path = copy.deepcopy(self.path)

    if not isinstance(path[0], list):
      path = [path]
      value = [value]

    all_overrides = {}
    for p, v in zip(path, value):
      overrides_nest = {}

      # Places value into bottom layer of overrides.
      overrides_nest[p.pop()] = v

      # Initialize overrides with bottom layer.
      overrides = overrides_nest

      # Builds up nested overrides dict from path.
      while p:
        overrides = {}
        overrides[p.pop()] = overrides_nest
        overrides_nest = overrides

      all_overrides = dict_util.MergeNestedDicts(
          all_overrides, overrides)

    return all_overrides


class CenterOfMassOffsetParameterRange(ParameterRange):
  """A parameter range for offsetting the simulated center-of-mass."""

  def __init__(self, dim, offsets, distribution, body, body_name=''):
    """Constructor.

    Args:
      dim: Dimension to offset (one of 0, 1, 2).
      offsets: Offsets to apply.
      distribution: Parameter for specifying probability distribution.
      body: String identifying the body in the 'sim' dictionary.
      body_name: Name to be used in the score name.
    """
    self.dim = dim
    self.body = body
    super(CenterOfMassOffsetParameterRange, self).__init__(
        body_name + ' %s COM Offset [m]' % ['X', 'Y', 'Z'][dim], offsets,
        distribution)

  def GetOverrides(self, value):
    return {
        'sim': {
            self.body: {
                'mass_prop_uncertainties': {
                    'center_of_mass_offset': {
                        self.dim: value
                    }
                }
            }
        }
    }


class MassScaleParameterRange(ParameterRange):
  """A parameter range scaling the simulated kite mass."""

  def __init__(self, scales, distribution, body, body_name=''):
    """Constructor.

    Args:
      scales: Scale factors to apply.
      distribution: Parameter for specifying probability distribution.
      body: String identifying the body in the 'sim' dictionary.
      body_name: Name to be used in the score name.
    """
    self.body = body
    super(MassScaleParameterRange, self).__init__(
        body_name + ' Mass Scaling [#]', scales, distribution)

  def GetOverrides(self, value):
    return {
        'sim': {
            self.body: {
                'mass_prop_uncertainties': {
                    'mass_scale': value,
                }
            }
        }
    }


class InertiaScaleParameterRange(ParameterRange):
  """A parameter range scaling the simulated kite moments-of-inertia."""

  def __init__(self, dim, scales, distribution, body, body_name=''):
    """Constructor.

    Args:
      dim: Dimension to apply the scaling to (one of 0, 1, 2).
      scales: Scale factors to apply.
      distribution: Parameter for specifying probability distribution.
      body: String identifying the body in the 'sim' dictionary.
      body_name: Name to be used in the score name.
    """
    self.dim = dim
    self.body = body
    super(InertiaScaleParameterRange, self).__init__(
        body_name + ' %s Inertia Scaling [#]' % ['X', 'Y', 'Z'][dim], scales,
        distribution)

  def GetOverrides(self, value):
    return {
        'sim': {
            self.body: {
                'mass_prop_uncertainties': {
                    'moment_of_inertia_scale': {
                        self.dim: value
                    }
                }
            }
        }
    }


class AeroSimOffsetParameterRange(ParameterRange):
  """A parameter range overriding the aerodynamic offsets."""

  def __init__(self, field, offsets, distribution):
    """Constructor.

    Args:
      field: Offset to adjust.
      offsets: A list of values to sweep over.
      distribution: Parameter for specifying probability distribution.
    """
    self.field = field
    super(AeroSimOffsetParameterRange, self).__init__(
        field + ' Offset', offsets, distribution)

  def GetOverrides(self, value):
    return {
        'sim': {
            'aero_sim': {
                'coeff_offsets': {
                    self.field: value
                }
            }
        }
    }


class AeroSimMomentBFlapScalingParameterRange(ParameterRange):
  """A parameter range for scaling the moments from flap deflections."""

  def __init__(self, label, moment, basis, scale_differences,
               distribution):
    """Constructor.

    Args:
      label: Name to display.
      moment: Index in (0, 1, 2) indicating which moment to scale.
      basis: Array of kNumFlaps values to multiply scale_differences by.
      scale_differences: Value to add to 1.0 to generate the scaling.
      distribution: Parameter for specifying probability distribution.
    """
    self.moment = moment
    self.basis = basis
    super(AeroSimMomentBFlapScalingParameterRange, self).__init__(
        label, scale_differences, distribution)

  def GetOverrides(self, value):
    scale_factor = 1.0 + value
    return {
        'sim': {
            'aero_sim': {
                'moment_coeff_b_scale_factors': {
                    'flap_derivatives': {
                        flap: {
                            self.moment: scale_factor * self.basis[flap]
                        }
                        for flap in range(system_types.kNumFlaps)
                        if numpy.abs(self.basis[flap]) > 0.0
                    }
                }
            }
        }
    }


class AeroSimForceBFlapScalingParameterRange(ParameterRange):
  """A parameter range for scaling the forces from flap deflections."""

  def __init__(self, label, force, basis, scale_differences,
               distribution):
    """Constructor.

    Args:
      label: Name to display.
      force: Index in (0, 1, 2) indicating which force to scale.
      basis: Array of kNumFlaps values to multiply scale_differences by.
      scale_differences: Value to add to 1.0 to generate the scaling.
      distribution: Parameter for specifying probability distribution.
    """
    self.force = force
    self.basis = basis
    super(AeroSimForceBFlapScalingParameterRange, self).__init__(
        label, scale_differences, distribution)

  def GetOverrides(self, value):
    scale_factor = 1.0 + value
    return {
        'sim': {
            'aero_sim': {
                'force_coeff_w_scale_factors': {
                    'flap_derivatives': {
                        flap: {
                            self.force: scale_factor * self.basis[flap]
                        }
                        for flap in range(system_types.kNumFlaps)
                        if numpy.abs(self.basis[flap]) > 0.0
                    }
                }
            }
        }
    }


class AeroSimMomentBRateScalingParameterRange(ParameterRange):
  """A parameter range for scaling the body aero moments from body rates."""
  moments = ['l', 'm', 'n']

  def __init__(self, moment, rate, scale_differences, distribution):
    """Constructor.

    Args:
      moment: Index in (0, 1, 2) indicating which moment to scale.
      rate: One of 'p', 'q', or 'r'.
      scale_differences: Value to add to 1.0 to generate the scaling.
      distribution: Parameter for specifying probability distribution.
    """
    self.moment = moment
    self.rate = rate
    super(AeroSimMomentBRateScalingParameterRange, self).__init__(
        'C%s%s Scaling Factor Offset [#]'
        % (self.moments[self.moment], self.rate),
        scale_differences, distribution)

  def GetOverrides(self, value):
    scaling = 1.0 + value
    return {
        'sim': {
            'aero_sim': {
                'moment_coeff_b_scale_factors': {
                    'rate_derivatives': {
                        self.rate: {
                            self.moment: scaling
                        }
                    }
                }
            }
        }
    }


class AeroSimForceBRateScalingParameterRange(ParameterRange):
  """A parameter range for scaling the wind aero forces from body rates."""
  forces = ['D', 'Y', 'L']

  def __init__(self, force, rate, scale_differences, distribution):
    """Constructor.

    Args:
      force: Index in (0, 1, 2) indicating which force to scale.
      rate: One of 'p', 'q', or 'r'.
      scale_differences: Value to add to 1.0 to generate the scaling.
      distribution: Parameter for specifying probability distribution.
    """
    self.force = force
    self.rate = rate
    super(AeroSimForceBRateScalingParameterRange, self).__init__(
        'C%s%s Scaling Factor Offset [#]'
        % (self.forces[self.force], self.rate),
        scale_differences, distribution)

  def GetOverrides(self, value):
    scaling = 1.0 + value
    return {
        'sim': {
            'aero_sim': {
                'force_coeff_w_scale_factors': {
                    'rate_derivatives': {
                        self.rate: {
                            self.force: scaling
                        }
                    }
                }
            }
        }
    }


class AeroSimFlapOffsetParameterRange(ParameterRange):
  """A parameter range overriding the flap offsets."""

  def __init__(self, label, basis, offsets, distribution):
    """Constructor.

    Args:
      label: Label for this range.
      basis: Basis vector to scale (numpy.array of kNumFlaps elements).
      offsets: A list of values to sweep over.
      distribution: Parameter for specifying probability distribution.
    """
    self.basis = basis
    super(AeroSimFlapOffsetParameterRange, self).__init__(label, offsets,
                                                          distribution)

  def GetOverrides(self, value):
    return {
        'sim': {
            'aero_sim': {
                'flap_offsets': {
                    flap: value * self.basis[flap]
                    for flap in range(system_types.kNumFlaps)
                    if numpy.abs(self.basis[flap]) > 0.0
                }
            }
        }
    }


class AirDensityParameterRange(ParameterRange):
  """A parameter range overriding the simulator's air density."""

  def __init__(self, air_densities, distribution):
    """Constructor.

    Args:
      air_densities: A list of air densities defining this parameter range.
      distribution: Parameter for specifying probability distribution.
    """
    super(AirDensityParameterRange, self).__init__(
        'Air Density [kg/m^3]', air_densities, distribution)

  def GetOverrides(self, value):
    return {
        'sim': {
            'phys_sim': {
                'air_density': value
            }
        }
    }


class WindSpeedParameterRange(ParameterRange):
  """A parameter range overriding the wind speed."""

  def __init__(self, wind_speeds, wind_shear_ref_height_agl=None,
               t_updates=None, max_wind_speed=None):
    """Constructor.

    Args:
      wind_speeds: A list of wind speeds defining this parameter range.
      wind_shear_ref_height_agl: Above-ground-level reference height [m] for
          the wind shear model.
      t_updates: A list of times [s] when wind speed updates are applied.
      max_wind_speed: Speed [m/s] used to saturate the mean wind speed before
          the second entry and after the third entry in t_updates.
    """
    if wind_shear_ref_height_agl is not None:
      label = 'Wind Speed [m/s] @ %.f [m] AGL' % wind_shear_ref_height_agl
    else:
      label = 'Wind Speed [m/s]'

    super(WindSpeedParameterRange, self).__init__(
        label, wind_speeds, distribution=None)
    self._wind_shear_ref_height_agl = wind_shear_ref_height_agl
    self._t_updates = t_updates
    self._max_wind_speed = max_wind_speed

  def GetOverrides(self, value):
    assert value >= 0, ('Wind speed must be positive. '
                        'Use WindDirectionDegParameterRange override to assign '
                        'appropriate direction.')
    overrides = {
        'sim': {
            'phys_sim': {
                'wind_speed': value
            }
        }
    }

    if self._wind_shear_ref_height_agl is not None:
      overrides['sim']['phys_sim']['wind_shear_ref_height_agl'] = (
          self._wind_shear_ref_height_agl)

    if self._t_updates is not None:
      num_updates = len(self._t_updates)
      assert num_updates == 3, (
          'The wind speed saturation logic in batch sims requires 3 updates.')

      offset_value = min(0.0, self._max_wind_speed - value)
      wind_speed_offsets = [{
          't_update': self._t_updates[0],
          'offset': offset_value,
      }, {
          't_update': self._t_updates[1],
          'offset': 0.0,
      }, {
          't_update': self._t_updates[2],
          'offset': offset_value,
      }]

      overrides['sim']['phys_sim']['wind_speed_update'] = (
          overrides_util.PreprocessWindSpeedUpdates(wind_speed_offsets))

    return overrides


class WindDirectionDegParameterRange(ParameterRange):
  """A parameter range overriding the wind_direction."""

  def __init__(self, wind_directions, distribution):
    """Constructor.

    Args:
      wind_directions: A list of wind directions [deg].
      distribution: Parameter for specifying probability distribution.
    """
    super(WindDirectionDegParameterRange, self).__init__(
        'Wind Direction [deg]', wind_directions, distribution)

  def GetOverrides(self, value):
    return {
        'sim': {
            'phys_sim': {
                'wind_direction': numpy.deg2rad(value % 360.0)
            }
        }
    }


class WindElevationDegParameterRange(ParameterRange):
  """A parameter range overriding the wind elevation."""

  def __init__(self, elevations, distribution):
    """Constructor.

    Args:
      elevations: A list of wind elevations [deg].
      distribution: Parameter for specifying probability distribution.
    """
    super(WindElevationDegParameterRange, self).__init__(
        'Wind Elevation [deg]', elevations, distribution)

  def GetOverrides(self, value):
    return {
        'sim': {
            'phys_sim': {
                'wind_elevation': numpy.deg2rad(value)
            }
        }
    }


class WindDatabaseInitialTimeParameterRange(ParameterRange):
  """A parameter range overriding the initial time of the wind database."""

  def __init__(self, times, distribution):
    """Constructor.

    Args:
      times: A list of initial times [s].
      distribution: Parameter for specifying probability distribution.
    """
    super(WindDatabaseInitialTimeParameterRange, self).__init__(
        'Wind Database Initial Time [s]', times, distribution)

  def GetOverrides(self, value):
    return {
        'sim': {
            'phys_sim': {
                'wind_database_initial_time': value
            }
        }
    }


class WindDatabaseYOffsetParameterRange(ParameterRange):
  """A parameter range overriding the y offset of the wind database."""

  def __init__(self, offset_positions, distribution):
    """Constructor.

    Args:
      offset_positions: A list of offset positions [m].
      distribution: Parameter for specifying probability distribution.
    """
    super(WindDatabaseYOffsetParameterRange, self).__init__(
        'Wind Database Y offset [m]', offset_positions, distribution)

  def GetOverrides(self, value):
    return {
        'sim': {
            'phys_sim': {
                'wind_database_y_offset': value
            }
        }
    }


class WindVeerDegParameterRange(ParameterRange):
  """A parameter range overriding the wind veer."""

  def __init__(self, wind_directions, distribution, start_height_agl=100.0,
               end_height_agl=400.0):
    """Constructor.

    Args:
      wind_directions: A list of changes in wind direction [deg].
      distribution: Parameter for specifying probability distribution.
      start_height_agl: Height [m] above-ground-level at which to start
          the direction change.
      end_height_agl: Height [m] above-ground-level at which to end
          the direction change.
    """
    label = 'Wind Veer [deg] from %.f to %.f [m] AGL' % (start_height_agl,
                                                         end_height_agl)
    super(WindVeerDegParameterRange, self).__init__(label, wind_directions,
                                                    distribution)
    self._start_height = start_height_agl
    self._end_height = end_height_agl

  def GetOverrides(self, value):
    return {
        'sim': {
            'phys_sim': {
                'wind_veer': numpy.deg2rad(value),
                'wind_veer_start_height_agl': self._start_height,
                'wind_veer_end_height_agl': self._end_height,
            }
        }
    }


class WindShearExponentParameterRange(ParameterRange):
  """A parameter range overriding the wind shear exponent."""

  def __init__(self, wind_shear_exponents):
    """Constructor.

    Args:
      wind_shear_exponents: A list of exponents defining this parameter range.
    """
    super(WindShearExponentParameterRange, self).__init__(
        'Wind Shear [#]', wind_shear_exponents, distribution=None)

  def GetOverrides(self, value):
    return {
        'sim': {
            'phys_sim': {
                'wind_shear_exponent': value,
            }
        }
    }


class PitotPitchDegParameterRange(ParameterRange):
  """A parameter range offsetting the pitch angle of the Pitot."""

  def __init__(self, angles_deg, distribution):
    """Constructor.

    Args:
      angles_deg: A list of pitch angles [deg] defining this parameter range.
      distribution: Parameter for specifying probability distribution.
    """
    super(PitotPitchDegParameterRange, self).__init__(
        'Pitot Pitch Angle Offset [deg]', angles_deg, distribution)

  def GetOverrides(self, value):
    return {
        'sim': {
            'pitots_sim': {
                0: {'pitch_offset': numpy.deg2rad(value)},
                1: {'pitch_offset': numpy.deg2rad(value)}
            }
        }
    }


class PitotYawDegParameterRange(ParameterRange):
  """A parameter range offsetting the yaw angle of the Pitot."""

  def __init__(self, angles_deg, distribution):
    """Constructor.

    Args:
      angles_deg: A list of yaw angles [deg] defining this parameter range.
      distribution: Parameter for specifying probability distribution.
    """
    super(PitotYawDegParameterRange, self).__init__(
        'Pitot Yaw Angle Offset [deg]', angles_deg, distribution)

  def GetOverrides(self, value):
    return {
        'sim': {
            'pitots_sim': {
                0: {'yaw_offset': numpy.deg2rad(value)},
                1: {'yaw_offset': numpy.deg2rad(value)}
            }
        }
    }


class PitotCpOffsetParameterRange(ParameterRange):
  """A parameter range offsetting the pressure coefficient of the Pitot."""

  def __init__(self, cp_offsets, distribution):
    """Constructor.

    Args:
      cp_offsets: A list of pressure coefficient offsets [#] defining this
          parameter range.
      distribution: Parameter for specifying probability distribution.
    """
    super(PitotCpOffsetParameterRange, self).__init__('Pitot Cp Offset [#]',
                                                      cp_offsets, distribution)

  def GetOverrides(self, value):
    return {
        'sim': {
            'pitots_sim': {
                0: {'local_pressure_coeff_offset': value},
                1: {'local_pressure_coeff_offset': value}
            }
        }
    }


class BuoyModelParameterRange(ParameterRange):
  """Class for defining uncertainties in buoy model parameters."""

  def __init__(self, name, offsets, distribution, category, variable):
    """Constructor.

    Args:
      name: Name of the uncertain variable.
      offsets: A list of uncertainties defining this parameter range.
      distribution: Parameter for specifying probability distribution.
      category: String identifying the model parameter category in the buoy_sim
          dictionary.
      variable: String identifying the variable.
    """
    super(BuoyModelParameterRange, self).__init__(name, offsets, distribution)
    self.category = category
    self.variable = variable

  def GetOverrides(self, value):
    return {
        'sim': {
            'buoy_sim': {
                self.category: {
                    'uncertainties': {
                        self.variable: value,
                    }
                }
            }
        }
    }


class FaultParameterRange(ParameterRange):
  """A parameter range adding a single fault to a simulation.

  Attributes:
    label: Name for this parameter range.
  """

  def __init__(self, label, t_start, t_end, component, fault_type,
               parameters_start, parameters_end,
               num_parameters):
    """Constructor for the range of faults.

    A given fault is applied with varying parameters by linearly interpolating
    between two parameter sets.

    Args:
      label: The name for this parameter range.
      t_start: When the fault should start.
      t_end: When the fault should end.
      component: Name of the component for the fault.
      fault_type: Type of fault to apply.
      parameters_start: Initial parameter values for the sweep.
      parameters_end: Final parameter values for the sweep.
      num_parameters: Number of parameters to include in the sweep.
    """
    self._t_start = t_start
    self._t_end = t_end
    self._component = component
    self._fault_type = fault_type
    self.values = [None for _ in range(num_parameters)]

    for parameter_idx in range(num_parameters):
      if num_parameters == 1:
        interp = 1.0
      else:
        interp = parameter_idx / (num_parameters - 1.0)

      self.values[parameter_idx] = (
          (1.0 - interp) * numpy.array(parameters_start)
          + interp * numpy.array(parameters_end))

    super(FaultParameterRange, self).__init__(
        label, self.values, distribution=None)

  def GetDisplayValue(self, value):
    return self._GetParameterValue(value)

  def _GetParameterValue(self, parameters):
    raise NotImplementedError

  def GetOverrides(self, value):
    return {
        'sim': {
            'faults_sim': [{
                't_start': self._t_start,
                't_end': self._t_end,
                'component': self._component,
                'type': self._fault_type,
                'parameters': value.tolist(),
            }]
        }
    }


class ParameterRangeTable(OverridesTable):
  """An OverridesTable built out of two ParameterRanges.

  Given two parameter ranges, constructs a Cartesian product of simulations
  whose overrides combine those provided by each range.
  """

  def __init__(self, name, x_parameter_range, y_parameter_range,
               base_overrides=None, turbsim_database_selector=None,
               wave_parameter_selector=None):
    """Constructor.

    Args:
      name: Name to give to this table.
      x_parameter_range: A ParameterRange.
      y_parameter_range: A ParameterRange.
      base_overrides: Overrides to apply to all table entries.
      turbsim_database_selector: Information about the online TurbSim folder
        from which to pull wind databases.
      wave_parameter_selector: Information about the sea state variations and
        the parameter correlations.
    """
    x_values = [
        x_parameter_range.GetDisplayValue(v) for v in x_parameter_range.values
    ]
    y_values = [
        y_parameter_range.GetDisplayValue(v) for v in y_parameter_range.values
    ]
    super(ParameterRangeTable, self).__init__(
        name, x_parameter_range.label, x_values,
        y_parameter_range.label, y_values, base_overrides=base_overrides)

    dim = self.GetDimensions()
    self._overrides_desc = [
        [None for _ in range(dim[1])] for _ in range(dim[0])
    ]
    self._overrides = [[None for _ in range(dim[1])] for _ in range(dim[0])]
    self._overrides_limits = [
        [None for _ in range(dim[1])] for _ in range(dim[0])
    ]

    for x_idx, y_idx in numpy.ndindex((len(x_parameter_range.values),
                                       len(y_parameter_range.values))):
      overrides_desc = [
          (x_parameter_range.label,
           x_parameter_range.GetDisplayValue(x_parameter_range.values[x_idx])),
          (y_parameter_range.label,
           y_parameter_range.GetDisplayValue(y_parameter_range.values[y_idx]))
      ]

      # Overrides specified in y_parameter range will step on those that
      # are also specified in x_parameter range to allow custom sweep to
      # modify overrides.
      overrides = dict_util.UpdateNestedDict(
          x_parameter_range.GetOverrides(x_parameter_range.values[x_idx]),
          y_parameter_range.GetOverrides(y_parameter_range.values[y_idx]))

      x_param_values = x_parameter_range.GetRangeValues()
      y_param_values = y_parameter_range.GetRangeValues()
      overrides_limits = [(x_param_values[0], x_param_values[-1])]
      overrides_limits += [(y_param_values[0], y_param_values[-1])]

      # If a TurbSimDatabaseSelector is given, get overrides based on wind
      # conditions and pick apppropriate TurbSim database.
      if turbsim_database_selector:
        # For shear sweeps, shear value is in overrides, not base_overrides, so
        # need to update them so turbsim_database_selector has the info needed.
        updated_overrides = dict_util.UpdateNestedDict(
            base_overrides, overrides)
        specific_override, file_index = (
            turbsim_database_selector.GetSpecificOverride(updated_overrides, 0))
        overrides_desc += [(turbsim_database_selector.label, file_index)]
        # Specific override effects turbsim only, so use merge instead of
        # update to ensure we're not specifying turbsim database in
        # conflicting flags.
        overrides = dict_util.MergeNestedDicts(overrides, specific_override)
        num_databases_with_wind_condition = (
            turbsim_database_selector.GetNumAppropriateDatabases(
                updated_overrides))
        overrides_limits += [(0, num_databases_with_wind_condition-1)]

      if wave_parameter_selector:
        overrides = wave_parameter_selector.AddWaveOverrides(
            overrides, base_overrides)

      self._overrides_desc[x_idx][y_idx] = overrides_desc
      self._overrides[x_idx][y_idx] = overrides
      self._overrides_limits[x_idx][y_idx] = overrides_limits

  def GetOverridesDesc(self, x_idx, y_idx):
    return self._overrides_desc[x_idx][y_idx]

  def GetOverridesLimits(self, x_idx, y_idx):
    return self._overrides_limits[x_idx][y_idx]

  def GetOverrides(self, x_idx, y_idx):
    return self._overrides[x_idx][y_idx]


class ParameterRangeMonteCarloTable(OverridesTable):
  """An OverridesTable for performing Monte Carlo Trials.

  Given a list of parameter ranges, randomly selects values and runs
  simulations.
  """

  def __init__(self, name, dim, parameter_ranges, randomize_seed=True,
               base_overrides=None, turbsim_database_selector=None,
               wave_parameter_selector=None):
    """Constructor.

    Args:
      name: Name to give to this table.
      dim: Tuple containing the number of rows and columns.
      parameter_ranges: List of ParameterRange.
      randomize_seed: Whether to randomize the simulator's seed.
      base_overrides: Overrides to be applied to each simulation.
      turbsim_database_selector: Information about the online TurbSim folder
        from which to pull wind databases from.
      wave_parameter_selector: Information about the sea state variations and
        the parameter correlations.

    """
    super(ParameterRangeMonteCarloTable, self).__init__(
        name, 'Row', range(0, dim[0]), 'Column', range(0, dim[1]),
        base_overrides=base_overrides)

    self._overrides_desc = [
        [None for _ in range(dim[1])] for _ in range(dim[0])
    ]
    self._overrides = [[None for _ in range(dim[1])] for _ in range(dim[0])]
    self._overrides_limits = [
        [None for _ in range(dim[1])] for _ in range(dim[0])
    ]

    for x_idx, y_idx in numpy.ndindex((dim[0], dim[1])):
      if randomize_seed:
        seed_offset = _random.Generator().randint(65536)
        overrides_desc = [('Random Seed Offset', seed_offset)]
        overrides = {'sim': {'random_seed_offset': seed_offset}}
        overrides_limits = [('NA', 'NA')]
      else:
        overrides_desc = []
        overrides = {}
        overrides_limits = []

      # Do not override any parameter for the first cell of each block.
      if x_idx == 0 and y_idx == 0:
        overrides_desc = []
        overrides = {}
        overrides_limits = []
      else:
        for parameter_range in parameter_ranges:
          random_override, value = parameter_range.GetRandomOverride()
          overrides_desc += [(parameter_range.label, value)]
          overrides = dict_util.MergeNestedDicts(overrides, random_override)
          param_values = parameter_range.GetRangeValues()
          overrides_limits += [(param_values[0], param_values[-1])]

      # If a TurbSimDatabaseSelector is given, get overrides based on wind
      # conditions and pick appropriate TurbSim database.
      if turbsim_database_selector:
        if x_idx == 0 and y_idx == 0:
          # Use baseline wind database for first cell of each block.
          merged_overrides = dict_util.MergeNestedDicts(overrides,
                                                        base_overrides)
          specific_override, file_index = (
              turbsim_database_selector.GetSpecificOverride(merged_overrides,
                                                            0))
          overrides = dict_util.MergeNestedDicts(overrides, specific_override)
          overrides_desc = []
          overrides_limits = []
        else:
          # With TurbsimDatabaseSelector class in its own util file,
          # the random number generation needs to be pulled out and
          # kept within parameter_tables.
          random_num = _random.Generator().randint(
              turbsim_database_selector.GetNumAppropriateDatabases(
                  base_overrides))
          random_override, file_index = (
              turbsim_database_selector.GetSpecificOverride(base_overrides,
                                                            random_num))
          overrides = dict_util.MergeNestedDicts(overrides, random_override)
          overrides_desc += [(turbsim_database_selector.label, file_index)]
          num_databases_with_wind_condition = (
              turbsim_database_selector.GetNumAppropriateDatabases(
                  base_overrides))
          overrides_limits += [(0, num_databases_with_wind_condition-1)]

      if wave_parameter_selector and not (x_idx == 0 and y_idx == 0):
        overrides = wave_parameter_selector.AddWaveOverrides(
            overrides, base_overrides)

      self._overrides_desc[x_idx][y_idx] = overrides_desc
      self._overrides[x_idx][y_idx] = overrides
      self._overrides_limits[x_idx][y_idx] = overrides_limits

  def GetOverridesDesc(self, x_idx, y_idx):
    return self._overrides_desc[x_idx][y_idx]

  def GetOverridesLimits(self, x_idx, y_idx):
    return self._overrides_limits[x_idx][y_idx]

  def GetOverrides(self, x_idx, y_idx):
    return self._overrides[x_idx][y_idx]


class DisturbancesSimClient(OverridesTableSimClient):
  """Abstract client for generating tables."""

  def __init__(self, output_dir, base_overrides, x_parameter_range,
               disturbances, scoring_functions, **kwargs):
    """Constructor for a disturbance sim.

    Args:
      output_dir: Directory to write outputs.
      base_overrides: Overrides to apply to each table.
      x_parameter_range: Variable to sweep along the x_axis.
      disturbances: A dictionary mapping table names to disturbance ranges.
      scoring_functions: A list of ScoringFunctions.
      **kwargs: See client_base.BatchSimClient.
    """
    tables = [
        ParameterRangeTable(disturbance_name,
                            x_parameter_range, disturbance_range,
                            base_overrides=base_overrides)
        for (disturbance_name, disturbance_range) in disturbances
    ]
    super(DisturbancesSimClient, self).__init__(
        output_dir, tables, scoring_functions, **kwargs)
