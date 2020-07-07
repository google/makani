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

"""Utility to plot histograms comparing two batch sims."""
# This script compares the results of two runs of the Monte Carlo
# batch simulation.
#
# To run it:
#   bazel run analysis/util:batch_sim_comparison -- -a old.json -b new.json
#
# old.json and new.json are the overview_data.json files from the HTML
# report of the batchsim Monte Carlo runs you wish to compare. For
# example, if your html report is
#
#  ...jenkins/job/manual_crosswind_sweeps/134/HTML_report
#
# Then add "overview_data.json" to the URL to download the text into a
# JSON file:
#
#  ...jenkins/job/manual_crosswind_sweeps/134/HTML_report/overview_data.json
#
# You can optionally specify a short description of change using '-d' flag.
#
# This script returns png files under makani/logs/batch_comparisons folder.
# You can review results by opening them in web browser.

import json
import os
import shutil
import sys

import gflags
from makani.analysis.util import batchsim_analysis_util as util
import matplotlib
# This needs to be before pyplot import.
matplotlib.use('Agg')
from matplotlib import cm  # pylint: disable=g-import-not-at-top
from matplotlib import pyplot as plt  # pylint: disable=g-import-not-at-top
import numpy as np  # pylint: disable=g-import-not-at-top


gflags.DEFINE_string('case_a', None,
                     'The baseline spec file.', short_name='a')

gflags.DEFINE_string('case_b', None,
                     'The change spec file.', short_name='b')

gflags.DEFINE_string('change_desc', None,
                     'Optional description of change.', short_name='d')

gflags.DEFINE_string('case_a_url', None,
                     'Optional link to baseline HTML report page.',
                     short_name='urla')

gflags.DEFINE_string('case_b_url', None,
                     'Optional link to change HTML report page.',
                     short_name='urlb')

gflags.DEFINE_float('threshold', 0.6,
                    'threshold above which is undesirable.', short_name='t')

gflags.DEFINE_string('output_dir', 'logs/batch_comparisons',
                     'Output directory.', short_name='dir')

FLAGS = gflags.FLAGS


# Define a sequence of colors for wind speeds [0:1:21] m/s
color_book = [cm.magma_r(item) for item in np.linspace(0, 1, 22)]


def _OutputFilename(label):
  label = label.replace(' ', '_')
  keep_chars = ['_']
  label = ''.join(c for c in label if c.isalnum() or c in keep_chars).rstrip()

  return label + '.png'


def Hist(ax, x, y, legend='', color='black',
         line_width=0.5, current_count=1, total_count=1):
  """Plot histograms using matplotlib pyplot."""

  bin_margin = 0.05 * np.gradient(x)
  bin_width = 0.90 * np.gradient(x) / total_count
  x = x[:-1] + bin_margin[:-1] + (current_count + 0.5)*bin_width[:-1]
  ax.bar(x, y, width=bin_width[0], color=color,
         label=legend, linewidth=line_width, edgecolor='k')
  return ax


def _SetExportFolder(export_folder):
  """Create a folder to export plots in makani/logs folder."""

  makani_home = os.environ['MAKANI_HOME']
  if os.path.isdir(export_folder):
    shutil.rmtree(export_folder)

  # Copy css, html and js files needed for displaying charts. This also
  # creates the batch_comparisons folder.
  src = os.path.join(makani_home, 'lib/python/batch_sim/frontend')
  shutil.copytree(src, export_folder)
  # Copy d3.v4.min.js to export folder.
  d3_js_file = os.path.join(makani_home, 'lib/python/batch_sim/d3.v4.min.js')
  shutil.copy(d3_js_file, export_folder)


def _GetCaseLabel(case_name, case_link):
  if case_link is None:
    case_label = ' (' + os.path.basename(case_name) + ')'
  else:
    case_label = '(<a href="' + case_link + '"> HTML Report</a>)'

  return case_label


def _ExportHistograms(case_a, case_b, change_desc, case_a_url, case_b_url,
                      export_folder):
  """Plot histograms and export them as png files."""

  with open(case_a) as fp:
    data_dict_a = json.load(fp)

  with open(case_b) as fp:
    data_dict_b = json.load(fp)

  # Assert the batches being compared were generated with same parameter seed.
  assert util.IsSameSeed(data_dict_a, data_dict_b)

  # Prepare folder for exporting the comparison charts.
  _SetExportFolder(export_folder)

  batch_comp = util.GetComparisonData(data_dict_a=data_dict_a,
                                      data_dict_b=data_dict_b)

  wind_speeds = batch_comp['wind_speeds']
  wind_shears = batch_comp['wind_shears']
  score_labels = batch_comp['score_labels']
  hist_edges = batch_comp['hist_edges']
  hist_values = batch_comp['hist_values']

  # Create a dictionary for png filenames.
  filenames = {}
  for kk in range(len(score_labels)):
    label = score_labels[kk]
    filenames[label] = []

  # Create change description annotations for the chart.
  chart_text = ['CHANGE HELPS', 'CHANGE HURTS']
  if change_desc is not None:
    chart_text = [change_desc.upper() + ' HELPS',
                  change_desc.upper() + ' HURTS']

  if util.IsSameBatch(data_dict_a, data_dict_b):
    x_range = [-5, 205]
    x_label = 'Score bins'
    bool_samebatch = True
  else:
    x_range = [-120, 120]
    x_label = 'Delta Score bins'
    bool_samebatch = False

  # Create and export comparison charts.
  for ii in range(len(wind_shears)):
    wind_shear = wind_shears[ii]
    for kk in range(len(score_labels)):
      label = score_labels[kk]
      chart = []
      ax = []
      for jj in range(len(wind_speeds)):
        wind_speed = wind_speeds[jj]
        score_hist = hist_values[ii][jj][kk]
        bar_color = color_book[int(wind_speed)]

        if jj == 0:
          chart, ax = plt.subplots(figsize=(8.6, 7.0))
          plt.xlim(x_range[0], x_range[-1])
          plt.ylim(-5, 110)
          plt.xlabel(x_label, fontsize=12)
          plt.ylabel('[ % ]', fontsize=12)
          plt.grid(color='grey', linestyle='--', linewidth=0.1)
          plt.xticks(hist_edges)
          plt.text(x=x_range[0] + 5, y=105, fontsize=12, fontweight='bold',
                   s=label)
          plt.text(x=x_range[0] + 5, y=101, fontsize=10, color='grey',
                   s='Shear exponent = ' + str(wind_shear))

          if not bool_samebatch:
            plt.barh(-1, 100, left=10, height=2,
                     color='lightcoral', edgecolor='white', alpha=0.5)
            plt.barh(-1, -100, left=-10, height=2,
                     color='greenyellow', edgecolor='white', alpha=0.5)
            plt.text(x=-100, y=-4.5, s=chart_text[0],
                     fontsize=10, fontweight='bold', color='dimgrey')
            plt.text(x=30, y=-4.5, s=chart_text[1],
                     fontsize=10, fontweight='bold', color='dimgrey')

        ax = Hist(ax, x=hist_edges, y=score_hist, color=bar_color,
                  legend=str(wind_speed) + ' m/s',
                  current_count=jj, total_count=len(wind_speeds))

      plt.legend(prop={'size': 11}, borderpad=0.5, labelspacing=1.0,
                 frameon=True, fancybox=True, framealpha=0.5,
                 bbox_to_anchor=(0.95, 0.92))
      plt.tight_layout()
      filename_png = label + '_shear_' + str(wind_shear)
      filename_png = _OutputFilename(filename_png)
      filenames[label].append(filename_png)
      png_save_path = os.path.join(export_folder, filename_png)
      chart.savefig(png_save_path)
      plt.close()
      print filename_png + ' exported.'

  # Append exported png filenames to batch_comparisons.json.
  batch_comp['filenames'] = filenames

  # Append case labels to batch_comparisons.json.
  case_label_a = _GetCaseLabel(case_a, case_a_url)
  case_label_b = _GetCaseLabel(case_b, case_b_url)
  batch_comp['batch_a'] = ''.join([batch_comp['batch_a'], case_label_a])
  batch_comp['batch_b'] = ''.join([batch_comp['batch_b'], case_label_b])

  # Export the batch comparisons results to json file.
  json_file = os.path.join(export_folder, 'batch_comparisons.json')
  with open(json_file, 'w') as fp:
    json.dump(batch_comp, fp, indent=2)
  print 'Comparison data exported as json file.'
  print 'Comparison report available at: '
  print os.path.join(export_folder, 'comparison_report.html')


def _WriteSummaryFile(case_a, case_b, threshold, export_folder):
  """Write the comparison summary to output_file."""
  output_txt_file = os.path.join(export_folder, 'summary.txt')
  with open(case_a) as fp:
    data_dict_a = json.load(fp)

  with open(case_b) as fp:
    data_dict_b = json.load(fp)

  util.WriteSummary(data_dict_a, data_dict_b, threshold, output_txt_file)


def main(argv):
  gflags.MarkFlagAsRequired('a')
  gflags.MarkFlagAsRequired('b')
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError as e:
    print ('%s\nUsage: %s -a filename_a.json -b filename_b.json\n%s'
           % (e, sys.argv[0], FLAGS))
    sys.exit(1)

  _ExportHistograms(FLAGS.case_a, FLAGS.case_b, FLAGS.change_desc,
                    FLAGS.case_a_url, FLAGS.case_b_url, FLAGS.output_dir)
  _WriteSummaryFile(FLAGS.case_a, FLAGS.case_b, FLAGS.threshold,
                    FLAGS.output_dir)


if __name__ == '__main__':
  main(sys.argv)
