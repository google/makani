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

"""Score evaluation module.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import functools
import importlib
import itertools
import os

from makani.analysis.util.simdata_analysis.data_import import BatchSimDataImporter
from makani.analysis.util.simdata_analysis.simdata import SimData
import matplotlib.pyplot as plt
import numpy as np


def _get_prob(simdata, score_thr, score_table):
  """Computes mean and bounds of the probability of a score above a threshold.

  Args:
    simdata: SimData object.
    score_thr: Score threshold at which to evaluate the probability.
    score_table: 2-element tuple containing (table, score).

  Returns:
    A tuple (mean, lower_bound, upper bound).
  """
  scoredata = simdata.get_score_data(score_table[0], score_table[1],
                                     verbose=False)
  probdata = scoredata.prob_above(score_thr)
  return (probdata.mean, probdata.bounds[0], probdata.bounds[1])


def process_score_probabilities(input_data, output_folder, process_inputs=False,
                                allow_different_commits=False, score_thr=0.999,
                                use_concurrency=False, ncores=None):
  """Processes batch sims in a folder list and outputs the data in CSV format.

  For concurrent evaluation this function relies in the library 'pathos':
  https://github.com/uqfoundation/pathos.

  Args:
    input_data: List of folders containing the batch sim data or string with
        HDF5 file containing the data.
    output_folder: Folder where to output the data.
    process_inputs: Boolean indicating whether to process input data.
    allow_different_commits: Boolean indicating wether data from different
            commits is allowed.
    score_thr: Float indicating the score threshold for probability
        computation.
    use_concurrency: Boolean indicating whether to use concurrency to
        speed up the computation.
    ncores: Integer with the number of cores to use for concurrent
        computation. If None, the number of cores is 80% of the
        available cores.
  """
  if not isinstance(input_data, (list, tuple)) and os.path.isfile(input_data):
    # Input data is a data file.
    simdata = SimData(input_data)

  else:
    # Create database from input files.
    db_fn = os.path.join(output_folder, 'db.h5')
    bsdi = BatchSimDataImporter(input_data, process_inputs,
                                allow_different_commits)
    bsdi.create_database(db_fn)

    # Create simdata object
    print('Creating simulation data object...')
    simdata = SimData(db_fn)
    print('...done.')

  # For each score, compute probabiltiy of being above 1
  print('Computing probabilities...')
  score_table_combs = list(itertools.product(range(simdata.num_scores),
                                             range(simdata.num_tables)))

  if use_concurrency:
    try:
      pathos = importlib.import_module('pathos.multiprocessing')
    except ImportError:
      print('Pathos library not installed. '
            'Get it here https://github.com/uqfoundation/pathos.')
      use_concurrency = False

  if use_concurrency:
    cpucount = pathos.cpu_count()
    if ncores is None:
      ncpus = int(np.floor(cpucount*0.8))
    else:
      assert ncores >= 1 and ncores <= cpucount, (
          'ncores must be between 1 and {0}.'.format(cpucount))
      ncpus = int(ncores)

    print('Using {0} cores (out of {1}).'.format(ncpus, cpucount))
    pool = pathos.ProcessingPool(ncpus)
    try:
      # I haven't found the way to check the pool status;
      # need to restart if the pool is still active.
      pool.restart()
    except AssertionError:
      pass

  else:
    print('Running without concurrency.')
    ncpus = 1

  processed_prob_list = []
  report_target = 0.1

  if ncpus > 1:
    # imap returns ordered data
    for prob in pool.imap(functools.partial(_get_prob,
                                            simdata,
                                            score_thr),
                          score_table_combs):
      processed_prob_list.append(prob)
      if len(processed_prob_list)/len(score_table_combs) > report_target:
        print('  {0:.0f}% completed.'.format(100.*report_target))
        report_target += 0.1

    pool.close()
    pool.join()
    pool.terminate()

  else:
    for score_table_comb in score_table_combs:
      processed_prob_list.append(_get_prob(simdata,
                                           score_thr,
                                           score_table_comb))
      if len(processed_prob_list)/len(score_table_combs) > report_target:
        print('  {0:.0f}% completed.'.format(100.*report_target))
        report_target += 0.1

  print('...done.')

  # Write CSVs
  shear_exponent_labels = ['Shear exponent 0',
                           'Shear exponent 0.1',
                           'Shear exponent 0.2']
  wind_speeds_label = ',Wind speed [m/s],1,3,5,7,9,11,13,15'

  mean_output_fn = 'prob_mean.csv'
  low_output_fn = 'prob_low.csv'
  high_output_fn = 'prob_high.csv'

  print('Saving mean probability data to file '
        '{0}'.format(os.path.join(output_folder, mean_output_fn)))
  print('Saving probability lower bound data to file '
        '{0}'.format(os.path.join(output_folder, low_output_fn)))
  print('Saving probability higher bound data to file '
        '{0}'.format(os.path.join(output_folder, high_output_fn)))

  for file_idx, fn in enumerate([mean_output_fn,
                                 low_output_fn,
                                 high_output_fn]):
    fp = open(os.path.join(output_folder, fn), 'w')

    idx = 0
    for score_idx, score in enumerate(simdata.scorelist):
      if simdata.get_score_info(score)['experimental']:
        fp.write('{0},{1} (Experimental)\n'.format(score_idx,
                                                   score.replace(',', '')))
      else:
        fp.write('{0},{1}\n'.format(score_idx, score.replace(',', '')))
      fp.write('{0}\n'.format(wind_speeds_label))
      table_idx = 0
      for row in range(3):
        fp.write(',{0},'.format(shear_exponent_labels[row]))
        for _ in range(8):
          prob = processed_prob_list[idx][file_idx]
          fp.write('{0},'.format(prob))

          table_idx += 1
          idx += 1
        fp.write('\n')
      fp.write('\n')

    fp.close()

  print('Done.')


def plot_score_prob(simdata, scorelist, wind_shear, score_thr=0.999,
                    plotbounds=True, draw_legend=True, plot_score_index=False):
  """Plots score probabilities as a function of wind speed.

  Processes the batch sims in a list of folders, and returns a figure with
  the probabilities of scores above a threshold as a function of wind speed.

  Args:
    simdata: SimData object containing the data to process.
    scorelist: List of strings or integers (indexes) for the scores to
        plot.
    wind_shear: Value in [0, 0.1, 0.2] identifying the wind shear to plot.
    score_thr: Float indicating the score threshold for probability
        computation.
    plotbounds: Flag to indicate whether to plot the confidence bounds.
    draw_legend: Flag to draw the lengend.
    plot_score_index: Flag to write the score index along the probability lines.

  Returns:
    matplotlib figure.
  """
  assert wind_shear in [0, 0.1, 0.2]

  # Check scores and report to user if not found or wrong
  scorenames = []
  for score in scorelist:
    scorename = simdata.find_score(score)
    if scorename is None:
      print('Score: {0} not found. Try again'.format(score))
      return
    scorenames.append(scorename)

  wind_speeds = np.arange(1, 17, 2)

  print('Computing probabilities...')
  for score in scorenames:
    scorevalues = np.empty((3, len(wind_speeds)))
    for idx, wind_speed in enumerate(wind_speeds):
      tablename = ('[m] AGL = {0},  '
                   'shear exponent = {1}'.format(wind_speed, wind_shear))
      table = simdata.find_table(tablename, verbose=False)
      vardata = simdata.get_score_data(score, table, exact=False, verbose=False)
      prob = vardata.prob_above(score_thr)
      scorevalues[0, idx] = prob.mean
      scorevalues[1, idx] = prob.bounds[0]
      scorevalues[2, idx] = prob.bounds[1]

    p = plt.plot(wind_speeds, scorevalues[0, :], label=score)
    if plot_score_index:
      score_index = simdata.get_score_info(score)['index']
      p[0].set_label(str(score_index) + ': ' + score)
      for i in range(len(wind_speeds)):
        plt.text(wind_speeds[i], scorevalues[0, i], str(score_index))
    if plotbounds:
      plt.plot(wind_speeds, scorevalues[1, :], '--', color=p[-1].get_color())
      plt.plot(wind_speeds, scorevalues[2, :], '--', color=p[-1].get_color())

  print('...done')

  plt.grid()
  if draw_legend:
    plt.legend()
  plt.xlabel('Wind Speed, m/s', fontsize=16)
  plt.ylabel('Probability of unacceptable score', fontsize=16)
  plt.xticks(wind_speeds)
  plt.ylim(-0.01, 1.01)
  plt.title('Wind shear exponent = {0}'.format(wind_shear))

  return plt.gcf()
