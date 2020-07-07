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

"""Batch sim comparison module.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import functools
import importlib

from makani.analysis.util.simdata_analysis import risk
import numpy as np
from scipy import stats
from scipy.spatial.distance import euclidean

CSV_DELIMITER = ';'


def get_score_comparison(prior_score, post_score, score_thr=0.999,
                         p_val_thr=0.05):
  """Compare two scores.

  Args:
    prior_score: VariableData object with the prior score.
    post_score: VariableData object with the posterior score.
    score_thr: Score threshold at which to evaluate the probabilities.
    p_val_thr: P-value threshold.

  Returns:
    Dictionary with the comparison result.
  """
  ks, p_val = stats.ks_2samp(prior_score.data, post_score.data)
  test_result = p_val < p_val_thr

  # Get prob above threshold
  prior_prob = prior_score.prob_above(score_thr).mean
  post_prob = post_score.prob_above(score_thr).mean

  # Risk evaluation.
  # Severity takes values in 1..5 (5 levels, see risk.py).
  # Here sometimes the severity takes a value of 0 to indicate no
  # severity at all, going against the standard. The idea is that if
  # a score has no severity, then it should not be a score.
  # We translate a severity of 0 into the standard lowest severity
  # level of 1.
  prior_severity = prior_score.variable_info['severity']
  if prior_severity == 0:
    prior_severity = 1
  post_severity = post_score.variable_info['severity']
  if post_severity == 0:
    post_severity = 1
  prior_risk_rank = risk.evaluate_risk(prior_prob, prior_severity)['rank']
  post_risk_rank = risk.evaluate_risk(post_prob, post_severity)['rank']

  # Append data
  return {'prior_table_idx': prior_score.table_info['index'],
          'post_table_idx': post_score.table_info['index'],
          'prior_score_idx': prior_score.variable_info['index'],
          'post_score_idx': post_score.variable_info['index'],
          'prior_score': prior_score.variable_info['name'],
          'post_score': post_score.variable_info['name'],
          'prior_table': prior_score.table_info['title'],
          'post_table': post_score.table_info['title'],
          'ks': ks,
          'p_val': p_val,
          'test_result': test_result,
          'prior_prob': prior_prob,
          'post_prob': post_prob,
          'delta_prob': post_prob - prior_prob,
          'prior_severity': prior_score.variable_info['severity'],
          'post_severity': post_score.variable_info['severity'],
          'prior_risk_rank': prior_risk_rank,
          'post_risk_rank': post_risk_rank,
          'delta_rank': post_risk_rank - prior_risk_rank}


def get_simdata_comparison(prior_simdata, post_simdata, table, score_thr=0.999,
                           p_val_thr=0.05):
  """Compares all scores in a table.

  Args:
    prior_simdata: SimData object with the prior data.
    post_simdata: SimData object with the posterior data.
    table: Table from which to evaluate the scores.
    score_thr: Score threshold at which to evaluate the probabilities.
    p_val_thr: P-value threshold.

  Returns:
    List of scores that are statistically different, based on the P-value.
  """
  table_dict = post_simdata.get_table_info(table, exact=True)
  if table_dict is None:
    return []

  table = table_dict['title']
  prior_score_dict = prior_simdata.get_all_score_data(table, exact=True)
  post_score_dict = post_simdata.get_all_score_data(table, exact=True)

  this_table_list = []
  for score in prior_simdata.get_score_list():
    if post_simdata.find_score(score, exact=True) is None:
      continue

    prior_score = prior_score_dict[score]
    post_score = post_score_dict[score]
    comparison = get_score_comparison(prior_score, post_score,
                                      score_thr=score_thr, p_val_thr=p_val_thr)
    if comparison['test_result']:
      # Append data
      this_table_list.append(comparison)

  return this_table_list


def compare_simulation_data(prior_simdata, post_simdata, output_file,
                            score_thr=0.999, p_val_thr=0.05,
                            use_concurrency=False, ncores=None):
  """Compares the data in two SimData objects and outputs data in CSV format.

  For concurrent evaluation this function relies in the library 'pathos':
  https://github.com/uqfoundation/pathos

  Args:
      prior_simdata: SimData object with prior (before) data.
      post_simdata: SimData object with post (after) data.
      output_file: Name of the CSV file where to output the data.
      score_thr: Float indicating the score threshold for probability
          commputation.
      p_val_thr: P-value threshold for Kolmogorov-Smirnov test (defaults to 5%).
      use_concurrency: Boolean indicating whether to use concurrency to
          speed up the computation.
      ncores: Integer with the number of cores to use for concurrent
          computation. If None, the number of cores is 80% of the
          available cores.
  """
  if use_concurrency:
    try:
      pathos = importlib.import_module('pathos.multiprocessing')
    except ImportError:
      print('Pathos library not installed. Get it here:')
      print('  https://github.com/uqfoundation/pathos.')
      use_concurrency = False

  if use_concurrency:
    cpucount = pathos.cpu_count()
    if ncores is None:
      ncpus = int(np.floor(cpucount*0.8))
    else:
      if not (ncores >= 1 and ncores <= cpucount):
        print('ncores must be between 1 and {0}.'.format(cpucount))
        return
      ncpus = int(ncores)

    print('Using {0} cores (out of {1}).'.format(ncpus, cpucount))
    pool = pathos.ProcessingPool(ncpus)
    try:
      pool.restart()  # Need to restart if the pool is still active.
    except AssertionError:
      pass

  else:
    print('Running without concurrency.')
    ncpus = 1

  ntables = prior_simdata.num_tables
  tablelist = prior_simdata.get_table_list()

  print('Comparing simulations...')
  result_list = []
  report_target = 0.1

  if ncpus > 1:
    # imap() returns data ordered.
    for table_comp in pool.imap(functools.partial(get_simdata_comparison,
                                                  prior_simdata,
                                                  post_simdata,
                                                  score_thr=score_thr,
                                                  p_val_thr=p_val_thr),
                                tablelist):
      result_list.append(table_comp)
      if len(result_list)/ntables > report_target:
        print('  {0:.0f}% completed.'.format(100.*report_target))
        report_target += 0.1

      pool.close()
      pool.join()
      pool.terminate()

  else:
    for table in tablelist:
      result_list.append(get_simdata_comparison(prior_simdata,
                                                post_simdata,
                                                table,
                                                score_thr,
                                                p_val_thr))
      if len(result_list)/ntables > report_target:
        print('  {0:.0f}% completed.'.format(100.*report_target))
        report_target += 0.1

  print('...done.')

    # Go over results and save to CSV
  columns_to_save = ['prior_score',
                     'prior_prob',
                     'post_prob',
                     'delta_prob',
                     'ks',
                     'p_val',
                     'prior_severity',
                     'post_severity',
                     'prior_risk_rank',
                     'post_risk_rank',
                     'delta_rank']

  print('Writing file {0}...'.format(output_file))
  with open(output_file, 'w') as fp:
    for ntable, table in enumerate(tablelist):
      score_comp_list = result_list[ntable]

      fp.write('Table #{0}: {1}\n\n'.format(ntable, table))
      if not score_comp_list:
        continue
      for col in columns_to_save:
        fp.write(CSV_DELIMITER + '{0}'.format(col))
      fp.write('\n')

      for score_comp in score_comp_list:
        for col in columns_to_save:
          fp.write(CSV_DELIMITER + '{0}'.format(score_comp[col]))
        fp.write('\n')
      fp.write('\n')
  print('...done.')


def find_closest_sample(simdata, table, datapoint, dist=None, weights=None,
                        **dist_kws):
  """Finds the closest sample in simdata to a datapoint.

  Args:
      simdata: SimData object.
      table: Table from which to evaluate the scores.
      datapoint: Dictionary with scores as keys, and score values as entries.
      dist: Distance function to use from scipy.spatial.distance. If None, the
          Euclidean distance is used.
      weights: Array of weights to be used in distance computation.
      **dist_kws: Additional keyword arguments for the distance function.

  Returns:
      Dictionary with information identifying the sample closest to datapont.
      Keywords:
          min_dist: Minimum distance.
          min_dist_idx: Index of the sample with the minimum distance.
          min_dist_job_id: Job_id of the sample with the minimum distance.
          min_dist_file: File containing the sample with the minimum distance.
  """
  if dist is None:
    dist = euclidean

  # Build the datapoint
  score_indexes = []
  datapoint_array = []
  weights_array = np.array([])
  for i, score_name in enumerate(datapoint):
    if (simdata.find_score(score_name, exact=False, verbose=True) is None or
        np.isnan(datapoint[score_name])):
      print('Skipping score {0}'.format(score_name))
      continue
    score_indexes.append(simdata.get_score_info(score_name,
                                                verbose=False)['index'])
    datapoint_array.append(datapoint[score_name])
    if weights is not None:
      weights_array = np.append(weights_array, weights[i])

  datapoint_array = np.array(datapoint_array)

  # Get the all distances to the datapoint
  table_df = simdata.get_all_table_data(table)
  crash_col = 'score_{0}'.format(simdata.get_score_info(
      'Crash indicator')['index'])
  all_scores = table_df.drop(['job_id', 'folder', crash_col],
                             axis=1).as_matrix()
  dist_store = np.array([])
  for sample in all_scores:
    sample_reduced = sample[score_indexes]
    non_nan_idxs = np.argwhere(~np.isnan(sample_reduced))
    if weights_array.size > 0:
      weights_filtered = weights_array[non_nan_idxs]
    else:
      weights_filtered = None

    distance = dist(sample_reduced[non_nan_idxs],
                    datapoint_array[non_nan_idxs],
                    w=weights_filtered,
                    **dist_kws)
    dist_store = np.append(dist_store, distance)

  # Populate output dictionary
  min_dist = np.min(dist_store)
  min_dist_idx = np.argmin(dist_store)
  min_dist_job_id = table_df.iloc[min_dist_idx]['job_id']
  min_dist_folder = table_df.iloc[min_dist_idx]['folder']

  return {'min_dist': min_dist,
          'min_dist_idx': min_dist_idx,
          'min_dist_job_id': min_dist_job_id,
          'min_dist_folder': min_dist_folder}
