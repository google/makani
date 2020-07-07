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

"""Utilities for analyzing batch sim results."""

import numpy as np


def IsSameSeed(data_dict_a, data_dict_b):
  return data_dict_a['parameter_seed'] == data_dict_b['parameter_seed']


def IsSameBatch(data_dict_a, data_dict_b):
  return data_dict_a['title'] == data_dict_b['title']


def GetScoringFunctionLabels(data_dict):
  score_labels = []
  for metric in data_dict['metrics']:
    score_labels.append(metric['name'])
  return score_labels


def GetScoreMetrics(data_dict, score_labels):
  all_score_labels = GetScoringFunctionLabels(data_dict)
  metrics = []
  for label in score_labels:
    label_idx = all_score_labels.index(label)
    metrics.append(data_dict['metrics'][label_idx])
  return metrics


def GetWindData(data_dict):
  """Get wind speeds and shear exponents contained in the data."""

  wind_speeds = []
  wind_shears = []
  for ii in range(GetGridInfo(data_dict=data_dict, only_return='num_blocks')):
    title = data_dict['table_data'][ii]['title']
    wind_speeds.append(float(title.split(',')[0].split('=')[-1]))
    wind_shears.append(float(title.split(',')[1].split('=')[-1]))
  return {
      'speeds': np.unique(wind_speeds),
      'shears': np.unique(wind_shears)
  }


def GetMatchedWindData(data_dict_a, data_dict_b):
  """Get wind speeds and shears common in both batches."""

  wind_data_a = GetWindData(data_dict=data_dict_a)
  wind_speeds_a = wind_data_a['speeds'].tolist()
  wind_shears_a = wind_data_a['shears'].tolist()

  wind_data_b = GetWindData(data_dict=data_dict_b)
  wind_speeds_b = wind_data_b['speeds'].tolist()
  wind_shears_b = wind_data_b['shears'].tolist()

  wind_speeds = list(set(wind_speeds_a).intersection(wind_speeds_b))
  wind_shears = list(set(wind_shears_a).intersection(wind_shears_b))
  return {
      'speeds': sorted(wind_speeds),
      'shears': sorted(wind_shears),
  }


def GetGridInfo(data_dict, only_return=''):
  """Returns how grids are arranged in display.

  Args:
    data_dict: overview_data.json read in to a dictionary.
    only_return: if only one info field is requested.
                 'num_blocks', 'num_major_cols','num_major_rows',
                 'num_minor_cols', 'num_minor_rows',
                 'num_sims_per_block'
                 Default is all of these in a dictionary.

  Returns:
    grid_info: [dict] with fields
                'num_blocks', 'num_major_cols','num_major_rows',
                'num_minor_cols', 'num_minor_rows', 'num_sims_per_block'
  """

  grid_info = {
      'num_blocks': len(data_dict['table_data']),
      'num_major_cols': data_dict['num_major_cols'],
      'num_minor_cols': data_dict['table_data'][0]['num_cols'],
      'num_minor_rows': data_dict['table_data'][0]['num_rows'],
  }
  grid_info['num_major_rows'] = (grid_info['num_blocks'] /
                                 grid_info['num_major_cols'])
  grid_info['num_sims_per_block'] = (grid_info['num_minor_rows'] *
                                     grid_info['num_minor_cols'])
  if only_return:
    return grid_info[only_return]
  else:
    return grid_info


def NormalizeScores(scores):
  """Normalize scores into a scale of 0 to 999."""

  norm_scores = []
  for score in scores:
    if score != 'None':
      if score < 0.0:
        norm_score = 0.0
      elif score*100.0 > 999.0:
        norm_score = 999.0
      else:
        norm_score = score*100.0
    else:
      norm_score = score

    norm_scores.append(norm_score)

  return norm_scores


def GetJobScores(data_dict, job_id):
  """Returns scores corresponding to requested job from table data."""

  num_sims_per_block = GetGridInfo(data_dict, only_return='num_sims_per_block')
  block_id, job_id = np.divmod(job_id, num_sims_per_block)
  sim_success = (data_dict['table_data'][block_id]['job_data'][job_id]
                 ['sim_success'])
  if sim_success:
    scores = data_dict['table_data'][block_id]['job_data'][job_id]['scores']
  else:
    # Treat simulation failure as all bad scores.
    scores = [999.0] * len(data_dict['metrics'])

  return NormalizeScores(scores)


def GetBlockId(data_dict, wind_speed, wind_shear):
  """Returns block corresponding to requested wind speed and shear."""

  wind_data = GetWindData(data_dict=data_dict)
  wind_speed_idx = int(np.interp(wind_speed, wind_data['speeds'],
                                 range(len(wind_data['speeds']))))
  wind_shear_idx = int(np.interp(wind_shear, wind_data['shears'],
                                 range(len(wind_data['shears']))))
  num_major_cols = GetGridInfo(data_dict=data_dict,
                               only_return='num_major_cols')

  return wind_speed_idx + (wind_shear_idx * num_major_cols)


def CollectBlockScores(data_dict, block_id):
  """Returns all scores in a block for a given batch."""

  num_sims_per_block = GetGridInfo(data_dict=data_dict,
                                   only_return='num_sims_per_block')
  job_ids = range(block_id * num_sims_per_block,
                  (block_id + 1) * num_sims_per_block)

  all_scores = []
  for job_id in job_ids:
    all_scores.append(GetJobScores(data_dict=data_dict, job_id=job_id))

  return all_scores


def GetComparisonData(data_dict_a, data_dict_b):
  """Returns dict with batch comparisons."""

  # Comparison is only valid if batches were run with same parameter seeds.
  assert IsSameSeed(data_dict_a, data_dict_b)

  # Find common wind speeds and shears in both batches.
  wind_data = GetMatchedWindData(data_dict_a, data_dict_b)
  wind_speeds = wind_data['speeds']
  wind_shears = wind_data['shears']
  num_speeds = len(wind_speeds)
  num_shears = len(wind_shears)

  # Find common score labels in both batches.
  score_labels_a = GetScoringFunctionLabels(data_dict=data_dict_a)
  score_labels_b = GetScoringFunctionLabels(data_dict=data_dict_b)
  score_labels = list(set(score_labels_a).intersection(score_labels_b))
  score_labels = sorted(score_labels)
  num_scores = len(score_labels)

  metrics = GetScoreMetrics(data_dict=data_dict_b, score_labels=score_labels)

  # Assert blocks in both batches have same number of simulations.
  # A block of simulations in crosswind monte carlo sweep is a wind
  # speed - shear combination.
  num_sims_per_block_a = GetGridInfo(data_dict=data_dict_a,
                                     only_return='num_sims_per_block')
  num_sims_per_block_b = GetGridInfo(data_dict=data_dict_b,
                                     only_return='num_sims_per_block')
  assert num_sims_per_block_a == num_sims_per_block_b
  num_sims_per_block = num_sims_per_block_a

  # Compute comparison data.
  num_bins = 11
  if IsSameBatch(data_dict_a, data_dict_b):
    data_bins = np.linspace(0, 200, num_bins)
  else:
    data_bins = np.linspace(-110, 110, num_bins + 1)
  all_hists = []
  for ii in range(num_shears):
    wind_shear = wind_shears[ii]
    wind_shear_hists = []
    for jj in range(num_speeds):
      wind_speed = wind_speeds[jj]

      block_id_a = GetBlockId(data_dict=data_dict_a, wind_shear=wind_shear,
                              wind_speed=wind_speed)
      scores_a = CollectBlockScores(data_dict=data_dict_a, block_id=block_id_a)
      block_id_b = GetBlockId(data_dict=data_dict_b, wind_shear=wind_shear,
                              wind_speed=wind_speed)
      scores_b = CollectBlockScores(data_dict=data_dict_b, block_id=block_id_b)

      # Initialize block histograms as a 2D array of zeros.
      block_hists = []
      for kk in range(num_scores):
        score_label = score_labels[kk]
        # If both batches don't have identical score labels, they could be in
        # different order. Get index from individual scores lists.
        score_idx_a = score_labels_a.index(score_label)
        score_idx_b = score_labels_b.index(score_label)

        data_a = np.array(scores_a)[:, score_idx_a]
        data_b = np.array(scores_b)[:, score_idx_b]
        if IsSameBatch(data_dict_a, data_dict_b):
          hist, _ = np.histogram(data_b, bins=data_bins)
        else:
          hist, _ = np.histogram(data_b - data_a, bins=data_bins)
        hist = 100.0 / num_sims_per_block * hist
        block_hists.append(hist.tolist())

      wind_shear_hists.append(block_hists)
    all_hists.append(wind_shear_hists)

  return {
      'title': 'Batch Simulations Comparison',
      'batch_a': data_dict_a['title'],
      'batch_b': data_dict_b['title'],
      'wind_speeds': wind_speeds,
      'wind_shears': wind_shears,
      'score_labels': score_labels,
      'metrics': metrics,
      'hist_edges': data_bins.tolist(),
      'hist_values': all_hists,
      'num_samples': num_sims_per_block,
  }


def WriteSummary(data_dict_a, data_dict_b, threshold, output_file):
  """Returns batch comparisons summary based on threshold."""

  def _ApplyThreshold(data_dict, threshold):
    """Read scores from raw data file."""
    metric_names = [m['name'] for m in data_dict['metrics']]
    scores = []
    for job_block in data_dict['table_data']:
      for job in job_block['job_data']:
        if job['scores']:
          scores.append(job['scores'])
    scores = np.array(scores, dtype=np.float)
    scores[scores >= threshold] = 1.0
    scores[scores < threshold] = 0.0
    total_scores = np.nansum(scores, axis=0).tolist()
    return dict(zip(metric_names, total_scores))

  def _Record(summary, name, value1, value2):
    """Record values to compare."""
    try:
      value1 = float(value1)
      value2 = float(value2)
      benefit = value1 - value2
    except ValueError:
      pass

    summary[name] = {
        'A': value1,
        'B': value2,
        'benefit': benefit
    }

  def _CompareJson(summary, data1, data2, root_path):
    """Compare two structures with scores."""
    if isinstance(data1, dict):
      assert isinstance(data2, dict)
      prefix = root_path + '.' if root_path else ''
      keys1 = set(data1.keys())
      keys2 = set(data2.keys())
      common_keys = keys1 & keys2
      for key in common_keys:
        value1 = data1[key]
        value2 = data2[key]
        _CompareJson(summary, value1, value2, prefix + key)
    elif isinstance(data1, list):
      assert isinstance(data2, list)
      for i in range(len(data1)):
        item1 = data1[i]
        if i >= len(data2):
          break
        item2 = data2[i]
        _CompareJson(summary, item1, item2, root_path + '[%d]' % i)
    else:
      _Record(summary, root_path, data1, data2)

  def _WriteDiff(name, value1, value2, output_file):
    """Write the summary of differences to output_file."""
    if value1 == value2:
      ratio = 0
    elif value1 == 0:
      ratio = float('nan')
    else:
      ratio = (value2 - value1)/value1 * 100.0
    diff = '%4d --> %4d (%+6.1f%%) ' % (value1, value2, ratio)

    with open(output_file, 'a') as f:
      f.write('%s [%s]\n' % (diff, name))

  summary = {}
  scores_a = _ApplyThreshold(data_dict_a, threshold)
  scores_b = _ApplyThreshold(data_dict_b, threshold)

  _CompareJson(summary, scores_a, scores_b, '')
  sorted_keys = sorted(summary, key=lambda k: summary[k]['benefit'],
                       reverse=True)

  with open(output_file, 'w') as f:
    f.write('Summary of batch sims comparison.\n')

  for k in sorted_keys:
    _WriteDiff(k, summary[k]['A'], summary[k]['B'], output_file)
