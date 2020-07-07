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

"""Risk definition.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import pandas as pd


#         Severity    1   2   3   4   5
_RISK_TABLE_RANKS = [[18, 9, 5, 2, 1],  # Likelihood 5
                     [21, 12, 7, 4, 3],  # Likelihood 4
                     [22, 14, 11, 8, 6],  # Likelihood 3
                     [24, 19, 15, 13, 10],  # Likelihood 2
                     [25, 23, 20, 17, 16]]  # Likelihood 1

_PROB_THRESHOLDS = [0.10, 0.25, 0.50, 0.70]


def evaluate_risk(probability, severity):
  """Evaluates the risk of a combination of probability and severity.

  Args:
    probability: Event probability.
    severity: Event severity as integer from 1 (least severe)
        to 5 (most severe).

  Returns:
    Dictionary with the fields:
      - row: Corresponding row in the risk matrix.
      - col: Corresponding column in the risk matrix.
      - likelihood: Likelihood of the event as integer from 1 (least likely)
            to 5 (most likely). Based on the probability thesholds
            defined in _PROB_THRESHOLDS.
      - rank: Risk rank as integer from 1 (highest risk) to 25 (lowest risk),
            based on the rank definition in _RISK_TABLE_RANKS.
  """
  assert probability >= .0 and probability <= 1.
  assert int(severity) in [1, 2, 3, 4, 5]

  if probability < _PROB_THRESHOLDS[0]:
    likelihood = 1
  elif probability < _PROB_THRESHOLDS[1]:
    likelihood = 2
  elif probability < _PROB_THRESHOLDS[2]:
    likelihood = 3
  elif probability < _PROB_THRESHOLDS[3]:
    likelihood = 4
  else:
    likelihood = 5

  row = 5 - likelihood
  col = int(severity) - 1

  return {'row': row,
          'col': col,
          'likelihood': likelihood,
          'rank': _RISK_TABLE_RANKS[row][col]}


def get_risk_table(simdata, table, score_thr=0.999,
                   skip_experimental_scores=True):
  """Returns string for risk table output in CSV format.

  Args:
    simdata: SimData object containing the data.
    table: Table identifier (integer or string).
    score_thr: Score threshold at which to evaluate the probabilities.
    skip_experimental_scores: Flag to skip experimental scores.

  Returns:
    risk_table: Numpy 2D array with the risk table.
    ranked_score_risk_df: Dataframe with the score and risk data, sorted by
        their risk rank.
  """
  table_info = simdata.get_table_info(table)
  if table_info is None:
    return None, None

  risk_table = np.zeros((5, 5))
  scores = simdata.get_all_score_data(table, verbose=False)
  ranked_scores = []
  for score_name in simdata.get_score_list():
    vardata = scores[score_name]
    if skip_experimental_scores and vardata.variable_info['experimental']:
      continue
    prob = vardata.prob_above(score_thr).mean
    severity = vardata.variable_info['severity']
    if severity == 0:
      severity = 1  # A severity of 0 is captured under a severity of 1.
    risk = evaluate_risk(prob, severity)

    ranked_scores.append((score_name, risk['likelihood'], severity,
                          risk['rank']))
    risk_table[risk['row'], risk['col']] += 1

  ranked_score_risk_df = pd.DataFrame(ranked_scores,
                                      columns=['score_name', 'likelihood',
                                               'severity', 'rank'])
  return risk_table, ranked_score_risk_df.sort_values(by='rank')


def get_risk_table_str(risk_table, simdata, table):
  """Returns string for risk table output in CSV format.

  Args:
    risk_table: Numpy 2D array with the risk table, conveniently returned by
        risk.get_risk_table().
    simdata: SimData object containing the data.
    table: Table identifier (integer or string).

  Returns:
    CSV-formatted string with the risk table.
  """
  table_info = simdata.get_table_info(table)
  if table_info is None:
    return None

  comma_less_title = table_info['title'].replace(',', '')
  out_str = 'Table #{0}: {1}\n\n'.format(table_info['index'],
                                         comma_less_title)
  out_str += ',Likelihood,,,,,,Number of scores\n'

  for i in range(5):
    out_str += ',{0}'.format(5 - i)
    for j in range(5):
      out_str += ',{0}'.format(int(risk_table[i, j]))
    out_str += '\n'
  out_str += ',,1,2,3,4,5,Severity\n\n'

  return out_str


def save_risk_tables_csv(filename, simdata, table_list, score_thr=0.999,
                         skip_experimental_scores=True, score_list_max_rank=6):
  """Saves risk tables in a CSV file.

  Args:
    filename: Name for the output CSV file.
    simdata: SimData object containing the data.
    table_list: List of table identifiers (integer indexes or strings).
    score_thr: Score threshold at which to evaluate the probabilities.
    skip_experimental_scores: Flag to skip experimental scores.
    score_list_max_rank: Maximum rank of the itemized score list.
  """
  f = open(filename, 'w')
  ntables = len(table_list)
  for i, table in enumerate(table_list):
    print('Processing table {0} of {1}...'.format(i+1, ntables))
    risk_table, ranked_score_risk_df = get_risk_table(simdata, table,
                                                      score_thr,
                                                      skip_experimental_scores)
    table_str = get_risk_table_str(risk_table, simdata, table)
    f.write(table_str)

    top_risk_scores = ranked_score_risk_df[
        ranked_score_risk_df['rank'] <= score_list_max_rank]
    if not top_risk_scores.empty:
      f.write('score,likelihood,severity,rank\n')
      for top_score in top_risk_scores.iterrows():
        score_name = top_score[1]['score_name'].replace(',', '')
        f.write('{0},{1},{2},{3}\n'.format(score_name,
                                           top_score[1]['likelihood'],
                                           top_score[1]['severity'],
                                           top_score[1]['rank']))
    f.write('\n\n')
  f.close()
  print('Done.')
