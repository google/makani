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

"""Batch sim data class.

"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import difflib
import os

from makani.analysis.util.simdata_analysis import vardata
import pandas as pd


class SimData(object):
  """Encapsulates simulation data.
  """

  def __init__(self, hdf_fn, verbose=True):
    """Initializes the object.

    The HDF5 file must be created by using the BatchSimDataImporter class.

    Args:
      hdf_fn: String with the filename of the HDF5 file containing the
          simulation data.
      verbose: Boolean describing whether to show terminal print-outs.
    """
    if not os.path.isfile(hdf_fn):
      print("File '{0}' does not exist.".format(hdf_fn))
      self.hdf_fn = None
      return None

    if verbose:
      print('Loading file {0}'.format(hdf_fn))
    self.hdf_fn = hdf_fn
    self.scoredata = pd.read_hdf(self.hdf_fn, 'scoredata')
    self.tabledata = pd.read_hdf(self.hdf_fn, 'tabledata')
    self.scorelist = self.scoredata['name'].tolist()
    self.num_scores = len(self.scorelist)
    self.tablelist = self.tabledata['title'].tolist()
    self.num_tables = len(self.tablelist)
    with pd.HDFStore(self.hdf_fn, 'r') as hdf:
      if 'inputs' in hdf:
        self.inputlist = [s.split('/inputs/')[1]
                          for s in hdf if s.startswith('/inputs/')]
      else:
        self.inputlist = []
    self.num_inputs = len(self.inputlist)
    if verbose:
      print('Done.')

  def get_metadata(self):
    """Returns a dictionary with the metadata.
    """
    metadata = pd.read_hdf(self.hdf_fn, 'metadata').to_dict('list')
    for key in metadata.keys():
      metadata[key] = metadata[key][0]
    return metadata

  def get_score_list(self):
    """Returns a list with all the score names.
    """
    return self.scorelist

  def get_table_list(self):
    """Returns a list with all the table titles.
    """
    return self.tablelist

  def find_item_in_list(self, item, item_list, item_category='', exact=False,
                        verbose=True):
    """Returns the closest score to score_name.

    Args:
      item: String to search.
      item_list: List where to look for the item.
      item_category: String to categorize item in print messages.
      exact: Flag to indicate whether to find the score using
             exact string comparison.
      verbose: Flag to indicate whether to display information to stdout.
    """
    if item in item_list:
      return item
    elif not exact:
      matches = difflib.get_close_matches(item, item_list, n=5, cutoff=0.2)
      if matches:
        if verbose:
          print("{0} '{1}' has not been found. "
                "These are the closest:".format(item_category.capitalize(),
                                                item))
          for i, candidate in enumerate(matches):
            print("  {0}: '{1}'".format(i+1, candidate))
          print("Using '{0}'.".format(matches[0]))
        return matches[0]
      else:
        if verbose:
          print("Could not find a match for {0} '{1}'.".format(item_category,
                                                               item))
        return None
    return None

  def find_score(self, score_name, exact=False, verbose=True):
    """Returns the closest score to score_name.

    Args:
      score_name: String with the score-like name to search.
      exact: Flag to indicate whether to find the score using
             exact string comparison.
      verbose: Flag to indicate whether to display information to stdout.
    """
    score_list = self.get_score_list()
    return self.find_item_in_list(score_name, score_list, 'score', exact,
                                  verbose)

  def get_score_info(self, score, exact=False, verbose=True):
    """Returns dictionary with the score information.

    Args:
      score: String with the score name or integer indicating index.
      exact: Flag to indicate whether to find the score using
             exact string comparison.
      verbose: Flag to indicate whether to display information to stdout.
    """
    if isinstance(score, int):
      if score >= 0 and score < self.num_scores:
        score_name = self.get_score_list()[score]
      else:
        if verbose:
          print('Score index {0} not found.'.format(score))
        return None
    else:
      score_name = score

    score_candidate = self.find_score(score_name, exact=exact, verbose=verbose)
    if score_candidate is None:
      return None

    scoredata = self.scoredata[self.scoredata['name'] ==
                               score_candidate].to_dict('list')
    for key in scoredata.keys():
      scoredata[key] = scoredata[key][0]
    scoredata['index'] = self.get_score_list().index(score_candidate)
    return scoredata

  def find_table(self, table_name, exact=False, verbose=True):
    """Returns the closest table to table_name.

    Args:
      table_name: String with the table-like name to search.
      exact: Flag to indicate whether to find the table using
             exact string comparison.
      verbose: Flag to indicate whether to display information to stdout.
    """
    table_list = self.get_table_list()
    return self.find_item_in_list(table_name, table_list, 'table', exact,
                                  verbose)

  def get_table_info(self, table, exact=False, verbose=True):
    """Returns dictionary with the table information.

    Args:
      table: String with the table name or integer indicating index.
      exact: Flag to indicate whether to find the table using
             exact string comparison.
      verbose: Flag to indicate whether to display information to stdout.
    """
    if isinstance(table, int):
      if table >= 0 and table < self.num_tables:
        table_name = self.get_table_list()[table]
      else:
        if verbose:
          print('Table index {0} not found.'.format(table))
        return None
    else:
      table_name = table

    table_candidate = self.find_table(table_name, exact=exact, verbose=verbose)
    if table_candidate is None:
      return None

    tabledata = self.tabledata[self.tabledata['title'] ==
                               table_candidate].to_dict('list')
    for key in tabledata.keys():
      tabledata[key] = tabledata[key][0]
    tabledata['index'] = self.get_table_list().index(table_candidate)
    return tabledata

  def get_input_list(self):
    """Returns a list with all the input names.
    """
    return self.inputlist

  def find_input(self, input_name, exact=False, verbose=True):
    """Returns the closest input to input_name.

    Args:
      input_name: String with the input-like name to search.
      exact: Flag to indicate whether to find the table using
             exact string comparison.
      verbose: Flag to indicate whether to display information to stdout.
    """
    input_list = self.get_input_list()
    return self.find_item_in_list(input_name, input_list, 'input', exact,
                                  verbose)

  def get_input_info(self, input_name, exact=False, verbose=True):
    """Returns dictionary with the input information.

    Args:
      input_name: String with the input name.
      exact: Flag to indicate whether to find the table using
             exact string comparison.
      verbose: Flag to indicate whether to display information to stdout.
    """
    input_candidate = self.find_input(input_name, exact=exact, verbose=verbose)
    if input_candidate is None:
      return None

    input_info = {'name': input_candidate}
    return input_info

  def get_score_data(self, score, table, exact=False, verbose=True):
    """Returns vardata.ScoreData object.

    Args:
      score: String or index identifying the score.
      table: String or index identifying the table.
      exact: Flag to indicate whether to find the score and table using
             exact string comparison.
      verbose: Flag to indicate whether to display information to stdout.
    """
    scoreinfo = self.get_score_info(score, exact=exact, verbose=verbose)
    if scoreinfo is None:
      return None

    tableinfo = self.get_table_info(table, exact=exact, verbose=verbose)
    if tableinfo is None:
      return None

    tabledata = pd.read_hdf(self.hdf_fn,
                            'tables/table_{0}'.format(tableinfo['index']))
    score_str = 'score_{0}'.format(scoreinfo['index'])
    var_df = tabledata[[score_str, 'job_id', 'folder']]
    var_df = var_df.rename(columns={score_str: 'score'})

    return vardata.ScoreData(scoreinfo, tableinfo, var_df)

  def get_all_table_data(self, table, exact=False, verbose=True):
    """Obtains all table data.

    Args:
      table: String or index identifying the table.
      exact: Flag to indicate whether to find the score and table using
             exact string comparison.
      verbose: Flag to indicate whether to display information to stdout.

    Returns:
      DataFrame with the table data.
    """
    tableinfo = self.get_table_info(table, exact=exact, verbose=verbose)
    if tableinfo is None:
      return None

    tabledata = pd.read_hdf(self.hdf_fn,
                            'tables/table_{0}'.format(tableinfo['index']))
    return tabledata

  def get_all_score_data(self, table, exact=False, verbose=True):
    """Obtains all data in a table.

    Returns dictionary with ScoreData objects representing all the
    scores in a table.

    Args:
      table: String or index identifying the table.
      exact: Flag to indicate whether to find the score and table using
             exact string comparison.
      verbose: Flag to indicate whether to display information to stdout.

    Returns:
      Dictionary of ScoreData objects.
    """
    tableinfo = self.get_table_info(table, exact=exact, verbose=verbose)
    if tableinfo is None:
      return None

    tabledata = pd.read_hdf(self.hdf_fn,
                            'tables/table_{0}'.format(tableinfo['index']))

    table_scores = {}
    for score_idx, score in enumerate(self.scorelist):
      scoreinfo = self.get_score_info(score_idx, exact=exact, verbose=verbose)
      score_str = 'score_{0}'.format(score_idx)
      var_df = tabledata[[score_str, 'job_id', 'folder']]
      var_df = var_df.rename(columns={score_str: 'score'})
      table_scores[score] = vardata.ScoreData(scoreinfo, tableinfo, var_df)

    return table_scores

  def get_input_data(self, input_name, table, exact=False, verbose=True):
    """Returns vardata.InputData object.

    Args:
      input_name: String identifying the score.
      table: String or index identifying the table.
      exact: Flag to indicate whether to find the score and table using
             exact string comparison.
      verbose: Flag to indicate whether to display information to stdout.
    """
    input_info = self.get_input_info(input_name, exact=exact, verbose=verbose)
    if input_info is None:
      return None

    table_info = self.get_table_info(table, exact=exact, verbose=verbose)
    if table_info is None:
      return None

    input_df = pd.read_hdf(self.hdf_fn, 'inputs/{0}'.format(input_name))

    # Select the input samples corresponding to this table.
    filtered_input_df = input_df.loc[
        (input_df['job_id'].isin(table_info['job_ids'])) &
        (input_df['folder'].isin(table_info['folders']))]

    # Sort the table so the samples are ordered as indicated by folders and
    # job_ids in table_info.
    # First create a list that contains the concatenated folder name and job_id.
    source_folder_jobid = [
        f + '_' + str(j) for f, j in zip(filtered_input_df['folder'].values,
                                         filtered_input_df['job_id'].values)]

    # Add as a new column to the filtered DataFrame.
    filtered_input_df = filtered_input_df.assign(
        folder_jobid=source_folder_jobid)

    # Sort according to the ordering of folder and job_id in table_info.
    target_folder_jobid = [
        f + '_' + str(j) for f, j in zip(table_info['folders'],
                                         table_info['job_ids'])]
    sorted_input_df = filtered_input_df.set_index('folder_jobid')
    sorted_input_df = sorted_input_df.loc[target_folder_jobid]
    sorted_input_df = sorted_input_df.reset_index(drop=True)

    return vardata.InputData(input_info, table_info, sorted_input_df)
