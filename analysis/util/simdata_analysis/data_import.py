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

"""Class for batch sim data import.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import glob
import json
import os
import tarfile
import warnings

from dateutil import parser
from makani.lib.python import dict_util
import numpy as np
import pandas as pd
import tables


# Ignore warning caused when saving input names to HDF5.
warnings.filterwarnings('ignore', category=tables.NaturalNameWarning)

# Ignore warning caused when saving mixed types to HDF5.
# This warning is in pandas.errors.PerformanceWarning in versions 0.20+.
warnings.filterwarnings(
    'ignore', category=pd.pandas.core.common.PerformanceWarning)


def get_monte_carlo_parameter_list():
  """Returns list of parameters dispersed in Monte Carlo analysis.

  As of now, this list is obtained by processing the command line overrides
  listed in a run in a batch sim, then using a script like the one below and
  finally removing some parameters by hand.

  >> from makani.lib.python import dict_util
  >> param_ranges = {"system": {"flight_plan": 8, "test_site": 2}...<paste_here>
  >> for entry in dict_util.GetAllDictPaths(param_ranges):
  >>   print("'" + '.'.join(entry[:-1]) + "',")

  TODO: Get this list automatically from crosswind_sweeps_client.py.
  """
  return ['sim.random_seed_offset',
          'sim.pitots_sim.0.local_pressure_coeff_offset',
          'sim.pitots_sim.0.pitch_offset',
          'sim.pitots_sim.0.yaw_offset',
          'sim.pitots_sim.1.local_pressure_coeff_offset',
          'sim.pitots_sim.1.pitch_offset',
          'sim.pitots_sim.1.yaw_offset',
          'sim.wing_sim.mass_prop_uncertainties.mass_scale',
          'sim.wing_sim.mass_prop_uncertainties.moment_of_inertia_scale.0',
          'sim.wing_sim.mass_prop_uncertainties.moment_of_inertia_scale.1',
          'sim.wing_sim.mass_prop_uncertainties.moment_of_inertia_scale.2',
          'sim.wing_sim.mass_prop_uncertainties.center_of_mass_offset.0',
          'sim.wing_sim.mass_prop_uncertainties.center_of_mass_offset.1',
          'sim.wing_sim.mass_prop_uncertainties.center_of_mass_offset.2',
          'sim.aero_sim.force_coeff_w_scale_factors.rate_derivatives.p.0',
          'sim.aero_sim.force_coeff_w_scale_factors.rate_derivatives.p.1',
          'sim.aero_sim.force_coeff_w_scale_factors.rate_derivatives.p.2',
          'sim.aero_sim.force_coeff_w_scale_factors.rate_derivatives.q.0',
          'sim.aero_sim.force_coeff_w_scale_factors.rate_derivatives.q.1',
          'sim.aero_sim.force_coeff_w_scale_factors.rate_derivatives.q.2',
          'sim.aero_sim.force_coeff_w_scale_factors.rate_derivatives.r.0',
          'sim.aero_sim.force_coeff_w_scale_factors.rate_derivatives.r.1',
          'sim.aero_sim.force_coeff_w_scale_factors.rate_derivatives.r.2',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.0.0',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.0.1',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.0.2',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.1.0',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.1.1',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.1.2',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.2.0',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.2.1',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.2.2',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.3.0',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.3.1',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.3.2',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.4.0',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.4.1',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.4.2',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.5.0',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.5.1',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.5.2',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.6.0',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.6.1',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.6.2',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.7.0',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.7.1',
          'sim.aero_sim.force_coeff_w_scale_factors.flap_derivatives.7.2',
          'sim.aero_sim.flap_offsets.2',
          'sim.aero_sim.flap_offsets.3',
          'sim.aero_sim.flap_offsets.6',
          'sim.aero_sim.flap_offsets.7',
          'sim.aero_sim.moment_coeff_b_scale_factors.rate_derivatives.p.0',
          'sim.aero_sim.moment_coeff_b_scale_factors.rate_derivatives.p.1',
          'sim.aero_sim.moment_coeff_b_scale_factors.rate_derivatives.p.2',
          'sim.aero_sim.moment_coeff_b_scale_factors.rate_derivatives.q.0',
          'sim.aero_sim.moment_coeff_b_scale_factors.rate_derivatives.q.1',
          'sim.aero_sim.moment_coeff_b_scale_factors.rate_derivatives.q.2',
          'sim.aero_sim.moment_coeff_b_scale_factors.rate_derivatives.r.0',
          'sim.aero_sim.moment_coeff_b_scale_factors.rate_derivatives.r.1',
          'sim.aero_sim.moment_coeff_b_scale_factors.rate_derivatives.r.2',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.0.0',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.0.1',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.0.2',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.1.0',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.1.1',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.1.2',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.2.0',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.2.1',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.2.2',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.3.0',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.3.1',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.3.2',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.4.0',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.4.1',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.4.2',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.5.0',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.5.1',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.5.2',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.6.0',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.6.1',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.6.2',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.7.0',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.7.1',
          'sim.aero_sim.moment_coeff_b_scale_factors.flap_derivatives.7.2',
          'sim.aero_sim.coeff_offsets.CD',
          'sim.aero_sim.coeff_offsets.CL',
          'sim.aero_sim.coeff_offsets.dCldbeta',
          'sim.aero_sim.coeff_offsets.dCmdalpha',
          'sim.aero_sim.coeff_offsets.dCndbeta',
          'sim.phys_sim.wind_speed',
          'sim.phys_sim.air_density',
          'sim.phys_sim.wind_shear_exponent',
          'sim.phys_sim.wind_direction',
          'sim.phys_sim.wind_database_initial_time',
          'sim.phys_sim.wind_shear_ref_height_agl',
          'sim.phys_sim.wind_model',
          'sim.phys_sim.wind_database_y_offset']


class BatchSimDataImporter(object):
  """Interface to data from a batch simulation.

  Used to generate a HDF5 datafile to use for analysis.
  """
  outputdata_filename = 'overview_data.json'
  inputdata_filename = 'configs.tar.gz'
  inputdata_foldername = 'gce_config'
  inputdata_fields = ['sim']

  def __init__(self, folders, process_inputs=True,
               allow_different_commits=False, verbose=True):
    """Initializes the object. The data is not loaded here.

    Args:
      folders: List of folders containing data to be appended.
      process_inputs: Boolean indicating whether to process input data contained
          in the self.inputdata_filename file.
      allow_different_commits: Boolean indicating wether data from different
          commits is allowed.
      verbose: Boolean indicating if terminal print-out are wanted.
    """
    if isinstance(folders, str):
      folders = [folders]

    self.folders = folders
    self.process_inputs = process_inputs

    for folder_str in self.folders:
      if not os.path.isdir(folder_str):
        print("Folder '{0}' does not exist.".format(folder_str))
        self.input_files = None
        return None

      if not os.path.isfile(os.path.join(folder_str, self.outputdata_filename)):
        print("Folder '{0}' does not contain "
              "the file '{1}'.".format(folder_str, self.outputdata_filename))
        self.input_files = None
        return None

    self.input_files = [
        os.path.abspath(os.path.join(input_folder, self.outputdata_filename))
        for input_folder in self.folders]

    self.load_output_data(allow_different_commits, verbose)
    if self.process_inputs:
      self.load_input_data(verbose)

  def load_output_data(self, allow_different_commits=False, verbose=True):
    """Loads output data from self.outputdata_filename.

    Incomplete simulations are skipped, instead of populating the data fields
    with NaNs.

    Args:
      allow_different_commits: Boolean indicating wether data from different
          commits is allowed.
      verbose: Boolean indicating if terminal print-out are wanted.
    """
    # TODO(b/147318624): Right now there is very little data, but in the future
    # there may be huge amounts of data. Changes will need to be done to avoid
    # loading too much data into memory.

    # Do it once for the first one
    input_filename0 = self.input_files[0]
    input_file = open(input_filename0, 'r')
    input_data = json.load(input_file)
    input_file.close()

    if verbose:
      print('Loading data from {0}.'.format(input_filename0))
    self.seed = input_data['parameter_seed']
    self.commit = [str(input_data['commit'])]  # str() to avoid unicode issues.
    if 'sim_name' in input_data:
      self.sim_name = [str(input_data['sim_name'])]
    else:
      # Older data files do not contain a sim_name field.
      self.sim_name = []
    self.title = str(input_data['title'])
    self.date = parser.parse(self.title[len(self.title)-17:])
    self.date_str = self.date.strftime('%Y-%m-%d %H:%M')
    self.scorelist = [str(score['name']) for score in input_data['metrics']]
    self.num_scores = len(self.scorelist)
    self.scoreunits = ['-']*len(self.scorelist)
    self.severitylist = [score['severity'] for score in input_data['metrics']]
    self.crash_score_idxs = [idx for idx in range(len(self.severitylist))
                             if self.severitylist[idx] == 5]
    self.experimental_flag = ['experimental' in
                              input_data['metrics'][idx]['system_labels']
                              for idx in range(self.num_scores)]
    self.num_tables = len(input_data['table_data'])
    self.table_titles = [str(input_data['table_data'][ntable]['title'])
                         for ntable in range(self.num_tables)]
    # table_data below is 3-dimensional:
    #   - first dimension is tables.
    #   - second dimension is number of completed simulations.
    #   - third dimension is scores.
    self.table_data = [
        [self.numerize_list(job_data['scores'])
         for job_data in input_data['table_data'][ntable]['job_data']
         if job_data['sim_success']] for ntable in range(self.num_tables)]
    # job_ids and file_folders are 2-dimensional:
    #   - first dimension is tables.
    #   - second dimension is job_ids of completed simuations.
    self.job_ids = [
        [job_data['job_id']
         for job_data in input_data['table_data'][ntable]['job_data']
         if job_data['sim_success']]
        for ntable in range(self.num_tables)]
    self.file_folders = [
        [os.path.dirname(input_filename0)]*len(self.job_ids[ntable])
        for ntable in range(self.num_tables)]
    self.num_table_samples = [len(t) for t in self.table_data]

    # Now do it again for the rest.
    for input_filename in self.input_files[1:]:
      input_file = open(input_filename, 'r')
      input_data = json.load(input_file)
      input_file.close()

      if verbose:
        print('Loading data from {0}.'.format(input_filename))
      if 'sim_name' in input_data:
        self.sim_name.append(str(input_data['sim_name']))
      if not allow_different_commits:
        assert input_data['commit'] in self.commit, (
            'File {0} has a different commit '
            'compared to {1}.'.format(input_filename, input_filename0))
      elif input_data['commit'] not in self.commit:
        self.commit.append(str(input_data['commit']))
      scorelist = [str(score['name']) for score in input_data['metrics']]
      assert len(scorelist) == len(self.scorelist), (
          'File {0} has a different number of scores compared to'
          ' {1}.'.format(input_filename, input_filename0))
      assert len(input_data['table_data']) == self.num_tables, (
          'File {0} has a different number of tables compared to'
          ' {1}.'.format(input_filename, input_filename0))

      # Get the new data.
      this_table_data = [[self.numerize_list(job_data['scores']) for job_data in
                          input_data['table_data'][ntable]['job_data']
                          if job_data['sim_success']]
                         for ntable in range(self.num_tables)]
      this_job_ids = [[job_data['job_id'] for job_data in
                       input_data['table_data'][ntable]['job_data']
                       if job_data['sim_success']]
                      for ntable in range(self.num_tables)]
      this_folders = [
          [os.path.dirname(input_filename)]*len(this_job_ids[ntable])
          for ntable in range(self.num_tables)]
      this_num_table_samples = [len(t) for t in self.table_data]

      # Append the data.
      for ntable in range(self.num_tables):
        self.table_data[ntable].extend(this_table_data[ntable])
        self.job_ids[ntable].extend(this_job_ids[ntable])
        self.file_folders[ntable].extend(this_folders[ntable])
        self.num_table_samples[ntable] += this_num_table_samples[ntable]

    if verbose:
      print('Done.')

    # Add overall crash indicator (score).
    for ntable in range(self.num_tables):
      for job in range(len(self.table_data[ntable])):
        scores = np.array(self.table_data[ntable][job])

        # Remove experimental scores.
        not_experimental = np.logical_not(np.array(self.experimental_flag))
        crash_not_experimental = np.logical_and(
            scores[self.crash_score_idxs] > 0.999,
            not_experimental[self.crash_score_idxs])

        # The crash indicator is true if any non-experimental crash scores is
        # unacceptable.
        crash_indicator = int(np.any(crash_not_experimental))
        # However, if any score is a NaN, then the crash indicator cannot be
        # determined.
        if np.any(np.isnan(scores)):
          crash_indicator = np.float('nan')

        # Add the crash indicator to the table.
        self.table_data[ntable][job].append(crash_indicator)

    self.scorelist.append('Crash indicator')
    self.scoreunits.append('-')
    self.num_scores += 1
    self.severitylist.append(5)
    self.crash_score_idxs.append(len(self.scorelist) - 1)
    self.experimental_flag.append(False)

  def load_input_data(self, verbose=True):
    """Loads input data.

    Args:
      verbose: Boolean indicating if terminal print-out are wanted.
    """
    # TODO(b/147318624): This is a slow process. Consider using multiprocessing
    # to speed it up.
    # Extract the archived data.
    for folder_str in self.folders:
      input_file_str = os.path.join(folder_str, self.inputdata_filename)
      if verbose:
        print('Extracting file {0}.'.format(input_file_str))
      if not os.path.isfile(input_file_str):
        print('Input data file {0} does not exist.'.format(input_file_str))
        return None
      with tarfile.open(input_file_str, 'r') as inputfile:
        inputfile.extractall(folder_str)

    # Load the input data.
    self.input_data = {}
    for folder_str in self.folders:
      if verbose:
        print('Processing input files in folder {0}.'.format(folder_str))
      input_folder_str = os.path.join(folder_str, self.inputdata_foldername)
      input_file_list = [f for f in glob.glob(input_folder_str + '/*.json')]
      for file_idx, input_file_str in enumerate(input_file_list):
        if verbose:
          percent_report = 20.0  # Report every this percentage of files.
          n_delta = int(np.floor(len(input_file_list)*percent_report/100.))
          if file_idx % n_delta == 0:
            print('  {0:3.0f}% files processed.'.format(
                float(file_idx)/float(len(input_file_list))*100.))
        job_id = int(os.path.basename(input_file_str).split('.')[0])
        with open(input_file_str) as json_file:
          job_raw_input_data = json.load(json_file)

        # Turn the input dictionary into a flat form for easier processing.
        # Use only the fields in self.inputdata_fields.
        flat_raw_job_input_data = []
        for field in self.inputdata_fields:
          flat_raw_job_input_data.extend(
              [[field] + path for path in
               dict_util.GetAllDictPaths(job_raw_input_data[field])])

        # Turn the list of input data in dictionary form compatible with the
        # list in get_monte_carlo_parameter_list().
        flat_job_input_data = {}
        for item in flat_raw_job_input_data:
          path = str('.'.join(item[:-1]))
          value = item[-1]
          if isinstance(value, list):
            # The numerical data needs to be unraveled in linear form to
            # acommodate the format in get_monte_carlo_parameter_list().
            value = np.array(value)
            for i in range(value.size):
              index_tuple = np.unravel_index(i, value.shape)
              complete_path = path + '.' + '.'.join(
                  [str(n) for n in index_tuple])
              flat_job_input_data[complete_path] = value[index_tuple]
          else:
            flat_job_input_data[path] = value

        # Go over the list of expected parameter variation in
        # get_monte_carlo_parameter_list() and create a dictionary that contains
        # all of the dispersion in table form.
        for input_var in get_monte_carlo_parameter_list():
          value = flat_job_input_data[input_var]
          folder_str = os.path.dirname(os.path.dirname(input_file_str))
          value_df = pd.DataFrame({'job_id': job_id,
                                   'folder': folder_str,
                                   'value': value}, index=[0])
          if input_var in self.input_data:
            self.input_data[input_var] = (
                self.input_data[input_var].append(value_df))
          else:
            self.input_data[input_var] = value_df

      if verbose:
        print('  {0:3.0f}% files processed.'.format(100.))

    # Clean up the DataFrames.
    for input_var, entry_df in self.input_data.iteritems():
      self.input_data[input_var] = entry_df.reset_index(drop=True)

    return

  def create_database(self, target_filename, verbose=True):
    """Generates an HDF5 file for analysis.

    Args:
      target_filename: String containing the HDF5 database filename.
      verbose: Boolean indicating if terminal print-out are wanted.
    """
    self.output_folder = os.path.dirname(os.path.abspath(target_filename))
    self.output_file = target_filename

    if not os.path.exists(self.output_folder):
      os.makedirs(self.output_folder)  # Create the folder.

    if verbose:
      print('Creating database {0}'.format(target_filename))
    # Create DataFrame with metadata.
    # Need an index for scalar values.
    metadata_df = pd.DataFrame({'commit': ','.join(self.commit),
                                'sim_name': ','.join(self.sim_name),
                                'seed': self.seed,
                                'date': self.date_str,
                                'title': self.title}, index=[0])

    # Create DataFrame with score data.
    score_df = pd.DataFrame({'name': self.scorelist,
                             'units': self.scoreunits,
                             'severity': self.severitylist,
                             'experimental': self.experimental_flag})

    # Create DataFrame with table data.
    tabledata_df = pd.DataFrame({'title': self.table_titles,
                                 'num_jobs': self.num_table_samples,
                                 'folders': self.file_folders,
                                 'job_ids': self.job_ids})

    # Create list of DataFrames containing data for each table.
    table_data = []
    for ntable in range(self.num_tables):
      table_scores = self.table_data[ntable]
      table_df = pd.DataFrame(table_scores,
                              columns=['score_{0}'.format(n) for
                                       n in range(self.num_scores)])
      # Add job_id column.
      table_df = table_df.assign(job_id=self.job_ids[ntable])
      # Add file column.
      table_df = table_df.assign(folder=self.file_folders[ntable])
      table_data.append(table_df)

    # Save HDF5 file.
    # Searchability is not necessary, so using 'fixed' seems the best
    # format option.
    df_format = 'fixed'
    df_data_columns = False

    # Metadata table.
    metadata_df.to_hdf(self.output_file, 'metadata', mode='w',
                       data_columns=df_data_columns, format=df_format)

    # Score info table.
    score_df.to_hdf(self.output_file, 'scoredata', mode='a',
                    data_columns=df_data_columns, format=df_format)

    # Table info.
    tabledata_df.to_hdf(self.output_file, 'tabledata', mode='a',
                        data_columns=df_data_columns, format=df_format)

    # Score tables.
    for ntable in range(self.num_tables):
      table_data[ntable].to_hdf(self.output_file,
                                'tables/table_{0}'.format(ntable), mode='a',
                                data_columns=df_data_columns, format=df_format)

    # Input table.
    if self.process_inputs:
      for input_var, input_var_df in self.input_data.items():
        input_var_df.to_hdf(self.output_file, 'inputs/{0}'.format(input_var),
                            mode='a', data_columns=df_data_columns,
                            format=df_format)

    if verbose:
      print('Done.')

  def numerize_list(self, input_list):
    """Replaces None values in input_list by a NaN.

    The rest of the values are casted to numpy.float.

    Args:
      input_list: List of values to process.

    Returns:
      List of values transformed to numpy types.
    """
    return [np.float(value) if value is not None else np.float('nan')
            for value in input_list]
