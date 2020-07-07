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

"""Utility to download logs from cloud storage for batch sims."""

# This is a wrapper script which uses the 'gsutil' utility to download
# h5 log files from the cloud storage to $MAKANI_HOME/logs folder.
#
# Examples:
# 1. ./get_batchsim_logs.py
#    In default configuration, this downloads the sparse logs for baseline
#    simulation at zero wind shear from latest nightly monte carlo sweeps.
#
# 2. ./get_batchsim_logs.py -j 45 -xs=False
#    downloads the full size sim log '45.h5'.
#
# 3. ./get_batchsim_logs.py -s 'nightly_crosswind_sweeps' -j 0:10:1
#    downloads the sparse logs 0 to 9 from latest nightly deterministic sweep.
#
# 4. ./get_batchsim_logs.py -s '20180610_test' -j 0 -e=True
#    downloads the error log '0.LOG' from user specified source.
#

import datetime
import os
import sys

import gflags
import makani
import numpy as np


gflags.DEFINE_string('source', 'nightly_crosswind_sweeps_monte_carlo',
                     'Source folder name, options:'
                     '"nightly_crosswind_sweeps_monte_carlo" (default),'
                     '"nightly_crosswind_sweeps" for deterministic sweeps,'
                     '"nightly_hover_disturbances",'
                     '"nightly_power_curve", or'
                     'any user specified batch sim_name.',
                     short_name='s')

gflags.DEFINE_string('job_ids', '0:399:50',
                     'Job ids for logs to be downloaded, comma-separated or'
                     'start:stop:step_size. Default option downloads baseline'
                     'h5 logs for zero wind sheer cases.',
                     short_name='j')

gflags.DEFINE_boolean('sparse_log', True,
                      'Change to False for downloading full sample rate logs.',
                      short_name='xs')

gflags.DEFINE_boolean('error_log', False,
                      'Change to True for downloading error logs.',
                      short_name='e')
FLAGS = gflags.FLAGS

nightly_runs = ['nightly_crosswind_sweeps_monte_carlo',
                'nightly_crosswind_sweeps',
                'nightly_hover_disturbances',
                'nightly_power_curve',
                'nightly_trans_in_sweeps_monte_carlo',
                'nightly_trans_in_sweeps']


def _ParseJobIds(job_ids, sparse_log, error_log):
  """Parse job ids."""
  if ':' in job_ids:
    job_ids = job_ids.split(':')
    assert len(job_ids) == 3
    job_ids = [int(x) for x in job_ids]
    job_ids = np.arange(job_ids[0], job_ids[1], job_ids[2])
    job_ids = [str(x) for x in job_ids]
  elif ',' in job_ids:
    job_ids = job_ids.split(',')
  else:
    job_ids = [job_ids]

  if not error_log:
    file_prefix = ''
    file_extension = '_sparse.h5' if sparse_log else '.h5'
  else:
    file_prefix = '*-'
    file_extension = '.LOG '

  return [file_prefix + x + file_extension for x in job_ids]


def _GetGcsPath(source, error_log):
  """Get the path to the folder on Google cloud storage."""
  batch_sim_bucket = 'gs://makani/batch_sim/'
  gcs_folder = batch_sim_bucket + source

  if not error_log:
    gcs_folder += '/h5_logs/'
  else:
    gcs_folder += '/error/'

  return gcs_folder


def _GetDate(gcs_file):
  """Check if file exists and get creation date."""
  date_string = os.popen('gsutil stat ' + gcs_file +
                         '| grep "Creation time"').read()
  print '\n'
  if not date_string:
    sys.exit()
  else:
    print date_string.strip()
    date_string = date_string.replace(' ', '')
    date_string = date_string.split(',')[-1]
    date_string = datetime.datetime.strptime(date_string, '%d%b%Y%H:%M:%S%Z\n')

    return str(date_string.year) + str(date_string.month) + str(date_string.day)


def _GetSavePath(source, date_stamp):
  """Get the path for saving the files locally."""
  logs_folder = makani.HOME + '/logs/'
  if source in nightly_runs:
    return logs_folder + date_stamp + '_' + source
  else:
    return logs_folder + source


def _DownloadLog(log_path, local_save_path):
  os.system('gsutil -m cp ' + log_path + ' ' + local_save_path)


def main(argv):
  argv = FLAGS(argv)

  job_ids = _ParseJobIds(FLAGS.job_ids, FLAGS.sparse_log, FLAGS.error_log)
  gcs_folder = _GetGcsPath(FLAGS.source, FLAGS.error_log)
  date_stamp = _GetDate(gcs_folder + job_ids[0])
  save_path = _GetSavePath(FLAGS.source, date_stamp)

  if not os.path.isdir(save_path):
    os.mkdir(save_path)

  for ii, job_id in enumerate(job_ids):
    log_path = gcs_folder + job_id
    print '\nDownloading file %s of %s.' %(ii+1, len(job_ids))
    _DownloadLog(log_path, save_path)
  print '\nAll files downloaded to %s.' %(save_path)


if __name__ == '__main__':
  main(sys.argv)
