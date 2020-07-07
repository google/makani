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

"""Utilities for using TurbSim wind databases."""

import os
import re
from makani.lib.python import gsutil


def CheckTurbsimFileName(file_name):
  """Checks if the database name follows the TurbSim naming convention."""

  # File naming convention:
  #   <date database generation started>-<time database generation started>-
  #   <unique folder number this file is a part of>-
  #   <0-based iteration of the random seed generator used to create this file>
  #   _<2 digit wind speed>mps_<2 digit wind shear exponent*100>shear.h5
  # For example, for the 2nd file within the 006 folder generated at 9 m/s and
  # a shear exponent of 0.1:  20181011-165912-006-01_09mps_10shear.h5
  turbsim_name_re = re.compile(r'\d\d\d\d[0-1]\d[0-3]\d'  # date
                               r'-[0-2]\d[0-5]\d[0-5]\d'  # time
                               r'-\d\d\d'  # online folder identifier
                               r'-\d\d'    # iteration number from start seed
                               r'_\d\dmps_\d\dshear\.h5')  # wind conditions
  if turbsim_name_re.match(os.path.basename(file_name)):
    return True
  else:
    return False


def GetCloudBasePath():
  """Returns the folder within the Makani bucket where all databases live."""

  return 'gs://makani_pub/turbsim_databases'


def GetOnlineFolder(database_file):
  """Extracts source id of the file and returns matching folder name."""

  assert CheckTurbsimFileName(database_file), (
      'File name does not follow TurbSim naming convention')
  online_folder_id = ('-' + database_file[16:19] + '-')
  gsutil_api = gsutil.GsutilApi()
  online_paths = gsutil_api.List(GetCloudBasePath())
  online_folder = [os.path.basename(os.path.normpath(path))
                   for path in online_paths
                   if online_folder_id in path]
  assert len(online_folder) == 1, (
      'Turbsim online folder identification failed.')
  return online_folder[0]


def DownloadDatabase(database_file, local_path):
  """Downloads TurbSim file to the specified local directory, if needed."""

  if os.path.exists(local_path):
    print 'TurbSim database already exists locally, skipping download.'
  else:
    gsutil_api = gsutil.GsutilApi()
    # TODO: Set up a class for handling more autoselection tasks so
    # that we don't have to pass as much back and forth,
    # or requery the folder list?
    cloud_path = os.path.join(
        GetCloudBasePath(), GetOnlineFolder(database_file),
        'h5_files', database_file)
    gsutil_api.Copy(cloud_path, local_path, False)


def GetWindSpeedFromName(file_name):
  """Returns the wind speed [m/s] in file_name, using the naming convention."""

  assert CheckTurbsimFileName(file_name), (
      'File name does not follow TurbSim naming convention')
  file_base_name = os.path.basename(file_name)
  return float(file_base_name[23:25])


def GetWindShearFromName(file_name):
  """Returns the wind shear exp [-], using the TurbSim naming convention."""

  assert CheckTurbsimFileName(file_name), (
      'File name does not follow TurbSim naming convention')
  file_base_name = os.path.basename(file_name)
  return float(file_base_name[29:31])/100.


class TurbSimDatabaseSelector(object):
  """Information about the online TurbSim folder to pull wind databases from.

  Attributes:
    folder: The name of the folder in the online TurbSim database bucket to
      pull files from.
    databases: The full list of database files in that folder.
    label: Description of this range object.
    base_params: The default config parameters as defined in the *.py files.
  """

  def __init__(self, folder, base_params):
    self._folder = folder
    self.label = 'Wind Database'
    # Pull down info from the cloud about files in this database set.
    gsutil_api = gsutil.GsutilApi()
    cloud_path = os.path.join(
        'gs://makani_pub/turbsim_databases', folder, 'h5_files')
    self._databases = gsutil_api.List(cloud_path)
    self._base_params = base_params

  def GetSpecificOverride(self, overrides, specific_num):
    """Select a specific database for override based on sorted position.

    Since the database list is ordered by filename, which starts with
    generation datetime, this will still be consistent even if more
    databases are added in the future.

    Args:
      overrides: Config dictionary containing info about the wind condition.
      specific_num: Index of which database from the list of appropriate options
        to select.

    Returns:
      (override, specific_num), where override is a dictionary with the selected
        wind database path name, and specific_num is the same as the input arg.
    """

    databases_with_wind_condition = self._ListAppropriateDatabases(overrides)
    wind_file_path = databases_with_wind_condition[specific_num]
    return self.GetOverrides(wind_file_path), specific_num

  def _GetWindCondition(self, overrides):
    """Get string describing wind conditions with database naming convention."""
    wind_speed = overrides['sim']['phys_sim']['wind_speed']
    if 'wind_shear_exponent' in overrides['sim']['phys_sim']:
      wind_shear = overrides['sim']['phys_sim']['wind_shear_exponent']
    else:
      wind_shear = self._base_params['sim']['phys_sim']['wind_shear_exponent']
    wind_condition = '{:02.0f}mps_{:02.0f}shear.h5'.format(
        wind_speed, wind_shear * 100.0)
    return wind_condition

  def _ListAppropriateDatabases(self, overrides):
    """Get the list of available and appropriate database options.

    Make a string with convention used in naming the turbsim databases
    with the requested wind speed and shear, then filter all the
    databases in the specificed folder to find files with the
    requested conditions.

    Args:
      overrides: Config dictionary containing info about the wind condition.

    Returns:
      A list of path names of databases in the TurbSim folder that are
      appropriate for the given wind condition.

    Raises:
      AssertionError: No appropriate databases were found.
    """

    wind_condition = self._GetWindCondition(overrides)
    # TODO: Ideally we should keep all database parameters in a sheet
    # or somewhere so that we can filter based on any condition, not just those
    # included in the filename by convention, as we may change conventions in
    # the future and it would be good to be backwards compatible.
    databases_with_wind_condition = [
        db_file for db_file in self._databases if wind_condition in db_file]
    assert databases_with_wind_condition, (
        'No databases in the flagged TurbSim folder \n' + self._folder
        + '\nfor the wind condition: ' + wind_condition
        + '\nSee go/makani-turbsim-databases for available '
        'databases (especially the -summary.csv file in the '
        'summary_files sub-directory of each folder) for '
        'available wind conditions.\nUse matching parameters in your batch run '
        'setup or see go/makani-turbsim for instructions on how '
        'to generate additional databases for your desired condition(s).\n'
        'Alternatively, set wind_model param to kWindModelDrydenTurbulence; '
        'if using run_sim, you will also need to pass flag:\n'
        '--with_online_turbsim_databases=f.')
    return databases_with_wind_condition

  def GetNumAppropriateDatabases(self, overrides):
    """Get the number of available, appropriate database options."""

    return len(self._ListAppropriateDatabases(overrides))

  def GetOverrides(self, wind_file_path):
    """Get the wind overrides to be included in the sim.

    Args:
      wind_file_path: Name of wind database to apply as an override.

    Returns:
      A set of overrides to be included in a simulation.
    """
    return {'sim': {'phys_sim': {'wind_database': {'name': wind_file_path}}}}
