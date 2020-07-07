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

"""Limit number of concurrent build processes."""

import os
import sys
import textwrap
import time

import gflags
from makani.lib.python.embedded import database
import pexpect


FLAGS = gflags.FLAGS

gflags.DEFINE_string('build_queue', 'build.db',
                     'Full path to shared build queue database file.')
gflags.DEFINE_integer('build_timeout', 300,
                      'Maximum build time [s].')
gflags.DEFINE_integer('max_builders', 1,
                      'Maximum number of concurrent builders.')


# Timeout [s] before considering a build dead.
WATCHDOG_TIMEOUT = 15.0


class BuildDatabase(database.DatabaseInterface):
  """Database to track build processes."""

  def __init__(self, name):
    super(BuildDatabase, self).__init__(name)
    self.SqlExecute(textwrap.dedent("""
        CREATE TABLE IF NOT EXISTS Jobs (
          JobId INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
          ProcessId INTEGER UNIQUE NOT NULL,
          IsBuilding INTEGER DEFAULT 0 CHECK (IsBuilding = 0 OR IsBuilding = 1),
          KeepAliveTime FLOAT
        )"""))


class BuildQueue(BuildDatabase):
  """Manage build queue to limit number of concurrent build processes."""

  def __init__(self, build_db=None, max_builders=None):
    if build_db is None:
      build_db = FLAGS.build_queue
    if max_builders is None:
      max_builders = FLAGS.max_builders
    super(BuildQueue, self).__init__(build_db)
    self._max_builders = max_builders

  def Iterate(self):
    # Delete stalled (or crashed) builds.
    self.SqlExecute(textwrap.dedent("""
        DELETE FROM Jobs
        WHERE KeepAliveTime + ? < ?"""), (WATCHDOG_TIMEOUT, time.time()))
    # Schedule builders.
    row = self.SqlSelectOne(textwrap.dedent("""
        SELECT SUM(IsBuilding)
        FROM Jobs"""))
    num_building = row['SUM(IsBuilding)']
    if not num_building:
      num_building = 0
    new_builders = self._max_builders - num_building
    if new_builders > 0:
      self.SqlExecute(textwrap.dedent("""
          UPDATE Jobs
          SET IsBuilding=1
          WHERE JobId IN (
            SELECT JobId
            FROM Jobs
            WHERE IsBuilding=0
            LIMIT %d)""" % new_builders))


class Builder(BuildDatabase):
  """Build when given ready signal from BuildQueue."""

  def __init__(self, build_db=None):
    if build_db is None:
      build_db = FLAGS.build_queue
    super(Builder, self).__init__(build_db)
    self._pid = os.getpid()

  def _KeepAlive(self):
    self.SqlExecute(textwrap.dedent("""
        UPDATE Jobs
        SET KeepAliveTime=?
        WHERE ProcessId=?"""), (time.time(), self._pid))

  def _InsertJob(self):
    self.SqlExecute(textwrap.dedent("""
        INSERT INTO Jobs (
          JobId,
          ProcessId,
          KeepAliveTime)
        VALUES (null, ?, ?)"""), (self._pid, time.time()))

  def _WaitForReady(self):
    while True:
      row = self.SqlSelectOne(textwrap.dedent("""
          SELECT IsBuilding
          FROM Jobs
          WHERE ProcessId=?
          LIMIT 1"""), (self._pid,))
      if not row:
        return False
      if row['IsBuilding']:
        return True
      self._KeepAlive()
      time.sleep(1.0)

  def _StartBuild(self, workpath, targets, extra_args):
    bazel_args = ['build'] + extra_args + targets
    child = pexpect.spawn(
        command='bazel',
        args=bazel_args,
        cwd=workpath,
        logfile=sys.stdout,
        env={'MAKANI_HOME': workpath})
    return child

  def _WaitForBuild(self, child, timeout):
    start_time = time.time()
    reason = 0
    while reason == 0 and (time.time() - start_time) < timeout:
      time.sleep(1.0)
      self._KeepAlive()
      reason = child.expect([pexpect.TIMEOUT, pexpect.EOF], timeout=0)
    child.close(force=True)
    return child.exitstatus == 0

  def _DeleteJob(self):
    self.SqlExecute(textwrap.dedent("""
      DELETE FROM Jobs
      WHERE ProcessId=?"""), (self._pid,))

  def Build(self, workpath, q7_targets, tms570_targets, timeout=None):
    if timeout is None:
      timeout = FLAGS.build_timeout
    self._InsertJob()
    if self._WaitForReady():
      success = True
      if q7_targets and success:
        child = self._StartBuild(workpath, q7_targets, ['--config q7'])
        success = self._WaitForBuild(child, timeout)
      if tms570_targets and success:
        child = self._StartBuild(workpath, tms570_targets, ['--config tms570'])
        success = self._WaitForBuild(child, timeout)
      self._DeleteJob()
      return success
    return False
