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

"""This module abstracts the database interface for common queries."""

from collections import defaultdict
import textwrap
import time

from makani.lib.python import string_util
from makani.lib.python.embedded import database


# Timeout [s] before considering a TestRunner dead.
WATCHDOG_TIMEOUT = 15.0


def TranslateYamlStyleToSqlStyle(snake_dict):
  """Translate Google YAML style to Google SQL style."""
  return {string_util.SnakeToCamel(key): value
          for (key, value) in snake_dict.iteritems()}


def _SplitOptions(options):
  """Split string of hardware options into a list."""
  if options:
    return options.split()
  else:
    return []


class HardwareDatabase(database.DatabaseInterface):
  """Database stores state of hardware under test."""

  def __init__(self, name):
    super(HardwareDatabase, self).__init__(name)

  def InitTables(self):
    """Initialize tables to a known state."""

    # Create RelayModules table to describe how to use each relay module.
    # Device contains an IP address or a /dev device. Note: Device should be
    # changed to contain a uniquely identifiable handle (udev can re-assign
    # /dev/ttyACM0 to /dev/ttyACM1 upon initialization) and the mapping
    # functionality should be updated to match.
    self.SqlExecute('DROP TABLE IF EXISTS RelayModules')
    self.SqlExecute(textwrap.dedent("""
        CREATE TABLE RelayModules (
          RelayModuleId INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
          Channels INTEGER NOT NULL,
          Type VARCHAR(64) NOT NULL,
          Device VARCHAR(32)
        )"""))

    # Create 'target' table to describe the hardware configuration of each
    # physical device. The test fixture populates this table on initialization.
    # After initialization, this table should remain constant.
    self.SqlExecute('DROP TABLE IF EXISTS Targets')
    self.SqlExecute(textwrap.dedent("""
        CREATE TABLE Targets (
          TargetId INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
          /* TMS570 AioNode (e.g., kAioNodeFcA) or NULL. */
          AioNodeTms570 VARCHAR(32),
          /* Q7 AioNode (e.g., kAioNodeFlightControlerA) or NULL. */
          AioNodeQ7 VARCHAR(32),
          /* Text board name (e.g., aio). */
          BoardName VARCHAR(32) NOT NULL,
          /* Text carrier board name (e.g., servo) or NULL. */
          CarrierName VARCHAR(32),
          /* Relay module ID or NULL. */
          RelayModuleId INTEGER NOT NULL REFERENCES RelayModules(RelayModuleId) ON DELETE CASCADE,
          RelayPort INTEGER NOT NULL,
          RelayAlwaysOn INTEGER NOT NULL DEFAULT 0
        )"""))

    # Create 'option' table to describe build options for each physical device
    # in the 'target' table. Each physical device may contain multiple hardware
    # options. The test fixture populates this table on initialization. After
    # initialization, this table should remain constant.
    self.SqlExecute('DROP TABLE IF EXISTS TargetHardwareOptions')
    self.SqlExecute(textwrap.dedent("""
        CREATE TABLE TargetHardwareOptions (
          TargetHardwareOptionId INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
          TargetId INTEGER REFERENCES Targets(TargetId) ON DELETE CASCADE,
          /* Text option name (e.g., imu). */
          Name VARCHAR(128)
        )"""))

    # Create 'test' table to describe the collection of nodes required for each
    # test description. The test runners (multiple unrelated processes)
    # create one row for each test (group of TestNodes). Each test runner
    # should then update the KeepAliveTime column with the current time. If
    # a test runner fails to update this column and enough time elapses, the
    # test fixture will delete the row and associated TestNodes. After
    # completion of a test, test runners should delete the associated row in
    # this table.
    self.SqlExecute('DROP TABLE IF EXISTS Tests')
    self.SqlExecute(textwrap.dedent("""
        CREATE TABLE Tests (
          TestId INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
          ProcessId INTEGER,
          KeepAliveTime FLOAT
        )"""))

    # Create 'node' table to describe the node hardware requirements for each
    # test description in the 'test' table. Each test may contain multiple
    # nodes under test. The test runners create one or more rows to associate
    # with each row in Tests. Thereafter, the test fixture updates the
    # TargetId column to associate the TestNode with a specific hardware
    # device. After completion of a test, the test runner should delete the
    # associated row in this table.
    self.SqlExecute('DROP TABLE IF EXISTS TestNodes')
    self.SqlExecute(textwrap.dedent("""
        CREATE TABLE TestNodes (
          TestNodeId INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
          TestId INTEGER REFERENCES Tests(TestId) ON DELETE CASCADE,
          TargetId INTEGER UNIQUE
            REFERENCES Targets(TargetId) ON DELETE CASCADE,
          /* Required board name or NULL. */
          BoardName VARCHAR(32),
          /* Required carrier board name or NULL. */
          CarrierName VARCHAR(32),
          /* List of required options or NULL. */
          HardwareOptions VARCHAR(128)
        )"""))

  def Insert(self, table, value_dict):
    defaults = defaultdict(lambda: None)
    defaults.update(value_dict)
    keys = defaults.keys()
    keys_colons = [':{}'.format(k) for k in keys]
    return self.SqlExecute(
        'INSERT INTO {} ({}) VALUES ({})'.format(table, ','.join(keys),
                                                 ','.join(keys_colons)),
        defaults)

  def InsertTarget(self, target_config):
    """Insert target configuration."""

    # HardwareOptions belong in another table, set aside for later.
    hardware_options = target_config.get('HardwareOptions')
    if hardware_options:
      del target_config['HardwareOptions']

    # Default values.
    target_id = self.Insert('Targets', target_config)

    if target_id is not None and hardware_options:
      target_config['TargetId'] = target_id
      # Each target may have multiple hardware options.
      for option in _SplitOptions(hardware_options):
        self.SqlExecute(textwrap.dedent("""
            INSERT INTO TargetHardwareOptions (
              TargetHardwareOptionId,
              TargetId,
              Name)
            VALUES (null, ?, ?)"""), (target_id, option))
      return target_config

  def InsertTest(self, test_config, pid):
    """Insert test configuration."""

    # Test runner inserts tests. Use pid=NULL to lock test while we insert
    # each test node. The test fixture should check for this condition.
    nodes = {}
    test_id = self.SqlExecute(textwrap.dedent("""
        INSERT INTO Tests (
          TestId,
          KeepAliveTime)
        VALUES (null, ?)"""), (time.time(),))
    if test_id is not None:
      for node_config in test_config:
        # Default values.
        node = {
            'BoardName': None,        # Required board name (e.g., aio).
            'CarrierName': None,      # Required carrier board name (e.g., fc).
            'HardwareOptions': None,  # Required hardware options (e.g., imu).
        }
        node.update(node_config)
        node_id = self.SqlExecute(textwrap.dedent("""
            INSERT INTO TestNodes (
              TestNodeId,
              TestId,
              BoardName,
              CarrierName,
              HardwareOptions)
            VALUES (null, ?, ?, ?, ?)"""), (
                test_id,
                node['BoardName'],
                node['CarrierName'],
                node['HardwareOptions']))
        if node_id is not None:
          nodes[node_id] = node
      # Update pid to unlock test node (and mark ready).
      self.SqlExecute(textwrap.dedent("""
          UPDATE Tests
          SET ProcessId=?
          WHERE TestId=?"""), (pid, test_id))
    return nodes

  def SelectTargetByNodeId(self, node_id):
    return self.SqlSelectOne(textwrap.dedent("""
        SELECT
          Targets.AioNodeTms570,
          Targets.AioNodeQ7,
          Targets.BoardName,
          Targets.CarrierName
        FROM Targets
        INNER JOIN TestNodes
        ON Targets.TargetId = TestNodes.TargetId
        WHERE TestNodes.TestNodeId=?
        LIMIT 1"""), (node_id,))

  def SelectRelayModules(self, relay_module_id=None):
    sql = 'SELECT * FROM RelayModules'
    where = []
    value = []
    if relay_module_id is not None:
      where = 'RelayModuleId=?'
      value = [relay_module_id]
      sql += ' WHERE {}'.format(where)
    return self.SqlSelectAll(sql, tuple(value))

  def SelectRelayModuleForTarget(self, target_id):
    return self.SqlSelectOne(textwrap.dedent("""
        SELECT *
        FROM Targets
        NATURAL JOIN RelayModules
        WHERE Targets.TargetId=?"""), (target_id,))

  def SelectTargets(self, target_id=None, tms570=None, q7=None,
                    relay_module_id=None, board=None,
                    carrier=None, options=None, busy=None):
    """Obtain the intersection of specified options."""

    # Build WHERE parameters for SELECT query.
    where = []
    value = []
    if target_id is not None:
      where += ['Targets.TargetId=?']
      value += [target_id]
    if tms570 is not None:
      where += ['Targets.AioNodeTms570=?']
      value += [tms570]
    if q7 is not None:
      where += ['Targets.AioNodeQ7=?']
      value += [q7]
    if relay_module_id is not None:
      where += ['Targets.RelayModuleId=?']
      value += [relay_module_id]
    if board is not None:
      where += ['Targets.BoardName=?']
      value += [board]
    if carrier is not None:
      where += ['Targets.CarrierName=?']
      value += [carrier]
    if busy:
      where += [textwrap.dedent("""
          EXISTS (
            SELECT NULL FROM TestNodes
            WHERE TestNodes.TargetId = Targets.TargetId)""")]
    elif busy is not None:
      where += [textwrap.dedent("""
          NOT EXISTS (
            SELECT NULL FROM TestNodes
            WHERE TestNodes.TargetId = Targets.TargetId)""")]

    # When specifying options, we want to the intersection of all options
    # (we require all options). We therefore construct multiple SQL SELECT
    # queries, each with a different option name set, and find the
    # intersection between all queries. We repeat all non-option values
    # in each query.
    sql = 'SELECT Targets.* FROM Targets '
    options = _SplitOptions(options)
    if options:
      where += ['TargetHardwareOptions.Name=?']
      value = [value + [o] for o in options]
      value = [j for i in value for j in i]
      sql += textwrap.dedent("""
          INNER JOIN TargetHardwareOptions
          ON Targets.TargetId = TargetHardwareOptions.TargetId
          """)
    if where:
      sql += 'WHERE ' + ' AND '.join(where)
    sql = ' INTERSECT '.join([sql] * max(1, len(options)))
    return self.SqlSelectAll(sql, tuple(value))

  def SelectTargetIds(self, *args, **kwargs):
    # Sort results for combination match testing.
    return sorted([o['TargetId'] for o in self.SelectTargets(*args, **kwargs)])

  def SelectTargetIdsByTest(self, node):
    return self.SelectTargetIds(
        board=node['BoardName'],
        carrier=node['CarrierName'],
        options=node['HardwareOptions'])

  def SelectTestNodes(self, test_id=None, pid=None, running=None):
    # Build WHERE parameters for SELECT query.
    where = []
    value = []
    if test_id is not None:
      where += ['Tests.TestId=?']
      value += [test_id]
    if pid is not None:
      where += ['Tests.ProcessId=?']
      value += [pid]
    else:
      where += ['Tests.ProcessId IS NOT NULL']  # Handle InsertTest ready lock.
    if running:
      where += ['TestNodes.TargetId IS NOT NULL']
    elif running is not None:
      where += ['TestNodes.TargetId IS NULL']
    sql = textwrap.dedent("""
        SELECT
          Tests.ProcessId,
          TestNodes.*
        FROM Tests
        INNER JOIN TestNodes
        ON Tests.TestId = TestNodes.TestId
        WHERE """ + ' AND '.join(where))
    # Create nodes=[{test desc}, {test desc}, ...].
    return self.SqlSelectAll(sql, tuple(value))

  def SelectTests(self, *args, **kwargs):
    # Create tests=[[{test desc}, {test desc}], [{test desc}], ...].
    tests = self.SqlSelectAll(textwrap.dedent("""
        SELECT TestId
        FROM Tests
        WHERE ProcessId IS NOT NULL
        ORDER BY TestId ASC"""))
    tests = [self.SelectTestNodes(t['TestId'], *args, **kwargs) for t in tests]
    return [t for t in tests if t]

  def StartTestNode(self, node_id, target_id):
    self.SqlExecute(textwrap.dedent("""
        UPDATE TestNodes
        SET TargetId=?
        WHERE TestNodeId=?"""), (target_id, node_id))

  def DeleteTestNode(self, node):
    self.SqlExecute(textwrap.dedent("""
        DELETE FROM TestNodes
        WHERE TestNodeId=?"""), (node['TestNodeId'],))

  def DeleteTestNodes(self, test):
    for node in test:
      self.SqlExecute(textwrap.dedent("""
          DELETE FROM Tests
          WHERE TestId=?"""), (node['TestId'],))

  def DeleteTests(self, pid):
    self.SqlExecute(textwrap.dedent("""
        DELETE FROM Tests
        WHERE Tests.ProcessId=?"""), (pid,))

  def KeepTestsAlive(self, pid):
    self.SqlExecute(textwrap.dedent("""
        UPDATE Tests
        SET KeepAliveTime=?
        WHERE ProcessId=?"""), (time.time(), pid))

  def GarbageCollection(self):
    # Delete stalled (or crashed) tests.
    self.SqlExecute(textwrap.dedent("""
        DELETE FROM Tests
        WHERE KeepAliveTime + ? < ?"""), (WATCHDOG_TIMEOUT, time.time()))
    # Delete nodes without a test group.
    self.SqlExecute(textwrap.dedent("""
        DELETE FROM TestNodes
        WHERE NOT EXISTS (
          SELECT NULL FROM Tests
          WHERE TestNodes.TestId = Tests.TestId
        )"""))
    # Delete test group after deleting all test nodes.
    self.SqlExecute(textwrap.dedent("""
        DELETE FROM Tests
        WHERE ProcessId IS NOT NULL
        AND
        NOT EXISTS (
          SELECT NULL FROM TestNodes
          WHERE Tests.TestId = TestNodes.TestId
        )"""))

  def GetBusy(self):
    return [t for t in self.SelectTargets(busy=True)
            if t['RelayModule'] is not None and t['RelayPort'] >= 0]
