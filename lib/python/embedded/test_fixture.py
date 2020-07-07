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

"""Coordinate hardware targets for software testing.

This module provides server-side coordination of test fixture hardware for
running software tests. Client test_runner.py inserts test request records
into the 'test' and 'node' database tables to request access to a particular
hardware configuration. The server compares this information with the
'target' table records (which lists available hardware) and then associates
a target record to a node record.
"""

import logging
import sys
import time

import gflags
from makani.lib.python.embedded import hardware_db
from makani.lib.python.embedded import relay_device
import yaml


logging.basicConfig(level=logging.INFO,
                    format='[%(levelname)s] %(message)s')


class RelayManager(object):
  """An interface for controlling the relays of each target."""

  def __init__(self, hardware_db_instance):
    self._db = hardware_db_instance
    self._relays = {}

  def AddRelay(self, relay_module_id, relay_module):
    """Add a relay to the RelayManager.

    Adds a relay to the internal dict _relays and runs GetMask() instance to
    ensure that the relay states in the underlying module are consistent with
    the physical state. GetMask is necessary or else a PowerOn call to a relay
    may do nothing (whereas that device is expected to be reset) if that relay
    was left on from a previous run.

    Args:
      relay_module_id: Integer represending the id of the relay module being
        added to the database table RelayModules.
      relay_module: Dict representation of a relay_module containing keys:
        ['Device', 'Channels', 'Type'] where the 'Type' value must be a valid
        class found in the relay_device python module.
    """
    always_on_bitmask = self._AlwaysOnBitmask(relay_module_id)
    modify_bitmask = (((1 << relay_module['Channels']) - 1) ^
                      always_on_bitmask)
    class_instantiater = getattr(relay_device, relay_module['Type'])
    self._relays[relay_module_id] = class_instantiater(
        device=relay_module['Device'],
        channels=relay_module['Channels'],
        mask=modify_bitmask,
        default=always_on_bitmask)

    # Update relay states in case some were left on.
    self._relays[relay_module_id].GetMask()

  def _GetRelay(self, target_id):
    target = self._db.SelectTargets(target_id=target_id)
    assert len(target) == 1, 'Invalid target match count.'
    port = target[0]['RelayPort']
    assert port >= 0, 'Invalid port number!'
    return (target[0]['RelayModuleId'], target[0]['RelayPort'])

  def _SetRelayMask(self, relay_id, mask):
    self._relays[relay_id].SetMask(mask)

  def _AlwaysOnBitmask(self, relay_module_id):
    results = self._db.SelectTargets(relay_module_id=relay_module_id)
    return sum([result['RelayAlwaysOn'] << result['RelayPort'] for
                result in results])

  def PowerOn(self, target_id):
    relay_id, relay_port = self._GetRelay(target_id)
    return self._relays[relay_id].PowerOn(relay_port)

  def PowerOff(self, target_id):
    relay_id, relay_port = self._GetRelay(target_id)
    return self._relays[relay_id].PowerOff(relay_port)

  def Update(self):
    # Select all targets with a valid RelayModuleId and RelayPort.
    targets = [t for t in self._db.SelectTargets() if t['RelayModuleId'] is not
               None and t['RelayPort'] >= 0]

    # Iterate over each RelayModuleId.
    for relay_id in set([t['RelayModuleId'] for t in targets]):
      bitmask = 0
      busy_targets = self._db.SelectTargets(relay_module_id=relay_id,
                                            busy=True)
      ports = [1 << t['RelayPort'] for t in busy_targets if
               t['RelayModuleId'] == relay_id]
      bitmask = sum(ports)
      bitmask |= self._AlwaysOnBitmask(relay_module_id=relay_id)
      self._SetRelayMask(relay_id, bitmask)


class TestFixture(object):
  """Coordinate hardware targets for software testing."""

  def __init__(self, database):
    self._db = hardware_db.HardwareDatabase(database)
    self._db.InitTables()
    self._relay = RelayManager(self._db)

  def LoadHardware(self, yaml_file):
    """Load hardware target configuration file into database."""

    relays = {}  # A dict to temporarily store RelayModuleId mapping.
    self._db.InitTables()

    def GetRelayId(relay_dict):
      for key, value in relays.iteritems():
        if value == relay_dict:
          return key
      raise ValueError('{} not found in dictionary.'.format(relay_dict))

    with open(yaml_file, 'r') as f:
      yaml_file = yaml.full_load(f)
      for r in yaml_file.get('relay_modules', []):
        relay_config = hardware_db.TranslateYamlStyleToSqlStyle(r)
        relay_id = self._db.Insert('RelayModules', relay_config)
        self._relay.AddRelay(relay_id, relay_config)
        relays[relay_id] = relay_config
      for t in yaml_file.get('targets', []):
        target_config = hardware_db.TranslateYamlStyleToSqlStyle(t)
        relay_config = hardware_db.TranslateYamlStyleToSqlStyle(
            target_config['RelayModule'])
        target_config['RelayModuleId'] = GetRelayId(relay_config)
        del target_config['RelayModule']  # This key does not exist in the db,
                                          # it is a yaml duplicated relay.
        self._db.InsertTarget(target_config)

  def GetCandidateTargets(self, test):
    return [self._db.SelectTargetIdsByTest(node) for node in test]

  def GetPossibleTargets(self, avail_targets, candidates):
    avail = set(avail_targets)
    return [list(set(c) & avail) for c in candidates]

  def IsTestPossible(self, avail_targets, candidates):
    return [] not in self.GetPossibleTargets(avail_targets, candidates)

  def SelectTestTargets(self, avail_targets, candidates, other=None):
    """Select targets to associate with a test.

    Args:
      avail_targets: A list of available target ids.
      candidates: A list of lists specifying the candidate targets (all targets
          matching hardware requirements) for each node.
      other: A list of lists specifying the candidate targets for another test
          that must run concurrently.

    Returns:
      A list of targets to schedule.
    """
    if avail_targets and candidates:
      # Perform a depth first search through each node in test to identify the
      # first possible target combination.
      for target in set(avail_targets) & set(candidates[0]):
        targets = [target]
        remaining = set(avail_targets) - set(targets)
        # Handle each node in test recursively.
        targets.extend(self.SelectTestTargets(remaining, candidates[1:], other))
        if len(targets) == len(candidates):
          # If we schedule this test, can the other test still run?
          remaining -= set(targets)
          if not other or self.SelectTestTargets(remaining, other):
            return targets
    return []

  def ScheduleTest(self, candidates, test, pending):
    """Schedule a test for execution.

    Args:
      candidates: A list of lists specifying the candidate targets (all targets
          matching hardware requirements) for each node.
      test: A list of node test descriptors.
      pending: A list of target candidates corresponding to tests that are
          pending.

    Returns:
      True when scheduled.
    """
    # Determine hardware available for test.
    avail_targets = self._db.SelectTargetIds(busy=False)
    # The current implementation only considers if this test blocks the
    # first pending test.
    if pending:
      pending = pending[0]
    # Filter nodes that prevent the first pending test from running.
    pending = [p for p in self.GetPossibleTargets(avail_targets, pending) if p]
    targets = self.SelectTestTargets(avail_targets, candidates, pending)
    # Can execute this test now?
    if targets:
      for target_id, node in zip(targets, test):
        target = self._db.SelectTargets(target_id=target_id)[0]
        logging.info('Assign node [test=%d node=%d]', node['TestId'],
                     node['TestNodeId'], extra={'TargetId': target['TargetId']})
        self._relay.PowerOn(target['TargetId'])
        self._db.StartTestNode(node['TestNodeId'], target_id)
    return bool(targets)

  def SchedulePendingTests(self):
    """Poll database for pending tests, then attempt to schedule each test."""
    pending = []
    all_targets = self._db.SelectTargetIds()
    for test in self._db.SelectTests(running=False):
      # Compute candidate targets for each node in a given test.
      candidates = self.GetCandidateTargets(test)
      # Can perform the given test with available targets?
      if not self.IsTestPossible(all_targets, candidates):
        logging.warning('Test %s not possible with available hardware.', test,
                        extra={'TargetId': -1})
        self._db.DeleteTestNodes(test)
      elif not self.ScheduleTest(candidates, test, pending):
        pending.append(candidates)

  def Iterate(self):
    # Perform garbage collection first to free up any staled nodes.
    self._db.GarbageCollection()
    self._relay.Update()
    # Schedule new tests.
    self.SchedulePendingTests()

  def Run(self, yaml_file):
    self.LoadHardware(yaml_file)
    while True:
      self.Iterate()
      time.sleep(1.0)


def ParseFlags(argv):
  """Define and parse gflags parameters."""
  gflags.DEFINE_string('database', 'hardware.db',
                       'Full path to shared hardware database file.')
  gflags.DEFINE_string('config', None,
                       'Full path to test fixture hardware configuration file.')
  gflags.MarkFlagAsRequired('config')
  return gflags.FLAGS(argv)


def main(argv):
  try:
    argv = ParseFlags(argv)
  except gflags.FlagsError, e:
    print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], gflags.FLAGS)
    sys.exit(1)

  fixture = TestFixture(database=gflags.FLAGS.database)
  fixture.Run(yaml_file=gflags.FLAGS.config)


if __name__ == '__main__':
  main(sys.argv)
