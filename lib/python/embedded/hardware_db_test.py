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

import unittest

from makani.lib.python.embedded import hardware_db


class TestHardwareDatabase(unittest.TestCase):

  def setUp(self):
    self._db = hardware_db.HardwareDatabase(name=':memory:')
    self._db.InitTables()

  def InsertRelayModules(self):
    # Insert two RelayModules
    relay_module = [
        {'Device': '/dev/ttyACM0', 'Channels': 8, 'Type': 'thing1'},
        {'Device': '192.168.1.200', 'Channels': 20, 'Type': 'thing2'}]
    for module in relay_module:
      self._db.Insert('RelayModules', module)

  def testInsertRelayModules(self):
    # Expect empty database.
    rows = self._db.SqlSelectAll('SELECT * FROM RelayModules')
    self.assertFalse(rows)

    self.InsertRelayModules()

    rows = self._db.SqlSelectAll('SELECT * FROM RelayModules')
    self.assertEqual(len(rows), 2)

    rows = self._db.SqlSelectAll(
        'SELECT Device,Channels FROM RelayModules')
    response = [{'Device': u'/dev/ttyACM0',
                 'Channels': 8},
                {'Device': u'192.168.1.200',
                 'Channels': 20}]
    self.assertEqual(rows, response)

  def testInsertTarget(self):
    self.InsertRelayModules()

    # Expect empty database.
    rows = self._db.SqlSelectAll('SELECT * FROM Targets')
    self.assertFalse(rows)
    rows = self._db.SqlSelectAll('SELECT * FROM TargetHardwareOptions')
    self.assertFalse(rows)

    # Insert one target.
    self._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcA',
        'AioNodeQ7': 'kAioNodeFlightControllerA',
        'BoardName': 'aio',
        'CarrierName': 'fc',
        'HardwareOptions': 'imu novatel',
        'RelayModuleId': 1,
        'RelayPort': 1,
    })
    rows = self._db.SqlSelectAll('SELECT * FROM Targets')
    self.assertEqual(len(rows), 1)
    rows = self._db.SqlSelectAll('SELECT * FROM TargetHardwareOptions')
    self.assertEqual(len(rows), 2)

    # Insert next target.
    self._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcB',
        'AioNodeQ7': 'kAioNodeFlightControllerB',
        'BoardName': 'aio',
        'CarrierName': 'fc',
        'HardwareOptions': 'imu septentrio',
        'RelayModuleId': 1,
        'RelayPort': 1,
    })
    rows = self._db.SqlSelectAll('SELECT * FROM Targets')
    self.assertEqual(len(rows), 2)
    rows = self._db.SqlSelectAll('SELECT * FROM TargetHardwareOptions')
    self.assertEqual(len(rows), 4)

  def testInsertTest(self):
    # Expect empty database.
    rows = self._db.SqlSelectAll('SELECT * FROM Tests')
    self.assertFalse(rows)
    rows = self._db.SqlSelectAll('SELECT * FROM TestNodes')
    self.assertFalse(rows)

    # Insert one test.
    test1 = [{'HardwareOptions': 'novatel'},
             {'HardwareOptions': 'septentrio'}]
    nodes = self._db.InsertTest(test1, 1)
    self.assertEqual(len(nodes), 2)
    rows = self._db.SqlSelectAll('SELECT * FROM Tests')
    self.assertEqual(len(rows), 1)
    rows = self._db.SqlSelectAll('SELECT * FROM TestNodes')
    self.assertEqual(len(rows), 2)

    # Insert next test.
    test2 = [{'HardwareOptions': 'hemisphere'}]
    nodes = self._db.InsertTest(test2, 2)
    self.assertEqual(len(nodes), 1)
    rows = self._db.SqlSelectAll('SELECT * FROM Tests')
    self.assertEqual(len(rows), 2)
    rows = self._db.SqlSelectAll('SELECT * FROM TestNodes')
    self.assertEqual(len(rows), 3)

  def testSelectRelayModule(self):
    # Insert RelayModules.
    relay_module = [
        {'Device': '/dev/ttyACM0', 'Channels': 8, 'Type': 'thing1'},
        {'Device': '192.168.1.200', 'Channels': 20, 'Type': 'thing2'}]

    relay_module_id = []

    for module in relay_module:
      relay_module_id.append(self._db.Insert('RelayModules', module))

    # Insert Targets.
    target_base = {
        'AioNodeTms570': 'kAioNodeFcA',
        'AioNodeQ7': 'kAioNodeFlightControllerA',
        'BoardName': 'aio',
        'CarrierName': 'fc',
        'RelayModuleId': 1,
        'RelayPort': 1,
    }
    target_id_module = []
    target_base.update({'RelayModuleId': relay_module_id[0],
                        'RelayPort': 2,
                        'HardwareOptions': 'imu novatel',
                        'TargetId': None})
    target_id_module.append(self._db.InsertTarget(target_base))
    target_base.update({'RelayModuleId': relay_module_id[1],
                        'RelayPort': 6,
                        'HardwareOptions': 'imu novatel',
                        'TargetId': None})
    target_id_module.append(self._db.InsertTarget(target_base))

    # Test SelectRelayModules method.
    for ind in range(len(target_id_module)):
      relay_index = relay_module_id.index(
          target_id_module[ind]['RelayModuleId'])
      self.assertEqual(
          self._db.SelectRelayModules(target_id_module[ind]['TargetId']),
          [{
              'Channels': relay_module[relay_index]['Channels'],
              'RelayModuleId': target_id_module[ind]['RelayModuleId'],
              'Device': relay_module[relay_index]['Device'],
              'Type': relay_module[relay_index]['Type'],
          }])

  def testSelectTargetByNodeId(self):
    self.InsertRelayModules()

    target_id = self._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcA',
        'AioNodeQ7': 'kAioNodeFlightControllerA',
        'BoardName': 'aio',
        'CarrierName': 'fc',
        'HardwareOptions': 'imu novatel',
        'RelayModuleId': 1,
        'RelayPort': 1,
    })
    test1 = [{'HardwareOptions': 'novatel'}]
    nodes = self._db.InsertTest(test1, 1)
    self.assertEqual(len(nodes), 1)
    node_id = nodes.keys()[0]

    # Target not associated with node.
    target = self._db.SelectTargetByNodeId(node_id)
    self.assertFalse(target)

    # Target assicated with node.
    self._db.StartTestNode(node_id, target_id['TargetId'])
    target = self._db.SelectTargetByNodeId(node_id)
    self.assertTrue(target)

  def testSelectTargets(self):
    self.InsertRelayModules()

    self._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcA',
        'AioNodeQ7': 'kAioNodeControllerA',
        'BoardName': 'aio',
        'CarrierName': 'fc',
        'HardwareOptions': 'imu novatel',
        'RelayModuleId': 1,
        'RelayPort': 1,
    })
    self._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcB',
        'AioNodeQ7': 'kAioNodeControllerB',
        'BoardName': 'aio',
        'CarrierName': 'fc',
        'HardwareOptions': 'imu septentrio',
        'RelayModuleId': 1,
        'RelayPort': 1,
    })
    self._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcC',
        'AioNodeQ7': 'kAioNodeControllerC',
        'BoardName': 'fc',
        'HardwareOptions': 'imu hemisphere',
        'RelayModuleId': 1,
        'RelayPort': 1,
    })
    test1 = [{'HardwareOptions': 'novatel'},
             {'HardwareOptions': 'septentrio'}]
    self._db.InsertTest(test1, 1)

    # Query for TMS570 node.
    targets = self._db.SelectTargets(tms570='kAioNodeFcB')
    self.assertEqual(len(targets), 1)
    self.assertEqual(targets[0]['AioNodeTms570'], 'kAioNodeFcB')
    targets = self._db.SelectTargets(tms570='kAioNodeControllerB')
    self.assertEqual(len(targets), 0)

    # Query for Q7 node.
    targets = self._db.SelectTargets(q7='kAioNodeControllerB')
    self.assertEqual(len(targets), 1)
    self.assertEqual(targets[0]['AioNodeQ7'], 'kAioNodeControllerB')
    targets = self._db.SelectTargets(q7='kAioNodeFcB')
    self.assertEqual(len(targets), 0)

    # Query for board type.
    targets = self._db.SelectTargets(board='aio')
    self.assertEqual(len(targets), 2)
    targets = self._db.SelectTargets(board='fc')
    self.assertEqual(len(targets), 1)

    # Query for carrier type.
    targets = self._db.SelectTargets(carrier='fc')
    self.assertEqual(len(targets), 2)

    # Query for hardware options.
    targets = self._db.SelectTargets(options='imu')
    self.assertEqual(len(targets), 3)
    targets = self._db.SelectTargets(options='imu novatel')
    self.assertEqual(len(targets), 1)
    targets = self._db.SelectTargets(options='septentrio novatel')
    self.assertEqual(len(targets), 0)

    # Query for busy.
    targets = self._db.SelectTargets(busy=False)
    self.assertEqual(len(targets), 3)
    targets = self._db.SelectTargets(busy=True)
    self.assertEqual(len(targets), 0)

    # Start one node.
    self._db.StartTestNode(1, 1)
    targets = self._db.SelectTargets(busy=False)
    self.assertEqual(len(targets), 2)
    targets = self._db.SelectTargets(busy=True)
    self.assertEqual(len(targets), 1)

    # Combined queries.
    targets = self._db.SelectTargets(board='aio', options='novatel')
    self.assertEqual(len(targets), 1)
    targets = self._db.SelectTargets(board='fc', options='novatel')
    self.assertEqual(len(targets), 0)
    targets = self._db.SelectTargets(tms570='kAioNodeFcB',
                                     q7='kAioNodeControllerB')
    self.assertEqual(len(targets), 1)
    targets = self._db.SelectTargets(tms570='kAioNodeFcB',
                                     q7='kAioNodeControllerC')
    self.assertEqual(len(targets), 0)

  def testSelectTargetIds(self):
    self.InsertRelayModules()

    self._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcA',
        'AioNodeQ7': 'kAioNodeFlightControllerA',
        'BoardName': 'aio',
        'CarrierName': 'fc',
        'HardwareOptions': 'imu novatel',
        'RelayModuleId': 1,
        'RelayPort': 1,
    })
    self._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcB',
        'AioNodeQ7': 'kAioNodeFlightControllerB',
        'BoardName': 'aio',
        'CarrierName': 'fc',
        'HardwareOptions': 'imu septentrio',
        'RelayModuleId': 1,
        'RelayPort': 1,
    })
    self._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcC',
        'AioNodeQ7': 'kAioNodeFlightControllerC',
        'BoardName': 'fc',
        'HardwareOptions': 'imu hemisphere',
        'RelayModuleId': 1,
        'RelayPort': 1,
    })
    target_ids = self._db.SelectTargetIds(board='aio')
    self.assertEqual(len(target_ids), 2)
    target_ids = self._db.SelectTargetIds(board='fc')
    self.assertEqual(len(target_ids), 1)
    target_ids = self._db.SelectTargetIds(options='imu')
    self.assertEqual(len(target_ids), 3)
    target_ids = self._db.SelectTargetIds(options='imu novatel')
    self.assertEqual(len(target_ids), 1)

  def testSelectTargetIdsByTest(self):
    self.InsertRelayModules()

    self._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcA',
        'AioNodeQ7': 'kAioNodeFlightControllerA',
        'BoardName': 'aio',
        'CarrierName': 'fc',
        'HardwareOptions': 'imu novatel',
        'RelayModuleId': 1,
        'RelayPort': 1,
    })
    self._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcB',
        'AioNodeQ7': 'kAioNodeFlightControllerB',
        'BoardName': 'aio',
        'CarrierName': 'fc',
        'HardwareOptions': 'imu septentrio',
        'RelayModuleId': 1,
        'RelayPort': 1,
    })
    self._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcC',
        'AioNodeQ7': 'kAioNodeFlightControllerC',
        'BoardName': 'fc',
        'HardwareOptions': 'imu hemisphere',
        'RelayModuleId': 1,
        'RelayPort': 1,
    })
    node = {
        'BoardName': 'fc',
        'CarrierName': None,
        'HardwareOptions': None
    }
    target_ids = self._db.SelectTargetIdsByTest(node)
    self.assertEqual(len(target_ids), 1)

  def testSelectTestNodes(self):
    self.InsertRelayModules()

    self._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcA',
        'AioNodeQ7': 'kAioNodeControllerA',
        'BoardName': 'aio',
        'CarrierName': 'fc',
        'HardwareOptions': 'imu novatel',
        'RelayModuleId': 1,
        'RelayPort': 1,
    })
    test1 = [{'HardwareOptions': 'novatel'},
             {'HardwareOptions': 'septentrio'}]
    self._db.InsertTest(test1, 1)
    test2 = [{'HardwareOptions': 'hemisphere'}]
    self._db.InsertTest(test2, 2)

    # Query for process id.
    tests = self._db.SelectTestNodes(pid=1)
    self.assertEqual(len(tests), 2)
    tests = self._db.SelectTestNodes(pid=2)
    self.assertEqual(len(tests), 1)
    tests = self._db.SelectTestNodes(pid=3)
    self.assertEqual(len(tests), 0)

    # Query for running.
    tests = self._db.SelectTestNodes(running=False)
    self.assertEqual(len(tests), 3)
    tests = self._db.SelectTestNodes(running=True)
    self.assertEqual(len(tests), 0)

    # Start one node.
    self._db.StartTestNode(1, 1)
    tests = self._db.SelectTestNodes(running=True)
    self.assertEqual(len(tests), 1)

    # Combined queries.
    tests = self._db.SelectTestNodes(pid=1, running=False)
    self.assertEqual(len(tests), 1)
    tests = self._db.SelectTestNodes(pid=1, running=True)
    self.assertEqual(len(tests), 1)

  def testSelectTests(self):
    test1 = [{'HardwareOptions': 'novatel'},
             {'HardwareOptions': 'septentrio'}]
    self._db.InsertTest(test1, 1)
    test2 = [{'HardwareOptions': 'hemisphere'}]
    self._db.InsertTest(test2, 2)

    tests = self._db.SelectTests(running=True)
    self.assertFalse(tests)
    tests = self._db.SelectTests(running=False)
    self.assertEqual(len(tests), 2)


if __name__ == '__main__':
  unittest.main()
