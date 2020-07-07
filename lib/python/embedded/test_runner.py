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

"""Execute software tests on test fixture.

This module provides a client-side interface to the test fixture. It inserts
test request records into the 'test' and 'node' database tables to request
access to a particular hardware configuration. When granted by the server,
the client executes the tests on the hardware.
"""

import logging
import os
import sys
import tempfile
import time

import gflags
import makani
from makani.avionics.common import aio
from makani.lib.python import string_util
from makani.lib.python.embedded import hardware_db
import pexpect
import yaml


logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s test=%(TestId)d node=%(TestNodeId)d] %(message)s')


class CommandWorker(object):
  """Worker to execute a program in the background and poll for completion."""

  def __init__(self, log_extra, command, **pexpect_kwargs):
    self._log_extra = {'TestId': -1, 'TestNodeId': -1}
    self._log_extra.update(log_extra)
    self._command = command
    self._pexpect_kwargs = pexpect_kwargs
    self._timeout = pexpect_kwargs['timeout']
    self._logfile = tempfile.TemporaryFile()
    self._child = None

    self._cmdline = self._command
    if 'args' in self._pexpect_kwargs:
      self._cmdline += ' ' + ' '.join(self._pexpect_kwargs['args'])

  def Execute(self):
    logging.info('%s (timeout=%d)', self._cmdline, self._timeout,
                 extra=self._log_extra)
    self._child = pexpect.spawn(command=self._command, logfile=self._logfile,
                                **self._pexpect_kwargs)
    self._start_time = time.time()

  def _PrintOutput(self, return_code):
    """Print command output from pexpect."""
    if return_code is None:
      return_code = -1
    if return_code == 0:
      log_level = logging.INFO
    else:
      log_level = logging.ERROR
    self._logfile.seek(0)
    output = self._logfile.read()
    if output:
      output = '\n' + output
    logging.log(log_level, 'Command output (return=%d):\n%s%s', return_code,
                self._cmdline, output, extra=self._log_extra)

  def _Close(self):
    return_code = -1
    if self._child:
      if self._child.isalive():
        logging.error('Command terminated prematurely.', extra=self._log_extra)
      self._child.close(force=True)
      return_code = self._child.exitstatus
    return return_code

  def Close(self):
    return_code = self._Close()
    self._PrintOutput(return_code)
    return return_code == 0

  def Iterate(self):
    if not self._child:
      self.Execute()
    reason = self._child.expect([pexpect.TIMEOUT, pexpect.EOF], timeout=0)
    run_time = time.time() - self._start_time
    timed_out = run_time >= self._timeout

    if reason == 0 and timed_out:
      logging.error('Command timed out.', extra=self._log_extra)
    return reason == 0 and not timed_out


class JobWorker(object):
  """Worker to execute jobs in serial."""

  def __init__(self):
    self._has_error = False
    self.jobs = []

  def Close(self):
    self.jobs = []
    return not self._has_error

  def Iterate(self):
    if self.jobs and not self.jobs[0].Iterate():
      self._has_error |= not self.jobs[0].Close()
      self.jobs.pop(0)
    return not self._has_error and self.jobs


class Tms570Worker(JobWorker):
  """Worker to execute jobs on TMS570 devices."""

  def __init__(self, config, target, paths):
    super(Tms570Worker, self).__init__()

    logging.info('Running %s on %s (TMS570)', config['FirmwareTms570'],
                 target['AioNodeTms570'], extra=config)

    # Use TMS570's bootloader for programming.
    bootloader_client = os.path.join(
        makani.HOME, 'avionics', 'bootloader', 'bootloader_client')
    snake_name = string_util.CamelToSnake(
        aio.aio_node_helper.ShortName(target['AioNodeTms570']))

    # Program TMS570-side test application.
    application = os.path.join(paths['tms570_bin'], config['FirmwareTms570'])
    self.jobs.append(CommandWorker(
        log_extra=config, command=bootloader_client,
        args=['--target=' + snake_name, application], timeout=30))

    # Program calibration parameters.
    if config['CalibParams']:
      params = os.path.join(paths['tms570_bin'], config['CalibParams'])
      self.jobs.append(CommandWorker(
          log_extra=config, command=bootloader_client,
          args=['--target=' + snake_name, params, '--calib'], timeout=30))

    # Program configuration parameters.
    if config['ConfigParams']:
      params = os.path.join(paths['tms570_bin'], config['ConfigParams'])
      self.jobs.append(CommandWorker(
          log_extra=config, command=bootloader_client,
          args=['--target=' + snake_name, params], timeout=30))


class Q7Worker(JobWorker):
  """Worker to execute jobs on Q7 devices."""

  def __init__(self, config, target, paths):
    super(Q7Worker, self).__init__()

    logging.info('Running %s on %s (Q7)', config['FirmwareQ7'],
                 target['AioNodeQ7'], extra=config)

    # Program Q7-side test application.
    application = os.path.join(paths['q7_bin'], config['FirmwareQ7'])
    aio_node = aio.aio_node_helper.Value(target['AioNodeQ7'])
    ip_address = aio.AioNodeToIpAddressString(aio_node)
    q7_address = 'root@' + ip_address
    self.jobs.append(CommandWorker(
        log_extra=config,
        command='scp', args=[application, q7_address + ':~'],
        timeout=30))

    # Run Q7-side test application.
    q7_cmdline = '~/' + os.path.basename(application)
    self.jobs.append(CommandWorker(
        log_extra=config,
        command='ssh', args=[q7_address, '\'' + q7_cmdline + '\''],
        timeout=config['Timeout']))


class HostWorker(JobWorker):
  """Worker to execute jobs on host computer."""

  def __init__(self, config, target):
    super(HostWorker, self).__init__()

    logging.info(
        'Running "%s" on host',
        ' '.join([config['HostApplication']] + [config['HostArguments']]),
        extra=config)

    # Place configuration and target into shell environment.
    env = {}
    env.update({string_util.CamelToSnake(k).upper(): str(v)
                for k, v in config.iteritems() if v is not None})
    env.update({string_util.CamelToSnake(k).upper(): ''
                for k, v in config.iteritems() if v is None})
    env.update({string_util.CamelToSnake(k).upper(): str(v)
                for k, v in target.iteritems() if v is not None})
    env.update({string_util.CamelToSnake(k).upper(): ''
                for k, v in target.iteritems() if v is None})

    # Temporarily set environment to calculate command line for logging in
    # CommandWorker.
    old = dict(os.environ)
    os.environ.update(env)
    args = os.path.expandvars(config['HostArguments'])
    os.environ.clear()
    os.environ.update(old)

    # Run host application.
    application = os.path.join(makani.HOME, config['HostApplication'])
    self.jobs.append(CommandWorker(
        log_extra=config,
        command=application, args=args.split(), env=env,
        timeout=config['Timeout']))


class NodeWorker(object):
  """Worker to coordinate jobs on a single hardware node."""

  def __init__(self, config, target, paths):
    self._has_error = False
    self._iterating_workers = []

    # Execute these workers first.
    self._bootload_workers = []
    if config['FirmwareTms570']:
      self._bootload_workers.append(Tms570Worker(config, target, paths))

    # Execute these workers second (in parallel).
    self._app_workers = []
    if config['FirmwareQ7']:
      self._app_workers.append(Q7Worker(config, target, paths))
    if config['HostApplication']:
      self._app_workers.append(HostWorker(config, target))

  def Close(self):
    for worker in self._iterating_workers:
      self._has_error |= not worker.Close()
    return not (self._has_error or self._bootload_workers or self._app_workers)

  def Iterate(self):
    """Process all workers in parallel. Iterate while return is True."""
    for worker in self._iterating_workers[:]:
      if not worker.Iterate():
        self._has_error |= not worker.Close()
        self._iterating_workers.remove(worker)

    # Iterate bootload workers first.
    if not self._iterating_workers:
      self._iterating_workers = self._bootload_workers
      self._bootload_workers = []
    if not self._iterating_workers:
      self._iterating_workers = self._app_workers
      self._app_workers = []

    return not self._has_error and self._iterating_workers


class TestRunner(object):
  """Coordinate with test fixture to execute software tests on hardware."""

  def __init__(self, paths, database):
    """Instantiate test runner.

    Args:
      paths: Dictionary that specifies q7_bin and tms570_bin paths.
      database: HardwareDatabase file path or object.
    """
    self._paths = paths
    if isinstance(database, hardware_db.HardwareDatabase):
      self._db = database
    else:
      self._db = hardware_db.HardwareDatabase(database)
    self._pid = os.getpid()
    self._workers = {}
    self._has_error = False
    self._config = {}

  def InsertTest(self, test_config):
    """Insert test into database."""
    test = []
    for node_config in test_config:
      node = {
          'FirmwareTms570': None,   # TMS570 test application binary.
          'FirmwareQ7': None,       # Q7 test application binary.
          'HostApplication': None,  # Host application binary.
          'HostArguments': '',      # Host application command line arguments.
          'CalibParams': None,      # TMS570 calibration parameters.
          'ConfigParams': None,     # TMS570 configuration parameters.
          'Timeout': 0,             # Test timeout [s].
          'BoardName': None,        # Required board name.
          'CarrierName': None,      # Required carrier board name.
          'HardwareOptions': None   # Required hardware options.
      }
      node.update(node_config)

      # Individual TMS570 applications require execution of the test_command
      # host command to coordinate test execution. If the configuration does
      # not specify a host application, default to test_command. For more
      # complex tests that do not require test_command (e.g., a HITL), set
      # HostApplication to '' or the sim.
      if node['FirmwareTms570'] and node['HostApplication'] is None:
        node['HostApplication'] = 'avionics/linux/test_command'
        node['HostArguments'] = ('--node=${AIO_NODE_TMS570} '
                                 '--timeout=${TIMEOUT} --logtostderr')

      test.append(node)
    config = self._db.InsertTest(test, self._pid)
    self._config.update(config)

  def LoadTests(self, tests):
    """Load a list of test configs into database."""
    for test_config in tests:
      self.InsertTest(test_config)

  def Iterate(self):
    """Iterate test fixture until completion."""

    # Select all nodes controlled by this process.
    nodes = self._db.SelectTestNodes(pid=self._pid)
    # Sub-select running nodes.
    for node in [n for n in nodes if n['TargetId'] is not None]:
      # Prevent server from reallocating our hardware.
      self._db.KeepTestsAlive(self._pid)
      node_id = node['TestNodeId']
      # New node entered active state.
      if node_id not in self._workers:
        target = self._db.SelectTargetByNodeId(node_id)
        config = self._config[node_id]
        config.update(node)
        self._workers[node_id] = NodeWorker(config, target, self._paths)
      # Iterate nodes until completion.
      worker = self._workers[node_id]
      if not worker.Iterate():
        self._has_error |= not worker.Close()
        self._db.DeleteTestNode(node)
    # Prevent server from reallocating our hardware.
    self._db.KeepTestsAlive(self._pid)
    # Handle when the server removes nodes before processing. This situation
    # may occur if the server cannot execute all tests because of insufficient
    # hardware.
    not_run = set(self._config.keys()) - set(self._workers.keys())
    if not nodes and not_run:
      for node_id in not_run:
        extra = self._config[node_id]
        extra['TargetId'] = -1
        extra['TestId'] = -1
        extra['TestNodeId'] = -1
        logging.error('Unable to execute test %s.', self._config[node_id],
                      extra=extra)
      self._has_error = True
    # Continue while nodes pending.
    return bool(nodes)

  def GetError(self):
    return self._has_error

  def Run(self, tests):
    self.LoadTests(tests)
    while self.Iterate():
      time.sleep(1.0)
    return not self._has_error


def ParseFlags(argv):
  """Define and parse gflags parameters."""
  gflags.DEFINE_string('database', 'hardware.db',
                       'Full path to shared hardware database file.')
  gflags.DEFINE_string('config', None,
                       'Full path to test configuration file.')
  gflags.DEFINE_string('q7_bin', None,
                       'Full path to Q7 binary directory.')
  gflags.DEFINE_string('tms570_bin', None,
                       'Full path to TMS570 binary directory.')
  gflags.MarkFlagAsRequired('config')
  gflags.MarkFlagAsRequired('q7_bin')
  gflags.MarkFlagAsRequired('tms570_bin')
  return gflags.FLAGS(argv)


def LoadTestsFromYaml(yaml_file=None):
  """Load tests from YAML file."""
  if not yaml_file:
    yaml_file = gflags.FLAGS.config
  tests = []
  with open(yaml_file, 'r') as f:
    for module in yaml.full_load(f):
      test_yaml = module['test']
      test = [hardware_db.TranslateYamlStyleToSqlStyle(n) for n in test_yaml]
      tests.append(test)
  return tests


def GetQ7Targets(tests):
  targets = []
  for test in tests:
    targets += [node['FirmwareQ7'] for node in test if 'FirmwareQ7' in node]
  return list(set(targets))


def GetTms570Targets(tests):
  targets = []
  for test in tests:
    targets += [node['FirmwareTms570'] for node in test
                if 'FirmwareTms570' in node]
    targets += [node['CalibParams'] for node in test if 'CalibParams' in node]
    targets += [node['ConfigParams'] for node in test if 'ConfigParams' in node]
  return list(set(targets))


def Run(tests):
  paths = {
      'q7_bin': gflags.FLAGS.q7_bin,
      'tms570_bin': gflags.FLAGS.tms570_bin}
  runner = TestRunner(paths=paths, database=gflags.FLAGS.database)
  runner.Run(tests=tests)
  return not runner.GetError()


def main(argv):
  try:
    argv = ParseFlags(argv)
  except gflags.FlagsError, e:
    print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], gflags.FLAGS)
    sys.exit(1)
  sys.exit(not Run(tests=LoadTestsFromYaml()))


if __name__ == '__main__':
  main(sys.argv)
