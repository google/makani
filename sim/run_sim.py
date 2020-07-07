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

"""Runs the simulator and its many friends.

Usage:
  run_sim <flags> -- <extra_sim_args>.
"""

import atexit
import contextlib
import datetime
import json
import os
import signal
import subprocess
import sys
import textwrap
import time

import gflags
import makani
from makani.lib.bazel import bazel_util
from makani.lib.python import dict_util
from makani.lib.python import os_util
from makani.lib.python import turbsim_util
from makani.lib.python import wing_flag

wing_flag.AppeaseLintWhenImportingFlagOnly()
makani.SetRunfilesDirFromBinaryPath()

# b/147502899: Workaround - when using the all_params flag, also pass
#   -w ''
# so that it doesn't detect a params incompatibility.
gflags.DEFINE_string('all_params', None,
                     'JSON file that includes all parameters.', short_name='p')
gflags.DEFINE_bool('async', False,
                   'Whether to run asynchronously.')
gflags.DEFINE_bool('callgrind_controller', False,
                   'Whether to run the controller under callgrind for '
                   'profiling.')
gflags.DEFINE_bool('callgrind_sim', False,
                   'Whether to run the sim under callgrind for profiling.')
gflags.DEFINE_bool('flightgear', False,
                   'Whether to run the Flight Gear interface. (Flight Gear '
                   'must be started separately.',
                   short_name='f')
gflags.DEFINE_string('interface', None,
                     'Multicast interface. This may not be loopback during '
                     'some HITL modes.',
                     short_name='i')
gflags.DEFINE_bool('show_cues', True,
                   'Show visual cues such as the flight path and flight mode.')
gflags.DEFINE_bool('show_landing_zone', False, 'Show landing zone.')

# Determines the joystick type that is used by the simulator.
#
# If --async, this is only relevant if use_software_joystick==True in
# config/m600/hitl.py. Note that use_software_joystick really refers to the
# simulator's joystick; with this option enabled, --joystick=hardware will
# forward hardware joystick commands through the simulator.
gflags.DEFINE_enum('joystick', None,
                   ['programmed', 'software', 'hardware'],
                   'Which type of joystick to use with the simulator.',
                   short_name='j')

gflags.DEFINE_string('flight_plan', None, 'Flight plan.')

gflags.DEFINE_bool('kill', False,
                   'Don''t run anything, but kill all sim-related processes.',
                   short_name='k')
gflags.DEFINE_bool('log', False,
                   'Whether to log.', short_name='l')
gflags.DEFINE_string('log_file', '',
                     'Generate a symlink to the log file with this name.')
gflags.DEFINE_multistring('monitor', None,
                          'Monitors to run.', short_name='m')
gflags.DEFINE_multistring('webmonitor', None,
                          'Web Monitors to run.', short_name='M')
gflags.DEFINE_bool('monitor_sim', False,
                   'Observe simulator data in the web monitor.', short_name='S')
gflags.DEFINE_integer('num_controllers', 1,
                      'Number of controllers to run.')
gflags.DEFINE_string('overrides', None,
                     'JSON string of override parameters.', short_name='o')
gflags.DEFINE_bool('load_state', False,
                   'Whether to load the sim and controller state from a '
                   'previous run.')
gflags.DEFINE_float('time', None,
                    'Duration of simulator run.', short_name='t')
gflags.DEFINE_float('save_state_time', None,
                    'Time at which to save the state.')
gflags.DEFINE_bool('valgrind_controller', False,
                   'Runs Valgrind on the controller to check for memory bugs.')
gflags.DEFINE_bool('vis', True,
                   'Whether to run the visualizer.')
gflags.DEFINE_bool('with_online_turbsim_databases', True,
                   'Searches for and downloads appropriate TurbSim wind '
                   'databases, and overrides wind_model to '
                   'kWindModelDatabase (unless it already had an override). '
                   'Set false for full offline capability and compatibility '
                   'with any Dryden wind case.')
gflags.DEFINE_bool('gs_estimator', True,
                   'Whether to run the ground station estimator.')
gflags.DEFINE_float('comms_timeout_sec', None,
                    'Timeout [s] for blocking during synchronous execution.')

FLAGS = gflags.FLAGS


def _KillSimRelatedProcesses():
  mbash_path = os.path.join(makani.HOME, 'lib/scripts/mbash.sh')
  subprocess.check_call([
      'bash', '-c',
      'source %s && mbash::kill_all_makani_procs' % mbash_path])


class _Process(object):
  """Context manager for running a process.

  Also reports required build targets via Targets().
  """

  def __init__(self, binary, extra_args=None, wait_for_completion=False,
               use_valgrind=False, use_callgrind=False, python=False):
    self._binary = binary
    self._extra_args = extra_args if extra_args else []
    self._wait_for_completion = wait_for_completion

    assert not (use_valgrind and use_callgrind)
    self._use_valgrind = use_valgrind
    self._use_callgrind = use_callgrind

    self._python = python

  def __enter__(self):
    args = []
    if self._use_valgrind:
      args += [
          'valgrind', '--quiet',
          '--log-file=' + os.path.join(
              'logs', 'valgrind-%s.out' % self._binary.replace('/', '-')),
      ]
    elif self._use_callgrind:
      args += [
          'valgrind', '--quiet',
          '--tool=callgrind',
          '--callgrind-out-file=' + os.path.join(
              'logs', 'callgrind-%s.out' % self._binary.replace('/', '-')),
      ]

    binary = os.path.join(makani.HOME, self._binary)

    args += [binary] + self._extra_args

    self._process = subprocess.Popen(args)

  def __exit__(self, *unused_args):
    if self._wait_for_completion:
      self._process.communicate()
      assert self._process.returncode == 0, (
          'Process for binary "%s" terminated abnormally.' % self._binary)
    else:
      self._process.send_signal(signal.SIGINT)


class _WebMonitor(object):
  """Context manager for running the web monitor."""

  def __init__(self, monitor_types, monitor_sim):
    self._monitor_types = monitor_types
    self._run_mode = 'sim' if monitor_sim else 'start'

  def __enter__(self):
    for monitor_type in self._monitor_types:
      subprocess.Popen([
          os.path.join(makani.HOME, 'gs/monitor2/webmonitor_launcher'),
          self._run_mode, monitor_type])

  def __exit__(self, *unused_args):
    subprocess.Popen([
        os.path.join(makani.HOME, 'gs/monitor2/webmonitor_launcher'), 'stop'])


class _Logger(object):
  """Context manager for running "the logger".

  Exposes the same interface as Process.
  """

  def __init__(self, interface, extra_args=None):
    self._interface = interface
    self._extra_args = extra_args if extra_args else []

  def __enter__(self):
    self._pcap_file = 'logs/%s.pcap' % datetime.datetime.now().strftime(
        '%Y%m%d-%H%M%S')
    self._tcpdump_process = subprocess.Popen(
        [os.path.join(makani.HOME, 'lib/scripts/sim_tcpdump'), self._pcap_file,
         self._interface])
    time.sleep(2)  # Give the recorder a chance to start
    assert self._tcpdump_process.poll() is None, (
        "Couldn't start tcpdump. Check permissions or re-run install_packages.")

  def _MakeLink(self, source, dest):
    if os.path.exists(source):
      if os.path.islink(dest):
        os.remove(dest)
      os.symlink(os.path.realpath(source), dest)

  def __exit__(self, *unused_args):
    self._tcpdump_process.send_signal(signal.SIGINT)
    self._tcpdump_process.wait()

    h5_file = self._pcap_file.replace('.pcap', '.h5')
    subprocess.check_call([os.path.join(makani.HOME,
                                        'lib/pcap_to_hdf5/pcap_to_hdf5'),
                           self._pcap_file,
                           '--output_file', h5_file] + self._extra_args)

    with os_util.ChangeDir('logs'):
      self._MakeLink('last_z2.h5', 'last_z3.h5')
      self._MakeLink('last_z1.h5', 'last_z2.h5')
      self._MakeLink('last.h5', 'last_z1.h5')
      self._MakeLink(os.path.basename(h5_file), 'last.h5')
      if FLAGS.log_file:
        os.symlink(os.path.basename(h5_file), FLAGS.log_file)


def PrepareOverrides():
  """Prepares overrides from flag values, and checks for conflicting options.

  Returns:
    Name of the all_params file, or None if there are no overrides.
  """

  override_flags = ('wing_model', 'overrides', 'joystick',
                    'flight_plan', 'callgrind_sim')

  if FLAGS.all_params:
    for flag in override_flags:
      if FLAGS.get(flag, None):
        print 'ERROR: --all_params is not compatible with --%s.' % flag
        if flag == 'wing_model':
          print(('To use the all_params flag, you must also explicitly set '
                 'the wing_model flag to an empty string: '
                 "-w ''"))
        sys.exit(1)

    if FLAGS.async:
      print 'WARNING: --all_params file does not affect the async controller.'

    all_params_path = FLAGS.all_params

    # Download the turbsim file, as needed.
    if FLAGS.with_online_turbsim_databases:
      PrepareTurbsimDatabase(all_params_path, None)

    return all_params_path

  # The with_online_turbsim_databases flag is compatible with all_params (so
  # that a database specified there can be downloaded), so it is not included
  # in override_flags; but if true, the wind_model may need to be overridden.
  if any(FLAGS.get(f, None) for f in override_flags) or (
      FLAGS.with_online_turbsim_databases):
    overrides = json.loads(FLAGS.overrides) if FLAGS.overrides else {}

    if FLAGS.flight_plan:
      overrides = dict_util.MergeNestedDicts(
          overrides,
          {'system': {'flight_plan': FLAGS.flight_plan}})

    if FLAGS.joystick:
      overrides = dict_util.MergeNestedDicts(
          overrides,
          {'sim': {'joystick_sim': {
              'joystick_type': FLAGS.joystick.capitalize()
          }}})

    # We should use the wakeless hover model when running with
    # callgrind. Otherwise, loading of the hover databases takes far too long
    # for this option to be useful.
    if FLAGS.callgrind_sim:
      try:
        aero_models = dict_util.GetByPath(overrides,
                                          ['sim', 'aero_sim', 'aero_models'])
      except KeyError:
        using_wakeless_model = False
      else:
        using_wakeless_model = (aero_models['large_deflection_aero_model']
                                == 'flat plate')
      if not using_wakeless_model:
        print textwrap.dedent("""\
            Loading hover databases under callgrind is very slow. You should
            probably use the wake-free model so there's only one database to
            load. Run with
                -o '{"sim": {"aero_sim": {"aero_models": {"large_deflection_aero_model": "flat plate"}}}}'
            to do this.""")

    if overrides and FLAGS.async:
      if 'control' in overrides or 'system' in overrides:
        print 'ERROR: Cannot override control or system params with --async.'
        sys.exit(1)

    # Set the wind_model to default to database, unless otherwise requested.
    # TODO: This check is a bit more complicated than desired because
    # run_sim does not have access to sim_types. Refactor so this is possible?
    if overrides:
      wind_database_name_is_override = dict_util.IsNestedField(
          overrides, ['sim', 'phys_sim', 'wind_database', 'name'])
      wind_model_is_override = dict_util.IsNestedField(
          overrides, ['sim', 'phys_sim', 'wind_model'])
    else:
      wind_database_name_is_override = False
      wind_model_is_override = False
    if (not wind_model_is_override) and (
        wind_database_name_is_override or FLAGS.with_online_turbsim_databases):
      overrides = dict_util.MergeNestedDicts(
          overrides,
          {'sim': {'phys_sim': {
              'wind_model': 'kWindModelDatabase'
          }}})

    if not os.path.exists('tmp'):
      os.mkdir('tmp')

    subprocess.check_call([os.path.join(makani.HOME, 'config/write_params'),
                           '--type', 'json',
                           '--wing_model', FLAGS.wing_model.lower(),
                           '--overrides', json.dumps(overrides),
                           '--input_file', 'config/common/all_params.py',
                           '--output_file', 'tmp/all_params.json'])
    all_params_path = 'tmp/all_params.json'

    # Select and/or download the turbsim file, as needed.
    if FLAGS.with_online_turbsim_databases:
      PrepareTurbsimDatabase(all_params_path, overrides)

    return all_params_path

  return None


def PrepareTurbsimDatabase(all_params_path, overrides):
  """Download the TurbSim database, if specified and needed."""
  with open(all_params_path, 'r') as params_path:
    all_params = json.load(params_path)

  # Check if a wind_database has been specified.
  database_file = os.path.basename(
      all_params['sim']['phys_sim']['wind_database']['name'])
  if not database_file:
    return

  # Check if the database name follows the TurbSim naming convention.
  if not turbsim_util.CheckTurbsimFileName(database_file):
    # No database will be downloaded - the file must already be present locally
    # and the full local path passed in the override.
    return

  # Verify that the wind settings in the overrides are consistent and update
  # the wind_database with an appropriate condition from the same TurbSim set
  # as needed.
  if overrides:
    wind_speed_is_overridden = dict_util.IsNestedField(
        overrides, ['sim', 'phys_sim', 'wind_speed'])
    wind_shear_is_overridden = dict_util.IsNestedField(
        overrides, ['sim', 'phys_sim', 'wind_shear_exponent'])
    wind_database_name_is_overridden = dict_util.IsNestedField(
        overrides, ['sim', 'phys_sim', 'wind_database', 'name'])

    if wind_database_name_is_overridden:
      # Check if it's consistent with other wind params, to within the
      # precision of the convention used on the TurbSim file name.
      database_file = os.path.basename(
          all_params['sim']['phys_sim']['wind_database']['name'])
      database_speed = turbsim_util.GetWindSpeedFromName(database_file)
      database_shear = turbsim_util.GetWindShearFromName(database_file)
      speeds_match = abs(database_speed -
                         all_params['sim']['phys_sim']['wind_speed']) < 1e-1
      shears_match = (
          abs(database_shear -
              all_params['sim']['phys_sim']['wind_shear_exponent']) < 1e-2)

      if (not speeds_match) or (not shears_match):
        # Check that no other wind conditions were passed as explicit overrides.
        assert ((not wind_speed_is_overridden) and
                (not wind_shear_is_overridden)), (
                    'Ambiguous wind conditions: Conflicting overrides passed '
                    'for both database name and wind speed/shear. Either '
                    'override the wind conditions and let run_sim select an '
                    'apporopriate database from the default folder, '
                    'override only the database name, or make sure the '
                    'settings are consistent.')
        # Override the wind conditions in params file to match so users can
        # query those fields in the logs or configs and get correct values.
        # Note: This means that if a user overrides both the database name and
        # the wind model to Dryden, then the default wind condition would be
        # overridden based on the wind database name conditions, even though
        # that file would now not be used. Run_sim is not set up to import
        # sim_types easily, so this condition is difficult to check, but it is
        # not an expected use case.
        all_params['sim']['phys_sim']['wind_speed'] = database_speed
        all_params['sim']['phys_sim']['wind_shear_exponent'] = database_shear
    elif wind_speed_is_overridden or wind_shear_is_overridden:
      # Select a new database to match.
      # TODO: Figure out if we want to support automatically selecting
      # an appropriate database from a TurbSim folder other than the default.
      # Otherwise, users that want to run a sim with a database from a different
      # folder will need to manually select an appropriate TurbSim
      # database from go/makani-turbsim-databases and pass the filename as an
      # override - see instructions and more information at go/makani-turbsim.
      online_folder = turbsim_util.GetOnlineFolder(database_file)
      turbsim_database_selector = turbsim_util.TurbSimDatabaseSelector(
          online_folder, all_params)
      specific_override, _ = (
          turbsim_database_selector.GetSpecificOverride(all_params, 0))
      database_file = os.path.basename(
          specific_override['sim']['phys_sim']['wind_database']['name'])

  local_path = os.path.join('tmp/', database_file)
  turbsim_util.DownloadDatabase(database_file, local_path)

  # Update wind database path in config file to downloaded location
  all_params['sim']['phys_sim']['wind_database']['name'] = (
      os.path.abspath(local_path))
  with open(all_params_path, 'w') as params_path:
    json.dump(all_params, params_path)


def main(argv):
  # Parse flags. Any args that remain after parsing (items following a bare
  # "--") will be forwarded to the sim.
  try:
    argv = FLAGS(argv)
    extra_sim_args = argv[1:]
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  # Kill all sim-related processes, and do the same again when the script exits.
  _KillSimRelatedProcesses()
  if FLAGS.kill:
    return
  atexit.register(_KillSimRelatedProcesses)

  aio_network_config = '/etc/makani/aio_network.conf'
  interface = 'lo'
  if FLAGS.interface:
    interface = FLAGS.interface
  elif os.path.exists(aio_network_config):
    with open(aio_network_config, 'r') as f:
      for line in f:
        line = line.strip()
        if line.startswith('AIO_NETWORK_INTERFACE='):
          interface = line[len('AIO_NETWORK_INTERFACE='):]

  try:
    subprocess.check_call([
        'bash', '-c',
        'source lib/scripts/mbash.sh && mbash::check_multicast_route ' +
        interface])
  except subprocess.CalledProcessError:
    print ('Multicast has not been configured for interface "%s".\n'
           'If trying to run a desktop simulation, please read '
           'lib/scripts/install/README_local_multicast.'
           % interface)
    sys.exit(1)

  all_params_path = PrepareOverrides()

  # Gather a list of all supporting processes. These will be launched in the
  # order they are added, and cleaned up in reverse order.
  processes = []
  if FLAGS.flightgear:
    processes += [_Process('lib/flight_gear/flight_gear_interface')]

  # TODO: Sanity-check against kSimOptProgrammedJoystick and the
  # use_software_joystick HITL option.
  if FLAGS.joystick == 'software':
    processes += [_Process('lib/joystick/joystick', python=True)]

  if FLAGS.monitor:
    extra_args = []
    if all_params_path:
      extra_args += ['--all_params', all_params_path]
    processes += [_Process('gs/aio_snapshot/aio_snapshot')]
    processes += [_Process('gs/monitor/monitor', extra_args + [monitor_type])
                  for monitor_type in FLAGS.monitor]

  if FLAGS.webmonitor:
    processes.append(_WebMonitor(FLAGS.webmonitor, FLAGS.monitor_sim))

  if FLAGS.log:
    extra_args = []
    if all_params_path:
      extra_args += ['--all_params', all_params_path]
    processes += [_Logger(interface, extra_args=extra_args)]

  if FLAGS.vis and os.environ.get('DISPLAY', False):
    extra_args = [FLAGS.FlagDict()[f].Serialize()
                  for f in ('show_cues', 'show_landing_zone')]
    if all_params_path:
      extra_args += ['--all_params', all_params_path]
    processes += [_Process('vis/vis', extra_args=extra_args)]

  controller_path = ('control/hitl_controller' if FLAGS.async
                     else 'control/sim_controller')
  for i in range(FLAGS.num_controllers):
    extra_args = ['--controller_label=%d' % i]
    if all_params_path:
      extra_args += ['--all_params', all_params_path]

    if FLAGS.comms_timeout_sec is not None:
      extra_args += ['--comms_timeout_sec=%f' % FLAGS.comms_timeout_sec]

    wait_for_completion = FLAGS.time and FLAGS.time > 0.0
    processes += [_Process(controller_path, extra_args=extra_args,
                           wait_for_completion=wait_for_completion,
                           use_valgrind=FLAGS.valgrind_controller,
                           use_callgrind=FLAGS.callgrind_controller)]

  if FLAGS.gs_estimator:
    gs_extra_args = []
    if all_params_path:
      gs_extra_args += ['--all_params', all_params_path]
    if FLAGS.comms_timeout_sec is not None:
      gs_extra_args += ['--comms_timeout_sec=%f' % FLAGS.comms_timeout_sec]
    ground_path = ('control/hitl_ground_estimator' if FLAGS.async
                   else 'control/sim_ground_estimator')
    processes += [_Process(ground_path, extra_args=gs_extra_args)]

  # Run everything!
  with contextlib.nested(*processes):
    # Run the simulator, taking ctrl+C as its termination signal.
    sim_args = ['--num_controllers=%d' % FLAGS.num_controllers]
    sim_args += ['--gs_estimator=%r' % FLAGS.gs_estimator]
    sim_args += extra_sim_args
    if FLAGS.time is not None:
      sim_args += ['--time=%g' % FLAGS.time]
    if FLAGS.load_state:
      sim_args += ['--load_state']
    if FLAGS.save_state_time is not None:
      sim_args += ['--save_state_time', str(FLAGS.save_state_time)]
    if FLAGS.async:
      sim_args += ['--async']
    if all_params_path:
      sim_args += ['--all_params', all_params_path]

    if FLAGS.comms_timeout_sec is not None:
      sim_args += ['--comms_timeout_sec=%f' % FLAGS.comms_timeout_sec]

    try:
      with _Process('sim/sim', extra_args=sim_args, wait_for_completion=True,
                    use_callgrind=FLAGS.callgrind_sim):
        pass
    except KeyboardInterrupt:
      pass


if __name__ == '__main__':
  with os_util.ChangeDir(bazel_util.GetWorkspaceRoot()):
    main(sys.argv)
