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


r"""A module for detecting a wing crash in an HDF5 log file.

Determine if a wing crashed during flight, failed to complete
a flight as expected, or exceeded any safety criteria.

E.g.:
  analysis/util/validate_flight.py a.h5
"""

import sys

import gflags
import h5py
from makani.config import mconfig
from makani.control import control_types
from makani.lib.python import c_helpers
from makani.lib.python.h5_utils import numpy_utils
import numpy

_FLIGHT_MODE_HELPER = c_helpers.EnumHelper('FlightMode', control_types)

FLAGS = gflags.FLAGS


class FlightError(Exception):
  pass


def CheckOverTension(monitor_params, sim_telem):
  """Tests that the tether tension stays below upper limits."""
  max_tension = monitor_params['tether']['tension']['very_high']
  tether_force = sim_telem['tether']['end_force_g']
  log_max_tension = numpy.max(numpy_utils.Vec3Norm(tether_force))

  print ('Max tension in flight was %3.1f kN (limit %3.1f kN).'
         % (log_max_tension / 1000.0, max_tension / 1000.0))
  if log_max_tension > max_tension:
    raise FlightError('Over tension (%3.1f > %3.1f [N]).' %
                      (log_max_tension / 1000.0, max_tension / 1000.0))


def CheckMinAltitude(system_params, sim_telem):
  """Checks that the wing does not penetrate the ground."""
  # TODO: The 8 m threshold is arbitrarily selected to avoid indicating
  # a crash when perching.  This check doesn't eliminate all crashes.  Come up
  # with a better way to check this, possibly considering wing orientation.
  min_altitude = 8.0
  log_min_altitude = (system_params['ground_frame']['ground_z']
                      - numpy.max(sim_telem['wing']['Xg']['z']))

  print ('Min altitude in flight was %4.2f m (limit %4.2f m)'
         % (log_min_altitude, min_altitude))
  if log_min_altitude < min_altitude:
    raise FlightError('Possible crash detected (%4.2f < %4.2f [m])' %
                      (log_min_altitude, min_altitude))


def CheckFlightModeProgression(control_telem):
  """Verify that all of the required modes are reached in order."""
  required_modes = [control_types.kFlightModePerched,
                    control_types.kFlightModeHoverAscend,
                    control_types.kFlightModeHoverPrepTransformGsUp,
                    control_types.kFlightModeHoverTransformGsUp,
                    control_types.kFlightModeHoverFullLength,
                    control_types.kFlightModeHoverAccel,
                    control_types.kFlightModeTransIn,
                    control_types.kFlightModeCrosswindNormal,
                    control_types.kFlightModeCrosswindPrepTransOut,
                    control_types.kFlightModeHoverTransOut,
                    control_types.kFlightModeHoverPrepTransformGsDown,
                    control_types.kFlightModeHoverTransformGsDown,
                    control_types.kFlightModeHoverReelIn,
                    control_types.kFlightModeHoverDescend]
  min_index = 0
  for mode in required_modes:
    mode_index = (numpy.argmax(control_telem['flight_mode'][min_index:] == mode)
                  + min_index)
    if control_telem['flight_mode'][mode_index] != mode:
      raise FlightError('Failed to reach mode %s in sequence.' %
                        _FLIGHT_MODE_HELPER.ShortName(mode))
    print 'Reached flight mode %s' % _FLIGHT_MODE_HELPER.ShortName(mode)
    min_index = mode_index


def _GetControlTelemetryCrosswindIndexes(control_telem):
  return numpy.where(numpy.logical_or(
      control_telem['flight_mode'] == control_types.kFlightModeCrosswindNormal,
      control_telem['flight_mode'] ==
      control_types.kFlightModeCrosswindPrepTransOut))[0]


def CheckCrosswindMinAltitude(system_params, control_telem, sim_telem):
  """Detect if wing got below 1.5 wingspans in crosswind."""
  crosswind_control_inds = _GetControlTelemetryCrosswindIndexes(control_telem)
  crosswind_start_time = control_telem['time'][crosswind_control_inds[0]]
  crosswind_end_time = control_telem['time'][crosswind_control_inds[-1]]
  crosswind_sim_inds = numpy.where(numpy.logical_and(
      crosswind_start_time <= sim_telem['time'],
      sim_telem['time'] <= crosswind_end_time))

  crosswind_xg_z = (
      sim_telem['wing']['Xg']['z'][crosswind_sim_inds])
  min_altitude = system_params['wing']['b'] * 1.5
  log_min_altitude = (system_params['ground_frame']['ground_z'] -
                      numpy.max(crosswind_xg_z))

  print ('Min altitude in crosswind was %4.2f m (limit %4.2f m).'
         % (log_min_altitude, min_altitude))
  if log_min_altitude < min_altitude:
    raise FlightError('Wing min altitude was %4.2f in crosswind (min = %4.2f).'
                      % (log_min_altitude, min_altitude))


def RunFlightChecks(log_file):
  """Run flight checks on an H5 log file."""
  system_params = log_file['parameters']['system_params']
  simulator = log_file['messages']['kAioNodeSimulator']
  sim_telem = simulator['kMessageTypeSimTelemetry']['message']

  controller = log_file['messages']['kAioNodeControllerA']
  control_telem = controller['kMessageTypeControlDebug']['message']

  monitor_params = mconfig.MakeParams('common.monitor.monitor_params')
  CheckOverTension(monitor_params, sim_telem)
  CheckMinAltitude(system_params, sim_telem)
  CheckFlightModeProgression(control_telem)
  CheckCrosswindMinAltitude(system_params, control_telem, sim_telem)


def _ShowUsageAndExit():
  print '\nUsage: %s <log_file>\n%s' % (sys.argv[0], FLAGS)
  sys.exit(1)


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print 'Error: %s' % e
    _ShowUsageAndExit()

  if len(argv) != 2:
    _ShowUsageAndExit()

  # Load data (may exit with IOError).
  with h5py.File(argv[1], 'r') as log_file:
    RunFlightChecks(log_file)


if __name__ == '__main__':
  main(sys.argv)
