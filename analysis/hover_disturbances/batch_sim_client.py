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

r"""Local client for running a hover disturbances batch simulation.

Sample usage, running 10 configs processed by 2 workers:
    bazel run analysis:hover_disturbances_batch_sim -- \
      --sim_name=my_shaky_hover_disturbance_test \
      --wind_speeds='3.0, 15.0, 10' \
      --num_workers=2 \
      --output_dir=/tmp/hover_disturbances
"""

import os
import sys

import gflags
import makani
from makani.analysis.hover_disturbances import batch_sim_params
from makani.config import overrides_util
from makani.lib.python import flag_types
from makani.lib.python.batch_sim import client as client_base
from makani.lib.python.batch_sim import parameter_tables
import numpy

gflags.DEFINE_string('output_dir',
                     os.path.join(makani.HOME, 'logs', 'hover_disturbances'),
                     'Directory in which to output .html files.  '
                     'This will be created if it does not exist.')

gflags.DEFINE_string('azimuth', 0.0,
                     'Azimuth [deg] of wing.')

gflags.DEFINE_string('elevation', 10.0,
                     'Elevation [deg] of wing.')

gflags.DEFINE_string('payout', 20.0,
                     'Tether payout [m].')

gflags.DEFINE_integer('num_amplitudes', 5,
                      'Number of amplitudes to apply.')

flag_types.DEFINE_linspace('wind_speeds', '0.0, 15.0, 7',
                           'Linspace range of wind speed values.',
                           nonempty=True)

FLAGS = gflags.FLAGS


class TorqueStepAmplitudeRange(parameter_tables.FaultParameterRange):

  def __init__(self, name, t_start, t_end, parameters_start, parameters_end):
    super(TorqueStepAmplitudeRange, self).__init__(
        'Disturbance [N-m]', t_start, t_end, 'Wing',
        'kSimFaultDisturbanceBodyTorqueStep', parameters_start, parameters_end,
        FLAGS.num_amplitudes)

  def _GetParameterValue(self, parameters):
    return numpy.linalg.norm(parameters[0:3])


class HoverDisturbancesSimClient(parameter_tables.DisturbancesSimClient):
  """Client for a hover disturbances batch simulation."""

  _BASE_PATH = 'analysis/hover_disturbances_batch_sim'

  def __init__(self, **kwargs):
    # The scale of the disturbances were calculated based on the following.
    #
    # m = 1499  # Mass of the wing [kg].
    # g = 9.81  # Gravitational acceleration [m/s^2]
    # num_rotors = 8  # Number [#] of rotors.
    # V = 15  # Wind speed [m/s].
    # y_motor = 3.6  # Distance [m] to the COM from the outboard motors.
    # z_motor = 1.6  # Distance [m] to the COM from the bottom row motors.
    # Vgust = 4  # Increased wind speed [m/s] due to a gust.
    # Cd = 1.2  # Flat plate drag coefficient [#].
    # rho = 1.225  # Air density [kg / m^3].
    # c = 1.28  # Wing chord [m].
    # b = 26  # Wing span [m].
    # A_ele = 4  # Elevator area [m^2].
    # A_rud = 4  # Rudder area [m^2].
    # r_tail = 7  # Distance of tail from COM [m].
    #
    #
    # For roll we consider a gust on the tip of the wing:
    #
    # roll_disturbance = 0.5 * rho * Cd * c * (
    #     (V+Vgust)**2.0 * 0.5 * ((b/2.0)**2.0 - (0.75 * b/2.0)**2.0)
    #     - V**2.0 * 0.5 * ((b/2.0)**2.0 - (0.75 * b/2.0)**2.0))
    #
    # For pitch we consider the elevator suddenly shifting to be 90
    # deg to the wind or a bottom row motor suddenly being unable to
    # provide thrust.
    #
    # pitch_disturbance = [0.5 * rho * Cd * A_ele * V**2 * r_tail,
    #                      z_motor * m * g / num_rotors]
    #
    # For yaw we consider a sudden 45 deg. shift in wind conditions
    # hitting the rudder or an outboard propellor suddenly being
    # unable to provide thrust.
    #
    # yaw_disturbance = [(0.5 * rho * Cd * A_rud *
    #                     (numpy.cos(numpy.pi / 4.0) * V)**2.0 * r_tail),
    #                    y_motor * m * g / num_rotors]
    #
    # We then increase these numbers by 50 percent.

    # Different disturbance directions.
    # NOTE: The way that FaultParameterRange uses numpy.min and
    #              numpy.max downstream of disturbances_data restricts these
    #              arrays to have only a single component in a 0, 1, or 2
    #              direction. This makes the min/max for a parameter sweep on
    #              a batch sim result page for a single sweep look a little odd
    #              as well, but it remains functional. Eventually someone may
    #              want multi-axis disturbances and this will have to be fixed.
    disturbances_data = [
        ('Pos. Roll', ([1500.0, 0.0, 0.0], [7500.0, 0.0, 0.0])),
        ('Pos. Pitch', ([0.0, 1500.0, 0.0], [0.0, 7500.0, 0.0])),
        ('Pos. Yaw', ([0.0, 0.0, 1500.0], [0.0, 0.0, 10500.0])),
        ('Neg. Roll', ([-7500, 0.0, 0.0], [-1500.0, 0.0, 0.0])),
        ('Neg. Pitch', ([0.0, -7500.0, 0.0], [0.0, -1500.0, 0.0])),
        ('Neg. Yaw', ([0.0, 0.0, -10500.0], [0.0, 0.0, -7500.0]))
    ]

    params = batch_sim_params.HoverDisturbancesParameters()
    t_start = params.setup_time
    t_end = params.setup_time + params.impulse_duration

    disturbances = []
    for (disturbance_name, parameters) in disturbances_data:
      name = '%g second %s Torque Step' % (t_end - t_start, disturbance_name)
      disturbances += [(name, TorqueStepAmplitudeRange(
          name, t_start, t_end, parameters[0], parameters[1]))]

    wind_speeds = parameter_tables.WindSpeedParameterRange(FLAGS.wind_speeds)

    base_overrides = overrides_util.PreprocessOverrides({
        'system': {'flight_plan': 'kFlightPlanStartDownwind'},
        'control': {'hover': {'inject': {'use_signal_injection': False}}},
        'sim': {
            'sim_opt': [
                'Faults',
                'GroundContact',
                'ImperfectSensors',
                'Perch',
                'PerchContact',
                'StackedPowerSystem'
            ],
            'telemetry_sample_period': 0.0,
            'sim_time': (params.setup_time + params.impulse_duration
                         + params.settle_time),
        }
    })

    super(HoverDisturbancesSimClient, self).__init__(
        FLAGS.output_dir, base_overrides, wind_speeds, disturbances,
        params.scoring_functions, title='Hover Disturbances', **kwargs)


def main(argv):
  client_base.InitMain(argv)
  client = HoverDisturbancesSimClient()
  client.Run()


if __name__ == '__main__':
  gflags.RegisterValidator(
      'output_dir',
      lambda o: not os.path.exists(o) or os.path.isdir(o),
      '--output_dir cannot refer to an existing, non-directory object.')

  main(sys.argv)
