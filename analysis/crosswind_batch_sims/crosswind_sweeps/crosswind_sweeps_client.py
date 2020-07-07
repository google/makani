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

r"""Local client for running a crosswind_sweeps batch simulation.

A list of parameter distributions and values used in Monte Carlo analysis can
be found in this spreadsheet:
https://docs.google.com/spreadsheets/d/1XZj-2hOPdUqwmIuRfzdV00E1n9Yej6XmY6QZhlEJi1A/edit#gid=0

Sample usage, running 10 configs processed by 2 workers:

    bazel run //analysis/crosswind_batch_sims:crosswind_sweeps_batch_sim -- \
      --sim_name=crosswind_sweeps_test \
      --num_workers=30 \
      --output_dir=${MAKANI_HOME}/logs/crosswind_sweeps

Sample usage of base_overrides:
  Example above is still relevant. Base overrides is used to add to or override
  the default overrides.
  Example:
    --base_overrides '{"sim": {"wing_sim": {"mass_scale": 0.9},
                               "sim_time": 1600.0}}'

Sample usage of custom sweep flags:
  Sample usage above is still relevant.
  Example:
    --only_custom_sweep \
    --custom_sweep '[{"name": "PB_Wind_Filter",
                      "path": ["control", "estimator",
                               "wind", "fc_speed_playbook"],
                      "values": [0.01, 0.05, 0.1]}]'

  Custom sweep can vary multiple parameters together.
  Example:
    --only_custom_sweep \
    --custom_sweep '[{"name": "Mass and Inertia",
                      "path": [["sim", "wing_sim", "mass_scale"],
                               ["sim", "wing_sim",
                                "moment_of_inertia_scale", 0],
                               ["sim", "wing_sim",
                                "moment_of_inertia_scale", 1],
                               ["sim", "wing_sim",
                                "moment_of_inertia_scale", 2]],
                      "values":[[0.9, 0.9, 0.9, 0.9],
                                [1.1, 1.1, 1.1, 1.1]]}]'

  Custom sweep can be used to specify monte-carlo variations.
  For monte-carlo, a distribution can be specified using the methodology
  specified in the parameter_tables ParameterRange object.
  Example:
    --custom_sweep '[{"name":"Servo Inertia",
                      "path":["sim", "servos_sim", 0, "moment_of_inertia"],
                      "distribution":{"type": "normal",
                                      "mean": 0.005,
                                      "sigma": 0.005,
                                      "bound": 2.0}}]'

For manual sweeps service instead of locally managed sweep:

  1) Go to go/makani-sweeps-service and follow instructions there.

  Note: JSON strings in "EXTRA_ARGS" must not be in single quotes
    and cannot contain spaces.
"""

import json
import os
import sys

import gflags
import makani
from makani.analysis.crosswind_batch_sims import batch_sim_params
from makani.config import mconfig
from makani.config import overrides_util
from makani.lib.python import dict_util
from makani.lib.python import flag_types
from makani.lib.python import turbsim_util
from makani.lib.python import wing_flag
from makani.lib.python.batch_sim import client as client_base
from makani.lib.python.batch_sim import parameter_sweeps
from makani.lib.python.batch_sim import parameter_tables
import numpy as np

gflags.DEFINE_string('output_dir',
                     os.path.join(makani.HOME, 'logs', 'crosswind_sweeps'),
                     'Directory in which to output .html files.  '
                     'This will be created if it does not exist.')

gflags.DEFINE_bool('monte_carlo', False,
                   'Whether to run Monte-Carlo variations.')

gflags.DEFINE_bool('offshore', False,
                   'Whether to run an offshore simulation.')

gflags.DEFINE_bool('randomize_waves', True,
                   'Whether to add wave variations to an offshore sim sweep.')

gflags.DEFINE_enum('flight_plan', 'TurnKey',
                   ['TurnKey', 'HighHover'],
                   'Flight plan to use in flight simulation.')

gflags.DEFINE_integer('monte_carlo_rows', 10,
                      'Number of rows for Monte Carlo tables.')

gflags.DEFINE_integer('monte_carlo_cols', 5,
                      'Number of columns for Monte Carlo tables.')

flag_types.DEFINE_linspace('wind_shears', '0.0, 0.2, 3',
                           'Linspace of wind shear exponents [#].')

flag_types.DEFINE_linspace('wind_speeds', '1.0, 15.0, 8',
                           'Linspace of wind speeds [m/s].')

gflags.DEFINE_float('max_hover_mean_wind_speed', 15.0,
                    'Max mean wind speed in hover [m/s].')

gflags.DEFINE_string('wind_model', 'kWindModelDrydenTurbulence',
                     'Wind turbulence model to use. This is overridden if '
                     '--turbsim_folder is set.')

gflags.DEFINE_string('turbsim_folder', '',
                     'Folder name within gs://makani_turbsim_databases from '
                     'which to pull TurbSim wind databases.')

gflags.DEFINE_string('custom_sweep', '[]',
                     'Sets up a sweep of a list of custom variables. '
                     'Format is JSON string of list of dicts: \n'
                     '[{\"name\": <name_in_report>, '
                     '\"path\": [<list_of_keys_to_variable>],'
                     '\"values\":[<param>]}]\n'
                     'This works for Monte-Carlo as well, where values '
                     'then describe the shape of the distribution. '
                     'Or, a distribution can be defined by adding '
                     '"distribution" to the dict, following the format '
                     'specified in the docstring for '
                     'parameter_tables.ParameterRange.')

gflags.DEFINE_bool('only_custom_sweep', False,
                   'If True, other parameters are cleared before adding '
                   'custom parameter sweep params.', short_name='ocs')

gflags.DEFINE_string('base_overrides', '{}',
                     'JSON string of override parameters added to '
                     'raw_base_overrides found in crosswind_sweeps_client. '
                     'Will overwrite items already in raw_base_overrides.')

FLAGS = gflags.FLAGS


class CrosswindSweepsSimClient(parameter_tables.OverridesTableSimClient):
  """Client for a crosswind disturbances batch simulation."""

  _BASE_PATH = 'analysis/crosswind_batch_sims/crosswind_sweeps_batch_sim'

  def __init__(self, **kwargs):
    base_params = mconfig.MakeParams('common.all_params', overrides=None,
                                     override_method='simple')

    # Time [s] to end simulation.
    if FLAGS.flight_plan == 'TurnKey':
      end_time = 1500.0
    elif FLAGS.flight_plan == 'HighHover':
      end_time = 1000.0
    else:
      assert False, 'Unsupported flight plan: %s' % FLAGS.flight_plan

    flight_plan = 'kFlightPlan' + FLAGS.flight_plan

    # Build the list of tables.

    raw_base_overrides = {
        'system': {'flight_plan': flight_plan},
        'sim': {
            'phys_sim': {
                'wind_model': ('kWindModelDatabase' if FLAGS.turbsim_folder
                               else FLAGS.wind_model)
            },
            'telemetry_sample_period': 0.0,
            'sim_opt': ['kSimOptExitOnCrash',
                        'kSimOptFaults',
                        'kSimOptGroundContact',
                        'kSimOptImperfectSensors',
                        'kSimOptPerch',
                        'kSimOptPerchContact',
                        'kSimOptStackedPowerSystem'],
            'sim_time': end_time
        }
    }

    # Only use sensor imperfections in Monte Carlo simulations.
    if FLAGS.monte_carlo:
      raw_base_overrides['sim']['sim_opt'] += ['kSimOptImperfectSensors']

    if FLAGS.offshore:
      raw_base_overrides['system']['test_site'] = 'kTestSiteNorway'
    else:
      raw_base_overrides['system']['test_site'] = 'kTestSiteParkerRanch'

    # Update raw_base_overrides with flag base_overrides.
    raw_base_overrides = dict_util.UpdateNestedDict(
        raw_base_overrides, json.loads(FLAGS.base_overrides))

    if ('system' in raw_base_overrides and
        'wing_serial' in raw_base_overrides['system']):
      assert False, ('Cannot override wing_serial; '
                     'it is set implicitly by flight plan and wing_model.')

    y_ranges = []

    if FLAGS.turbsim_folder:
      # Initialize object for selecting appropriate wind databases for each run.
      turbsim_database_selector = turbsim_util.TurbSimDatabaseSelector(
          FLAGS.turbsim_folder, base_params)

      # TODO: Rather than hardcode the shear reference height,
      # pull from a database/file with properties of each database set.
      # Also think about overriding this value in the base_params.
      x_range = parameter_tables.WindSpeedParameterRange(
          FLAGS.wind_speeds, wind_shear_ref_height_agl=21.0)

      if FLAGS.monte_carlo:
        # TODO: Pull the range of options for these from a
        # database/file with properties of each database set. Would need to be
        # able to query a sheet w/ these parameters; maybe make a descriptive
        # csv or json file in each database folder.
        y_ranges += [
            parameter_tables.WindDatabaseInitialTimeParameterRange(
                [30.0, 270.0], distribution={
                    'lower_bound': 30.0, 'upper_bound': 270.0,
                    'type': 'uniform'}),
            parameter_tables.WindDatabaseYOffsetParameterRange(
                [-100.0, 100.0], distribution={
                    'lower_bound': -100.0, 'upper_bound': 100.0,
                    'type': 'uniform'}),
        ]

    else:
      turbsim_database_selector = None

      # Sweep wind speeds.
      # If times for wind speed updates are specified (a 3-by-1 list of time
      # values in seconds), then saturate the mean wind speed to
      # FLAGS.max_hover_mean_wind_speed before the second time entry and after
      # the third time entry.
      # NOTE:
      # - This flag will not affect simulations which use a TurbSim database.
      # - Here, the second value in t_updates corresponds to the approximate
      #   time when the kite enters crosswind, and the third value is the time
      #   when the joystick throttle is updated to
      #   kSimJoystickThrottleReturnToPerch.
      x_range = parameter_tables.WindSpeedParameterRange(
          FLAGS.wind_speeds,
          wind_shear_ref_height_agl=base_params['sim']['phys_sim'][
              'wind_shear_ref_height_agl'],
          t_updates=[0.0, 460.0, 820.0],
          max_wind_speed=FLAGS.max_hover_mean_wind_speed)

      # Sweep environmental properties.
      # Original source for WindElevation is unknown.
      # TODO: Update WindElevation distribution based on Parker Ranch
      # wind measurements.
      # Wind veer range is based on Parker Ranch wind measurements as discussed
      # in http://b/117942530.
      y_ranges += [
          parameter_tables.WindElevationDegParameterRange(
              [-6.0, 6.0], distribution={
                  'mean': 0.0, 'sigma': 3.0, 'bound': 2.0, 'type': 'normal'}),
          parameter_tables.WindVeerDegParameterRange(
              [-45.0, -21.0, 21.0, 45.0], distribution={
                  'mean': 13.5, 'sigma': 7.5, 'bound': 3.0, 'type': 'normal'}),
      ]

    shear_parameter_range = parameter_tables.WindShearExponentParameterRange(
        FLAGS.wind_shears)
    # Add shear exponents as a sweep if this is not a Monte Carlo run.
    if not FLAGS.monte_carlo:
      y_ranges.append(shear_parameter_range)

    # Sweep mass properties.
    y_ranges += [
        # Center of mass location: maximum 0.05 m error in each dimension for
        # Monte Carlo sweeps, and 0.05 m variation in crosswind parameters
        # sweeps.
        parameter_tables.CenterOfMassOffsetParameterRange(
            0, [-0.05, 0.05], distribution={
                'mean': 0.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
            body='wing_sim', body_name='Wing'),
        parameter_tables.CenterOfMassOffsetParameterRange(
            1, [-0.05, 0.05], distribution={
                'mean': 0.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
            body='wing_sim', body_name='Wing'),
        parameter_tables.CenterOfMassOffsetParameterRange(
            2, [-0.05, 0.05], distribution={
                'mean': 0.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
            body='wing_sim', body_name='Wing'),
        # Mass scaling: 1.4% error for both Monte Carlo and crosswind sweeps.
        parameter_tables.MassScaleParameterRange(
            [0.986, 1.014], distribution={
                'mean': 1.0, 'sigma': 0.014, 'bound': 2.0, 'type': 'normal'},
            body='wing_sim', body_name='Wing'),
        # Inertia scaling: maximum errors for Monte Carlo sweeps of 5% for Ixx,
        # 2% for Iyy and 4.4% for Izz. 10% variation in crosswind sweeps.
        parameter_tables.InertiaScaleParameterRange(
            0, [0.90, 1.1], distribution={
                'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
            body='wing_sim', body_name='Wing'),
        parameter_tables.InertiaScaleParameterRange(
            1, [0.90, 1.1], distribution={
                'mean': 1.0, 'sigma': 0.01, 'bound': 2.0, 'type': 'normal'},
            body='wing_sim', body_name='Wing'),
        parameter_tables.InertiaScaleParameterRange(
            2, [0.90, 1.1], distribution={
                'mean': 1.0, 'sigma': 0.022, 'bound': 2.0, 'type': 'normal'},
            body='wing_sim', body_name='Wing'),
    ]

    # Sweep aerodynamics parameters.
    # The following parameters override existing aerodynamic offsets in our
    # config files.
    if FLAGS.wing_model == 'm600':
      y_ranges += [
          # CD offset: maximum 0.050 error.
          # Known offset in CD from RPX-07 glide data analysis is 0.075.
          # Use 0.075 as mean for Monte Carlo sweeps.
          # This is chosen since this will override existing offsets.
          # [0.075 - 0.050, 0.075 + 0.050] variation in crosswind sweeps.
          parameter_tables.AeroSimOffsetParameterRange(
              'CD', [0.025, 0.125], distribution={
                  'mean': 0.075, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'}
          ),
          # CL offset: maximum 0.3 error for Monte Carlo sweeps,
          # 0.2 variation in crosswind sweeps.
          # Flight data and aero database comparison shows CL is over predicted,
          # thus setting the mean at -0.125 from Monte Carlo sweeps.
          parameter_tables.AeroSimOffsetParameterRange(
              'CL', [-0.2, 0.2], distribution={
                  'mean': -0.125, 'sigma': 0.15, 'bound': 2.0, 'type': 'normal'}
          ),
      ] + parameter_sweeps.GetAeroDerivativeParameterRanges()
    elif FLAGS.wing_model == 'oktoberkite':
      y_ranges += [

          parameter_tables.AeroSimOffsetParameterRange(
              'CD', [-0.011, 0.033], distribution={
                  'mean': 0.011, 'sigma': 0.011, 'bound': 2.0, 'type': 'normal'}
          ),
          parameter_tables.AeroSimOffsetParameterRange(
              'CL', [-0.275, 0.275], distribution={
                  'mean': 0.0, 'sigma': 0.15, 'bound': 2.0, 'type': 'normal'}),
      ] + parameter_sweeps.GetAeroDerivativeParameterRanges()
    else:
      assert False, '{} wing_model is not supported.'.format(FLAGS.wing_model)

    # Sweep Pitot parameters.
    y_ranges += [
        # Pitot angles offsets: maximum 1 deg offset for Monte Carlo and
        # crosswind sweeps.
        # Pitot Cp offset: maximum 0.01 offset for Monte Carlo and
        # 0.02 variation in crosswind sweeps.
        parameter_tables.PitotPitchDegParameterRange(
            [-1.0, 1.0], distribution={
                'mean': 0.0, 'sigma': 0.5, 'bound': 2.0, 'type': 'normal'}),
        parameter_tables.PitotYawDegParameterRange(
            [-1.0, 1.0], distribution={
                'mean': 0.0, 'sigma': 0.5, 'bound': 2.0, 'type': 'normal'}),
        parameter_tables.PitotCpOffsetParameterRange(
            [-0.02, 0.02], distribution={
                'mean': 0.0, 'sigma': 0.01, 'bound': 2.0, 'type': 'normal'}),
    ]

    # Sweep actuator parameters.
    y_ranges += parameter_sweeps.GetFlapOffsetParameterRanges()

    # Sweep offshore parameters.
    if FLAGS.offshore:
      # Current uncertainties:
      #   - 5% on mass properties.
      #   - 5% on hydrodynamic model parameters.
      #   - 5% on mooring line model parameters.
      #   - 5 deg on yaw equilibrium heading.
      #   - 50 cm on axial mooring line attachment point model.
      #   - 20 cm on orthogonal mooring line attachment point model.
      # TODO: Refine the parameter variations once we have measured
      # data and knowledge of sensitivity.
      # TODO: Add a deterministic offshore crosswind sweep to the
      # nightly (b/137648033).
      y_ranges += [
          # Center of mass offset.
          parameter_tables.CenterOfMassOffsetParameterRange(
              0, [-0.2, 0.2], distribution={
                  'mean': 0.0, 'sigma': 0.1, 'bound': 2.0, 'type': 'normal'},
              body='buoy_sim', body_name='Buoy'),
          parameter_tables.CenterOfMassOffsetParameterRange(
              1, [-0.2, 0.2], distribution={
                  'mean': 0.0, 'sigma': 0.1, 'bound': 2.0, 'type': 'normal'},
              body='buoy_sim', body_name='Buoy'),
          parameter_tables.CenterOfMassOffsetParameterRange(
              2, [-1.0, 1.0], distribution={
                  'mean': 0.0, 'sigma': 0.39, 'bound': 2.0, 'type': 'normal'},
              body='buoy_sim', body_name='Buoy'),

          # Mass scaling.
          parameter_tables.MassScaleParameterRange(
              [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              body='buoy_sim', body_name='Buoy'),

          # Inertia scaling.
          parameter_tables.InertiaScaleParameterRange(
              0, [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              body='buoy_sim', body_name='Buoy'),
          parameter_tables.InertiaScaleParameterRange(
              1, [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              body='buoy_sim', body_name='Buoy'),
          parameter_tables.InertiaScaleParameterRange(
              2, [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              body='buoy_sim', body_name='Buoy'),

          # Hydrodynamic model uncertainties.
          parameter_tables.BuoyModelParameterRange(
              'Buoy Torsional Damping X Scale [#]',
              [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              category='hydrodynamics',
              variable='torsional_damping_x_scale'),
          parameter_tables.BuoyModelParameterRange(
              'Buoy Torsional Damping Y Scale [#]',
              [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              category='hydrodynamics',
              variable='torsional_damping_y_scale'),
          parameter_tables.BuoyModelParameterRange(
              'Buoy Torsional Damping Z Scale [#]',
              [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              category='hydrodynamics',
              variable='torsional_damping_z_scale'),
          parameter_tables.BuoyModelParameterRange(
              'Buoy Buoyancy Damping Coeff. Scale [#]',
              [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              category='hydrodynamics',
              variable='buoyancy_damping_coeff_scale'),
          parameter_tables.BuoyModelParameterRange(
              'Buoy Added Mass Coeff. Scale [#]',
              [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              category='hydrodynamics',
              variable='Ca_scale'),
          parameter_tables.BuoyModelParameterRange(
              'Buoy Effective Heave Diameter Scale [#]',
              [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              category='hydrodynamics',
              variable='Dh_scale'),
          parameter_tables.BuoyModelParameterRange(
              'Buoy Added Inertia Coeff. Scale [#]',
              [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              category='hydrodynamics',
              variable='ki_scale'),

          # Mooring line model uncertainties.
          parameter_tables.BuoyModelParameterRange(
              'Buoy Equilibrium Yaw Angle Delta [deg]',
              np.arange(-180., 180., 30.), distribution={
                  'mean': 0.0, 'sigma': 2.5, 'bound': 2.0, 'type': 'normal'},
              category='mooring_lines',
              variable='yaw_equilibrium_heading_delta'),
          parameter_tables.BuoyModelParameterRange(
              'Buoy Yaw Torsional Stiffness Scale [#]',
              [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              category='mooring_lines',
              variable='torsional_stiffness_z_scale'),
          parameter_tables.BuoyModelParameterRange(
              'Buoy Effective Mooring Line X Attachment Delta [m]',
              [-0.2, 0.2], distribution={
                  'mean': 0.0, 'sigma': 0.1, 'bound': 2.0, 'type': 'normal'},
              category='mooring_lines',
              variable='mooring_attach_pos_x_delta'),
          parameter_tables.BuoyModelParameterRange(
              'Buoy Effective Mooring Line Y Attachment Delta [m]',
              [-0.2, 0.2], distribution={
                  'mean': 0.0, 'sigma': 0.1, 'bound': 2.0, 'type': 'normal'},
              category='mooring_lines',
              variable='mooring_attach_pos_y_delta'),
          parameter_tables.BuoyModelParameterRange(
              'Buoy Effective Mooring Line Z Attachment Delta [m]',
              [-0.5, 0.5], distribution={
                  'mean': 0.0, 'sigma': 0.25, 'bound': 2.0, 'type': 'normal'},
              category='mooring_lines',
              variable='mooring_attach_pos_z_delta'),
          parameter_tables.BuoyModelParameterRange(
              'Buoy Mooring Line Linear Spring Coeff. Scale [#]',
              [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              category='mooring_lines',
              variable='kt0_scale'),
          parameter_tables.BuoyModelParameterRange(
              'Buoy Mooring Line Quadratic Spring Coeff. Scale [#]',
              [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              category='mooring_lines',
              variable='kt1_scale'),
          parameter_tables.BuoyModelParameterRange(
              'Buoy Mooring Line Damping Coeff. Scale [#]',
              [0.9, 1.1], distribution={
                  'mean': 1.0, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'},
              category='mooring_lines',
              variable='ct_scale'),
      ]

      # Sweep environmental properties.
      # Air density at Stavanger test site is 1.220 kg/m^3.
      # Use this as mean value and maximum 5% error
      # in density measurement for Monte Carlo sweeps. For crosswind sweeps,
      # high limit is at sea-level density and low limit is at roughly 2500 m
      # density altitude.
      # NOTE: Ideally, we would like to point the mean value of
      # the air density parameter range to the air_density in the config
      # structure. However, it is a parameter that is derived from the test_site
      # param, which is also overridden at the current level.
      y_ranges += [parameter_tables.AirDensityParameterRange(
          [0.976, 1.225], distribution={
              'mean': 1.220, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'})]

      # Parameters used to define the 'WaveParameterSelector' class used in
      # Monte Carlo analyses and define the distribution in the
      # parameter_tables. See the WaveParameterSelector and ParameterRange class
      # definitions in parameter_tables.py for more description.
      # Note, for example, that the 'mean' used in a lognormal distribution is
      # the mean of the underlying normal distribution, and can therefore be
      # outside the upper/lower_bounds.
      # Generated from Colab notebook: b/130682761#comment15
      # Source: 'NORA10_5922N_0509E.txt'
      # Data filtered for Jun to Sep, between hours of 6 and 20, and when
      # wind speed at 10 m is between 5 and 15 m/s.'
      # Overwriting the upper_bound of the significant wave height to 3.0 m
      # to account for wave envelope due to max significant wave height for
      # personnel transfers at sea: b/130682761#comment18
      wave_variation_params = {
          'peak_period': {
              'correlation_fit': {
                  'coefficient': 1.186394,
                  'description':
                      'TP = coefficient * HS + intercept'
                      ' + (lognorm(mean, sigma) + loc)',
                  'intercept': 5.822409,
                  'lower_bound': 3.923101,
                  'upper_bound': 13.687049,
              },
              'description':
                  'Peak wave period of the total sea state, based on total sea '
                  'significant wave height.',
              'distribution': {
                  'loc': -4.342332,
                  'lower_bound': -2.478684,
                  'mean': 1.384812,
                  'sigma': 0.405295,
                  'type': 'lognormal',
                  'upper_bound': 4.217562,
              },
              'label': 'Peak wave period variation [s]',
              'values': [-2.478684, 4.217562],
          },
          'significant_height': {
              'correlation_fit': {
                  'coefficient': 0.012127,
                  'description':
                      'HS = coefficient * speed^2 + intercept + (lognorm(mean, '
                      'sigma) + loc). Note the distribution is closer to '
                      'exponnorm, but lognorm should be close enough and keeps '
                      'things simpler.',
                  'intercept': 0.653568,
                  'lower_bound': 0.427477,
                  'upper_bound': 3.0,
              },
              'description':
                  'Significant wave height of the total sea state, '
                  'based on wind speed at 10 m.',
              'distribution': {
                  'loc': -2.647313,
                  'lower_bound': -0.854369,
                  'mean': 0.953525,
                  'sigma': 0.196548,
                  'type': 'lognormal',
                  'upper_bound': 1.108073,
              },
              'label': 'Significant wave height variation [m]',
              'values': [-0.854369, 1.108073],
          },
          'wave_wind_alignment': {
              'description':
                  'Alignment [deg] of total sea peak wave direction minus wind '
                  'direction at 10 m. Corrected for use of wave heading '
                  'as wave-direction-of-travel.',
              'distribution': {
                  'bound': 2.000000,
                  'mean': 191.708319,
                  'sigma': 51.891563,
                  'type': 'normal',
              },
              'label': 'Wave alignment [deg] relative to wind',
              'values': [87.925192, 295.491445],
          },
          'wind_direction': {
              'description': 'Measured at 10 m.',
              'distribution': {
                  'distributions': [{
                      'mean': 170.364074,
                      'sigma': 29.122998,
                      'type': 'vonmises',
                      'units': 'deg',
                  }, {
                      'mean': 335.172882,
                      'sigma': 14.044122,
                      'type': 'vonmises',
                      'units': 'deg',
                  }],
                  'type': 'multimodal',
                  'weights': [0.510851, 0.489149],
              },
              'label': 'Wind direction [deg]',
              'values': [170.364074, 335.172882],
          },
      }

      y_ranges += [parameter_tables.WindDirectionDegParameterRange(
          wave_variation_params['wind_direction']['values'],
          distribution=wave_variation_params['wind_direction']['distribution'])]
      if FLAGS.randomize_waves:
        del wave_variation_params['wind_direction']
        for param in wave_variation_params:
          y_ranges += [parameter_tables.CustomParameterRange(
              wave_variation_params[param]['label'],
              ['sim', 'sea_sim', 'waves', param],
              wave_variation_params[param]['values'],
              wave_variation_params[param]['distribution'])]
        wave_parameter_selector = parameter_tables.WaveParameterSelector(
            wave_variation_params)
      else:
        wave_parameter_selector = None
    else:
      wave_parameter_selector = None

      # Sweep environmental properties.
      # Air density at Parker Ranch test site is 1.026 kg/m^3. This is roughly
      # 1800 m density altitude. Use this as mean value and maximum 5% error
      # in density measurement for Monte Carlo sweeps. For crosswind sweeps,
      # high limit is at sea-level density and low limit is at roughly 2500 m
      # density altitude.
      y_ranges += [parameter_tables.AirDensityParameterRange(
          [0.976, 1.225], distribution={
              'mean': 1.026, 'sigma': 0.025, 'bound': 2.0, 'type': 'normal'})]

      # Wind at Parker Ranch test site is nominally from the NorthEast.
      # From the initial analysis of our 2018 Aug-Oct Lidar data at the test
      # site, the mean is roughly at 55 degrees azimuth.
      # From the analysis presented in go/makani-cw09-wind-envelope, the
      # operational envelope for the wind direction was set to [0 - 75] degrees.
      # The normal distribution is clipped 5 degrees past these limits.
      # NOTE: Wind direction is an epistemic random variable that
      # we treat as an aleatory random variable to simplify the Monte Carlo
      # analysis.
      y_ranges += [parameter_tables.WindDirectionDegParameterRange(
          [30, 80], distribution={
              'mean': 55.0, 'sigma': 15.0, 'lower_bound': -5.0,
              'upper_bound': 80.0, 'type': 'normal'})]

    base_overrides = overrides_util.PreprocessOverrides(raw_base_overrides)

    # Wipe out existing y_range if only_custom_sweep flag is passed.
    if FLAGS.only_custom_sweep:
      y_ranges = []

    # Parse the custom_sweep flag.
    custom_sweep_entries = json.loads(FLAGS.custom_sweep)

    # Add custom sweep entries to y_ranges.
    for entry in custom_sweep_entries:
      y_ranges += [parameter_tables.CustomParameterRange(
          entry['name'], entry['path'],
          # Values can be unspecified for monte-carlo usage.
          entry.get('values', None),
          # Distribution can be unspecifed for deterministic sweep usage.
          entry.get('distribution', None))]

    tables = []
    if FLAGS.monte_carlo:
      num_cols = len(x_range.values)

      for shear_exponent in shear_parameter_range.values:
        for value in x_range.values:
          table_base_overrides = dict_util.MergeNestedDicts(
              base_overrides, x_range.GetOverrides(value))
          table_base_overrides = dict_util.MergeNestedDicts(
              table_base_overrides,
              shear_parameter_range.GetOverrides(shear_exponent))
          tables.append(parameter_tables.ParameterRangeMonteCarloTable(
              'Monte Carlo %s = %g, shear exponent = %g' % (
                  x_range.label, x_range.GetDisplayValue(value),
                  shear_exponent),
              [FLAGS.monte_carlo_cols, FLAGS.monte_carlo_rows],
              y_ranges, base_overrides=table_base_overrides,
              turbsim_database_selector=turbsim_database_selector,
              wave_parameter_selector=wave_parameter_selector))
      title = FLAGS.flight_plan + ' Monte Carlo'
    else:
      num_cols = 4
      for y_range in y_ranges:
        tables.append(parameter_tables.ParameterRangeTable(
            y_range.label, x_range, y_range, base_overrides=base_overrides,
            turbsim_database_selector=turbsim_database_selector))
      title = FLAGS.flight_plan + ' Parameter Sweeps'

    wing_model = wing_flag.FlagToWingModelName(FLAGS.wing_model)
    params = batch_sim_params.CrosswindSweepsParameters(
        flight_plan=flight_plan, wing_model=wing_model,
        offshore=FLAGS.offshore)
    super(CrosswindSweepsSimClient, self).__init__(
        FLAGS.output_dir, tables, params.scoring_functions,
        title=title, columns=num_cols, **kwargs)

  def _GetWorkerArgs(self, config_range):
    args = super(CrosswindSweepsSimClient, self)._GetWorkerArgs(config_range)
    args.append('--wing_model=' + FLAGS.wing_model)
    return args


def main(argv):
  client_base.InitMain(argv)
  client = CrosswindSweepsSimClient()
  client.Run()


if __name__ == '__main__':
  gflags.RegisterValidator(
      'output_dir',
      lambda o: not os.path.exists(o) or os.path.isdir(o),
      '--output_dir cannot refer to an existing, non-directory object.')

  main(sys.argv)
