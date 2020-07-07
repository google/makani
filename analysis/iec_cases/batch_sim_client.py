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

r"""Local client for an IEC cases batch simulation.

Sample usage, restricting to a few specific cases:
    bazel build analysis:iec_cases_batch_sim -- \
      --sim_name=my_cranky_iec_cases_test \
      --cases=1.1,2.3a,3.2c,4.2a \
      --num_workers=2 \
      --output_dir=/tmp/iec_cases

Typical local simulation usage:
     bazel build analysis:iec_cases_batch_sim -- \
      --sim_name=my_local_iec_cases_test \
      --cases=1.1,1.3 \
      --use_local_worker \
      --keep_h5_logs \
      --local_h5_logs_dir=/home/my_username/makani/logs/iec_cases
"""

import importlib
import os
import sys

import gflags
import makani
from makani.config import mconfig
from makani.config import overrides_util
from makani.lib.python.batch_sim import client as client_base
from makani.sim import sim_types
import matplotlib
import numpy

IEC_CASES = {
    '1.1': ('Normal turbulence model, '
            r'$V_{\mathrm{hub}} = 10$ m/s'),
    '1.3': ('Extreme turbulence model, '
            r'$V_{\mathrm{hub}} = 10$ m/s'),
    '1.4a': ('Extreme coherent gust with direction change, '
             r'$V_{\mathrm{hub}} = 10$ m/s'),
    '1.4b': ('Extreme coherent gust with direction change, '
             r'$V_{\mathrm{hub}} = 8$ m/s'),
    '1.4c': ('Extreme coherent gust with direction change, '
             r'$V_{\mathrm{hub}} = 12$ m/s'),
    '1.5a': ('Extreme horiztonal wind shear, '
             r'$V_{\mathrm{hub}} = 10$ m/s'),
    '1.5b': ('Extreme vertical wind shear, '
             r'$V_{\mathrm{hub}} = 10$ m/s'),
    '2.1': ('Normal turbulence model, '
            r'$V_{\mathrm{hub}} = 10$ m/s, motor fault'),
    '2.2': ('Normal turbulence model, '
            r'$V_{\mathrm{hub}} = 10$ m/s, GSG fault'),
    '2.3a': ('Extreme operating gust, '
             r'$V_{\mathrm{hub}} = 10$ m/s, motor fault'),
    '2.3b': ('Extreme operating gust, '
             r'$V_{\mathrm{hub}} = 8$ m/s, motor fault'),
    '2.3c': ('Extreme operating gust, '
             r'$V_{\mathrm{hub}} = 12$ m/s, motor fault'),
    '2.4': ('Normal turbulence model, '
            r'$V_{\mathrm{hub}} = 10$ m/s, servo fault'),
    '3.1': ('Normal wind profile, '
            r'$V_{\mathrm{hub}} = 10$ m/s, start-up'),
    '3.2a': ('Extreme operating gust, '
             r'$V_{\mathrm{hub}} = 5$ m/s, start-up'),
    '3.2b': ('Extreme operating gust, '
             r'$V_{\mathrm{hub}} = 8$ m/s, start-up'),
    '3.2c': ('Extreme operating gust, '
             r'$V_{\mathrm{hub}} = 12$ m/s, start-up'),
    '3.2d': ('Extreme operating gust, '
             r'$V_{\mathrm{hub}} = 14$ m/s, start-up'),
    '3.3a': ('Extreme direction change, '
             r'$V_{\mathrm{hub}} = 5$ m/s, start-up'),
    '3.3b': ('Extreme direction change, '
             r'$V_{\mathrm{hub}} = 8$ m/s, start-up'),
    '3.3c': ('Extreme direction change, '
             r'$V_{\mathrm{hub}} = 12$ m/s, start-up'),
    '3.3d': (r'Extreme direction change, '
             r'$V_{\mathrm{hub}} = 14$ m/s, start-up'),
    '4.1': ('Normal wind profile, '
            r'$V_{\mathrm{hub}} = 10$ m/s, shut-down'),
    '4.2a': ('Extreme operating gust, '
             r'$V_{\mathrm{hub}} = 8$ m/s, shut-down'),
    '4.2b': ('Extreme operating gust, '
             r'$V_{\mathrm{hub}} = 12$ m/s, shut-down'),
    '4.2c': ('Extreme operating gust, '
             r'$V_{\mathrm{hub}} = 14$ m/s, shut-down')
}

gflags.DEFINE_string('output_dir', os.path.join(makani.HOME, 'logs/iec_cases'),
                     'Directory in which to output .png file and .html page.  '
                     'This will be created if it does not exist.')

gflags.DEFINE_list('cases', [],
                   'If nonempty, only the specified cases will be run. '
                   'Possible cases are: %s.' %
                   ', '.join(sorted(IEC_CASES.keys())))

FLAGS = gflags.FLAGS


class IecCasesSimClient(client_base.BatchSimClient):
  """Client for an IEC case batch simulation."""

  _BASE_PATH = 'analysis/iec_cases_batch_sim'

  def __init__(self, **kwargs):
    super(IecCasesSimClient, self).__init__(**kwargs)

    if FLAGS.cases:
      self._cases = sorted(FLAGS.cases)
    else:
      self._cases = sorted(IEC_CASES.keys())

  def _GenerateConfigs(self):
    builder = IecConfigBuilder()
    for case in self._cases:
      yield builder.BuildConfig(case)

  @client_base.JsonReducer
  def _ReduceWorkerOutput(self, outputs):
    matplotlib.use('Agg')
    pylab = importlib.import_module('pylab')
    if pylab.rcParams['figure.max_open_warning'] < len(outputs):
      pylab.rcParams['figure.max_open_warning'] = len(outputs) + 1

    if not os.path.exists(FLAGS.output_dir):
      os.makedirs(FLAGS.output_dir)

    image_files = []
    for i in range(len(outputs)):
      case = self._cases[i]
      output = outputs[i]
      pylab.figure(i)

      time = output['time']
      wing_Xg = output['wing_Xg']  # pylint: disable=invalid-name

      pylab.subplot(2, 1, 1)
      pylab.plot(time, wing_Xg['x'],
                 time, wing_Xg['y'],
                 time, wing_Xg['z'])

      if output['sim_successful']:
        pylab.title('IEC Case %s: %s' % (case, IEC_CASES[case]))
      else:
        pylab.title('FAILURE: IEC Case %s: %s' % (case, IEC_CASES[case]),
                    color='r')
      pylab.ylabel('Position [m]')
      pylab.grid()

      pylab.subplot(2, 1, 2)
      pylab.plot(time, [o / 1000.0 for o in output['tether_tension']])
      pylab.xlabel('Time [s]')
      pylab.ylabel('Tension [kN]')
      pylab.grid()

      image_files.append('pos_tension_%s.png' % case.replace('.', '_'))
      pylab.savefig(os.path.join(FLAGS.output_dir, image_files[-1]))

    with open(os.path.join(FLAGS.output_dir, 'index.html'), 'w') as f:
      f.write('<html>\n<head><title>IEC Test Cases</title></head>\n<body>\n')
      for file_name in image_files:
        f.write('<img width="22%%" src="%s">\n' % file_name)
        f.write('</body>\n</html>\n')


class IecConfigBuilder(object):
  """Builds configs for the IEC cases."""

  def __init__(self):
    iec_params = mconfig.MakeParams(FLAGS.wing_model + '.sim.iec_sim', None)
    sys_params = mconfig.MakeParams(FLAGS.wing_model + '.system_params')

    v_in = float(iec_params['v_in'])  # Cut-in wind speed.
    v_out = float(iec_params['v_out'])  # Cut-out wind speed.
    self._v_hub = float(iec_params['v_r1'])  # Rated hub speed.

    # alpha is the wind shear exponent defined in IEC 61400-1:2005 on page 24.
    alpha = 0.2

    # Take the hub height to be the tether length * sin(30 deg.) + height of
    # the GSG.
    z_hub = float(sys_params['tether']['length'] * numpy.sin(numpy.pi / 6.0)
                  + sys_params['ground_frame']['ground_z'])

    # Height at which wind_speed is measured.
    # This assumes that the perch z-axis is parallel to the
    # g-coordinate z-axis are aligned.
    z_sensor = float(sys_params['ground_frame']['ground_z']
                     - sys_params['wind_sensor']['pos_parent'][2])

    # v_hub * shear_correction = wind_speed.
    shear = float((z_sensor / z_hub)**alpha)

    # The *_sensed variables are the speed measured at the ground to
    # arrive at a given speed at the hub assuming the shear model
    # above.
    self._v_in_sensed = v_in * shear
    self._v_out_sensed = v_out * shear
    self._v_hub_sensed = self._v_hub * shear
    self._v_hub_m2_sensed = (self._v_hub - 2.0) * shear  # m2 means minus two.
    self._v_hub_p2_sensed = (self._v_hub + 2.0) * shear  # p2 means plus two.

    self._fault_power_sys_zero = {
        't_start': 20.0,
        't_end': 25.0,
        'component': 'PowerSys/motor_connections[0]',
        'type': sim_types.kSimFaultActuatorZero,
    }

  def BuildConfig(self, case_name):
    """Build a config for a specific IEC case.

    Args:
      case_name: Name of the case.

    Returns:
      A config suitable for running a simulation corresponding to the provided
          case.

    Raises:
      ValueError: `case_name` does not correspond to a known case.
    """

    overrides = {
        'sim': {
            'iec_sim': {},
            'phys_sim': {
                'wind_model': sim_types.kWindModelIec,
                'wind_shear_exponent': 0.2,
            },
            'sim_opt': (sim_types.kSimOptFaults
                        | sim_types.kSimOptPerch
                        | sim_types.kSimOptStackedPowerSystem
                        | sim_types.kSimOptGroundContact
                        | sim_types.kSimOptImperfectSensors),
            'sim_time': 150.0,
        },
        'system': {
            'flight_plan': sim_types.kFlightPlanStartDownwind
        }
    }
    iec_sim = overrides['sim']['iec_sim']
    phys_sim = overrides['sim']['phys_sim']

    # Power production
    # DLC 1.1: NTM V_in < V_hub < V_out, Ultimate
    # DLC 1.2: NTM V_in < V_hub < V_out, Fatigue
    if case_name in ('1.1', '1.2'):
      iec_sim['load_case'] = sim_types.kIecCaseNormalTurbulenceModel
      phys_sim['wind_speed'] = self._v_hub

    # DLC 1.3: ETM V_in < V_hub < V_out, Ultimate
    elif case_name == '1.3':
      iec_sim['load_case'] = sim_types.kIecCaseExtremeTurbulenceModel

    # DLC 1.4: ECD V_hub = V_r - 2, V_r, V_r + 2, Ultimate
    elif case_name.startswith('1.4'):
      iec_sim['event_t_start'] = 60.0
      iec_sim['load_case'] = (
          sim_types.kIecCaseExtremeCoherentGustWithDirectionChange)
      if case_name == '1.4b':
        phys_sim['wind_speed'] = self._v_hub_m2_sensed
      elif case_name == '1.4c':
        phys_sim['wind_speed'] = self._v_hub_p2_sensed

    # DLC 1.5: EWS V_in < V_hub < V_out, Ultimate
    elif case_name == '1.5a':
      iec_sim['event_t_start'] = 20.0
      iec_sim['load_case'] = (
          sim_types.kIecCaseExtremeWindShearHorizontal)
    elif case_name == '1.5b':
      iec_sim['event_t_start'] = 20.0
      iec_sim['load_case'] = (
          sim_types.kIecCaseExtremeWindShearVertical)

    # Power production plus occurrence of fault.
    #
    # I'm not sure how the different types of IEC faults should be interpreted,
    # so I'm assuming rotor-out, servo-out, GSG fails faults.
    #
    # DLC 2.1: NTM V_in < V_hub < V_out, Control system fault, Ultimate
    elif case_name == '2.1':
      iec_sim['load_case'] = sim_types.kIecCaseNormalTurbulenceModel
      phys_sim['wind_speed'] = self._v_hub
      overrides['sim']['faults_sim'] = [self._fault_power_sys_zero]

    # DLC 2.2: NTM V_in < V_hub < V_out, Protection system fault, Ultimate
    elif case_name == '2.2':
      iec_sim['load_case'] = sim_types.kIecCaseNormalTurbulenceModel
      phys_sim['wind_speed'] = self._v_hub
      overrides['sim']['faults_sim'] = [{
          't_start': 20.0,
          't_end': 25.0,
          'component': 'GSG/elevation[0]',
          'type': sim_types.kSimFaultMeasurementRescale,
          'parameters': [0.0]
      }, {
          't_start': 20.0,
          't_end': 25.0,
          'component': 'GSG/elevation[1]',
          'type': sim_types.kSimFaultMeasurementRescale,
          'parameters': [0.0]
      }]

    # DLC 2.3: EOG V_hub = V_r-2, V_r+2, V_out, Electrical fault, Ultimate
    elif case_name.startswith('2.3'):
      iec_sim['event_t_start'] = 20.0
      iec_sim['load_case'] = sim_types.kIecCaseExtremeOperatingGust
      overrides['sim']['faults_sim'] = [self._fault_power_sys_zero]
      if case_name == '2.3b':
        phys_sim['wind_speed'] = self._v_hub_sensed - 2.0
      elif case_name == '2.3c':
        phys_sim['wind_speed'] = self._v_hub_sensed + 2.0

    # DLC 2.4: NTM V_in < V_hub < V_out, Fault, Fatigue
    elif case_name == '2.4':
      iec_sim['load_case'] = sim_types.kIecCaseNormalTurbulenceModel
      phys_sim['wind_speed'] = self._v_hub
      overrides['sim']['faults_sim'] = [{
          't_start': 150.0,
          't_end': 170.0,
          'component': 'Servo[1]',
          'type': sim_types.kSimFaultActuatorZero,
      }]

    # Start up
    # DLC 3.1: NWP V_in < V_hub < V_out, Fatigue
    elif case_name == '3.1':
      iec_sim['load_case'] = sim_types.kIecCaseNormalWindProfile

    # DLC 3.2: EOG V_hub = V_in, V_r-2, V_r+2, V_out, Ultimate
    elif case_name.startswith('3.2'):
      iec_sim['event_t_start'] = 10.0
      iec_sim['load_case'] = sim_types.kIecCaseExtremeOperatingGust
      phys_sim['wind_speed'] = self._v_in_sensed
      if case_name == '3.2b':
        phys_sim['wind_speed'] = self._v_hub_m2_sensed
      elif case_name == '3.2c':
        phys_sim['wind_speed'] = self._v_hub_p2_sensed
      elif case_name == '3.2d':
        phys_sim['wind_speed'] = self._v_out_sensed

    # DLC 3.3: EDC V_hub = V_in, V_r-2, V_r+2, V_out, Ultimate
    elif case_name.startswith('3.3'):
      iec_sim['event_t_start'] = 10.0
      iec_sim['load_case'] = sim_types.kIecCaseExtremeDirectionChange
      phys_sim['wind_speed'] = self._v_in_sensed
      if case_name == '3.3b':
        phys_sim['wind_speed'] = self._v_hub_m2_sensed
      elif case_name == '3.3c':
        phys_sim['wind_speed'] = self._v_hub_p2_sensed
      elif case_name == '3.3d':
        phys_sim['wind_speed'] = self._v_out_sensed

    # Normal shut down
    # DLC 4.1: NWP V_in < V_hub < V_out, Fatigue
    elif case_name == '4.1':
      iec_sim['load_case'] = sim_types.kIecCaseNormalWindProfile

    # DLC 4.2: EOG V_hub = V_r-2, V_r+2, V_out, Ultimate
    elif case_name.startswith('4.2'):
      iec_sim['event_t_start'] = 50.0
      iec_sim['load_case'] = sim_types.kIecCaseExtremeOperatingGust
      if case_name == '4.2a':
        phys_sim['wind_speed'] = self._v_hub_m2_sensed
      elif case_name == '4.2b':
        phys_sim['wind_speed'] = self._v_hub_p2_sensed
      elif case_name == '4.2c':
        phys_sim['wind_speed'] = self._v_out_sensed

    else:
      raise ValueError('Unknown IEC Case: ' + case_name)

    # Missing cases:
    #  - Emergency shut down.
    #  - Parked (standing still or idling).
    #  - Parked and fault conditions.
    #  - Transport assembly, maintenance, and repair.

    return mconfig.MakeParams('common.all_params',
                              overrides_util.PreprocessOverrides(overrides),
                              override_method='derived')


def main(argv):
  client_base.InitMain(argv)
  client = IecCasesSimClient()
  client.Run()


if __name__ == '__main__':
  gflags.RegisterValidator(
      'output_dir',
      lambda o: not os.path.exists(o) or os.path.isdir(o),
      '--output_dir cannot refer to an existing, non-directory object.')
  gflags.RegisterValidator(
      'cases',
      lambda cases: all([c in IEC_CASES for c in cases]),
      'Each element of --cases must refer to a valid IEC case.')

  main(sys.argv)
