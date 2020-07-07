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

"""Simulation parameters."""
import sys

from makani.config import mconfig
from makani.sim import sim_types as m


@mconfig.Config(deps={
    'aero_sim': mconfig.WING_MODEL + '.sim.aero_sim',
    'buoy_sim': 'base_station.sim.buoy_sim',
    'sea_sim': 'common.sim.sea_sim',
    'constraint_sim': 'base_station.sim.constraint_sim',
    'contact_sim': 'm600.sim.contact_sim',
    'dyno_sim': 'base_station.sim.dyno_sim',
    'faults_sim': 'common.sim.faults_sim',
    'gps_sim': 'common.sim.gps_sim',
    'gs_gps_sim': 'common.sim.gs_gps_sim',
    'gsg_sim': 'base_station.sim.gsg_sim',
    'ground_frame_sim': 'base_station.sim.ground_frame_sim',
    'gs_imu_mount_sim': 'base_station.sim.gs_imu_mount_sim',
    'gs_imus_sim': 'base_station.sim.gs_imus_sim',
    'gs02_sim': 'base_station.sim.gs02_sim',
    'iec_sim': 'm600.sim.iec_sim',
    'joystick_sim': 'common.sim.joystick_sim',
    'loadcell_sim': 'common.sim.loadcell_sim',
    'ode_solver': 'common.sim.ode_solver',
    'high_voltage_harness_sim': 'm600.sim.high_voltage_harness_sim',
    'perch_sim': 'base_station.sim.perch_sim',
    'phys_sim': 'common.sim.phys_sim',
    'pitots_sim': 'm600.sim.pitots_sim',
    'power_sys_sim': 'base_station.sim.power_sys_sim',
    'rotor_sim': 'm600.sim.rotor_sim',
    'servos_sim': 'm600.sim.servos_sim',
    'sim_opt': 'common.sim.sim_options',
    'system': mconfig.WING_MODEL + '.system_params',
    'tether_sim': 'common.sim.tether_sim',
    'winch_sim': 'base_station.sim.winch_sim',
    'wind_sensor_sim': 'common.sim.wind_sensor_sim',
    'wing_imu_mount_sim': 'm600.sim.wing_imu_mount_sim',
    'wing_imus_sim': 'm600.sim.wing_imus_sim',
    'wing_sim': 'common.sim.wing_sim',
})
def MakeParams(params):
  # Interval [s] to wait between publishing SimTelemetry.  The
  # simulator will publish every step if this number is smaller than
  # the system time step.
  telemetry_sample_period = 0.02

  # Time [s] at the beginning of a simulation during which no sensor updates
  # are provided.
  sensor_blackout_duration = 0.1

  sim_params = {
      'sim_opt': params['sim_opt'],
      'ode_solver': params['ode_solver'],
      'sim_time': sys.float_info.max,
      'random_seed_offset': 0,
      'sensor_blackout_duration': sensor_blackout_duration,
      'telemetry_sample_period': telemetry_sample_period,
      'phys_sim': params['phys_sim'],
      'iec_sim': params['iec_sim'],
      'wing_sim': params['wing_sim'],
      'aero_sim': params['aero_sim'],
      'contact_sim': params['contact_sim'],
      'ground_frame_sim': params['ground_frame_sim'],
      'high_voltage_harness_sim': params['high_voltage_harness_sim'],
      'tether_sim': params['tether_sim'],
      'buoy_sim': params['buoy_sim'],
      'sea_sim': params['sea_sim'],
      'perch_sim': params['perch_sim'],
      'gs02_sim': params['gs02_sim'],
      'rotor_sim': params['rotor_sim'],
      'constraint_sim': params['constraint_sim'],
      'dyno_sim': params['dyno_sim'],
      'faults_sim': params['faults_sim'],
      'power_sys_sim': params['power_sys_sim'],
      'servos_sim': params['servos_sim'],
      'winch_sim': params['winch_sim'],
      'wing_imu_mount_sim': params['wing_imu_mount_sim'],
      'wing_imus_sim': params['wing_imus_sim'],
      'gs_imu_mount_sim': params['gs_imu_mount_sim'],
      'gs_imus_sim': params['gs_imus_sim'],
      'loadcell_sim': params['loadcell_sim'],
      'pitots_sim': params['pitots_sim'],
      'wind_sensor_sim': params['wind_sensor_sim'],
      'gps_sim': params['gps_sim'],
      'gs_gps_sim': params['gs_gps_sim'],
      'gsg_sim': params['gsg_sim'],
      'joystick_sim': params['joystick_sim'],
  }
  assert mconfig.MatchesCStruct(sim_params, m.SimParams)
  return sim_params
