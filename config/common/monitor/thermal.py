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

"""Thermal parameters."""
from makani.config import mconfig
from makani.control import system_types
import numpy as np


_DOUBLE_MAX = np.finfo(np.double).max


@mconfig.Config(deps={
    'wing_serial': 'common.wing_serial'
})
def MakeParams(params):
  wing_serial = params['wing_serial']
  if wing_serial == system_types.kWingSerial01:
    return Sn1ThermalParams()
  elif wing_serial in [system_types.kWingSerial04Hover,
                       system_types.kWingSerial04Crosswind,
                       system_types.kWingSerialOktoberKite01]:
    return Sn4ThermalParams()
  elif wing_serial in [system_types.kWingSerial05Hover,
                       system_types.kWingSerial05Crosswind]:
    return Sn5ThermalParams()
  elif wing_serial in [system_types.kWingSerial06Hover,
                       system_types.kWingSerial06Crosswind]:
    return Sn6ThermalParams()
  elif wing_serial in [system_types.kWingSerial07Hover,
                       system_types.kWingSerial07Crosswind]:
    return Sn7ThermalParams()
  else:
    raise ValueError('Missing thermal limits for Wing Serial %s' % wing_serial)


def _CommonThermalParams():
  return {
      'aiomon_default': {
          'very_low': -40.0,
          'low': -10.0,
          'high': 70.0,
          'very_high': 85.0,
      },
      # TODO: Need indicator.
      'bridle_box': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': _DOUBLE_MAX,
          'very_high': _DOUBLE_MAX,
      },
      # TODO: Need indicator.
      'capacitor_box': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': _DOUBLE_MAX,
          'very_high': 85.0,  # Default to that of SN2.
      },
      'detwist': {
          'very_low': -_DOUBLE_MAX,
          'low': 5.0,
          'high': 100.0,
          'very_high': 110.0,
      },
      # TODO: Need indicator.
      'eop_boxes': {  # geo/kurt
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': _DOUBLE_MAX,
          'very_high': 105.0,
      },
      # TODO: Need indicator.
      'fcu_aio_atom_q7': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 75.0,  # Default to that of SN2.
          'very_high': 85.0,  # Default to that of SN2.
      },
      # TODO: Need indicator.
      'fcu_aio_boardcom': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 75.0,  # Default to that of SN2.
          'very_high': 85.0,  # Default to that of SN2.
      },
      # TODO: Need indicator.
      'fcu_flight_recorder': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 75.0,  # Default to that of SN2.
          'very_high': 85.0,  # Default to that of SN2.
      },
      # TODO: Need indicator.
      'fcu_imu_chip': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 75.0,  # Default to that of SN2.
          'very_high': 85.0,  # Default to that of SN2.
      },
      # TODO: Need indicator.
      'gs_slip_ring': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': _DOUBLE_MAX,
          'very_high': 70.0,  # Default to that of SN2.
      },
      # TODO: Need indicator.
      'lipo_batteries': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 75.0,  # Default to that of SN2.
          'very_high': 80.0,  # Default to that of SN2.
      },
      'motor_controller_board': {  # horton
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 75.0,
          'very_high': 85.0,
      },
      'motor_controller_capacitor': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 85.0,
          'very_high': 90.0,
      },
      'motor_controller_heat_plate': {  # b/63139806
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 78.0,
          'very_high': 89.5,
      },
      'motor_controller_module': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 100.0,
          'very_high': 120.0,
      },
      'motor_stator_core': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 100.0,
          'very_high': 110.0,
      },
      'motor_stator_winding': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 145.0,
          'very_high': 160.0,
      },
      'motor_rotor': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 65.0,
          'very_high': 70.0,
      },
      # TODO: Need indicator.
      'rpx_soft_bridle': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 70.0,  # Default to that of SN2.
          'very_high': 80.0,  # Default to that of SN2.
      },
      # TODO: Need indicator.
      'satcontainer_ambient': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': _DOUBLE_MAX,
          'very_high': 60.0,  # Default to that of SN2.
      },
      'servo_controller': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 65.0,  # Default to that of SN2.
          'very_high': 75.0,  # Default to that of SN2.
      },
      # TODO: Need indicator.
      'tether_electronics': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 75.0,
          'very_high': 80.0,
      },
      # TODO: Need indicator.
      'wingside_radio': {
          'very_low': -_DOUBLE_MAX,
          'low': -_DOUBLE_MAX,
          'high': 55.0,  # Default to that of SN2.
          'very_high': 60.0,  # Default to that of SN2.
      }
  }


def Sn1ThermalParams():
  params = _CommonThermalParams()
  params.update({
      # Add SN1-specific thermal limits here.
  })
  return params


def Sn4ThermalParams():
  params = _CommonThermalParams()
  params.update({
      # Add SN4-specific thermal limits here.
  })
  return params


def Sn5ThermalParams():
  params = _CommonThermalParams()
  params.update({
      # Add SN5-specific thermal limits here.
  })
  return params


def Sn6ThermalParams():
  params = _CommonThermalParams()
  params.update({
      # Add SN6-specific thermal limits here.
  })
  return params


def Sn7ThermalParams():
  params = _CommonThermalParams()
  params.update({
      # Add SN7-specific thermal limits here.
  })
  return params
