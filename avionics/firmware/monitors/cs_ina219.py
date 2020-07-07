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

"""Core Switch INA219 hardware monitor configuration."""

from makani.avionics.firmware.drivers import ina219_types
from makani.avionics.firmware.serial import cs_serial_params as rev

ina219_default = {
    'name': '',
    'address': -1,
    'shunt_resistor': -1,
    'bus_voltage': ina219_types.kIna219BusVoltage16V,
    'range': ina219_types.kIna219Range40mv,
    'bus_adc': ina219_types.kIna219Adc128Samples,
    'shunt_adc': ina219_types.kIna219Adc128Samples,
    'mode': ina219_types.kIna219ModeShuntAndBusContinuous,
    'current_max': -1,
    'voltage_limits_percent': [95, 105],
}

ina219_16v_40mv = dict(ina219_default, **{
    'bus_voltage': ina219_types.kIna219BusVoltage16V,
    'range': ina219_types.kIna219Range40mv,
})

ina219_16v_80mv = dict(ina219_default, **{
    'bus_voltage': ina219_types.kIna219BusVoltage16V,
    'range': ina219_types.kIna219Range80mv,
})

ina219_16v_160mv = dict(ina219_default, **{
    'bus_voltage': ina219_types.kIna219BusVoltage16V,
    'range': ina219_types.kIna219Range160mv,
})

ina219_16v_320mv = dict(ina219_default, **{
    'bus_voltage': ina219_types.kIna219BusVoltage16V,
    'range': ina219_types.kIna219Range320mv,
})

ina219_32v_40mv = dict(ina219_default, **{
    'bus_voltage': ina219_types.kIna219BusVoltage32V,
    'range': ina219_types.kIna219Range40mv,
})

ina219_32v_160mv = dict(ina219_default, **{
    'bus_voltage': ina219_types.kIna219BusVoltage32V,
    'range': ina219_types.kIna219Range160mv,
})

rev_aa = [
    dict(ina219_32v_40mv, name='12v', address=0x40, shunt_resistor=0.012),
    dict(ina219_16v_40mv, name='1v2', address=0x42, shunt_resistor=0.012),
    dict(ina219_16v_40mv, name='2v5', address=0x48, shunt_resistor=0.012),
    dict(ina219_16v_80mv, name='3v3', address=0x45, shunt_resistor=0.012),
    dict(ina219_16v_40mv, name='3v3_vrl', address=0x4A, shunt_resistor=0.012,
         voltage_limits_percent=[90, 110]),
]

rev_ac = [
    dict(ina219_32v_160mv, name='12v', address=0x41, shunt_resistor=0.05),
    dict(ina219_16v_160mv, name='1v2', address=0x42, shunt_resistor=0.05),
    dict(ina219_16v_160mv, name='2v5', address=0x48, shunt_resistor=0.05),
    dict(ina219_16v_320mv, name='3v3', address=0x45, shunt_resistor=0.05),
    dict(ina219_16v_320mv, name='3v3_vrl', address=0x4A, shunt_resistor=0.05,
         voltage_limits_percent=[90, 110]),
]

ina219_config = (rev.CsHardware, {
    rev.CsHardware.REV_AA: rev_aa,
    rev.CsHardware.REV_AB: rev_aa,  # No changes from rev_aa.
    rev.CsHardware.REV_AC: rev_ac,
    rev.CsHardware.REV_AD_CLK8: rev_ac,  # No changes from rev_ac
    rev.CsHardware.REV_AD_CLK16: rev_ac,  # No changes from rev_ac
})
