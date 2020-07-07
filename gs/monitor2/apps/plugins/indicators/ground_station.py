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

""""Monitor indicators from the ground station."""

import collections
import ctypes
from datetime import datetime
import operator

from makani.analysis.checks import avionics_util
from makani.analysis.control import geometry
from makani.avionics.common import encoder_types
from makani.avionics.common import pack_avionics_messages
from makani.avionics.common import plc_messages
from makani.avionics.common import winch_messages
from makani.avionics.network import aio_labels
from makani.common.c_math import util as c_util
from makani.control import common as control_common
from makani.control import control_types
from makani.control import sensor_util
from makani.control import system_params
from makani.control import system_types
from makani.gs.monitor import monitor_params
from makani.gs.monitor2.apps.layout import indicator
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import avionics
from makani.gs.monitor2.apps.plugins.indicators import gs_model
from makani.gs.monitor2.apps.receiver import aio_util
from makani.lib.python import c_helpers
from makani.lib.python import ctype_util
from makani.lib.python import struct_tree

import numpy

ACTUATOR_STATE_HELPER = c_helpers.EnumHelper('ActuatorState',
                                             pack_avionics_messages,
                                             exclude='ActuatorStateCommand')

GS_NODE = None

AMO_4306_FLAG_HELPER = c_helpers.EnumHelper('Amo4306', encoder_types)

GROUND_STATION_ERROR_HELPER = c_helpers.EnumHelper(
    'GsErrorFlag', plc_messages)

GROUND_STATION_MODE_HELPER = c_helpers.EnumHelper(
    'GroundStationMode', plc_messages)

GROUND_STATION_WARNING_HELPER = c_helpers.EnumHelper(
    'GsWarningFlag', plc_messages)

DRUM_ENCODER_WARNINGS_HELPER = c_helpers.EnumHelper(
    'GsDrumEncodersWarning', pack_avionics_messages)

DRUM_ENCODER_ERRORS_HELPER = c_helpers.EnumHelper(
    'GsDrumEncodersError', pack_avionics_messages)

DRUM_SENSORS_HELPER = c_helpers.EnumHelper(
    'DrumLabel', aio_labels, prefix='kDrumSensors')

PLATFORM_SENSORS_HELPER = c_helpers.EnumHelper(
    'PlatformSensorsLabel', aio_labels, prefix='kPlatformSensors')

WINCH_PROXIMITY_FLAG_HELPER = c_helpers.EnumHelper(
    'WinchProximity', winch_messages)

PERCH_ENCODER_WARNINGS_HELPER = c_helpers.EnumHelper(
    'GsPerchEncodersWarning', pack_avionics_messages)

PERCH_ENCODER_ERRORS_HELPER = c_helpers.EnumHelper(
    'GsPerchEncodersError', pack_avionics_messages)

SYSTEM_PARAMS = system_params.GetSystemParams().contents

MONITOR_PARAMS = monitor_params.GetMonitorParams().contents


def GetOrNone(value):
  return value if struct_tree.IsValidElement(value) else None


class BasePlatformSensorsIndicator(indicator.BaseAttributeIndicator):

  def __init__(self, name=None, source_short_names=None, **base_kwargs):
    self._sources = (PLATFORM_SENSORS_HELPER.ShortNames()
                     if source_short_names is None else source_short_names)
    super(BasePlatformSensorsIndicator, self).__init__(
        [('PlatformSensors', 'PlatformSensors' + source)
         for source in self._sources],
        name, **base_kwargs)


class BaseGroundStationWeatherIndicator(indicator.SingleAttributeIndicator):

  def __init__(self, name=None, **base_kwargs):
    super(BaseGroundStationWeatherIndicator, self).__init__(
        ('GroundStationWeather', None), name, **base_kwargs)


class BaseGroundStationStatusIndicator(indicator.SingleAttributeIndicator):

  def __init__(self, name=None, **base_kwargs):
    super(BaseGroundStationStatusIndicator, self).__init__(
        ('GroundStationStatus', None), name, **base_kwargs)


class BaseDrumSensorsIndicator(indicator.BaseAttributeIndicator):

  def __init__(self, name=None, source_short_names=None, **base_kwargs):
    self._sources = (DRUM_SENSORS_HELPER.ShortNames()
                     if source_short_names is None else source_short_names)
    super(BaseDrumSensorsIndicator, self).__init__(
        [('DrumSensors', 'DrumSensors' + source) for source in self._sources],
        name, **base_kwargs)


class BaseWinchPlcIndicator(indicator.SingleAttributeIndicator):

  def __init__(self, name=None, source_short_names=None):
    super(BaseWinchPlcIndicator, self).__init__(
        ('GroundStationWinchStatus', 'PlcTophat'), name)


class AirDensityIndicator(BaseGroundStationWeatherIndicator):

  def __init__(self):
    super(AirDensityIndicator, self).__init__('Air Density')

  def _Filter(self, weather):
    phys = SYSTEM_PARAMS.phys

    # Temperature [Celcius].
    temperature = weather.weather.temperature

    # Pressure [Pa].
    p = weather.weather.pressure * 100.0

    # Relative humidity [fraction].
    humidity = weather.weather.humidity / 100.0

    # Density [kg/m^3] of dry air (calculated).
    # TODO: Consider using the validity checking available
    # from the CalcAirDensity function to check whether the arguments
    # are sensible.
    rho = sensor_util.CalcAirDensity(p, temperature, humidity, None)

    # Density [kg/m^3] (configured)
    rho_configured = phys.rho

    text = '{rho: 5.3f} kg/m^3 ({percent: 5.1f}% of configured)'.format(
        rho=rho, percent=(100.0 * rho / rho_configured))

    # TODO(b/28324436): These are bits in the weather status that we
    # can ignore.
    dont_care_status_bits = 0xb

    if weather.weather_latency * 1.0e-6 > 2.0:
      text = 'Weather data is stale.'
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif weather.weather.status & ~dont_care_status_bits & 0xff:
      text = 'Weather data is invalid.'
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif abs(rho - rho_configured) / rho_configured > 0.05:
      # Indicate a warning if the computed density differs from the
      # configured density by more than 5%.
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    return text, stoplight


class MaxServoTempIndicator(indicator.BaseAttributeIndicator):
  """Indicator to display maximum of PlcGs02 servo temperatures."""

  def __init__(self):
    super(MaxServoTempIndicator, self).__init__([
        ('GroundStationStatus', 'PlcGs02', 'status.azimuth.motor'),
        ('GroundStationStatus', 'PlcGs02', 'status.detwist.motor'),
        ('GroundStationStatus', 'PlcGs02', 'status.levelwind.motor'),
        ('GroundStationStatus', 'PlcGs02', 'status.winch.motor'),
    ], 'Max Servo Temp [C]')

  def _IsValidInput(self, *attributes):
    for attr in attributes:
      if attr is None: return False
    return True

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, azimuth, detwist, lvlwind, winch):
    temps = {'Azimuth': max(azimuth[0].temperature, azimuth[1].temperature),
             'Detwist': max(detwist[0].temperature, detwist[1].temperature),
             'Levelwind': max(lvlwind[0].temperature, lvlwind[1].temperature),
             'Winch': max(winch[0].temperature, winch[1].temperature)}

    (servo, max_temp) = max(temps.iteritems(), key=operator.itemgetter(1))
    return '%s: %5.1f' % (servo, max_temp), stoplights.STOPLIGHT_NORMAL


class WeatherSensorIndicator(BaseGroundStationWeatherIndicator):

  def __init__(self):
    super(WeatherSensorIndicator, self).__init__('Weather', default='--\n\n')

  def _Filter(self, weather):

    text = ('Temp:  {temperature: 5.1f} C, '
            'Pres: {pressure_pa: 6.0f} Pa\n'
            'DewPt: {dewpoint: 5.1f} C, '
            'RH: {humidity: 3.0f}%')

    measurements = {
        'pressure_pa': weather.weather.pressure * 100.0,
        'temperature': weather.weather.temperature,
        'dewpoint': weather.weather.dewpoint,
        'humidity': weather.weather.humidity,
    }
    text = text.format(**measurements)

    status = weather.weather.status
    # TODO(b/28324436): These are bits in the weather status that we
    # can ignore.
    dont_care_status_bits = 0xb

    if (((status | dont_care_status_bits) != dont_care_status_bits) or
        weather.weather_latency * 1.0e-6 > 2.0):
      # Weather sensor data is stale or invalid.
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    return text, stoplight


class WindSensorSpeedIndicator(BaseGroundStationWeatherIndicator):

  def __init__(self):
    super(WindSensorSpeedIndicator, self).__init__('Wind [m/s]')

  def _Filter(self, weather):
    velocity = weather.wind.wind_velocity
    text = '%2.1f (u:% 3.1f v:% 3.1f w:% 3.1f)' % (
        numpy.linalg.norm(velocity[:]), velocity[0], velocity[1], velocity[2])

    # An implausible speed-of-sound measurement presumably also
    # indicates an incorrect wind speed measurement.  The range given
    # here covers -25 to +50 degrees Celsius.  The relationship is
    # c_air = (331.45 m/s) * sqrt(1 + T / 273.15 degrees Celsius) where
    # T is the temperature in Celsius.  Effects due to humidity are of
    # order one percent at 100% humidity and high temperature.
    # Reference: http://www.rane.com/pdf/eespeed.pdf
    speed_of_sound = weather.wind.speed_of_sound
    if (weather.wind_latency * 1.0e-6 > 1.0 or
        weather.wind.status != 0):
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif speed_of_sound < 315.0 or speed_of_sound > 360.0:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    return text, stoplight


class WindSensorStatusIndicator(BaseGroundStationWeatherIndicator):

  def __init__(self):
    super(WindSensorStatusIndicator, self).__init__('Wind Sensor Status')

  def _Filter(self, weather):
    return str(weather.wind.status), stoplights.STOPLIGHT_NORMAL


class LevelwindElevationIndicator(BasePlatformSensorsIndicator):

  def __init__(self):
    super(LevelwindElevationIndicator, self).__init__('Level Ele [deg]', None)

  def _Filter(self, *platform_sensors):
    any_stale = False
    results = []
    for platform_sensor, short_name in zip(platform_sensors, self._sources):
      if platform_sensor is None:
        text = '--'
        any_stale = True
      else:
        ele = platform_sensor.encoders.levelwind_ele
        text = '% 4.0f' % numpy.rad2deg(ele)
      results.append('%s: %s' % (short_name, text))

    if any_stale:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL
    return '\n'.join(results), stoplight


class BaseGsgAnglesIndicator(BaseDrumSensorsIndicator):

  def __init__(self, name, encoder_field, status_name, source_short_names):
    super(BaseGsgAnglesIndicator, self).__init__(name, source_short_names)
    self._encoder_field = encoder_field
    self._status_name = status_name

  def _Filter(self, *drum_sensors):
    any_stale = False
    any_invalid = False
    results = []
    for drum_sensor, short_name in zip(drum_sensors, self._sources):
      if drum_sensor is None:
        text = '--'
        any_stale = True
      else:
        status = drum_sensor.encoders.status
        gsg_angle = getattr(drum_sensor.encoders, self._encoder_field)
        text = '% 3.0f' % numpy.rad2deg(gsg_angle)
        if avionics_util.CheckWarning(
            status, DRUM_ENCODER_WARNINGS_HELPER.Value(self._status_name)):
          text += ' Warning'
          any_invalid = True
        if avionics_util.CheckError(
            status, DRUM_ENCODER_ERRORS_HELPER.Value(self._status_name)):
          text += ' Error'
          any_invalid = True
      results.append('%s: %s' % (short_name, text))

    if any_stale or any_invalid:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL
    return '\n'.join(results), stoplight


class GsgAngleCrosswindDictChart(indicator.DictChartIndicator):
  """Displays a Dict value list and 2D view of yoke vs. term."""

  def DrawPolygonFromLimits(self, low_x, high_x, low_y, high_y, color='yellow'):
    # Return polygon coordinates from low and high limit values
    # and specified color string.
    polygon_x = [low_x, low_x, high_x, high_x, low_x]
    polygon_y = [low_y, high_y, high_y, low_y, low_y]
    return {'x': polygon_x, 'y': polygon_y, 'color': color}

  def __init__(self, src_names=None):
    # Form rectangular polygons from yoke and term monitor limits.
    yoke_limits = MONITOR_PARAMS.est.crosswind_tether_angles[0]
    term_limits = MONITOR_PARAMS.est.crosswind_tether_angles[1]
    yellow_limit = self.DrawPolygonFromLimits(
        yoke_limits.low, yoke_limits.high,
        term_limits.low, term_limits.high, 'yellow')
    red_limit = self.DrawPolygonFromLimits(
        yoke_limits.very_low, yoke_limits.very_high,
        term_limits.very_low, term_limits.very_high, 'red')
    attributes = [('DrumSensors', 'DrumSensors' + src) for src in src_names]
    self._sources = src_names

    super(GsgAngleCrosswindDictChart, self).__init__(
        attributes, name='Crosswind',
        xlim=[-100.0, 100.0], ylim=[-100.0, 100.0],
        xlabel='Yoke [deg]', ylabel='Term [deg]',
        background_polygons=[yellow_limit, red_limit],
        line_properties={
            'gsg_azi_ele': {'color': 'blue'},
        },
        history_len=50, show_legend=True,
        keys=['Yoke [deg]', 'Term [deg]'],
        num_xticks=5, num_yticks=5)

  @indicator.ReturnIfInputInvalid(None, None, None,
                                  stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *messages):
    pos_pointer = {'x': None, 'y': None}
    value_dict = {}
    for i_src in range(len(self._sources)):
      if messages[i_src]:
        pos_pointer['x'] = numpy.rad2deg(messages[i_src].encoders.gsg_azi)
        value_dict['Yoke [deg]'] = pos_pointer['x']
        pos_pointer['y'] = numpy.rad2deg(messages[i_src].encoders.gsg_ele)
        value_dict['Term [deg]'] = pos_pointer['y']

    return (value_dict, {'gsg_azi_ele': pos_pointer}, {},
            stoplights.STOPLIGHT_NORMAL)


class GsgElevationIndicator(BaseGsgAnglesIndicator):

  def __init__(self, source_short_names=None):
    super(GsgElevationIndicator, self).__init__(
        'GSG Elev [deg]', 'gsg_ele', 'GsgElevation', source_short_names)


class GsgAzimuthIndicator(BaseGsgAnglesIndicator):

  def __init__(self, source_short_names=None):
    super(GsgAzimuthIndicator, self).__init__(
        'GSG Azi [deg]', 'gsg_azi', 'GsgAzimuth', source_short_names)


class WinchProximityIndicator(BaseWinchPlcIndicator):

  def __init__(self):
    super(WinchProximityIndicator, self).__init__('Proximity')

  def _Filter(self, winch_plc):
    return (self._DisplayProximitySensors(winch_plc.plc.proximity),
            stoplights.STOPLIGHT_NORMAL)

  def _DisplayProximitySensors(self, flag):
    references = ['FinalA', 'FinalB', 'EarlyA', 'EarlyB']
    symbols = []
    for reference in references:
      if flag & WINCH_PROXIMITY_FLAG_HELPER.Value(reference):
        symbols.append('1')
      else:
        symbols.append('-')
    return 'Final [%s] Early' % ''.join(symbols)


class WinchArmedIndicator(BaseWinchPlcIndicator):

  def __init__(self):
    super(WinchArmedIndicator, self).__init__('Winch Armed')

  def _Filter(self, winch_plc):
    if common.IsActuatorStateArmed(winch_plc.plc.state):
      return 'Armed', stoplights.STOPLIGHT_NORMAL
    else:
      return 'Disarmed', stoplights.STOPLIGHT_WARNING


class PerchAzimuthIndicator(BasePlatformSensorsIndicator):
  """Indicator for Perch Azimuth."""

  def __init__(self):
    super(PerchAzimuthIndicator, self).__init__('Perch Azi [deg]')

  def _Filter(self, *platform_sensors):
    all_stale = True
    any_stale = False
    stoplight = stoplights.STOPLIGHT_NORMAL
    results = []

    # TODO(b/25368865): Add 'B' to the following list once the
    # hardware is installed.
    expected_short_names = ['A']

    for platform_sensor, short_name in zip(platform_sensors, self._sources):
      if not platform_sensor:
        text = '--'
        if short_name in expected_short_names: any_stale = True
      else:
        perch_azi = platform_sensor.encoders.perch_azi
        flags = platform_sensor.encoders.perch_azi_flags
        status = platform_sensor.encoders.status

        if short_name in expected_short_names: all_stale = False
        flag_string, flag_stoplight = self._FlagToString(flags)
        text = '% 3.0f' % numpy.rad2deg(perch_azi) + flag_string
        stoplight = stoplights.MostSevereStoplight(stoplight, flag_stoplight)
        status_string, status_stoplight = self._StatusToString(status)
        if status_string:
          text += ', ' + status_string
        stoplight = stoplights.MostSevereStoplight(stoplight, status_stoplight)
      results.append('%s: %s' % (short_name, text))

    if all_stale:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif any_stale:
      stoplight = stoplights.MostSevereStoplight(
          stoplight, stoplights.STOPLIGHT_WARNING)
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL
    return '\n'.join(results), stoplight

  def _StatusToString(self, status):
    warnings = []
    if avionics_util.CheckWarning(
        status, PERCH_ENCODER_WARNINGS_HELPER.Value('PerchAzimuth')):
      warnings.append('Warning status')
    if avionics_util.CheckError(
        status, PERCH_ENCODER_ERRORS_HELPER.Value('PerchAzimuth')):
      warnings.append('Error status')

    text = ', '.join(warnings)
    stoplight = (stoplights.STOPLIGHT_WARNING if warnings else
                 stoplights.STOPLIGHT_NORMAL)
    return text, stoplight

  def _FlagToString(self, flag):
    warnings = []
    if flag & AMO_4306_FLAG_HELPER.Value('InvalidParity'):
      warnings.append('Invalid parity')
    if flag & AMO_4306_FLAG_HELPER.Value('Warning'):
      warnings.append('Encoder warning')
    if flag & AMO_4306_FLAG_HELPER.Value('Error'):
      warnings.append('Encoder error')

    text = ', '.join(warnings)
    stoplight = (stoplights.STOPLIGHT_WARNING if warnings else
                 stoplights.STOPLIGHT_NORMAL)
    return text, stoplight


class DrumStateIndicator(BaseWinchPlcIndicator):

  def __init__(self):
    super(DrumStateIndicator, self).__init__('Drum')

  def _Filter(self, winch_plc):
    drum_velocity_cal = SYSTEM_PARAMS.winch.drum_velocity_cal
    drum_velocity_cal_ptr = ctypes.cast(
        ctypes.pointer(drum_velocity_cal),
        ctypes.POINTER(c_util.CalParams))
    velocity = c_util.ApplyCal(winch_plc.plc.winch_drum.velocity,
                               drum_velocity_cal_ptr)
    text = 'error: %d, position: %.1f rad, velocity: %.2f rad/s' % (
        winch_plc.plc.flags.error, winch_plc.plc.drum_position,
        velocity)
    return text, stoplights.STOPLIGHT_NORMAL


class PlcStatusIndicator(BaseWinchPlcIndicator):

  def __init__(self):
    super(PlcStatusIndicator, self).__init__('PLC Status')

  def _Filter(self, winch_plc):
    text = 'winch: %d\nlevelwind:%d' % (
        winch_plc.plc.winch_drum.flags.status,
        winch_plc.plc.levelwind.flags.status)
    return text, stoplights.STOPLIGHT_NORMAL


class WindGustIndicator(indicator.BaseAttributeIndicator):

  def __init__(self, **base_kwargs):
    self._warning_threshold = 3
    super(WindGustIndicator, self).__init__([
        ('filtered', 'wind_gust', 'tke'),
        ('filtered', 'wind_gust', 'valid'),
    ], 'Wind Gust [(m/s)^2]', **base_kwargs)

  def _Filter(self, tke, valid):
    if not valid:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE
    # Obtain the turbulence kinetic energy.
    text = 'tke : %5.3f' % tke
    if tke > self._warning_threshold:
      return text, stoplights.STOPLIGHT_WARNING
    return text, stoplights.STOPLIGHT_NORMAL


class WindIndicator(indicator.BaseAttributeDictChart):
  """Wind indicator that shows speed, direction, and gust."""

  def __init__(self, **base_kwargs):
    self._speed_label = 'speed [m/s]'
    self._lowpass_speed_label = 'filtered [m/s]'
    self._raw_speed_label = 'instant [m/s]'
    self._wind_aloft_label = 'wind-aloft [m/s]'
    self._alignment_label = 'alignment [deg]'
    self._direction_label = 'direction [deg]'
    self._sidewind_label = 'side wind [m/s]'
    self._source_label = 'source'
    self._ti_label = 'turb. int [ # ]'
    self._tke_label = 'tke [(m/s)^2]'
    super(WindIndicator, self).__init__(
        [('filtered', 'wind_gust', 'turbulence_intensity'),
         ('filtered', 'wind_gust', 'tke'),
         ('filtered', 'wind_gust', 'valid'),
         ('GroundStationWeather', None),
         ('GroundStationStatus', None, 'status'),
         ('ControlTelemetry', None),
         ('GroundEstimate', None),
         ('PlatformSensors', None, 'encoders.perch_azi')],
        [self._speed_label, self._alignment_label, self._sidewind_label,
         self._direction_label, self._ti_label, self._tke_label,
         self._source_label], 'Wind',
        chart_keys=[self._lowpass_speed_label, self._raw_speed_label,
                    self._wind_aloft_label],
        panel_ratio=0.42, ylim=[0.0, 20.0], precision=1, num_yticks=5,
        **base_kwargs)

  def _GetStateEstWind(self, wind_g, alignment_ref_g):
    """Get wind from state est.

    Args:
      wind_g: The "going to" wind direction vector.
      alignment_ref_g: Reference angle for the direction the wind comes from.

    Returns:
      values: The dict for each field and their values.
      stoplight: The stoplight for the wind.
      message: Additional message for the user.
    """
    values = {}
    if wind_g.valid:
      values[self._lowpass_speed_label] = wind_g.speed_f
      values[self._direction_label] = numpy.rad2deg(
          common.WindAziToWindNed(wind_g.dir_f))
      if alignment_ref_g is None:
        values[self._alignment_label] = '--'
        values[self._sidewind_label] = '--'
      else:
        # wind_g.dir_f + numpy.pi is where wind comes from in ground frame.
        alignment = wind_g.dir_f + numpy.pi - alignment_ref_g
        values[self._alignment_label] = c_util.Wrap(
            numpy.rad2deg(alignment), -180.0, 180.0)
        values[self._sidewind_label] = wind_g.speed_f * numpy.sin(alignment)
      values[self._source_label] = 'est.'
      stoplight = stoplights.STOPLIGHT_NORMAL
      message = ''
    else:
      stoplight = stoplights.STOPLIGHT_WARNING
      message = 'Invalid values'
    return values, stoplight, message

  def _GetGsWind(self, weather, gs_status, alignment_ref_g, ground_estimate,
                 platformsensor_azimuth):
    """Get wind from ground station.

    Args:
      weather: The `weather` struct.
      gs_status: The `gs_status` struct.
      alignment_ref_g: Reference angle for the direction the wind comes from.
      ground_estimate: Estimate from ground station.
      platformsensor_azimuth: Ground station azimuth reading in PlatformSensors.

    Returns:
      values: The dict for each field and their values.
      stoplight: The stoplight for the wind.
      message: Additional message for the user.
    """
    speed = alignment = sidewind = direction = '--'
    stoplight = stoplights.STOPLIGHT_UNAVAILABLE

    gs_valid = (struct_tree.IsValidElement(weather) and
                struct_tree.IsValidElement(gs_status) and
                struct_tree.IsValidElement(ground_estimate))

    if gs_valid:
      velocity = weather.wind.wind_velocity
      wind_g = avionics.WindWsToWindG(velocity, ground_estimate.pqr,
                                      ground_estimate.dcm_g2p)
      speed = numpy.linalg.norm([wind_g.x, wind_g.y, wind_g.z])
      # `dir_rad` is where wind comes from.
      dir_rad = numpy.arctan2(-wind_g.y, -wind_g.x)
      if alignment_ref_g is not None:
        alignment = dir_rad - alignment_ref_g
        sidewind = speed * numpy.sin(alignment)
        alignment = c_util.Wrap(numpy.rad2deg(alignment), -180.0, 180.0)
      direction = c_util.Wrap(
          numpy.rad2deg(dir_rad + SYSTEM_PARAMS.ground_frame.heading),
          0.0, 360.0)

      if (weather.wind_latency * 1.0e-6 < 1.0 and
          weather.wind.status == 0):
        stoplight = stoplights.STOPLIGHT_NORMAL

    values = {self._raw_speed_label: speed,
              self._alignment_label: alignment,
              self._sidewind_label: sidewind,
              self._direction_label: direction, self._source_label: 'raw'}
    return values, stoplight, ''

  def _BoomPointsAtWing(self, flight_mode):
    if flight_mode == control_types.kFlightModePerched:
      return True

    if flight_mode in [
        control_types.kFlightModeHoverTransOut,
        control_types.kFlightModeHoverTransformGsUp,
        control_types.kFlightModeHoverTransformGsDown]:
      return False

    return control_common.AnyHoverFlightMode(flight_mode)

  def _GsgPointsAtWing(self, flight_mode):
    if flight_mode == control_types.kFlightModeHoverTransOut:
      return True

    if flight_mode in [
        control_types.kFlightModeHoverTransformGsUp,
        control_types.kFlightModeHoverTransformGsDown]:
      return False

    return control_common.AnyCrosswindFlightMode(flight_mode)

  def _Filter(self, ti, tke, gust_valid, weather, gs_status, control_telemetry,
              ground_estimate, platformsensor_azimuth):
    est_valid = control_telemetry and control_telemetry.state_est.wind_g.valid
    gs_valid = (struct_tree.IsValidElement(weather) and
                struct_tree.IsValidElement(ground_estimate) and
                (SYSTEM_PARAMS.gs_model != system_types.kGroundStationModelGSv2
                 or struct_tree.IsValidElement(gs_status)))

    alignment_ref_base = None
    if SYSTEM_PARAMS.gs_model == system_types.kGroundStationModelGSv2:
      if control_telemetry and gs_status:
        perch_azi = gs_status.azimuth.position
        if self._BoomPointsAtWing(control_telemetry.flight_mode):
          alignment_ref_base = perch_azi
          alignment_ref_base += (
              SYSTEM_PARAMS.ground_station.gs02.boom_azimuth_p - numpy.pi)
        elif self._GsgPointsAtWing(control_telemetry.flight_mode):
          alignment_ref_base = perch_azi
    else:
      # For top hat, assuming the "virtual perch" is pointing towards the
      # container henge.
      alignment_ref_base = numpy.pi

    if est_valid:
      wind_g = control_telemetry.state_est.wind_g
      est_info, est_stoplight, est_message = self._GetStateEstWind(
          wind_g, alignment_ref_base)

    if gs_valid:
      gs_info, gs_stoplight, gs_message = self._GetGsWind(
          weather, gs_status, alignment_ref_base, ground_estimate,
          platformsensor_azimuth)

    # Default to the wind measurement from the estimator, if available.
    if est_valid:
      stoplight = est_stoplight
      message = est_message
      info = est_info
      info[self._speed_label] = info[self._lowpass_speed_label]
      if gs_valid:
        info[self._raw_speed_label] = gs_info[self._raw_speed_label]
    else:  # Estimator is not available.
      if gs_valid:
        stoplight = gs_stoplight
        message = gs_message
        info = gs_info
        info[self._speed_label] = info[self._raw_speed_label]
      else:
        info = {self._source_label: '--'}
        stoplight = stoplights.STOPLIGHT_UNAVAILABLE
        message = ''

    if struct_tree.IsValidElement(control_telemetry):
      wind_aloft = control_telemetry.state_est.wind_aloft_g.speed_f
      info[self._wind_aloft_label] = wind_aloft

    # Obtain the turbulence intensity.
    info[self._ti_label] = '%5.3f' % ti if gust_valid else '--'
    info[self._tke_label] = '%5.3f' % tke if gust_valid else '--'

    current_time = (
        (datetime.now() - datetime.utcfromtimestamp(0)).total_seconds())
    timestamps = {
        self._lowpass_speed_label: current_time,
        self._raw_speed_label: current_time,
        self._wind_aloft_label: current_time,
    }
    return timestamps, info, stoplight, message


def WrapDetwist(angle):
  return c_util.Wrap(
      angle, 0.0, 2.0 * numpy.pi * pack_avionics_messages.TETHER_DETWIST_REVS)


def WrapDetwistError(detwist_cmd, detwist_pos):
  return c_util.Wrap(
      detwist_cmd - detwist_pos,
      -numpy.pi * pack_avionics_messages.TETHER_DETWIST_REVS,
      numpy.pi * pack_avionics_messages.TETHER_DETWIST_REVS)


def StoplightForDetwistPosError(detwist_cmd, detwist_obs):
  abs_error = abs(WrapDetwistError(detwist_cmd, detwist_obs))
  if abs_error < numpy.pi / 8.0:
    return stoplights.STOPLIGHT_NORMAL
  elif abs_error < numpy.pi / 6.0:
    return stoplights.STOPLIGHT_WARNING
  else:
    return stoplights.STOPLIGHT_ERROR


class BaseDetwistChart(indicator.BaseAttributeDictChart):
  """Plots error in the detwist position."""

  def __init__(self, labels, name, additional_attributes=None,
               **widget_kwargs):
    attributes = [
        ('filtered', 'merge_tether_down',
         'tether_down.control_command.detwist_angle'),
        ('filtered', 'merge_tether_down', 'valid'),
        ('filtered', 'merge_tether_down',
         'tether_down.control_command.no_update_count'),
        aio_util.TetherDownTimestampAttributePath(),
        ('GroundStationStatus', 'PlcGs02', 'status.detwist.position'),
        aio_util.AioTimestampAttributePath('GroundStationStatus',
                                           'PlcGs02'),
    ]
    if additional_attributes:
      attributes += additional_attributes

    super(BaseDetwistChart, self).__init__(
        attributes, labels, name, **widget_kwargs)

  def _IsValidCmd(self, tether_down_valid, no_update_count):
    return (tether_down_valid and
            no_update_count < common.MAX_NO_UPDATE_COUNT_CONTROLLER_COMMAND)


class DetwistPositionChart(BaseDetwistChart):
  """Plots detwist position."""

  def __init__(self, **widget_kwargs):
    self._obs_label = 'obs'
    self._cmd_label = 'cmd'
    super(DetwistPositionChart, self).__init__(
        [self._obs_label, self._cmd_label], 'Detwist Pos. (deg)',
        precision=1, **widget_kwargs)

  def _Filter(self, detwist_cmd, tether_down_valid, no_update_count,
              detwist_cmd_timestamp, detwist_pos, detwist_pos_timestamp):

    is_cmd_valid = self._IsValidCmd(tether_down_valid, no_update_count)
    is_obs_valid = struct_tree.IsValidElement(detwist_pos)
    values = {
        self._cmd_label: numpy.rad2deg(detwist_cmd) if is_cmd_valid else None,
        self._obs_label: (numpy.rad2deg(WrapDetwist(detwist_pos))
                          if is_obs_valid else None),
    }

    timestamps = {
        self._cmd_label: detwist_cmd_timestamp if is_cmd_valid else None,
        self._obs_label: detwist_pos_timestamp if is_obs_valid else None,
    }

    if is_cmd_valid and is_obs_valid:
      # Check the error if both command and observation messages are received.
      stoplight = StoplightForDetwistPosError(detwist_cmd, detwist_pos)
    elif not (is_cmd_valid or is_obs_valid):
      # Gray if none of command nor observation is received.
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    else:
      # Yellow if any of command or observation messages is missing.
      stoplight = stoplights.STOPLIGHT_WARNING

    return timestamps, values, stoplight


class DetwistErrorChart(BaseDetwistChart):
  """Plots error in the detwist position."""

  def __init__(self, **widget_kwargs):
    self._error_label = 'detwist error'
    super(DetwistErrorChart, self).__init__(
        [self._error_label], 'Detwist Error (deg)',
        [('ControlTelemetry', None, 'flight_mode')],
        precision=1, ylim=[-45.0, 45.0], **widget_kwargs)

  def _Filter(self, detwist_cmd, tether_down_valid, no_update_count,
              detwist_cmd_timestamp, detwist_pos, detwist_pos_timestamp,
              flight_mode):
    is_valid = (self._IsValidCmd(tether_down_valid, no_update_count) and
                struct_tree.IsValidElement(detwist_pos))
    if is_valid:
      detwist_error = WrapDetwistError(detwist_cmd, detwist_pos)
      values = {
          self._error_label: numpy.rad2deg(detwist_error)
      }
      timestamps = {
          self._error_label: detwist_pos_timestamp
      }
      if common.AnyDetwistFlightMode(flight_mode):
        stoplight = StoplightForDetwistPosError(detwist_cmd, detwist_pos)
      else:
        stoplight = stoplights.STOPLIGHT_ANY
      return timestamps, values, stoplight
    else:
      return None, None, stoplights.STOPLIGHT_UNAVAILABLE


class DetwistErrorIndicator(indicator.BaseAttributeIndicator):
  """Show error in the detwist position."""

  def __init__(self):
    super(DetwistErrorIndicator, self).__init__([
        ('filtered', 'merge_tether_down',
         'tether_down.control_command.detwist_angle'),
        ('filtered', 'merge_tether_down', 'valid'),
        ('filtered', 'merge_tether_down',
         'tether_down.control_command.no_update_count'),
        ('GroundStationStatus', 'PlcGs02', 'status.detwist.position'),
        ('filtered', 'merge_tether_down',
         'tether_down.control_telemetry.flight_mode'),
    ], 'Detwist Error')

  def _IsValidCmd(self, tether_down_valid, no_update_count):
    return (tether_down_valid and
            no_update_count < common.MAX_NO_UPDATE_COUNT_CONTROLLER_COMMAND)

  def _IsValidInput(self, *attributes):
    tether_down_valid, no_update_count, detwist_pos = attributes[1:4]
    return (tether_down_valid and
            no_update_count < common.MAX_NO_UPDATE_COUNT_CONTROLLER_COMMAND and
            struct_tree.IsValidElement(detwist_pos))

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, detwist_cmd, tether_down_valid, no_update_count,
              detwist_pos, flight_mode):
    error = numpy.rad2deg(WrapDetwistError(detwist_cmd, detwist_pos))
    if common.AnyDetwistFlightMode(flight_mode):
      stoplight = StoplightForDetwistPosError(detwist_cmd, detwist_pos)
    else:
      stoplight = stoplights.STOPLIGHT_ANY
    return '{: 4.1f} deg'.format(error), stoplight


class DetwistKiteLoopErrorIndicator(indicator.BaseAttributeIndicator):
  """Show error between # of detwist loops and # of kite loops."""

  def __init__(self):
    super(DetwistKiteLoopErrorIndicator, self).__init__([
        ('ControlTelemetry', None, 'detwist_loop_count'),
        ('ControlTelemetry', None, 'crosswind.loop_count'),
    ], 'Detwist-Kite Error')

  def _IsValidInput(self, detwist_loop_count, kite_loop_count):
    return detwist_loop_count is not None and kite_loop_count is not None

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, detwist_loop_count, kite_loop_count):
    error = int(round(kite_loop_count - detwist_loop_count))
    error_lims = self._GetLimits(MONITOR_PARAMS.est.detwist_kite_loop_diff)
    stoplight = stoplights.SetByLimits(error, error_lims)
    return '{: d} loops'.format(error), stoplight


class DetwistTemperatureIndicator(avionics.BaseConfigLimitIndicator):

  def __init__(self):
    super(DetwistTemperatureIndicator, self).__init__(
        'Detwist Temp [C]', 'GroundStationPlcStatus', 'PlcTophat',
        'plc.detwist_motor_temp', MONITOR_PARAMS.thermal.detwist, '% 5.1f C')


_PLC_INFO = c_helpers.EnumHelper('PlcInfoFlag', pack_avionics_messages)
_PLC_WARNINGS = c_helpers.EnumHelper('PlcWarningFlag', pack_avionics_messages)
_PLC_ERRORS = c_helpers.EnumHelper('PlcErrorFlag', pack_avionics_messages)


class DetwistStatusIndicator(indicator.BaseAttributeIndicator):

  def __init__(self):
    super(DetwistStatusIndicator, self).__init__(
        [('GroundStationPlcStatus', 'PlcTophat')],
        'Detwist status')

  def _Filter(self, plc_status):
    def FlagToString(field, enum_helper):
      active = []
      for value in enum_helper.Values():
        if field & value:
          active.append(enum_helper.ShortName(value))
      return ', '.join(active) if active else 'None'

    if not struct_tree.IsValidElement(plc_status):
      text = '\n'.join([
          'State: --',
          'Position: --',
          'Velocity: --',
          'Torque:   --',
          'Temp:     --',
          'Info:     --',
          'Warnings: --',
          'Errors:   --',
      ])
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    else:
      plc = plc_status.plc
      detwist_state = plc_status.detwist_state
      text = '\n'.join([
          'State:  : %s' % ACTUATOR_STATE_HELPER.ShortName(detwist_state),
          'Position:% 5.1f deg' % numpy.rad2deg(plc.detwist_position),
          'Velocity:% 5.1f deg/s' % numpy.rad2deg(plc.detwist_velocity),
          'Torque:  % 5.1f N-m' % plc.detwist_torque,
          'Temp:    % 5.1f C' % plc.detwist_motor_temp,
          'Info:     ' + FlagToString(plc.info_flags, _PLC_INFO),
          'Warnings: ' + FlagToString(plc.warning_flags, _PLC_WARNINGS),
          'Errors:   ' + FlagToString(plc.error_flags, _PLC_ERRORS)
      ])
      armed = common.IsActuatorStateArmed(detwist_state)
      error = common.IsActuatorStateError(detwist_state)
      if error or not armed:
        stoplight = stoplights.STOPLIGHT_ERROR
      else:
        stoplight = stoplights.STOPLIGHT_NORMAL

    return text, stoplight


class DetwistStatusInfoIndicator(indicator.BaseAttributeIndicator):

  def __init__(self):
    super(DetwistStatusInfoIndicator, self).__init__(
        [('GroundStationPlcStatus', 'PlcTophat')],
        'Detwist Info')

  def _Filter(self, plc_status):
    def Flags(field, enum_helper):
      active = []
      for value in enum_helper.Values():
        if field & value:
          active.append(enum_helper.ShortName(value))
      return active

    if not struct_tree.IsValidElement(plc_status):
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
      text = '\n\n\n'
    else:
      plc = plc_status.plc
      info_flags = Flags(plc.info_flags, _PLC_INFO)
      warning_flags = Flags(plc.warning_flags, _PLC_WARNINGS)
      error_flags = Flags(plc.error_flags, _PLC_ERRORS)
      stoplight = stoplights.STOPLIGHT_NORMAL
      text = ''
      if info_flags:
        text += 'Info: ' + ', '.join(info_flags)
      text += '\n'
      if warning_flags:
        text += 'Warning: ' + ', '.join(warning_flags)
        stoplight = stoplights.STOPLIGHT_WARNING
      text += '\n'
      if error_flags:
        text += 'Error: ' + ', '.join(error_flags)
        stoplight = stoplights.STOPLIGHT_ERROR

    return text, stoplight


class DetwistArmedIndicator(indicator.SingleAttributeIndicator):

  def __init__(self):
    super(DetwistArmedIndicator, self).__init__(
        ('GroundStationPlcStatus', 'PlcTophat', 'detwist_state'),
        'Detwist Status')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, detwist_state):
    state = ACTUATOR_STATE_HELPER.ShortName(detwist_state)
    armed = common.IsActuatorStateArmed(detwist_state)
    error = common.IsActuatorStateError(detwist_state)
    # TODO: Flight-plan dependent stoplight.
    if error or not armed:
      stoplight = stoplights.STOPLIGHT_ERROR
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL
    return state, stoplight


class EStopStatusIndicator(indicator.BaseAttributeIndicator):

  def __init__(self, **base_kwargs):
    super(EStopStatusIndicator, self).__init__(
        [('GroundStationStatus', None, 'status')],
        'GS E-Stop', **base_kwargs)

  def _Filter(self, status):
    if struct_tree.IsValidElement(status):
      error = status.flags.error
      estopped = (error &
                  GROUND_STATION_ERROR_HELPER.Value('Estopped'))
      if estopped:
        text = 'E-Stop active.'
        stoplight = stoplights.STOPLIGHT_ERROR
      else:
        text = 'Not e-stopped.'
        stoplight = stoplights.STOPLIGHT_NORMAL
    else:
      text = 'Status Unavailable'
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    return text, stoplight


class Ground480VIndicator(indicator.SingleAttributeIndicator):

  def __init__(self):
    super(Ground480VIndicator, self).__init__(
        ('GroundStationStatus', None, 'status.flags'),
        '480V Power')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, flags):
    if flags.error & plc_messages.kGsErrorFlagNo480Vac:
      return 'Off', stoplights.STOPLIGHT_ERROR
    else:
      return 'On', stoplights.STOPLIGHT_NORMAL


class GsWingProximityIndicator(indicator.SingleAttributeIndicator):

  def __init__(self):
    super(GsWingProximityIndicator, self).__init__(
        ('GroundStationStatus', GS_NODE, 'status'),
        'Wing Proximity')

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, status):
    text = 'Active' if status.bridle_proximity.proximity else 'Inactive'
    return text, stoplights.STOPLIGHT_NORMAL


class GsProxSensorsIndicator(indicator.BaseAttributeIndicator):

  def __init__(self, **base_kwargs):
    super(GsProxSensorsIndicator, self).__init__([
        ('TetherUp', None, 'ground_station.proximity'),
        ('GroundStationStatus', None, 'status.bridle_proximity'),
        ], 'GS Prox Sensors', **base_kwargs)

  def _Filter(self, tetherup_proximity, bridle_proximity):
    if ((not struct_tree.IsValidElement(bridle_proximity)) and
        (not struct_tree.IsValidElement(tetherup_proximity))):
      return 'Status Unavailable', stoplights.STOPLIGHT_UNAVAILABLE

    prox = collections.OrderedDict()
    text_by_value = {True: 'Active  ', False: 'Inactive', None: '   --   '}
    if struct_tree.IsValidElement(bridle_proximity):
      prox['Combined'] = bridle_proximity.proximity
      prox['A'] = bridle_proximity.sensor_raw[0]
      prox['B'] = bridle_proximity.sensor_raw[1]
    else:
      if struct_tree.IsValidElement(tetherup_proximity):
        prox['Combined'] = tetherup_proximity
      else:
        prox['Combined'] = None
      prox['A'] = None
      prox['B'] = None

    text = '\t'.join([key + ': ' + text_by_value[prox[key]] for key in prox])
    # Show warning stoplight if either sensor sees the mark.
    # This means indicator is yellow when we're reeled in.
    if prox['Combined']:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL
    return text, stoplight


class GsTetherEngagementIndicator(indicator.BaseAttributeIndicator):

  def __init__(self, **base_kwargs):
    super(GsTetherEngagementIndicator, self).__init__(
        [('GroundStationStatus', None, 'status.tether_engagement'),
         ('TetherUp', None, 'ground_station.tether_engaged'),
         ('ControlTelemetry', None, 'flight_mode')],
        'GS Tether Engagement Sensors', **base_kwargs)

  def _Filter(self, gs_tether_engagement, tether_up_tether_engaged,
              flight_mode):
    if struct_tree.IsValidElement(gs_tether_engagement):
      engaged = gs_tether_engagement.engaged
      dark_on = bool(gs_tether_engagement.sensor_raw[0])
      light_on = bool(gs_tether_engagement.sensor_raw[1])
    elif struct_tree.IsValidElement(tether_up_tether_engaged):
      engaged = tether_up_tether_engaged
      dark_on = '--'
      light_on = '--'
    else:
      return 'Status Unavailable', stoplights.STOPLIGHT_UNAVAILABLE

    text = '%s\tDarkOn: %s\tLightOn: %s' % (
        'Engaged' if engaged else 'Not engaged', dark_on, light_on)
    if engaged:
      stoplight = stoplights.STOPLIGHT_NORMAL
    elif (struct_tree.IsValidElement(flight_mode)
          and flight_mode in (control_types.kFlightModeHoverPayOut,
                              control_types.kFlightModeHoverReelIn)):
      stoplight = stoplights.STOPLIGHT_ERROR
    else:
      stoplight = stoplights.STOPLIGHT_WARNING

    return text, stoplight


class BaseOrientationSketch(indicator.BaseSketch2D):
  """A base class for ground station orientations."""

  def _OrientationVertices(self, rad, length):
    return {
        'x': length * numpy.cos(rad),
        'y': length * numpy.sin(rad),
    }

  def _PointToSegment(self, point, color, from_point=None):
    """Return the coordinates of two points to represent a segment."""
    if from_point is None:
      from_point = {'x': 0.0, 'y': 0.0}
    return {
        'x': [from_point['x'], point['x']],
        'y': [from_point['y'], point['y']],
        'color': color,
    }


class GsOrientationWindow(BaseOrientationSketch):
  """Displays top-down position of the ground station."""

  def __init__(self):
    # Add a 10-meter circle around bouy for visual reference.
    ten_meter_circle = self._PolygonForCircle(10, 0.0, 0.0, 30, 'gray')
    attributes = [
        ('ControlTelemetry', None),
        ('GroundStationWeather', None, 'wind.wind_velocity'),
        ('GroundStationStatus', None, 'status.flags'),
        ('GroundStationStatus', GS_NODE, 'status.azimuth'),
        ('GroundEstimate', None),
        ('GroundEstimateSim', None),
        ('PlatformSensors', None, 'encoders.perch_azi'),
        ('TetherUp', None, 'platform_a.perch_azi'),
    ]

    super(GsOrientationWindow, self).__init__(
        attributes, name='',
        xlim=[-50.0, 50.0], ylim=[-50.0, 50.0],
        background_polygons=[ten_meter_circle],
        line_properties={
            'Wing': {'color': 'green'},
            'Wind': {'color': 'blue'},
            'Gs': {'color': 'red'},
        },
        marker_properties={
            'Ground': {'color': 'purple', 'size': 3},
        },
        history_len=50, show_legend=True,
        num_xticks=5, num_yticks=5)

  def _NedToScreen(self, angle):
    # Negate to make the angle go clockwise as in NED.
    # Add 90 deg to make the angle start from upward direction (North).
    return -angle + numpy.pi * 0.5

  @indicator.ReturnCallbackOutputIfInputInvalid(dict)
  def _Filter(self, control_telemetry, wind_uvw, gs_status_flags,
              gs_status_azimuth, ground_estimate, ground_estimate_sim,
              platformsensor_azimuth, tetherup_azimuth):
    # If 480V is not up, GroundStationStatus azimuth data is not valid.
    no_480v = True
    if gs_status_flags is not None:
      no_480v = gs_status_flags.error & plc_messages.kGsErrorFlagNo480Vac

    vessel_pos_g = None
    if struct_tree.IsValidElement(ground_estimate):
      vessel_pos_g = ground_estimate.Xg
    elif struct_tree.IsValidElement(ground_estimate_sim):
      vessel_pos_g = ground_estimate_sim.Xg

    # The vector length to draw on the figure.
    wing_vec_len = 35.0
    if control_telemetry and vessel_pos_g:
      wing_pos_ned = control_telemetry.state_est.Xg
      wing_pointer = self._OrientationVertices(
          self._NedToScreen(
              numpy.arctan2(wing_pos_ned.y - vessel_pos_g.y,
                            wing_pos_ned.x - vessel_pos_g.x)),
          wing_vec_len)
    else:
      wing_pointer = {'x': None, 'y': None}

    wind_vec_len = 45.0
    if control_telemetry and control_telemetry.state_est.wind_g.valid:
      wind_dir_g = common.WindAziToWindNed(
          control_telemetry.state_est.wind_g.dir_f)
    elif no_480v and wind_uvw and struct_tree.IsValidElement(ground_estimate):
      wind_vel_g = avionics.WindWsToWindG(
          wind_uvw, pqr=ground_estimate.pqr, dcm_g2p=ground_estimate.dcm_g2p)
      wind_dir_g = numpy.arctan2(-wind_vel_g.y, -wind_vel_g.x)
    else:
      wind_dir_g = None

    if wind_dir_g:
      # Showing the "going to" vector (as if it is a wind sock).
      wind_pointer = self._OrientationVertices(
          self._NedToScreen(wind_dir_g + numpy.pi), wind_vec_len)
    else:
      wind_pointer = {'x': None, 'y': None}

    boom_vec_len = 25.0
    if control_telemetry:
      # Note that this won't work in the following two cases:
      # 1. We lost wideband wifi and only have long range radio
      # 2. Wing on the perch but controller is off.
      #
      # TODO(b/138342907): For the scenarios above, estimate perch heading
      # directly from compass or GroundEstimate messages.
      heading_ref = control_telemetry.control_input.perch.perch_heading

      if SYSTEM_PARAMS.gs_model == system_types.kGroundStationModelGSv2:
        heading_ref += SYSTEM_PARAMS.ground_station.gs02.boom_azimuth_p

      gs_pointer = self._OrientationVertices(
          self._NedToScreen(heading_ref), boom_vec_len)
    else:
      gs_pointer = {'x': None, 'y': None}

    markers = {}
    if vessel_pos_g is not None:
      heading = numpy.arctan2(-vessel_pos_g.y, -vessel_pos_g.x)
      vec_len = numpy.hypot(vessel_pos_g.x, vessel_pos_g.y)
      markers['Ground'] = self._OrientationVertices(self._NedToScreen(heading),
                                                    vec_len)

    return {
        'pointers': {
            'Wing': wing_pointer,
            'Wind': wind_pointer,
            'Gs': gs_pointer,
        },
        'segments': {
            'Wing': self._PointToSegment(wing_pointer, 'green'),
            'Wind': self._PointToSegment(wind_pointer, 'blue'),
            'Gs': self._PointToSegment(gs_pointer, 'red'),
        },
        'markers': markers
    }


class DrumOrientationWindow(BaseOrientationSketch):
  """Displays a view of the drum."""

  def __init__(self):
    drum = self._PolygonForCircle(5, 0.0, 0.0, 30, 'gray')
    attributes = [
        ('GroundStationStatus', GS_NODE, 'status.winch.position'),
        ('TetherUp', None, 'ground_station.drum_angle')
    ]

    super(DrumOrientationWindow, self).__init__(
        attributes, name='',
        xlim=[-7.0, 7.0], ylim=[-7.0, 7.0],
        background_polygons=[drum],
        line_properties={
            'Gimbal': {'color': 'blue'},
        },
        history_len=50, show_legend=True,
        invert_xaxis=True, invert_yaxis=True,
        num_xticks=5, num_yticks=5)

  @indicator.ReturnCallbackOutputIfInputInvalid(dict)
  def _Filter(self, winch_pos, tetherup_winch_pos):

    if winch_pos is None and tetherup_winch_pos is None:
      winch_pointer = {'x': None, 'y': None}
    else:
      # Transform the angle from winch-frame to the screen frame.
      # On the screen, the angle goes counter-clockwise. Negate it
      # so that it goes clockwise to match the drum. Also, the schnozz
      # points down, not left, at angle 0. So we add another 90 deg.
      winch_pos = winch_pos if winch_pos is not None else tetherup_winch_pos
      winch_angle = -winch_pos + numpy.pi * 0.5
      winch_pointer = self._OrientationVertices(winch_angle, 4.0)

    return {
        'pointers': {'Gimbal': winch_pointer},
        'segments': {'Arm': self._PointToSegment(winch_pointer, 'gray')},
    }


class DrumVelocityChart(indicator.BaseAttributeDictChart):
  """Indicator that shows the drum velocity."""

  def __init__(self, **base_kwargs):
    self._field = 'velocity'
    super(DrumVelocityChart, self).__init__(
        [('GroundStationStatus', GS_NODE, 'capture_info.timestamp'),
         ('GroundStationStatus', GS_NODE, 'status.winch.velocity')],
        [self._field], 'Velocity Error', precision=2, num_yticks=5,
        trail_length=80, **base_kwargs)

  def _Filter(self, timestamp, velocity):
    if velocity is None or timestamp is None:
      return {}, {}, stoplights.STOPLIGHT_UNAVAILABLE

    info = {self._field: velocity}
    timestamps = {self._field: timestamp}
    stoplight = stoplights.STOPLIGHT_NORMAL
    return timestamps, info, stoplight


class AzimuthErrorChart(indicator.BaseAttributeDictChart):
  """Plots error in the azimuth position."""

  def __init__(self, **widget_kwargs):
    self._error_label = 'Error'
    self._raw_error_label = 'Raw error'
    self._wrap = [-180, 180]
    attributes = [('filtered', 'merge_tether_down',
                   'tether_down.control_command.gs_azi_target'),
                  ('ControlTelemetry', None, 'gs_azi_target_raw'),
                  ('filtered', 'merge_tether_down', 'valid'),
                  ('filtered', 'merge_tether_down',
                   'tether_down.control_command.no_update_count'),
                  aio_util.TetherDownTimestampAttributePath(),
                  ('GroundStationStatus', 'PlcGs02', 'status.azimuth.position'),
                  aio_util.AioTimestampAttributePath('GroundStationStatus',
                                                     'PlcGs02'),
                  ('TetherUp', None, 'platform_a.perch_azi'),
                  ('GroundStationStatus', 'PlcGs02', 'status.mode'),
                  ('TetherUp', None, 'ground_station.mode'),
                  ('GroundStationStatus', 'PlcGs02', 'status.transform_stage'),
                  ('TetherUp', None, 'ground_station.transform_stage')]
    super(AzimuthErrorChart, self).__init__(
        attributes, [self._error_label, self._raw_error_label],
        'Azi Error [deg]', precision=1, ylim=[-25.0, 25.0], **widget_kwargs)

  def _IsValidCmd(self, tether_down_valid, no_update_count):
    return (tether_down_valid and
            no_update_count < common.MAX_NO_UPDATE_COUNT_CONTROLLER_COMMAND)

  def _IsInReel(self, gs_mode, transform_stage):
    return (gs_mode == GROUND_STATION_MODE_HELPER.Value('Reel') or
            (gs_mode == GROUND_STATION_MODE_HELPER.Value('Transform') and
             transform_stage in (3, 4)))  # Stages 3 and 4 are reel-like.

  def _IsInHighTension(self, gs_mode, transform_stage):
    return (gs_mode == GROUND_STATION_MODE_HELPER.Value('HighTension') or
            (gs_mode == GROUND_STATION_MODE_HELPER.Value('Transform') and
             transform_stage == 1))  # Stage 1 is high-tension-like.

  def _IsValidInput(self, azimuth_cmd, azimuth_cmd_raw, tether_down_valid,
                    no_update_count, azi_cmd_tstamp, gs_azi_pos, gs_azi_tstamp,
                    tether_azi_pos, gs_status_mode, tether_mode, gs_stage,
                    tether_stage):
    # Ensure enough data present to calculate mode and error.
    return (self._IsValidCmd(tether_down_valid, no_update_count) and
            (tether_mode is not None or gs_status_mode is not None) and
            (tether_stage is not None or gs_stage is not None) and
            (gs_azi_pos is not None or tether_azi_pos is not None) and
            (azi_cmd_tstamp is not None or gs_azi_tstamp is not None) and
            (azimuth_cmd is not None) and (azimuth_cmd_raw is not None))

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, azimuth_cmd, azimuth_cmd_raw, tether_down_valid,
              no_update_count, azi_cmd_tstamp, gs_azi_pos, gs_azi_tstamp,
              tether_azi_pos, gs_status_mode, tether_mode, gs_stage,
              tether_stage):
    # Get data from GroundStationStatusMessage if available, else get it from
    # TetherUp.
    gs_mode = gs_status_mode if gs_status_mode is not None else tether_mode
    transform_stage = gs_stage if gs_stage is not None else tether_stage
    azimuth_pos = gs_azi_pos if gs_azi_pos is not None else tether_azi_pos
    azi_tstamp = gs_azi_tstamp if gs_azi_tstamp is not None else azi_cmd_tstamp
    azi_error_raw = numpy.rad2deg(azimuth_cmd_raw - azimuth_pos)
    azi_error = numpy.rad2deg(azimuth_cmd - azimuth_pos)
    # Choose azi command offset based on mode and transform stage.
    if self._IsInHighTension(gs_mode, transform_stage):
      azi_offset = 180.0
    else:  # Default to 90 degree offset for Reel-like or ambiguous modes.
      azi_offset = 90.0
    azimuth_error = c_util.Wrap(azi_error - azi_offset, self._wrap[0],
                                self._wrap[1])
    azimuth_error_raw = c_util.Wrap(azi_error_raw - azi_offset, self._wrap[0],
                                    self._wrap[1])
    values = {
        self._error_label: azimuth_error,
        self._raw_error_label: azimuth_error_raw
    }
    timestamps = {
        self._error_label: azi_tstamp,
        self._raw_error_label: azi_tstamp
    }
    # Stoplight based on limits for modes with offset, else greyed.
    if self._IsInHighTension(gs_mode, transform_stage):
      error_lims = self._GetLimits(MONITOR_PARAMS.est.gs_azi_error_ht)
      stoplight = stoplights.SetByLimits(numpy.deg2rad(azimuth_error),
                                         error_lims)
    elif self._IsInReel(gs_mode, transform_stage):
      error_lims = self._GetLimits(MONITOR_PARAMS.est.gs_azi_error_reel)
      stoplight = stoplights.SetByLimits(numpy.deg2rad(azimuth_error),
                                         error_lims)
    else:
      stoplight = stoplights.STOPLIGHT_ANY
    return timestamps, values, stoplight


class GsAzimuthErrorIndicator(indicator.BaseAttributeIndicator):
  """Indicator displaying difference between observed azimuth and command."""

  def __init__(self, name='Azi Error [deg]', precision=1, **base_kwargs):
    sources = [('GroundStationStatus', GS_NODE, 'status.azimuth.position'),
               ('TetherUp', None, 'platform_a.perch_azi'),
               ('ControllerCommand', None, 'gs_azi_target'),
               ('TetherUp', None, 'ground_station.mode'),
               ('GroundStationStatus', None, 'status.mode'),
               ('TetherUp', None, 'ground_station.transform_stage'),
               ('GroundStationStatus', GS_NODE, 'status.transform_stage'),
              ]
    self._wrap = [-180, 180]
    self._precision = precision
    super(GsAzimuthErrorIndicator, self).__init__(sources, name, **base_kwargs)

  def _IsInReel(self, gs_mode, transform_stage):
    return (gs_mode == GROUND_STATION_MODE_HELPER.Value('Reel') or
            (gs_mode == GROUND_STATION_MODE_HELPER.Value('Transform') and
             transform_stage in (3, 4)))  # Stages 3 and 4 are reel-like.

  def _IsInHighTension(self, gs_mode, transform_stage):
    return (gs_mode == GROUND_STATION_MODE_HELPER.Value('HighTension') or
            (gs_mode == GROUND_STATION_MODE_HELPER.Value('Transform') and
             transform_stage == 1))  # Stage 1 is high-tension-like.

  def _IsValidInput(self, gs_azi_data, tether_azi_data, azimuth_cmd,
                    tether_mode, gs_status_mode, tether_stage, gs_stage):
    # Ensure enough data present to calculate mode and error.
    return ((tether_mode is not None or gs_status_mode is not None) and
            (tether_stage is not None or gs_stage is not None) and
            (gs_azi_data is not None or tether_azi_data is not None) and
            azimuth_cmd is not None)

  @indicator.ReturnIfInputInvalid(None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, gs_azi_data, tether_azi_data, azimuth_cmd,
              tether_mode, gs_status_mode, tether_stage, gs_stage):
    # Get azimuth position and GS mode + transform stage from
    # GroundStationStatusMessage if available, else get it from TetherUp.
    gs_mode = gs_status_mode if gs_status_mode is not None else tether_mode
    transform_stage = gs_stage if gs_stage is not None else tether_stage
    azi_data = gs_azi_data if gs_azi_data is not None else tether_azi_data
    azi_error_raw = numpy.rad2deg(azimuth_cmd - azi_data)

    # Choose azi command offset based on mode and transform stage.
    if self._IsInHighTension(gs_mode, transform_stage):
      azi_offset = 180.0
    else:  # Default to 90 degree offset for Reel-like or ambiguous modes.
      azi_offset = 90.0
    azi_error = c_util.Wrap(azi_error_raw - azi_offset,
                            self._wrap[0], self._wrap[1])

    if self._IsInHighTension(gs_mode, transform_stage):
      # Stoplight based on limits for modes with offset, else greyed.
      error_lims = self._GetLimits(MONITOR_PARAMS.est.gs_azi_error_ht)
      stoplight = stoplights.SetByLimits(numpy.deg2rad(azi_error), error_lims)
    elif self._IsInReel(gs_mode, transform_stage):
      error_lims = self._GetLimits(MONITOR_PARAMS.est.gs_azi_error_reel)
      stoplight = stoplights.SetByLimits(numpy.deg2rad(azi_error), error_lims)
    else:
      stoplight = stoplights.STOPLIGHT_ANY

    text = 'Error: {: 5.1f}'.format(azi_error)
    return text, stoplight


class BaseGsChartWithControllerCommand(indicator.BaseAttributeDictChart):
  """Indicator that charts GS attributes with a controller command."""

  def __init__(self, name, actuator, gs_field, controller_field,
               tetherup_message, tetherup_field, scale_obs=1.0, scale_cmd=1.0,
               wrap=None, **base_kwargs):
    sources = [('GroundStationStatus', GS_NODE, 'capture_info.timestamp'),
               ('GroundStationStatus', GS_NODE,
                'status.%s.%s' % (actuator, gs_field)),
              ]
    self._use_tetherup = False
    if (tetherup_message is not None) and (tetherup_field is not None):
      sources.append(('TetherUp', None, 'capture_info.timestamp'))
      sources.append(('TetherUp', None, '%s.%s' % (tetherup_message,
                                                   tetherup_field)))
      self._use_tetherup = True
    labels = ['obs']
    if controller_field is not None:
      sources.append(('ControllerCommand', None, 'capture_info.timestamp'))
      sources.append(('ControllerCommand', None, controller_field))
      labels.append('cmd')
    self._controller_field = controller_field
    self._scale_obs = scale_obs
    self._scale_cmd = scale_cmd
    self._wrap = wrap

    super(BaseGsChartWithControllerCommand, self).__init__(
        sources, labels, name, precision=2, num_yticks=5,
        trail_length=80, **base_kwargs)

  def _IsValidInput(self, *sources):
    value = sources[1]
    if struct_tree.IsValidElement(value):
      return True
    if self._controller_field is not None:
      controller_field = sources[-1]
      if struct_tree.IsValidElement(controller_field):
        return True
    if self._use_tetherup:
      tetherup_field = sources[3]
      if struct_tree.IsValidElement(tetherup_field):
        return True
    return False

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, timestamp, value, *args):
    if self._use_tetherup:
      tetherup_timestamp = args[0]
      tetherup_field = args[1]
      if self._controller_field is not None:
        cmd = args[2:]
    elif self._controller_field is not None:
      cmd = args
    info = {'obs': GetOrNone(value)}
    timestamps = {'obs': GetOrNone(timestamp)}
    if info['obs'] is None:
      # Revert to TetherUp messages if GroundStationStatus is not available.
      if self._use_tetherup:
        info = {'obs': GetOrNone(tetherup_field)}
        timestamps = {'obs': GetOrNone(tetherup_timestamp)}
    # Check for controller command
    if self._controller_field is not None:
      info['cmd'] = GetOrNone(cmd[1])
      timestamps['cmd'] = GetOrNone(cmd[0])
      if info['cmd'] is not None:
        info['cmd'] *= self._scale_cmd
        if self._wrap:
          info['cmd'] = c_util.Wrap(info['cmd'], self._wrap[0], self._wrap[1])
    if info['obs'] is not None:
      info['obs'] *= self._scale_obs
      if self._wrap:
        info['obs'] = c_util.Wrap(info['obs'], self._wrap[0], self._wrap[1])

    stoplight = stoplights.STOPLIGHT_NORMAL
    return timestamps, info, stoplight


class BaseGsChartWithGsCommand(indicator.BaseAttributeDictChart):
  """Indicator that charts GS attributes with their gs commands."""

  def __init__(self, name, actuator, field, scale_obs=1.0, scale_cmd=1.0,
               **base_kwargs):
    sources = [('GroundStationStatus', GS_NODE, 'capture_info.timestamp'),
               ('GroundStationStatus', GS_NODE,
                'status.%s.%s' % (actuator, field)),
              ]
    labels = ['obs']
    for n in xrange(2):
      sources.append(
          ('GroundStationStatus', GS_NODE,
           'status.%s.motor[%d].command' % (actuator, n)))
    labels += ['cmd.A', 'cmd.B']
    self._target = 'command'
    self._scale_obs = scale_obs
    self._scale_cmd = scale_cmd

    super(BaseGsChartWithGsCommand, self).__init__(
        sources, labels, name, precision=2, num_yticks=5,
        trail_length=80, **base_kwargs)

  def _IsValidInput(self, timestamp, *args):
    return timestamp is not None

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, timestamp, value, *cmd):
    info = {'obs': GetOrNone(value)}
    timestamp = GetOrNone(timestamp)
    timestamps = {'obs': timestamp}
    if self._target:
      info.update({
          'cmd.A': GetOrNone(cmd[0]),
          'cmd.B': GetOrNone(cmd[1]),
      })
      timestamps.update({
          'cmd.A': timestamp,
          'cmd.B': timestamp,
      })
      if info['cmd.A'] is not None:
        info['cmd.A'] *= self._scale_cmd
      if info['cmd.B'] is not None:
        info['cmd.B'] *= self._scale_cmd
    if info['obs'] is not None:
      info['obs'] *= self._scale_obs

    stoplight = stoplights.STOPLIGHT_NORMAL
    return timestamps, info, stoplight


class GsAzimuthChart(BaseGsChartWithControllerCommand):

  def __init__(self, **base_kwargs):
    # TODO: gs_azi_target is currently not in the same coordinate frame
    # as azimuth.position. We still use it to see if 'obs' follows the changes
    # in 'cmd'. Note the offset between both is 90 deg in reel mode, and about
    # 180 deg in high tension mode.
    super(GsAzimuthChart, self).__init__(
        'Gs Azimuth [deg]', 'azimuth', 'position', 'gs_azi_target',
        'platform_a', 'perch_azi', numpy.rad2deg(1.0), numpy.rad2deg(1.0),
        **base_kwargs)


class GsAzimuthVelocityChart(BaseGsChartWithGsCommand):

  def __init__(self, **base_kwargs):
    super(GsAzimuthVelocityChart, self).__init__(
        'Gs Azimuth Velocity [deg/s]', 'azimuth', 'velocity',
        numpy.rad2deg(1.0), numpy.rad2deg(1.0), ylim=[-7.0, 7.0],
        **base_kwargs)


class GsLevelwindPositionChart(BaseGsChartWithControllerCommand):

  def __init__(self, **base_kwargs):
    super(GsLevelwindPositionChart, self).__init__(
        'Gs Levelwind Shuttle Pos [mm]', 'levelwind', 'position', None, None,
        None, 1000.0, 1000.0, ylim=[0.0, 2000.0], **base_kwargs)


class GsLevelwindVelocityChart(BaseGsChartWithControllerCommand):

  def __init__(self, **base_kwargs):
    super(GsLevelwindVelocityChart, self).__init__(
        'Gs Levelwind Velocity [mm/s]', 'levelwind', 'velocity', None, None,
        None, 1000.0, 1000.0, **base_kwargs)


class GsWinchPositionChart(BaseGsChartWithControllerCommand):

  def __init__(self, **base_kwargs):
    super(GsWinchPositionChart, self).__init__(
        'Winch Position [m]', 'winch', 'position', None, 'ground_station',
        'drum_angle', SYSTEM_PARAMS.ground_station.gs02.drum_radius,
        SYSTEM_PARAMS.ground_station.gs02.drum_radius, ylim=[-435.0, 1.0],
        **base_kwargs)


class GsDetwistPositionChart(BaseGsChartWithControllerCommand):

  def __init__(self, **base_kwargs):
    super(GsDetwistPositionChart, self).__init__(
        'Detwist Position [deg]', 'detwist', 'position', 'detwist_position',
        'ground_station', 'detwist_angle', numpy.rad2deg(1.0),
        numpy.rad2deg(1.0), (0.0, 360.0), ylim=[0.0, 360.0], **base_kwargs)


class GsWinchVelocityChart(BaseGsChartWithControllerCommand):
  """Indicator that shows winch velocity."""

  def __init__(self, **base_kwargs):
    super(GsWinchVelocityChart, self).__init__(
        'Winch Velocity [m/s]', 'winch', 'velocity', 'winch_velocity', None,
        None, SYSTEM_PARAMS.ground_station.gs02.drum_radius, 1.0,
        ylim=[-3.0, 3.0], **base_kwargs)


class GsTorqueChart(indicator.BaseAttributeDictChart):
  """Indicator that charts GS torques."""

  _LIMITS = {
      'azimuth': 100.0,
      'winch': 100.0,
  }

  def __init__(self, **base_kwargs):
    sources = [('GroundStationStatus', GS_NODE, 'capture_info.timestamp')]
    for actuator in ['azimuth', 'winch']:
      for n in xrange(2):
        sources.append(
            ('GroundStationStatus', GS_NODE,
             'status.%s.motor[%d].torque' % (actuator, n)))
    labels = ['azimuth.A', 'azimuth.B', 'winch.A', 'winch.B',
              'azimuth.limit', 'winch.limit']
    super(GsTorqueChart, self).__init__(
        sources, labels, 'Torques', precision=2, num_yticks=5,
        trail_length=80, **base_kwargs)

  def _IsValidInput(self, timestamp, *args):
    return timestamp is not None

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, timestamp, azi_torque_a, azi_torque_b,
              winch_torque_a, winch_torque_b):

    info = {
        'azimuth.A': GetOrNone(azi_torque_a),
        'azimuth.B': GetOrNone(azi_torque_b),
        'winch.A': GetOrNone(winch_torque_a),
        'winch.B': GetOrNone(winch_torque_b),
        'azimuth.limit': self._LIMITS['azimuth'],
        'winch.limit': self._LIMITS['winch'],
    }

    timestamp = GetOrNone(timestamp)
    timestamps = {
        'azimuth.A': timestamp,
        'azimuth.B': timestamp,
        'winch.A': timestamp,
        'winch.B': timestamp,
        'azimuth.limit': timestamp,
        'winch.limit': timestamp,
    }

    stoplight = stoplights.STOPLIGHT_NORMAL
    if self._LIMITS['azimuth'] <= max(azi_torque_a, azi_torque_b):
      stoplight = stoplights.STOPLIGHT_WARNING
    elif self._LIMITS['winch'] <= max(winch_torque_a, winch_torque_b):
      stoplight = stoplights.STOPLIGHT_WARNING

    return timestamps, info, stoplight


class GsModeIndicator(indicator.BaseAttributeIndicator):

  def __init__(self, **base_kwargs):
    super(GsModeIndicator, self).__init__(
        [('TetherUp', None, 'ground_station.mode'),
         ('GroundStationStatus', None, 'status'),
         ('ControllerCommand', None, 'gs_mode_request')],
        'GS Mode', **base_kwargs)

  def _Filter(self, tetherup_mode, status, mode_request):
    any_available = False
    any_missing = False
    mode = None
    if struct_tree.IsValidElement(status):
      mode = status.mode
    else:
      if struct_tree.IsValidElement(tetherup_mode):
        mode = tetherup_mode
    if mode is not None:
      text = GROUND_STATION_MODE_HELPER.ShortName(mode)
      any_available = True
    else:
      text = 'Status Unavailable'
      any_missing = True

    if struct_tree.IsValidElement(mode_request):
      text += '\n%s (Requested)' % (
          GROUND_STATION_MODE_HELPER.ShortName(mode_request))
      any_available = True
    else:
      text += '\nRequest Unavailable'
      any_missing = True

    if not any_missing:
      stoplight = stoplights.STOPLIGHT_NORMAL
    elif not any_available:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    else:
      stoplight = stoplights.STOPLIGHT_WARNING
    return text, stoplight


class GsCommsIndicator(indicator.BaseAttributeIndicator):

  def __init__(self, **base_kwargs):
    super(GsCommsIndicator, self).__init__(
        [('GroundStationStatus', None, 'status')],
        'GS Comms', **base_kwargs)

  def _Filter(self, status):
    if struct_tree.IsValidElement(status):
      warning = status.flags.warning
      ignoring_comms = (warning &
                        GROUND_STATION_WARNING_HELPER.Value('IgnoringComms'))
      if ignoring_comms:
        text = 'Ignoring comms'
        stoplight = stoplights.STOPLIGHT_WARNING
      else:
        text = 'Listening to comms'
        stoplight = stoplights.STOPLIGHT_NORMAL
    else:
      text = 'Status Unavailable'
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    return text, stoplight


class GsArmingStatusIndicator(indicator.BaseAttributeIndicator):

  def __init__(self, **base_kwargs):
    super(GsArmingStatusIndicator, self).__init__(
        [('GroundStationStatus', None, 'actuator_state')],
        'GS Actuator States', **base_kwargs)

  def _Filter(self, actuator_states):
    # There are four GS actuators -- this indicator gives a single
    # status based on the status of these four actuators.
    all_armed = True  # Green stoplight if all armed.
    any_error = False  # Text displays "Error" if any have errors.
    if struct_tree.IsValidElement(actuator_states):
      for state in actuator_states:
        if common.IsActuatorStateError(state):
          any_error = True
        if not common.IsActuatorStateArmed(state):
          all_armed = False

      if any_error:
        text = 'Error'
        stoplight = stoplights.STOPLIGHT_ERROR
      elif all_armed:
        text = 'Armed'
        stoplight = stoplights.STOPLIGHT_NORMAL
      else:
        text = 'Disarmed'
        stoplight = stoplights.STOPLIGHT_ERROR
    else:
      text = 'Status Unavailable'
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    return text, stoplight


class GsModeChart(indicator.BaseAttributeDictChart):
  """Indicator that shows the ground station's mode."""

  def __init__(self, **base_kwargs):
    super(GsModeChart, self).__init__(
        [('GroundStationStatus', GS_NODE, 'capture_info.timestamp'),
         ('GroundStationStatus', GS_NODE, 'status.mode'),
         ('ControllerCommand', None, 'capture_info.timestamp'),
         ('ControllerCommand', None, 'gs_mode_request')],
        ['obs', 'cmd'], 'Mode', precision=2, num_yticks=5,
        trail_length=80, **base_kwargs)

  def _IsValidInput(self, timestamp, *args):
    return timestamp is not None

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, gs_timestamp, gs_mode, cmd_timestamp, cmd_mode):
    info = {}
    gs_timestamp = GetOrNone(gs_timestamp)
    gs_mode = GetOrNone(gs_mode)
    cmd_timestamp = GetOrNone(cmd_timestamp)
    cmd_mode = GetOrNone(cmd_mode)

    if gs_mode is None or cmd_mode is None:
      return {}, {}, stoplights.STOPLIGHT_UNAVAILABLE

    info = {
        'obs': gs_mode,
        'cmd': cmd_mode,
    }

    timestamps = {
        'obs': gs_timestamp,
        'cmd': cmd_timestamp,
    }

    stoplight = stoplights.STOPLIGHT_NORMAL
    return timestamps, info, stoplight


class SensorConsistentIndicator(indicator.BaseAttributeIndicator):
  """Display set of redundant readings with warning if inconsistent."""

  # Acceptable difference between sensor readings.
  _acceptable_difference = None  # Pass in via child class init function.
  _is_angle = False  # Pass in via child class init function.

  def __init__(self, name, input_fields, acceptable_difference, is_angle):
    self._acceptable_difference = acceptable_difference
    self._is_angle = is_angle
    super(SensorConsistentIndicator, self).__init__(input_fields, name)

  def DifferenceWarning(self, readings):
    """Return True to indicate warning if max difference > acceptable."""
    if len(readings) == 1:
      return False  # Readings agree within acceptable difference.
    for comp_reading in readings[1:]:
      if self._is_angle:
        if abs(c_util.Wrap(readings[0] - comp_reading, -numpy.deg2rad(180),
                           numpy.deg2rad(180))) > self._acceptable_difference:
          return True  # Readings disagree - display warning.
      else:
        if abs(readings[0] - comp_reading) > self._acceptable_difference:
          return True  # Readings disagree - display warning.
    return self.DifferenceWarning(readings[1:])

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *sensors):
    stoplight = stoplights.STOPLIGHT_NORMAL
    readings = []
    text = []
    for value in sensors:
      if value is not None:
        if self._is_angle:
          # Assuming sensor reading in rad. We want to display in deg.
          text.append('% 8.1f' % c_util.Wrap(numpy.rad2deg(value), -180, 180))
          readings.append(value)
        else:
          # Assuming sensor reading in m.  We want to display in mm.
          text.append('% 8.1f' % (1000 * numpy.float(value)))
          readings.append(numpy.float(value))
      else:
        text.append('   --')
        stoplight = stoplights.STOPLIGHT_WARNING  # Warning if sensors missing.

    if len(readings) < 1:  # No sensor readings available.
      return ' '.join(text), stoplights.STOPLIGHT_UNAVAILABLE

    if self.DifferenceWarning(readings):
      stoplight = stoplights.STOPLIGHT_WARNING  # Warning if sensors disagree.

    return ' '.join(text), stoplight


class PerchAziConsistentIndicator(SensorConsistentIndicator):
  """Displays perch azi readings with warning stoplight if inconsistent."""

  def __init__(self, name='Perch Azi [deg]'):
    acceptable_difference = MONITOR_PARAMS.est.perch_azimuth_max_disagreement
    input_fields = [
        ('PlatformSensors', 'PlatformSensorsA', 'encoders.perch_azi'),
        ('PlatformSensors', 'PlatformSensorsB', 'encoders.perch_azi'),
        ('GroundStationStatus', None, 'status.azimuth.motor[0].position'),
        ('GroundStationStatus', None, 'status.azimuth.motor[1].position')]
    super(PerchAziConsistentIndicator, self).__init__(
        name, input_fields, acceptable_difference, is_angle=True)


class DetwistConsistentIndicator(SensorConsistentIndicator):
  """Displays detwist readings with warning stoplight if inconsistent."""

  def __init__(self, name='Detwist [deg]'):
    acceptable_difference = MONITOR_PARAMS.est.detwist_max_disagreement
    input_fields = [
        ('GroundStationStatus', None, 'status.detwist.motor[0].position'),
        ('GroundStationStatus', None, 'status.detwist.motor[1].position')]
    super(DetwistConsistentIndicator, self).__init__(
        name, input_fields, acceptable_difference, is_angle=True)


class WinchConsistentIndicator(SensorConsistentIndicator):
  """Displays winch readings with warning stoplight if inconsistent."""

  def __init__(self, name='Winch [deg]'):
    acceptable_difference = MONITOR_PARAMS.est.winch_max_disagreement
    input_fields = [
        ('GroundStationStatus', None, 'status.winch.motor[0].position'),
        ('GroundStationStatus', None, 'status.winch.motor[1].position')]
    super(WinchConsistentIndicator, self).__init__(
        name, input_fields, acceptable_difference, is_angle=True)


class LvlShoulderConsistentIndicator(SensorConsistentIndicator):
  """Displays lev. shoulder readings with warning stoplight if inconsistent."""

  def __init__(self, name='Lev. Shoulder [deg]'):
    # TODO(b/117862206): Re-enable when enabling Levelwind Shoulder A encoder in
    # the flight controller.
    acceptable_difference = numpy.deg2rad(360.0)
    input_fields = [
        ('PlatformSensors', 'PlatformSensorsA', 'encoders.levelwind_shoulder'),
        ('PlatformSensors', 'PlatformSensorsB', 'encoders.levelwind_shoulder')]
    super(LvlShoulderConsistentIndicator, self).__init__(
        name, input_fields, acceptable_difference, is_angle=True)


class LvlWristConsistentIndicator(SensorConsistentIndicator):
  """Displays lev. wrist readings with warning stoplight if inconsistent."""

  def __init__(self, name='Lev. Wrist [deg]'):
    acceptable_difference = MONITOR_PARAMS.est.lvlwind_max_disagreement
    input_fields = [
        ('PlatformSensors', 'PlatformSensorsA', 'encoders.levelwind_wrist'),
        ('PlatformSensors', 'PlatformSensorsB', 'encoders.levelwind_wrist')]
    super(LvlWristConsistentIndicator, self).__init__(
        name, input_fields, acceptable_difference, is_angle=True)


class LvlShuttleConsistentIndicator(SensorConsistentIndicator):
  """Displays lev. shuttle readings with warning stoplight if inconsistent."""

  def __init__(self, name='Lev. Shuttle [mm]'):
    acceptable_difference = MONITOR_PARAMS.est.lvlwind_shuttle_max_disagreement
    input_fields = [
        ('GroundStationStatus', None, 'status.levelwind.motor[0].position'),
        ('GroundStationStatus', None, 'status.levelwind.motor[1].position')]
    super(LvlShuttleConsistentIndicator, self).__init__(
        name, input_fields, acceptable_difference, is_angle=False)


class GsgYokeConsistentIndicator(SensorConsistentIndicator):
  """Displays gsg yoke readings with warning stoplight if inconsistent."""

  def __init__(self, name='Gsg Yoke [deg]'):
    acceptable_difference = MONITOR_PARAMS.est.gsg_max_disagreement
    input_fields = [
        ('DrumSensors', 'DrumSensorsA', 'encoders.gsg_azi'),
        ('DrumSensors', 'DrumSensorsB', 'encoders.gsg_azi')]
    super(GsgYokeConsistentIndicator, self).__init__(
        name, input_fields, acceptable_difference, is_angle=True)


class GsgTermConsistentIndicator(SensorConsistentIndicator):
  """Displays gsg term. readings with warning stoplight if inconsistent."""

  def __init__(self, name='Gsg Term. [deg]'):
    acceptable_difference = MONITOR_PARAMS.est.gsg_max_disagreement
    input_fields = [
        ('DrumSensors', 'DrumSensorsA', 'encoders.gsg_ele'),
        ('DrumSensors', 'DrumSensorsB', 'encoders.gsg_ele')]
    super(GsgTermConsistentIndicator, self).__init__(
        name, input_fields, acceptable_difference, is_angle=True)


class BaseTetherUpGsStatusIndicator(indicator.SingleAttributeIndicator):

  def __init__(self, name, field, enum_helper, chars_per_line=15,
               **base_kwargs):
    super(BaseTetherUpGsStatusIndicator, self).__init__(
        ('TetherUp', None, field), name, **base_kwargs)
    self._enum_helper = enum_helper
    self._chars_per_line = chars_per_line

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, bitmask):
    items = []
    for enum_bitmask in self._enum_helper.Values():
      if bitmask & enum_bitmask:
        items.append(self._enum_helper.ShortName(enum_bitmask))
    if items:
      return (common.TimeCycle(items, self._chars_per_line),
              stoplights.STOPLIGHT_ERROR)
    else:
      return 'Normal', stoplights.STOPLIGHT_NORMAL


class TetherUpPlatformSensorsStatusIndicator(BaseTetherUpGsStatusIndicator):

  def __init__(self, node):
    super(TetherUpPlatformSensorsStatusIndicator, self).__init__(
        'Platform Sensor %s' % node.upper(),
        'platform_%s.flags' % node.lower(),
        c_helpers.EnumHelper('TetherPlatformFlag', pack_avionics_messages))
    self._node = node


class TetherUpDrumSensorsStatusIndicator(BaseTetherUpGsStatusIndicator):

  def __init__(self, node):
    super(TetherUpDrumSensorsStatusIndicator, self).__init__(
        'Drum Sensor %s' % node.upper(),
        'drum_%s.flags' % node.lower(),
        c_helpers.EnumHelper('TetherDrumFlag', pack_avionics_messages))


class TetherUpPlcStatusIndicator(BaseTetherUpGsStatusIndicator):

  def __init__(self):
    super(TetherUpPlcStatusIndicator, self).__init__(
        'PLC Status', 'plc.flags',
        c_helpers.EnumHelper('TetherPlcFlag', pack_avionics_messages))


class TetherUpGsStatusIndicator(BaseTetherUpGsStatusIndicator):

  def __init__(self):
    super(TetherUpGsStatusIndicator, self).__init__(
        'GS02 Status', 'ground_station.flags',
        c_helpers.EnumHelper('TetherGroundStationFlag', pack_avionics_messages))


class TetherAngleChart(indicator.BaseAttributeDictChart):
  """Show tether elevation angle."""

  def __init__(self, name='Hover', **widget_kwargs):
    # TODO: Infer which nodes to use from
    # FaultDetectionDisabledParams.
    self._ele_label = 'obs [deg]'
    self._ele_cmd_label = 'cmd [deg]'
    self._source_label = 'Source'
    super(TetherAngleChart, self).__init__(
        [
            ('TetherUp', None),
            ('PlatformSensors', 'PlatformSensorsB'),
            ('DrumSensors', 'DrumSensorsB'),
            ('GroundStationStatus', None),
            ('TetherDown', None),
            ('ControlTelemetry', None),
        ],
        [self._ele_label, self._ele_cmd_label, self._source_label],
        name, chart_keys=[self._ele_label, self._ele_cmd_label],
        ylim=[-10.0, 80.0], **widget_kwargs)

  def _IsValidInput(self, tether_up, platform_sensors, drum_sensors, gs_status,
                    tether_down, telemetry):
    if tether_up is not None:
      return True

    if gs_status is None:
      return False

    if (telemetry is not None and
        telemetry.state_est.tether_ground_angles.elevation_valid):
      return True

    if self._IsInReelMode(gs_status.status.mode):
      if platform_sensors is None:
        return False
    elif self._IsInHighTensionMode(gs_status.status.mode):
      if drum_sensors is None:
        return False
    elif self._IsInTransformMode(gs_status.status.mode):
      if platform_sensors is None or drum_sensors is None:
        return False
    return True

  def _IsInReelMode(self, mode):
    return (mode == GROUND_STATION_MODE_HELPER.Value('Reel') or
            mode == GROUND_STATION_MODE_HELPER.Value('Manual'))

  def _IsInTransformMode(self, mode):
    return mode == GROUND_STATION_MODE_HELPER.Value('Transform')

  def _IsInHighTensionMode(self, mode):
    return mode == GROUND_STATION_MODE_HELPER.Value('HighTension')

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, tether_up, platform_sensors, drum_sensors, gs_status,
              tether_down, telemetry):

    # Determine GS mode and GS transform stage.
    if tether_up is not None:
      gs_mode = tether_up.ground_station.mode
      transform_stage = tether_up.ground_station.transform_stage
    else:
      gs_mode = gs_status.status.mode
      transform_stage = None

    # Determine the flight mode.
    if tether_down is not None:
      flight_mode = tether_down.control_telemetry.flight_mode
    elif telemetry is not None:
      flight_mode = telemetry.flight_mode
    else:
      flight_mode = None

    # Determine which of the levelwind or GSG tether angle estimate to use.
    if self._IsInReelMode(gs_mode):
      use_levelwind = True
    elif self._IsInHighTensionMode(gs_mode):
      use_levelwind = False
    else:
      # Use tether engagement to determine whether to use levelwind or GSG.
      if tether_up is not None:
        use_levelwind = tether_up.ground_station.tether_engaged
      else:
        use_levelwind = gs_status.status.tether_engagement.engaged

    # Get the tether elevation angle.
    if (telemetry is not None and
        telemetry.state_est.tether_ground_angles.elevation_valid):
      source = 'levelwind' if use_levelwind else 'gsg'
      tether_ele = telemetry.state_est.tether_ground_angles.elevation_p
      tether_ele_cmd = telemetry.hover.tether_elevation_cmd
      timestamp = telemetry.capture_info['timestamp']
    else:
      tether_ele_cmd = None
      if tether_up is not None:
        if use_levelwind:
          tether_ele = tether_up.platform_b.levelwind_ele
          timestamp = tether_up.capture_info['timestamp']
        else:
          platform_azi = tether_up.platform_b.perch_azi
          drum_angle = tether_up.ground_station.drum_angle
          detwist_angle = tether_up.ground_station.detwist_angle
          gsg_azi = tether_up.drum_b.gsg_axis1
          gsg_ele = tether_up.drum_b.gsg_axis2
          timestamp = tether_up.capture_info['timestamp']
      else:
        if use_levelwind:
          tether_ele = platform_sensors.encoders.levelwind_ele
          timestamp = platform_sensors.capture_info['timestamp']
        else:
          platform_azi = gs_status.status.azimuth.position
          drum_angle = gs_status.status.winch.position
          detwist_angle = gs_status.status.detwist.position
          gsg_azi = drum_sensors.encoders.gsg_azi
          gsg_ele = drum_sensors.encoders.gsg_ele
          timestamp = max(
              drum_sensors.capture_info['timestamp'],
              gs_status.capture_info['timestamp'])
      if use_levelwind:
        source = 'levelwind'
      else:
        source = 'gsg'
        gs = gs_model.GroundStation(
            platform_pos_ned=[0.0, 0.0, 0.0],
            drum_center_p=ctype_util.Vec3ToList(
                SYSTEM_PARAMS.ground_station.gs02.drum_origin_p),
            platform_azimuth=platform_azi,
            winch_azimuth_p=0.0,
            detwist_ele=SYSTEM_PARAMS.ground_station.gs02.detwist_elevation,
            drum_rotation=drum_angle,
            gsg_position_drum=ctype_util.Vec3ToList(
                SYSTEM_PARAMS.ground_station.gs02.gsg_pos_drum),
            detwist_angle=detwist_angle,
            gsg_yoke=gsg_azi,
            gsg_termination=gsg_ele)
        tether_ele = gs.GetTetherAngle()

    angle = numpy.rad2deg(tether_ele)
    if flight_mode in [control_types.kFlightModeHoverPayOut,
                       control_types.kFlightModeHoverReelIn,
                       control_types.kFlightModeHoverPrepTransformGsUp,
                       control_types.kFlightModeHoverTransformGsUp,
                       control_types.kFlightModeHoverPrepTransformGsDown,
                       control_types.kFlightModeHoverTransformGsUp]:
      angle_cmd = (None if tether_ele_cmd is None
                   else numpy.rad2deg(tether_ele_cmd))
    else:
      angle_cmd = None
    angle_lims = None
    stoplight = stoplights.STOPLIGHT_ANY

    # If possible, choose stoplights based on flight mode and transform stage.
    if flight_mode is not None and transform_stage is not None:
      if ((flight_mode in [control_types.kFlightModeHoverPrepTransformGsUp,
                           control_types.kFlightModeHoverPayOut,  # Reel modes.
                           control_types.kFlightModeHoverReelIn]) or
          (flight_mode == control_types.kFlightModeHoverTransformGsUp and
           transform_stage in [0])):
        angle_lims = self._GetLimits(MONITOR_PARAMS.est.hover_tether_angles[0])
      elif ((flight_mode == control_types.kFlightModeHoverTransformGsUp and
             transform_stage in [1, 2, 3]) or
            (flight_mode == control_types.kFlightModeHoverTransformGsDown and
             transform_stage in [0, 1, 2, 3])):
        angle_lims = self._GetLimits(MONITOR_PARAMS.est.hover_tether_angles[1])
      elif (flight_mode in [control_types.kFlightModeHoverFullLength,
                            control_types.kFlightModeHoverAccel,  # HT modes.
                            control_types.kFlightModeTransIn,
                            control_types.kFlightModeCrosswindNormal,
                            control_types.kFlightModeCrosswindPrepTransOut,
                            control_types.kFlightModeHoverTransOut,
                            control_types.kFlightModeHoverPrepTransformGsDown]):
        angle_lims = self._GetLimits(MONITOR_PARAMS.est.hover_tether_angles[2])
    if angle_lims:
      stoplight = stoplights.SetByLimits(numpy.deg2rad(angle), angle_lims)

    return ({self._ele_label: timestamp, self._ele_cmd_label: timestamp},
            {self._ele_label: angle, self._ele_cmd_label: angle_cmd,
             self._source_label: source},
            stoplight)


class VesselPositionChart(indicator.BaseAttributeDictChart):
  """Indicator that charts the vessel altitude."""

  def __init__(self, **base_kwargs):
    sources = [
        ('GroundEstimate', None),
        ('GroundEstimateSim', None),
    ]
    labels = ['zg']
    super(VesselPositionChart, self).__init__(
        sources, labels, 'Altitude [m]', precision=2, ylim=[-3.0, 3.0],
        trail_length=80, **base_kwargs)

  def _IsValidInput(self, ground_estimate, ground_estimate_sim, *args):
    return (struct_tree.IsValidElement(ground_estimate) or
            struct_tree.IsValidElement(ground_estimate_sim))

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, ground_estimate, ground_estimate_sim):
    vessel_pos_g = None
    timestamp = None
    ground_estimate = (ground_estimate
                       if struct_tree.IsValidElement(ground_estimate)
                       else ground_estimate_sim)
    vessel_pos_g = ground_estimate.Xg
    timestamp = ground_estimate.capture_info['timestamp']

    info = {
        'zg': vessel_pos_g.z,
    }

    timestamps = {
        'zg': timestamp,
    }

    return timestamps, info, stoplights.STOPLIGHT_NORMAL


class VesselVelocityChart(indicator.BaseAttributeDictChart):
  """Indicator that charts the vessel velocity."""

  def __init__(self, **base_kwargs):
    sources = [
        ('GroundEstimate', None),
        ('GroundEstimateSim', None),
    ]
    labels = ['vx.g', 'vy.g', 'vz.g']
    super(VesselVelocityChart, self).__init__(
        sources, labels, 'Vg [m/s]', precision=2, ylim=[-3.0, 3.0],
        trail_length=80, **base_kwargs)

  def _IsValidInput(self, ground_estimate, ground_estimate_sim, *args):
    return (struct_tree.IsValidElement(ground_estimate) or
            struct_tree.IsValidElement(ground_estimate_sim))

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, ground_estimate, ground_estimate_sim):
    vessel_vel_g = None
    timestamp = None
    ground_estimate = (ground_estimate
                       if struct_tree.IsValidElement(ground_estimate)
                       else ground_estimate_sim)
    vessel_vel_g = ground_estimate.Vg
    timestamp = ground_estimate.capture_info['timestamp']

    info = {
        'vx.g': vessel_vel_g.x,
        'vy.g': vessel_vel_g.y,
        'vz.g': vessel_vel_g.z,
    }

    timestamps = {
        'vx.g': timestamp,
        'vy.g': timestamp,
        'vz.g': timestamp,
    }

    return timestamps, info, stoplights.STOPLIGHT_NORMAL


class VesselOrientationChart(indicator.BaseAttributeDictChart):
  """Indicator that charts the vessel velocity."""

  def __init__(self, **base_kwargs):
    sources = [
        ('ControlTelemetry', None, 'capture_info.timestamp'),
        ('ControlTelemetry', None, 'state_est.vessel.dcm_g2v.d'),
    ]
    labels = ['yaw', 'pitch', 'roll']
    super(VesselOrientationChart, self).__init__(
        sources, labels, 'Vessel angles [deg]', precision=2, ylim=[-40.0, 40.0],
        trail_length=80, **base_kwargs)

  def _IsValidInput(self, timestamp, dcm_g2v, *args):
    return timestamp is not None and dcm_g2v is not None

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, timestamp, dcm_g2v):
    yaw, pitch, roll = geometry.DcmToAngle(numpy.matrix(dcm_g2v))
    yaw_deg, pitch_deg, roll_deg = (numpy.rad2deg(yaw),
                                    numpy.rad2deg(pitch),
                                    numpy.rad2deg(roll))

    info = {
        'yaw': yaw_deg,
        'pitch': pitch_deg,
        'roll': roll_deg,
    }

    timestamps = {
        'yaw': timestamp,
        'pitch': timestamp,
        'roll': timestamp,
    }

    return timestamps, info, stoplights.STOPLIGHT_NORMAL
