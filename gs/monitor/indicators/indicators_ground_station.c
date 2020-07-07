/*
 * Copyright 2020 Makani Technologies LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gs/monitor/indicators/indicators_ground_station.h"

#include <math.h>
#include <stdint.h>

#include "avionics/common/encoder_types.h"
#include "avionics/common/winch_messages.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "gs/monitor/indicators/indicators_util.h"
#include "gs/monitor/monitor.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/indicator.h"

void UpdateAirDensity(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Air Density");

  if (!CheckAioComms() || !CheckGroundStationWeatherComms()) {
    MON_PRINTF(ind, "%s", "");
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const PhysParams *const phys = &GetSystemParams()->phys;
  const GsWeatherData *const weather = &aio_1->ground_station_weather.weather;

  // Temperature [Celcius].
  const double T = (double)weather->temperature;

  // Pressure [Pa].
  const double p = (double)weather->pressure * 100.0;

  // Humidity [fraction].
  const double relative_humidity = (double)weather->humidity / 100.0;

  // Density of air [kg/m^3]
  bool rho_valid;
  const double rho = CalcAirDensity(p, T, relative_humidity, &rho_valid);

  // Density of air [kg/m^3] as configured.
  const double rho_configured = phys->rho;

  if (rho_valid) {
    MON_PRINTF(ind, "%5.3f kg/m³ (%5.1f%% of configured)", rho,
               100.0 * rho / rho_configured);
  } else {
    MON_PRINTF(ind, "%s", "Weather data is out of range.");
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
    return;
  }

  // These are bits in the weather status that we can ignore.
  const uint8_t dont_care_status_bits = 0xb;

  if (1.0e-6 * aio_1->ground_station_weather.weather_latency > 2.0) {
    MON_PRINTF(ind, "%s", "Weather data is stale.");
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else if (weather->status & ~dont_care_status_bits) {
    MON_PRINTF(ind, "%s", "Weather data is invalid.");
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else if (fabs(rho - rho_configured) / rho_configured > 0.05) {
    // Indicate a warning if the computed density differs from the
    // configured density by more than 5%.
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateDrumState(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Drum");

  if (!CheckAioComms() || !CheckWinchPlcComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  double velocity = ApplyCal(aio_1->winch_plc.plc.winch_drum.velocity,
                             &GetSystemParams()->winch.drum_velocity_cal);

  MON_PRINTF(ind, "error: %d\nposition: %0.1f rad\nvelocity: %0.2f rad/s",
             aio_1->winch_plc.plc.flags.error,
             aio_1->winch_plc.plc.drum_position, velocity);
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
}

void UpdateWinchArmed(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Winch Armed");

  if (!CheckAioComms() || !CheckWinchPlcComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool armed = (aio_1->winch_plc.plc.state == kActuatorStateArmed) ||
               (aio_1->winch_plc.plc.state == kActuatorStateRunning);
  MON_PRINTF(ind, "%d", armed);

  if (armed) {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  }
}

void UpdateLevelwindElevation(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Level Ele [rad]");

  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool stale[kNumPlatforms];
  bool any_stale = false;
  bool all_stale = true;

  bool invalid[kNumDrums];
  bool any_invalid = false;

  for (int32_t i = 0; i < kNumPlatforms; ++i) {
    stale[i] = !CheckPlatformComms(i);
    any_stale |= stale[i];
    all_stale &= stale[i];

    invalid[i] = CheckWarning(&aio_1->platform_sensors[i].encoders.status,
                              kGsPerchEncodersWarningLevelwindElevation);
    invalid[i] |= CheckError(&aio_1->platform_sensors[i].encoders.status,
                             kGsPerchEncodersErrorLevelwindElevation);
    any_invalid |= invalid[i];
  }

  MON_PRINTF(ind, "A%s: % 3.4f%s\nB%s: % 3.4f%s",
             stale[kPlatformSensorsA] ? "(stale)" : "",
             aio_1->platform_sensors[kPlatformSensorsA].encoders.levelwind_ele,
             invalid[kPlatformSensorsA] ? " Invalid" : "",
             stale[kPlatformSensorsB] ? "(stale)" : "",
             aio_1->platform_sensors[kPlatformSensorsB].encoders.levelwind_ele,
             invalid[kPlatformSensorsB] ? " Invalid" : "");

  if (all_stale) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else if (any_stale) {
    // TODO: Once we have a levelwind again, warn if any_invalid.
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdatePerchAzimuthEncoders(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Perch Azi [rad]");

  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool parity[kNumPlatforms], warning[kNumPlatforms], error[kNumPlatforms];
  bool stale[kNumPlatforms];
  bool any_stale = false;
  bool all_stale = true;
  bool any_invalid = false;

  for (int32_t i = 0; i < kNumPlatforms; ++i) {
    stale[i] = !CheckPlatformComms(i);

    uint8_t perch_azi_flags =
        aio_1->platform_sensors[i].encoders.perch_azi_flags;

    parity[i] = perch_azi_flags & kAmo4306InvalidParity;
    warning[i] = perch_azi_flags & kAmo4306Warning;
    error[i] = perch_azi_flags & kAmo4306Error;

    warning[i] |= CheckWarning(&aio_1->platform_sensors[i].encoders.status,
                               kGsPerchEncodersWarningPerchAzimuth);
    error[i] |= CheckError(&aio_1->platform_sensors[i].encoders.status,
                           kGsPerchEncodersErrorPerchAzimuth);

    // TODO(b/25368865): Remove the following statement once there is a
    // redundant perch azimuth encoder.
    if ((PlatformLabel)i != kPlatformSensorsA) continue;

    any_stale |= stale[i];
    all_stale &= stale[i];
    any_invalid |= parity[i] || warning[i] || error[i];
  }

  MON_PRINTF(ind,
             "A%s: % 3.4f%s%s%s\n"
             "B%s: % 3.4f%s%s%s",
             stale[kPlatformSensorsA] ? "(stale)" : "",
             aio_1->platform_sensors[kPlatformSensorsA].encoders.perch_azi,
             error[kPlatformSensorsA] ? " Error" : "",
             warning[kPlatformSensorsA] ? " Warning" : "",
             parity[kPlatformSensorsA] ? " Parity" : "",
             stale[kPlatformSensorsB] ? "(stale)" : "",
             aio_1->platform_sensors[kPlatformSensorsB].encoders.perch_azi,
             error[kPlatformSensorsB] ? " Error" : "",
             warning[kPlatformSensorsB] ? " Warning" : "",
             parity[kPlatformSensorsB] ? " Parity" : "");

  if (all_stale) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else if (any_stale || any_invalid) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateGsgAzimuth(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "GSG Azi [rad]");

  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool stale[kNumDrums];
  bool any_stale = false;
  bool all_stale = true;

  bool invalid[kNumDrums];
  bool any_invalid = false;

  for (int32_t i = 0; i < kNumDrums; ++i) {
    stale[i] = !CheckDrumComms(i);
    any_stale |= stale[i];
    all_stale &= stale[i];

    invalid[i] = CheckWarning(&aio_1->drum_sensors[i].encoders.status,
                              kGsDrumEncodersWarningGsgAzimuth);
    invalid[i] |= CheckError(&aio_1->drum_sensors[i].encoders.status,
                             kGsDrumEncodersErrorGsgAzimuth);

    // TODO: Remove the following line once we have redundant
    // GSG Azimuth encoders.
    if (i != kDrumSensorsA) continue;

    any_invalid |= invalid[i];
  }

  MON_PRINTF(ind,
             "A%s: % 3.4f%s\n"
             "B%s: % 3.4f%s",
             stale[kDrumSensorsA] ? "(stale)" : "",
             aio_1->drum_sensors[kDrumSensorsA].encoders.gsg_azi,
             invalid[kDrumSensorsA] ? " Invalid" : "",
             stale[kDrumSensorsB] ? "(stale)" : "",
             aio_1->drum_sensors[kDrumSensorsB].encoders.gsg_azi,
             invalid[kDrumSensorsB] ? " Invalid" : "");

  if (all_stale) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else if (any_stale || any_invalid) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateGsgElevation(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "GSG Ele [rad]");

  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool stale[kNumDrums];
  bool any_stale = false;
  bool all_stale = true;

  bool invalid[kNumDrums];
  bool any_invalid = false;

  for (int32_t i = 0; i < kNumDrums; ++i) {
    stale[i] = !CheckDrumComms(i);
    any_stale |= stale[i];
    all_stale &= stale[i];

    invalid[i] = CheckWarning(&aio_1->drum_sensors[i].encoders.status,
                              kGsDrumEncodersWarningGsgElevation);
    invalid[i] |= CheckError(&aio_1->drum_sensors[i].encoders.status,
                             kGsDrumEncodersErrorGsgElevation);
    any_invalid |= invalid[i];
  }

  MON_PRINTF(ind,
             "A%s: % 3.4f%s\n"
             "B%s: % 3.4f%s",
             stale[kDrumSensorsA] ? "(stale)" : "",
             aio_1->drum_sensors[kDrumSensorsA].encoders.gsg_ele,
             invalid[kDrumSensorsA] ? " Invalid" : "",
             stale[kDrumSensorsB] ? "(stale)" : "",
             aio_1->drum_sensors[kDrumSensorsB].encoders.gsg_ele,
             invalid[kDrumSensorsB] ? " Invalid" : "");

  if (all_stale) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else if (any_stale || any_invalid) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdatePlcStatus(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "PLC Status");

  if (!CheckAioComms() || !CheckWinchPlcComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  MON_PRINTF(ind, "winch: %d\nlevelwind: %d",
             aio_1->winch_plc.plc.winch_drum.flags.status,
             aio_1->winch_plc.plc.levelwind.flags.status);
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
}

void UpdateWeather(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Weather");

  if (!CheckAioComms() || !CheckGroundStationWeatherComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  GsWeatherData weather = aio_1->ground_station_weather.weather;

  MON_PRINTF(ind, "%5.1f°C, DewPt: %4.1f°C (%3.0f%%), %4.0f mb",
             (double)weather.temperature, (double)weather.dewpoint,
             (double)weather.humidity, (double)weather.pressure);

  // These are bits in the weather status that we can ignore.
  const uint8_t dont_care_status_bits = 0xb;

  if ((1.0e-6 * aio_1->ground_station_weather.weather_latency > 2.0) ||
      (weather.status & ~dont_care_status_bits)) {
    // Weather sensor data is stale or invalid.
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateWinchProximitySensor(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Proximity");

  if (!CheckAioComms() || !CheckWinchPlcComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  uint16_t proximity_flag = aio_1->winch_plc.plc.proximity;
  MON_PRINTF(ind, "Final [%c%c%c%c] Early",
             (proximity_flag & kWinchProximityFinalA) ? '1' : '-',
             (proximity_flag & kWinchProximityFinalB) ? '1' : '-',
             (proximity_flag & kWinchProximityEarlyA) ? '1' : '-',
             (proximity_flag & kWinchProximityEarlyB) ? '1' : '-');
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
}

void UpdateWindSensor(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Wind [m/s]");

  if (!CheckAioComms() || !CheckGroundStationWeatherComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  Vec3 wind_velocity_ws = {aio_1->ground_station_weather.wind.wind_velocity[0],
                           aio_1->ground_station_weather.wind.wind_velocity[1],
                           aio_1->ground_station_weather.wind.wind_velocity[2]};

  MON_PRINTF(ind, "u:% 5.1f v:% 5.1f w:% 5.1f, speed: %4.1f",
             wind_velocity_ws.x, wind_velocity_ws.y, wind_velocity_ws.z,
             Vec3Norm(&wind_velocity_ws));

  if ((1.0e-6 * aio_1->ground_station_weather.wind_latency > 1.0) ||
      (aio_1->ground_station_weather.wind.status != 0)) {
    // Weather sensor data is stale or invalid.
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else if (aio_1->ground_station_weather.wind.speed_of_sound < 315.0 ||
             aio_1->ground_station_weather.wind.speed_of_sound > 360.0) {
    // An implausible speed-of-sound measurement presumably also
    // indicates an incorrect wind speed measurement.  The range given
    // here covers -25 to +50 degrees Celsius.  The relationship is
    // c_air = (331.45 m/s) * sqrt(1 + T / 273.15 degrees Celsius) where
    // T is the temperature in Celsius.  Effects due to humidity are of
    // order one percent at 100% humidity and high temperature.
    // Reference: http://www.rane.com/pdf/eespeed.pdf
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}
