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

#include "avionics/platform/firmware/output.h"

#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/monitors/aio_types.h"
#include "avionics/firmware/monitors/ground_io_types.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/platform/firmware/encoders.h"
#include "avionics/platform/firmware/monitor.h"
#include "avionics/platform/firmware/weather.h"

static PlatformSensorsMessage g_output;
static PlatformSensorsMonitorMessage g_mon_output;
static GroundStationWeatherMessage g_weather_output;
static int64_t g_weather_timestamp = INT32_MIN;
static int64_t g_wind_timestamp = INT32_MIN;

static bool IsWeatherStation(void) {
  return AppConfigGetAioNode() == kAioNodePlatformSensorsA;
}

void PlatformOutputInit(void) {
  memset(&g_output, 0, sizeof(g_output));
  memset(&g_mon_output, 0, sizeof(g_mon_output));
  memset(&g_weather_output, 0, sizeof(g_weather_output));
  EncodersInit();
  WeatherInit();
  PlatformMonInit();
}

AioModuleMonitorData *PlatformOutputGetAioModuleMonitors(void) {
  return &g_mon_output.aio_mon;
}

GroundIoMonitorData *PlatformOutputGetGroundIoMonitors(void) {
  return &g_mon_output.ground_io_mon;
}

void PlatformOutputPoll(int64_t now) {
  PlatformMonPoll();

  if (IsWeatherStation()) {
    GillDataMetPakFull weather;
    if (WeatherPollMetPak(&weather)) {
      g_weather_output.weather.pressure = weather.pressure;
      g_weather_output.weather.humidity = weather.humidity;
      g_weather_output.weather.dewpoint = weather.dewpoint;
      g_weather_output.weather.temperature = weather.temperature;
      g_weather_output.weather.status = weather.status;
      g_weather_timestamp = now;
    }
    if (WeatherPollWind(&g_weather_output.wind)) {
      g_wind_timestamp = now;
    }
  }
}

void PlatformOutputSendStatus(void) {
  EncodersRead(&g_output.encoders);
  NetSendAioPlatformSensorsMessage(&g_output);
}

void PlatformOutputSendMonitorStatus(void) {
  NetSendAioPlatformSensorsMonitorMessage(&g_mon_output);
}

void PlatformOutputSendWeather(int64_t now) {
  if (IsWeatherStation()) {
    g_weather_output.weather_latency =
        SaturateLatency(now - g_weather_timestamp);
    g_weather_output.wind_latency =
        SaturateLatency(now - g_wind_timestamp);
    NetSendAioGroundStationWeatherMessage(&g_weather_output);
  }
}
