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

#include "avionics/fc/firmware/output.h"

#include <assert.h>
#include <limits.h>
#include <string.h>

#include "avionics/common/adis16488_types.h"
#include "avionics/common/avionics_messages.h"
#include "avionics/fc/firmware/config_params.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/drivers/adis16488.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/monitors/fc.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"

static int64_t g_pitot_timestamp = INT32_MIN;
static int64_t g_pps_timestamp = INT32_MIN;

// TODO(b/123896334): Process non-exist/invalid hardware at driver level.

static bool IgnoreImuWarning(void) {
  // The followling nodes do not have an Imu.
  const AioNode node = AppConfigGetAioNode();
  return (node == kAioNodeLightTailBottom || node == kAioNodeLightTailTop);
}

static bool IgnorePitotWarning(void) {
  // The following nodes do not have pitot tube connected.
  const AioNode node = AppConfigGetAioNode();
  return (IsLightNode(node));
}

static float ApplyPressureCalib(const HscPitotCalib *cal, const HscData *data) {
  float p = HscPressureRawToPa(cal->model, data);
  return p * cal->scale + cal->bias;
}

static void UpdatePitot(const HscPitotCalib *cal, const HscData *data,
                        FlightComputerWarning warning,
                        FlightComputerFlag diag_flag, float *pressure,
                        float *temperature, StatusFlags *flags) {
  if (!data->invalid) {
    *pressure = ApplyPressureCalib(cal, data);
    *temperature = HscTemperatureRawToC(data);

    // Check for diagnostic condition.
    SetStatus(diag_flag, data->status == 0x03, flags);
  }
  SignalWarning(warning, data->invalid || data->status != 0x0, flags);
}

void FcOutputInit(FlightComputerSensorMessage *sensor_message) {
  // Status messages.
  memset(sensor_message, 0, sizeof(*sensor_message));
  g_pitot_timestamp = INT32_MIN;
  g_pps_timestamp = INT32_MIN;
}

void FcOutputGpsFailure(bool failure,
                        FlightComputerSensorMessage *sensor_message) {
  bool no_gps = (kFcConfigParams->gps_receiver == kGpsReceiverTypeNone);

  SetStatus(kFlightComputerFlagNoGps, no_gps, &sensor_message->flags);
  SignalWarning(kFlightComputerWarningGps, !no_gps && failure,
                &sensor_message->flags);
}

void FcOutputFpvState(bool enabled,
                      FlightComputerSensorMessage *sensor_message) {
  SignalWarning(kFlightComputerWarningFpvEnabled, enabled,
                &sensor_message->flags);
}

void FcOutputPitotState(uint8_t status,
                        FlightComputerSensorMessage *sensor_message) {
  sensor_message->pitot_cover_status = status;
}

void FcOutputPitot(HscDevice dev, const PitotCalib *cal, const HscData *data,
                   FlightComputerSensorMessage *sensor_message, int64_t now) {
  if (!data->invalid) {
    g_pitot_timestamp = now;
  }
  switch (dev) {
    case kHscDeviceAltitude:
      UpdatePitot(&cal->altitude, data, kFlightComputerWarningPitotAltitude,
                  kFlightComputerFlagPitotAltitudeDiag,
                  &sensor_message->pitot.altitude,
                  &sensor_message->pitot.altitude_temp, &sensor_message->flags);
      break;
    case kHscDevicePitch:
      UpdatePitot(&cal->pitch, data, kFlightComputerWarningPitotPitch,
                  kFlightComputerFlagPitotPitchDiag,
                  &sensor_message->pitot.pitch,
                  &sensor_message->pitot.pitch_temp, &sensor_message->flags);
      break;
    case kHscDeviceSpeed:
      UpdatePitot(&cal->speed, data, kFlightComputerWarningPitotSpeed,
                  kFlightComputerFlagPitotSpeedDiag,
                  &sensor_message->pitot.speed,
                  &sensor_message->pitot.speed_temp, &sensor_message->flags);
      break;
    case kHscDeviceYaw:
      UpdatePitot(&cal->yaw, data, kFlightComputerWarningPitotYaw,
                  kFlightComputerFlagPitotYawDiag, &sensor_message->pitot.yaw,
                  &sensor_message->pitot.yaw_temp, &sensor_message->flags);
      break;
    default:
      assert(false);
      break;
  }
}

void FcOutputPps(int64_t now) { g_pps_timestamp = now; }

void FcOutputUpdateSensorMessage(FlightComputerSensorMessage *sensor_message,
                                 int64_t now) {
  if (IgnoreImuWarning()) {
    uint32_t imu_warnings =
        kFlightComputerWarningImu | kFlightComputerWarningImuData;
    sensor_message->flags.warning &= ~imu_warnings;
  }

  if (IgnorePitotWarning()) {
    uint32_t pitot_warnings =
        kFlightComputerWarningPitotAltitude | kFlightComputerWarningPitotPitch |
        kFlightComputerWarningPitotSpeed | kFlightComputerWarningPitotYaw;
    sensor_message->flags.warning &= ~pitot_warnings;
  }

  // GPS 1-PPS latency.
  sensor_message->pps_latency_usec = SaturateLatency(now - g_pps_timestamp);

  // Pitot latency.
  sensor_message->pitot.latency_usec = SaturateLatency(now - g_pitot_timestamp);
}
