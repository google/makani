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

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/imu_output.h"
#include "avionics/fc/firmware/config_params.h"
#include "avionics/fc/firmware/fpv_pitot_control.h"
#include "avionics/fc/firmware/monitor.h"
#include "avionics/fc/firmware/output.h"
#include "avionics/fc/firmware/selftest.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/drivers/adis16488.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/drivers/faa_light.h"
#include "avionics/firmware/drivers/hsc.h"
#include "avionics/firmware/drivers/led.h"
#include "avionics/firmware/drivers/q7_watchdog.h"
#include "avionics/firmware/gps/gps_interface.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/network/net_diag.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/firmware/network/switch_config.h"
#include "avionics/firmware/output/slow_status.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_stats.h"
#include "common/macros.h"

// Reserve time at the end of each iteration to prevent processes from
// overrunning the maximum processing time. All trivial polling functions
// called at the end ScheduleLoop() should take less than this amount of time.
#define GUARD_TIME_US 50

// Define maximum loop processing time.
#define LOOP_PROCESSING_TIME_US (1000000 / ADIS16488_SYNC_FREQ_HZ)

// Define message rate decimations of loop iteration rate.
#define SLOW_STATUS_MESSAGE_DECIMATION                  \
  (ADIS16488_SYNC_FREQ_HZ / SLOW_STATUS_FREQUENCY_HZ)
#define SERIAL_DEBUG_DECIMATION                         \
  (ADIS16488_SYNC_FREQ_HZ / SERIAL_DEBUG_FREQUENCY_HZ)
#define GPS_SOLUTION_DECIMATION                                 \
  (ADIS16488_SYNC_FREQ_HZ / NOV_ATEL_SOLUTION_FREQUENCY_HZ)
#define GPS_OBSERVATIONS_DECIMATION                             \
  (ADIS16488_SYNC_FREQ_HZ / NOV_ATEL_OBSERVATIONS_FREQUENCY_HZ)
#define GPS_SATELLITES_DECIMATION                               \
  (ADIS16488_SYNC_FREQ_HZ / GPS_SATELLITES_FREQUENCY_HZ)
#define GPS_TIME_DECIMATION (ADIS16488_SYNC_FREQ_HZ / GPS_TIME_FREQUENCY_HZ)
#define FAA_LIGHT_TIME_DECIMATION                               \
  (ADIS16488_SYNC_FREQ_HZ / FAA_LIGHT_STATUS_FREQUENCY_HZ)

// Define the amount of time before selecting a new RTCM source.
#define GPS_RTCM_OBS_TIMEOUT_US 5000000  // Observables.
#define GPS_RTCM_AUX_TIMEOUT_US 5000000  // Auxiliary information.

// Septentrio's support indicates that the AsteRx-m receiver has limited
// processing capabilities and will not be able to track all satellites
// with a 10 Hz RTCM rate and 10 Hz PVT rate.
#define SEPTENTRIO_RTCM_MIN_PERIOD_US 500000

typedef bool (*const CvtGetFunction)(AioNode source, void *message,
                                     uint16_t *seq_num, int64_t *timestamp);

static int64_t g_rtcm_obs_timestamp = INT32_MIN;
static int64_t g_rtcm1006_timestamp = INT32_MIN;
static int64_t g_rtcm1033_timestamp = INT32_MIN;
static int64_t g_rtcm1230_timestamp = INT32_MIN;
static int64_t g_rtcm_timestamp = INT32_MIN;

static int32_t g_rtcm_obs_source_index = -1;
static int32_t g_rtcm1006_source_index = -1;
static int32_t g_rtcm1033_source_index = -1;
static int32_t g_rtcm1230_source_index = -1;
static int32_t g_rtcm_source_index = -1;

static AioNode g_current_node;

static FlightComputerSensorMessage g_sensor_message;

static void HandleGpsRtcm(CvtGetFunction cvt_get, int32_t timeout,
                          int32_t max_length, const int16_t *length,
                          const uint8_t *data, void *message,
                          int32_t *last_source_index, int64_t *last_timestamp) {
  // List RTCM sources according to descending priority.
  const AioNode sources[] = {kAioNodeGpsBaseStation, kAioNodeCsA, kAioNodeCsB};

  for (int32_t i = 0; i < ARRAYSIZE(sources); ++i) {
    int64_t timestamp;
    if (cvt_get(sources[i], message, NULL, &timestamp) && 0 < *length &&
        *length <= max_length &&
        (timestamp - *last_timestamp > timeout || i <= *last_source_index)) {
      // Rate limit corrections to Septentrio receiver.
      if (kFcConfigParams->gps_receiver != kGpsReceiverTypeSeptentrio ||
          timestamp - *last_timestamp > SEPTENTRIO_RTCM_MIN_PERIOD_US) {
        // Select new source.
        *last_timestamp = timestamp;
        *last_source_index = i;

        // Forward differential GPS corrections to wing receiver.
        GpsInsertRtcm(&kGpsConfigRover, kFcConfigParams->gps_receiver, *length,
                      data);
      }
    }
  }
}

static void PollGpsRtcmMessages(void) {
  // Declare static to prevent stack overflow.
  static union {
    GpsRtcm1006Message rtcm1006;
    GpsRtcm1033Message rtcm1033;
    GpsRtcm1230Message rtcm1230;
    GpsRtcm1072Message rtcm1072;
    GpsRtcm1074Message rtcm1074;
    GpsRtcm1082Message rtcm1082;
    GpsRtcm1084Message rtcm1084;
    GpsRtcmMessage rtcm;
  } u;

#define HANDLE_GPS_RTCM(cvt_get, timeout, message, source_index, timestamp) \
  HandleGpsRtcm((CvtGetFunction)(cvt_get), (timeout),                   \
                ARRAYSIZE((message)->data), &(message)->length,         \
                (message)->data, (message), (source_index), (timestamp))

  // Observables.
  HANDLE_GPS_RTCM(CvtGetGpsRtcm1072Message, GPS_RTCM_OBS_TIMEOUT_US,
                  &u.rtcm1072, &g_rtcm_obs_source_index, &g_rtcm_obs_timestamp);
  HANDLE_GPS_RTCM(CvtGetGpsRtcm1074Message, GPS_RTCM_OBS_TIMEOUT_US,
                  &u.rtcm1074, &g_rtcm_obs_source_index, &g_rtcm_obs_timestamp);
  HANDLE_GPS_RTCM(CvtGetGpsRtcm1082Message, GPS_RTCM_OBS_TIMEOUT_US,
                  &u.rtcm1082, &g_rtcm_obs_source_index, &g_rtcm_obs_timestamp);
  HANDLE_GPS_RTCM(CvtGetGpsRtcm1084Message, GPS_RTCM_OBS_TIMEOUT_US,
                  &u.rtcm1084, &g_rtcm_obs_source_index, &g_rtcm_obs_timestamp);

  // Auxiliary information.
  HANDLE_GPS_RTCM(CvtGetGpsRtcm1006Message, GPS_RTCM_AUX_TIMEOUT_US,
                  &u.rtcm1006, &g_rtcm1006_source_index, &g_rtcm1006_timestamp);
  HANDLE_GPS_RTCM(CvtGetGpsRtcm1033Message, GPS_RTCM_AUX_TIMEOUT_US,
                  &u.rtcm1033, &g_rtcm1033_source_index, &g_rtcm1033_timestamp);
  HANDLE_GPS_RTCM(CvtGetGpsRtcm1230Message, GPS_RTCM_AUX_TIMEOUT_US,
                  &u.rtcm1230, &g_rtcm1230_source_index, &g_rtcm1230_timestamp);

  // Catch all.
  HANDLE_GPS_RTCM(CvtGetGpsRtcmMessage, GPS_RTCM_OBS_TIMEOUT_US, &u.rtcm,
                  &g_rtcm_source_index, &g_rtcm_timestamp);

#undef HANDLE_GPS_RTCM
}

static void PollExtWatchdog(void) { ExtWatchdogPoll(); }

static void PollFaaLight(void) {
  if (IsLightNode(g_current_node)) {
    FaaLightPoll();
  }
}

static void PollSwitch(void) { Bcm53101Poll(GetSwitchConfig()); }

static void PollGps(void) {
  int64_t now = ClockGetUs();
  if (GpsDevicePollPps(&kGps)) {
    FcOutputPps(now);
    GpsUpdatePpsTimestamp(now);
  }
  GpsPoll(&kGpsConfigRover, kFcConfigParams->gps_receiver, now);
}

static HscDevice PollPitot(HscDevice pitot, int64_t now) {
  HscDevice pitot_next = (pitot + 1) % kNumHscDevices;
  HscData data;
  if (HscRead(&data)) {
    FcOutputPitot(pitot, &kFcConfigParams->pitot, &data, &g_sensor_message,
                  now);
  }
  HscTrigger(pitot_next);
  return pitot_next;
}

static void ScheduleLoop(void) {
  // Current pitot tube device.
  HscDevice pitot = 0;

  // Define iteration counters for loop decimation.
  int32_t sensor_message_count = 0;
  int32_t slow_status_message_count = 0;
  int32_t faa_light_status_message_count = 0;
  int32_t imu_count = 0;
  int32_t serial_debug_count = SERIAL_DEBUG_DECIMATION / 4;
  int32_t gps_solution_count = 0;
  int32_t gps_observations_count = GPS_OBSERVATIONS_DECIMATION / 2;
  int32_t gps_satellites_count = 0;
  int32_t gps_time_count = 0;

  // GpsSolutionMessage and GpsObservationsMessage should be out of phase.
  assert(GPS_SOLUTION_DECIMATION == GPS_OBSERVATIONS_DECIMATION);
  assert(gps_solution_count == 0 &&
         gps_observations_count == GPS_OBSERVATIONS_DECIMATION / 2);

  // Poll trivial processes.
  void (*const poll_fn[])(void) = {
    FcMonPoll,       NetDiagPoll,      NetMonPoll,     NetPoll,
    PollExtWatchdog, PollFaaLight,     PollGps,        PollGpsRtcmMessages,
    PollSwitch,      Q7WatchdogFcPoll, FpvPitotControlPoll,
  };

  // Loop forever.
  for (;;) {
    // Align schedule with the ePWM output that synchronizes the IMU. The ePWM
    // frequency is 2400 Hz, so we have <416 us to process everything.
    Adis16488WaitForSyncPulse();

    int64_t now = ClockGetUs();
    int64_t start_time = now;

    // Use LEDs for loop timing measurements.
    LedOn(true, true);

    // At this time, the MibSPI should have an IMU transfer in progress for
    // the current synchronization pulse. If we trigger a pitot tube transfer
    // now, the MibSPI handle this request immediately after the IMU transfer
    // completes.
    pitot = PollPitot(pitot, now);

    // Sample the IMU output from the previous synchronization pulse.
    Adis16488OutputData imu_output;
    bool imu_cs_update = false;
    if (Adis16488PollOutputData(now, &imu_output)) {
      imu_cs_update = ImuOutputData(&imu_output, &g_sensor_message, now);
      ++imu_count;
    }

    // Output sensor warnings.
    ImuOutputUpdateFailure(!Adis16488IsReady(), &g_sensor_message);
    FcOutputGpsFailure(!GpsIsReady(kFcConfigParams->gps_receiver),
                       &g_sensor_message);

    // Send FlightComputerImuMessage.
    if (imu_count >= IMU_OUTPUT_DECIMATION) {
      imu_count = 0;
      now = ClockGetUs();
      ImuOutputSendImuMessage(kFcConfigParams->imu.fir_coefs, now);
    }

    // Send FlightComputerSensorMessage. Synchronize to c/s update.
    ++sensor_message_count;
    if (imu_cs_update || sensor_message_count >= CONING_SCULLING_DECIMATION) {
      sensor_message_count = 0;
      now = ClockGetUs();
      ImuOutputUpdateSensorMessage(&g_sensor_message, now);
      FcOutputUpdateSensorMessage(&g_sensor_message, now);
      NetSendAioFlightComputerSensorMessage(&g_sensor_message);
    }

    // Send SerialDebugMessage.
    ++serial_debug_count;
    if (serial_debug_count >= SERIAL_DEBUG_DECIMATION) {
      serial_debug_count = 0;
      GpsSendSerialDebugMessage();
    }

    // Send Hemisphere/NovAtel/SeptentrioSolutionMessage.
    ++gps_solution_count;
    if (gps_solution_count >= GPS_SOLUTION_DECIMATION) {
      gps_solution_count = 0;
      now = ClockGetUs();
      GpsSendSolutionMessage(kFcConfigParams->gps_receiver, now);
    }

    // Send Hemisphere/NovAtel/SeptentrioObservationsMessage.
    ++gps_observations_count;
    if (gps_observations_count >= GPS_OBSERVATIONS_DECIMATION) {
      gps_observations_count = 0;
      now = ClockGetUs();
      GpsSendObservationsMessage(kFcConfigParams->gps_receiver, now);
    }

    // Send GpsSatellitesMessage.
    ++gps_satellites_count;
    if (gps_satellites_count >= GPS_SATELLITES_DECIMATION) {
      gps_satellites_count = 0;
      now = ClockGetUs();
      GpsSendSatellitesMessage(now);
    }

    // Send GpsTimeMessage.
    ++gps_time_count;
    if (gps_time_count >= GPS_TIME_DECIMATION) {
      gps_time_count = 0;
      now = ClockGetUs();
      GpsSendTimeMessage(now);
    }

    // Send FaaLightStatusMessage.
    if (IsLightNode(g_current_node)) {
      ++faa_light_status_message_count;
      if (faa_light_status_message_count >= FAA_LIGHT_TIME_DECIMATION) {
        faa_light_status_message_count = 0;
        FaaLightSendStatusMessage();
      }
    }

    // Send SlowStatusMessage.
    ++slow_status_message_count;
    if (slow_status_message_count >= SLOW_STATUS_MESSAGE_DECIMATION) {
      slow_status_message_count = 0;
      now = ClockGetUs();
      OutputSendSlowStatusMessage(now);
    }

    // Poll all trivial processes once.
    for (int32_t i = 0; i < ARRAYSIZE(poll_fn); ++i) {
      poll_fn[i]();
    }

    // Complete required processing.
    LedOff(true, false);

    // Poll trivial processes until end time.
    now = ClockGetUs();
    int64_t end_time = start_time + LOOP_PROCESSING_TIME_US - GUARD_TIME_US;
    for (int32_t cycle = 0; now < end_time; ++cycle) {
      cycle %= ARRAYSIZE(poll_fn);
      poll_fn[cycle]();
      now = ClockGetUs();
    }
  }
}

int main(void) {
  // Initialize watchdog.
  ExtWatchdogInit();
  Q7WatchdogFcInit();

  // Initialize TMS570 hardware.
  MibSPIInit(1, kSpiPinmuxAll);  // For access switch.
  MibSPIInit(3, kSpiPinmuxAll);  // For IMU, pitot.
  I2cInit(400000);               // For voltage and temperature monitoring.

  g_current_node = AppConfigGetAioNode();
  if (IsLightNode(g_current_node)) {
    FaaLightInit();  // Initialize FAA lights.
  }

  // Initialize network.
  SwitchConfigInit();
  Bcm53101Init(true);
  NetInit(g_current_node);

  // Perform self test as soon as possible and immediately after NetInit()
  // such that we validate parameters before calling dependent code.
  SelfTest();
  NetMonInit(GetSwitchConfig()->info);
  NetDiagInit(GetSwitchConfig()->info);

  // Initialize IMU.
  Adis16488Init();

  // Initialize pitot.
  HscInit();

  // Initialize GPS.
  GpsInit();

  // Initialize flight computer output.
  FcOutputInit(&g_sensor_message);

  // Initialize IMU output.
  ImuOutputInit(&g_sensor_message);

  // FPV power control.
  FpvPitotControlInit(&g_sensor_message);

  // Initialize LEDs for timing measurements.
  LedInit();

  // Initialize flight computer monitors.
  FcMonInit(&g_sensor_message.fc_mon, &g_sensor_message.aio_mon);

  // Initialize slow status message.
  OutputInitSlowStatusMessage();

  // Enable interrupts for SCI driver.
  VimEnableIrq();

  // Run!
  ScheduleLoop();

  return 0;
}
