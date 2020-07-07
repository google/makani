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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/gps_receiver.h"
#include "avionics/common/imu_output.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/drivers/adis16488.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/drivers/faa_light.h"
#include "avionics/firmware/drivers/q7_watchdog.h"
#include "avionics/firmware/gps/gps_interface.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/network/net_diag.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/firmware/network/switch_config.h"
#include "avionics/firmware/output/slow_status.h"
#include "avionics/firmware/serial/aio_serial_params.h"
#include "avionics/firmware/serial/fc_serial_params.h"
#include "avionics/firmware/util/selftest.h"
#include "avionics/gps/firmware/config_params.h"
#include "avionics/gps/firmware/monitor.h"
#include "avionics/gps/firmware/output.h"
#include "avionics/gps/firmware/selftest.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_stats.h"
#include "common/macros.h"

// Reserve time at the end of each iteration to prevent processes from
// overrunning the maximum processing time. All trivial polling functions
// called at the end ScheduleLoop() should take less than this amount of time.
#define GUARD_TIME_US 50

// Define maximum loop processing time.
#define LOOP_PROCESSING_TIME_US (1000000 / ADIS16488_SYNC_FREQ_HZ)

#define SLOW_STATUS_MESSAGE_DECIMATION \
  (ADIS16488_SYNC_FREQ_HZ / SLOW_STATUS_FREQUENCY_HZ)
#define SERIAL_DEBUG_DECIMATION \
  (ADIS16488_SYNC_FREQ_HZ / SERIAL_DEBUG_FREQUENCY_HZ)
#define GPS_SOLUTION_DECIMATION \
  (ADIS16488_SYNC_FREQ_HZ / NOV_ATEL_SOLUTION_FREQUENCY_HZ)
#define GPS_OBSERVATIONS_DECIMATION \
  (ADIS16488_SYNC_FREQ_HZ / NOV_ATEL_OBSERVATIONS_FREQUENCY_HZ)
#define GPS_COMPASS_DECIMATION \
  (ADIS16488_SYNC_FREQ_HZ / NOV_ATEL_COMPASS_FREQUENCY_HZ)
#define GPS_SATELLITES_DECIMATION \
  (ADIS16488_SYNC_FREQ_HZ / GPS_SATELLITES_FREQUENCY_HZ)
#define GPS_TIME_DECIMATION (ADIS16488_SYNC_FREQ_HZ / GPS_TIME_FREQUENCY_HZ)
#define FAA_LIGHT_TIME_DECIMATION \
  (ADIS16488_SYNC_FREQ_HZ / FAA_LIGHT_STATUS_FREQUENCY_HZ)

static void PollExtWatchdog(void) { ExtWatchdogPoll(); }

static void PollFaaLight(void) { FaaLightPoll(); }

static void PollSwitch(void) { Bcm53101Poll(GetSwitchConfig()); }

static void PollGps(void) {
  int64_t now = ClockGetUs();
  if (GpsDevicePollPps(&kGps)) {
    GpsUpdatePpsTimestamp(now);
  }
  GpsPoll(&kGpsConfigBase, kGpsConfigParams->gps_receiver, now);
}

static FlightComputerSensorMessage g_sensor_message;

int main(void) {
  // Initialize watchdog.
  ExtWatchdogInit();
  Q7WatchdogFcInit();

  // Initialize TMS570 hardware.
  MibSPIInit(1, kSpiPinmuxAll);  // For access switch.
  MibSPIInit(3, kSpiPinmuxAll);  // For IMU, pitot.

  // Initialize network.
  SwitchConfigInit();
  Bcm53101Init(true);
  NetInit(AppConfigGetAioNode());

  // Perform self test as soon as possible and immediately after NetInit()
  // such that we validate parameters before calling dependent code.
  SelfTest();
  NetMonInit(GetSwitchConfig()->info);
  NetDiagInit(GetSwitchConfig()->info);
  I2cInit(400e3);
  GpsMonInit();

  // Initialize IMU.
  Adis16488Init();
  ImuOutputInit(&g_sensor_message);

  // Initialize GPS.
  GpsInit();

  // Initialize FAA light.
  FaaLightInit();

  // Initialize output.
  GpsOutputInit();
  OutputInitSlowStatusMessage();

  // Enable interrupts last.
  VimEnableIrq();

  // Define iteration counters for loop decimation.
  int32_t sensor_message_count = 0;
  int32_t slow_status_message_count = 0;
  int32_t faa_light_status_message_count = 0;
  int32_t imu_count = 0;
  int32_t serial_debug_count = SERIAL_DEBUG_DECIMATION / 4;
  int32_t gps_solution_count = 0;
  int32_t gps_compass_count = 0;
  int32_t gps_observations_count = GPS_OBSERVATIONS_DECIMATION / 2;
  int32_t gps_satellites_count = 0;
  int32_t gps_time_count = 0;

  // GpsSolutionMessage and GpsObservationsMessage should be out of phase.
  assert(GPS_SOLUTION_DECIMATION == GPS_OBSERVATIONS_DECIMATION);
  assert(gps_solution_count == 0 &&
         gps_observations_count == GPS_OBSERVATIONS_DECIMATION / 2);

  // Poll trivial processes.
  void (*const poll_fn[])(void) = {
      NetDiagPoll, NetMonPoll, NetPoll,          PollExtWatchdog, PollFaaLight,
      PollGps,     PollSwitch, Q7WatchdogFcPoll, GpsMonPoll};

  while (true) {
    // Align schedule with the ePWM output that synchronizes the IMU. The ePWM
    // frequency is 2400 Hz, so we have <416 us to process everything.
    Adis16488WaitForSyncPulse();

    int64_t now = ClockGetUs();
    int64_t start_time = now;

    // Sample the IMU output from the previous synchronization pulse.
    Adis16488OutputData imu_output;
    bool imu_cs_update = false;
    if (Adis16488PollOutputData(now, &imu_output)) {
      imu_cs_update = ImuOutputData(&imu_output, &g_sensor_message, now);
      ++imu_count;
    }

    ImuOutputUpdateFailure(!Adis16488IsReady(), &g_sensor_message);

    // Send FlightComputerImuMessage.
    if (imu_count >= IMU_OUTPUT_DECIMATION) {
      imu_count = 0;
      now = ClockGetUs();
      ImuOutputSendImuMessage(kGpsConfigParams->imu.fir_coefs, now);
    }

    // Send FlightComputerSensorMessage. Synchronize to c/s update.
    ++sensor_message_count;
    if (imu_cs_update || sensor_message_count >= CONING_SCULLING_DECIMATION) {
      sensor_message_count = 0;
      now = ClockGetUs();
      ImuOutputUpdateSensorMessage(&g_sensor_message, now);
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
      GpsSendSolutionMessage(kGpsConfigParams->gps_receiver, now);
    }

    // Send Hemisphere/NovAtel/SeptentrioObservationsMessage.
    ++gps_observations_count;
    if (gps_observations_count >= GPS_OBSERVATIONS_DECIMATION) {
      gps_observations_count = 0;
      now = ClockGetUs();
      GpsSendObservationsMessage(kGpsConfigParams->gps_receiver, now);
    }

    // Send GpsCompassMessage.
    ++gps_compass_count;
    if (gps_compass_count >= GPS_COMPASS_DECIMATION) {
      gps_compass_count = 0;
      now = ClockGetUs();
      GpsSendCompassMessage(kGpsConfigParams->gps_receiver, now);
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
    ++faa_light_status_message_count;
    if (faa_light_status_message_count >= FAA_LIGHT_TIME_DECIMATION) {
      faa_light_status_message_count = 0;
      FaaLightSendStatusMessage();
    }

    // Send SlowStatusMessage.
    ++slow_status_message_count;
    if (slow_status_message_count >= SLOW_STATUS_MESSAGE_DECIMATION) {
      slow_status_message_count = 0;
      now = ClockGetUs();
      OutputSendSlowStatusMessage(now);
    }

    // Poll trivial processes until end time.
    now = ClockGetUs();
    int64_t end_time = start_time + LOOP_PROCESSING_TIME_US - GUARD_TIME_US;
    for (int32_t cycle = 0; now < end_time; ++cycle) {
      cycle %= ARRAYSIZE(poll_fn);
      poll_fn[cycle]();
      now = ClockGetUs();
    }
  }
  return 0;
}
