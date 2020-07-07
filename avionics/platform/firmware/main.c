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
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/cpu/io.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/network/net_diag.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/switch_config.h"
#include "avionics/firmware/output/slow_status.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_stats.h"
#include "avionics/platform/firmware/config_params.h"
#include "avionics/platform/firmware/output.h"
#include "avionics/platform/firmware/selftest.h"
#include "common/macros.h"

int main(void) {
  // Initialize watchdog.
  ExtWatchdogInit();
  // Initialize network.
  MibSPIInit(1, kSpiPinmuxAll);
  SwitchConfigInit();
  Bcm53101Init(true);
  NetInit(AppConfigGetAioNode());
  // Perform self test as soon as possible and immediately after NetInit()
  // such that we validate parameters before calling dependent code.
  SelfTest();
  NetMonInit(GetSwitchConfig()->info);
  NetDiagInit(GetSwitchConfig()->info);
  I2cInit(400000);  // Required for integrity monitoring.
  IoInit();

  // Initialize output.
  PlatformOutputInit();
  OutputInitSlowStatusMessage();

  // Enable interrupts last.
  VimEnableIrq();

  int64_t now = ClockGetUs();
  int64_t wakeup_sensors = now + PLATFORM_SENSORS_PERIOD_US * 0U / 4U;
  int64_t wakeup_monitor = now + PLATFORM_SENSORS_PERIOD_US * 1U / 4U;
  int64_t wakeup_weather = now + PLATFORM_SENSORS_PERIOD_US * 2U / 4U;
  int64_t wakeup_slow_status = now + PLATFORM_SENSORS_PERIOD_US * 3U / 4U;
  while (true) {
    now = ClockGetUs();
    if (wakeup_sensors <= now) {
      wakeup_sensors += PLATFORM_SENSORS_PERIOD_US;
      PlatformOutputSendStatus();
    } else if (wakeup_monitor <= now) {
      wakeup_monitor += PLATFORM_SENSORS_MONITOR_PERIOD_US;
      PlatformOutputSendMonitorStatus();
    } else if (wakeup_weather <= now) {
      wakeup_weather += GROUND_STATION_WEATHER_PERIOD_US;
      PlatformOutputSendWeather(now);
    } else if (wakeup_slow_status <= now) {
      wakeup_slow_status += SLOW_STATUS_PERIOD_US;
      OutputSendSlowStatusMessage(now);
    } else {
      NetPoll();
      NetMonPoll();
      NetDiagPoll();
      Bcm53101Poll(GetSwitchConfig());
      PlatformOutputPoll(now);
      ExtWatchdogPoll();
    }
  }
  return 0;
}
