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

#include <string.h>

#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/monitors/aio.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/network/net_diag.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/firmware/network/switch_config.h"
#include "avionics/firmware/output/slow_status.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/network/message_stats.h"
#include "avionics/plc/firmware/config_params.h"
#include "avionics/plc/firmware/plc_interface.h"
#include "avionics/plc/firmware/selftest.h"
#include "avionics/plc/firmware/winch_interface.h"
#include "common/macros.h"

#define MON_STATUS_PERIOD_US GROUND_STATION_PLC_MONITOR_STATUS_PERIOD_US
static GroundStationPlcMonitorStatusMessage g_mon_status;

static void PollNetwork(int64_t now) {
  uint16_t dest_port;
  static uint8_t data[MAX_UDP_PAYLOAD_SIZE];
  int32_t length = NetPollUdp(ARRAYSIZE(data), data, NULL, NULL, NULL,
                              &dest_port);
  if (kPlcConfigParams->plc_type == kPlcTypeDetwist
      || kPlcConfigParams->plc_type == kPlcTypeGs02) {
    PlcPollInput(now);
    HandlePlcMessage(dest_port, length, data, now);
  } else if (kPlcConfigParams->plc_type == kPlcTypeWinch) {
    // TODO: Remove winch (GS01) interface.
    WinchPollCvt();
    HandleWinchMessage(dest_port, length, data);
  }
}

int main(void) {
  ExtWatchdogInit();
  MibSPIInit(1, kSpiPinmuxAll);
  SwitchConfigInit();
  Bcm53101Init(true);

  NetInit(AppConfigGetAioNode());
  // Perform self test as soon as possible and immediately after NetInit()
  // such that we validate parameters before calling dependent code.
  SelfTest();

  NetMonInit(GetSwitchConfig()->info);
  NetDiagInit(GetSwitchConfig()->info);
  I2cInit(400e3);
  AioMonitorInit();

  // Initialize outputs.
  OutputInitSlowStatusMessage();
  int64_t now = ClockGetUs();
  PlcInterfaceInit(now + MON_STATUS_PERIOD_US / 2U);

  int64_t wakeup_mon_status = now + MON_STATUS_PERIOD_US / 4U;
  int64_t wakeup_slow_status =
      now + SLOW_STATUS_PERIOD_US - MON_STATUS_PERIOD_US * 3U / 4U;
  while (true) {
    now = ClockGetUs();
    if (now >= wakeup_mon_status) {
      wakeup_mon_status += MON_STATUS_PERIOD_US;
      NetSendAioGroundStationPlcMonitorStatusMessage(&g_mon_status);
    } else if (now >= wakeup_slow_status) {
      wakeup_slow_status += SLOW_STATUS_PERIOD_US;
      OutputSendSlowStatusMessage(now);
    } else {
      PollNetwork(now);
      NetMonPoll();
      NetDiagPoll();
      Bcm53101Poll(GetSwitchConfig());
      ExtWatchdogPoll();
      AioMonitorPoll(GetBoardHardwareRevision(), &g_mon_status.aio_mon);
    }
  }

  return 0;
}
