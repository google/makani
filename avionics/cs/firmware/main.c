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

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/network_addresses.h"
#include "avionics/common/network_config.h"
#include "avionics/cs/firmware/config_params.h"
#include "avionics/cs/firmware/eop_interface.h"
#include "avionics/cs/firmware/monitor.h"
#include "avionics/cs/firmware/output.h"
#include "avionics/cs/firmware/selftest.h"
#include "avionics/cs/firmware/tether_comms.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/cpu/io.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/drivers/bcm53284.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/drivers/microhard.h"
#include "avionics/firmware/drivers/pca9546a.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/network/net_diag.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/switch_config.h"
#include "avionics/firmware/output/slow_status.h"
#include "avionics/network/aio_latency.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_stats.h"
#include "avionics/network/routes.h"
#include "avionics/plc/firmware/winch_interface.h"
#include "common/macros.h"

#define STATUS_PERIOD_CYCLES                                    \
  CLOCK32_MSEC_TO_CYCLES(CORE_SWITCH_STATUS_PERIOD_US / 1000)

static MicrohardData g_microhard_data;
static const Microhard kMicrohard = {
  .data = &g_microhard_data,
  .sci = &kSci1Interrupt,
  .baud_rate = 115200,
  .set_status = CsOutputMicrohardStatus,
};

static StaticAddress g_static_addresses[
    ARRAYSIZE(kCsConfigParams->static_addresses)];

static void PopulateStaticAddresses(void) {
  SwitchOptions *switch_options = SwitchConfigGetSwitchOptions();
  switch_options->num_static_addresses = ARRAYSIZE(
      kCsConfigParams->static_addresses);
  for (int i = 0; i < ARRAYSIZE(g_static_addresses); i++) {
    g_static_addresses[i].port = kCsConfigParams->static_addresses[i].port;
    g_static_addresses[i].mac.a = kCsConfigParams->static_addresses[i].mac[0];
    g_static_addresses[i].mac.b = kCsConfigParams->static_addresses[i].mac[1];
    g_static_addresses[i].mac.c = kCsConfigParams->static_addresses[i].mac[2];
    g_static_addresses[i].mac.d = kCsConfigParams->static_addresses[i].mac[3];
    g_static_addresses[i].mac.e = kCsConfigParams->static_addresses[i].mac[4];
    g_static_addresses[i].mac.f = kCsConfigParams->static_addresses[i].mac[5];
  }
  switch_options->static_addresses = g_static_addresses;
}

static void SetDisablePortMask(uint32_t disable_port_mask) {
  SwitchConfigGetSwitchOptions()->port_disable_mask_current = disable_port_mask;
  Bcm53284ReconfigureOptions();
  CsOutputDisabledPortMask(disable_port_mask);
}

static void HandleSwitchConnectionCommand(AioNode node) {
  CoreSwitchConnectionSelectMessage message;
  if (CvtGetCoreSwitchConnectionSelectMessage(kAioNodeOperator, &message, NULL,
                                              NULL)) {
    if (node == message.target) {
      SetDisablePortMask(message.disable_port_mask);
    }
  }
}

static void PollNetwork(AioNode me) {
  IpAddress source_ip;
  uint16_t source_port, dest_port;
  static uint8_t data[MAX_UDP_PAYLOAD_SIZE];
  int32_t length = NetPollUdp(ARRAYSIZE(data), data, &source_ip, NULL,
                              &source_port, &dest_port);
  bool handled = length <= 0;

  if (!handled && (me == kAioNodeCsGsA || me == kAioNodeCsGsB)) {
    WinchPollCvt();
    handled = HandleWinchMessage(dest_port, length, data);
  }
  if (!handled && (me == kAioNodeCsA || kAioNodeCsGsA)) {
    handled = HandleEopMessage(&source_ip, source_port, dest_port, length,
                               data);
  }
}

int main(void) {
  AioNode node = AppConfigGetAioNode();
  MibSPIInit(1, kSpiPinmuxAll);
  ExtWatchdogInit();
  SwitchConfigInit();
  PopulateStaticAddresses();
  Bcm53284Init(true);
  NetInit(node);
  // Perform self test as soon as possible and immediately after NetInit()
  // such that we validate parameters before calling dependent code.
  SelfTest();
  NetMonInit(GetSwitchConfig()->info);
  NetDiagInit(GetSwitchConfig()->info);
  I2cInit(400e3);
  IoInit();
  CsOutputInit();
  OutputInitCoreSwitchSlowStatusMessage();
  TetherCommsInit(kCsConfigParams->xlr_network_id);
  MicrohardInit(&kMicrohard, &kCsConfigParams->microhard_params);

  // Set default state for Microhard nCONFIG and nRESET.
  IoConfigureAsOutputOpenDrain(kIoN2het1Pin22, 1);  // nCONFIG.
  IoConfigureAsOutputOpenDrain(kIoGioPinA5, 1);     // nRESET.

  // Enable/disable radio power.
  IoConfigureAsOutputOpenDrain(kIoN2het1Pin20, kCsConfigParams->enable_radio);

  // Enable interupts for SCI driver.
  VimEnableIrq();

  uint32_t now = Clock32GetCycles();
  uint32_t status_wakeup = now + STATUS_PERIOD_CYCLES;
  uint32_t slow_status_wakeup = now + SLOW_STATUS_PERIOD_CYCLES;
  while (true) {
    if (PollPeriodicCycles(STATUS_PERIOD_CYCLES, &status_wakeup)) {
      CsOutputSendStatusMessage();
    } else if (PollPeriodicCycles(SLOW_STATUS_PERIOD_CYCLES,
                                  &slow_status_wakeup)) {
      OutputSendCoreSwitchSlowStatusMessage(ClockGetUs());
    } else {
      PollNetwork(node);
      HandleSwitchConnectionCommand(node);
      CsMonPoll();
      NetMonPoll();
      NetDiagPoll();
      Bcm53284Poll(GetSwitchConfig());
      ExtWatchdogPoll();
      if (kCsConfigParams->microhard_params.enabled) {
        MicrohardPoll(&kMicrohard);
      }
      TetherCommsPoll(node, CsOutputGetCsMonitors(),
                      &OutputGetCoreSwitchSlowStatusMessage()->switch_stats,
                      CsOutputGetMicrohardStatus()->rssi);
      AioLatencyPoll();
    }
  }
  return 0;
}
