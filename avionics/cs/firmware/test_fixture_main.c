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

#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/drivers/bcm53284.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/network/switch_config.h"
#include "avionics/network/switch_types.h"

#define DIAG_PORT 20
#define HOST_PORT 24
#define MASK_ALL ((1 << NUM_SWITCH_PORTS_BCM53284) - 1)

static const SwitchInfo kSwitchInfo = {
  .type = kSwitchTypeBcm53284,
  .num_ports = NUM_SWITCH_PORTS_BCM53284,
  .num_fiber_ports = NUM_SWITCH_FIBER_PORTS_BCM53284,
  .host_port = HOST_PORT,
  .mirror_port = -1,
  .forward_mask_a = MASK_ALL,
  .forward_mask_b = 0,
  .isolate_mask = MASK_ALL ^ (1 << HOST_PORT | 1 << DIAG_PORT),
  .unicast_mask = MASK_ALL,
  .segment_vlans = NULL,
  .trunk = {0, 0, 0, 0, NULL}
};

static SwitchOptions g_switch_options;

int main(void) {
  ExtWatchdogInit();
  MibSPIInit(1, kSpiPinmuxAll);
  Bcm53284Init(true);
  NetInit(AppConfigGetAioNode());

  GetDefaultSwitchOptions(&kSwitchInfo, &g_switch_options);
  SwitchConfig switch_config = {&kSwitchInfo, &g_switch_options, NULL};
  while (true) {
    ExtWatchdogPoll();
    Bcm53284Poll(&switch_config);
    NetPoll();
  }
  return 0;
}
