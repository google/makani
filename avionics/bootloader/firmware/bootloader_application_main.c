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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/bootloader/firmware/update_server.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/drivers/bcm_unified.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/output/slow_status.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/route_config.h"
#include "avionics/network/switch_config.h"
#include "avionics/network/switch_info.h"

int main(void) {
  ExtWatchdogInit();
  MibSPIInit(1, kSpiPinmuxAll);
  BcmUnifiedInit(true);
  // TODO: Remove this delay once the bootloader asynchronously waits
  // for initialization.
  while (!BcmUnifiedReady()) {
    ExtWatchdogPoll();
    BcmUnifiedPoll(NULL);
  }
  NetInit(AppConfigGetAioNode());

  // For now, simply send one BootloaderSlowStatus message.
  // TODO: Remove this once bootloader asynchronously waits for init.
  OutputInitBootloaderSlowStatusMessage();
  OutputSendBootloaderSlowStatusMessage();

  // Identity functions reference the bootloader's flash memory. Read and
  // store these values now before erasing the bootloader's flash memory.
  HardwareType hardware_type = BootConfigGetHardwareType();

  // Run bootloader.
  while (!UpdateServer(hardware_type)) {
    ExtWatchdogPoll();
  }

  printf("About to reset.\n");
  fflush(stdout);

  // Flush that last printf out, hopefully.
  NetPoll();
  NetPoll();
  NetPoll();

  SYS.ECR.RESET = 2;  // Software reset.
  return 0;
}
