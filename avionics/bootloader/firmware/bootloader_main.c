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
#include "avionics/common/crc.h"
#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/drivers/bcm_unified.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/output/slow_status.h"
#include "avionics/firmware/startup/flash_tms570.h"
#include "avionics/firmware/startup/ldscript.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/route_config.h"
#include "avionics/network/switch_config.h"
#include "avionics/network/switch_info.h"

static bool ValidateApplication(void) {
  // Disable flash ECC protection while reading possibly erased or corrupted
  // flash to avoid ESM errors.
  StartupFlashDisableEcc();

  // Calculate CRC in blocks of 1024 bytes to allow external watchdog kick.
  const uint8_t *position = ldscript_app_flash_begin;
  uint32_t crc = 0;
  while (position < ldscript_app_flash_crc) {
    uint32_t block_len = ldscript_app_flash_crc - position;
    if (block_len > 1024) {
      block_len = 1024;
    }
    crc = Crc32(crc, block_len, position);
    position += block_len;
    ExtWatchdogPoll();
  }

  uint32_t stored_crc;
  ReadUint32Be(ldscript_app_flash_crc, &stored_crc);

  // Re-enable ECC protection after completion.
  StartupFlashEnableEcc();

  return crc == stored_crc;
}

static void ShutdownDma(void) {
  DMA.GCTRL.DMAEN = 0;  // 0 = Disable.
}

static void ShutdownAdc(void) {
  ADC(1).RSTCR.RESET = 1;  // 1 = Reset.
  ADC(2).RSTCR.RESET = 1;  // 1 = Reset.
}

static void ShutdownGio(void) {
  GIO.GCR0.RESET = 0;  // 0 = Reset.
}

static void ShutdownMibSpi(void) {
  SPI(1).GCR0.NRESET = 0;  // 0 = Reset.
  SPI(3).GCR0.NRESET = 0;  // 0 = Reset.
}

static void ShutdownEmac(void) {
  EMAC.TXCONTROL.TXEN = 0;  // 0 = Disable.
  EMAC.RXCONTROL.RXEN = 0;  // 0 = Disable.
  EMAC.SOFTRESET.SOFTRESET = 1;  // 1 = Reset.
}

static void ShutdownPeripherals(void) {
  ShutdownVim();
  ShutdownDma();
  ShutdownAdc();
  ShutdownGio();
  ShutdownMibSpi();
  ShutdownEmac();
}

static bool BootApplication(void) {
  if (ValidateApplication()) {
    // Kick watchdog before starting application.
    while (!ExtWatchdogPoll()) {}
    ShutdownPeripherals();
    __asm__ __volatile__("bx %0" : : "r"(ldscript_app_flash_begin));
    assert(!(bool)"Branch should never return!");
  }
  return false;
}

// Expected behavior:
// - On power-on, the boot loader should listen for client communications
//   for a short time period. If it does not receive boot loader instructions
//   from a client, it should then attempt to load the application. If the
//   application fails to load, the boot loader should listen indefinitely.
// - On software reset, the boot loader should function exactly the same
//   as on power-on.
// - On processor or watchdog reset, the boot loader load the application
//   immediately.
int main(void) {
  ExtWatchdogInit();

  // Attempt to boot application immediately following processor or watchdog
  // resets.
  bool boot_failure = false;
  if (!SYS.ESR.PORST && !SYS.ESR.SWRST) {
    boot_failure = !BootApplication();
  }
  SYS.ESR.PORST = 1;
  SYS.ESR.SWRST = 1;

  // Load network stack.
  MibSPIInit(1, kSpiPinmuxAll);
  BcmUnifiedInit(true);
  // TODO: Remove this delay once the bootloader asynchronously waits
  // for initialization.
  while (!BcmUnifiedReady()) {
    ExtWatchdogPoll();
    BcmUnifiedPoll(NULL);
  }
  // Bootloader should not require knowledge of the AioNode. This parameter
  // sets the AioNode in the AioHeader. We only use this information for
  // printf traffic.
  NetInit(kAioNodeUnknown);

  // For now, simply send one BootloaderSlowStatus message.
  // TODO: Remove this once bootloader asynchronously waits for init.
  OutputInitBootloaderSlowStatusMessage();
  OutputSendBootloaderSlowStatusMessage();

  // Listen.
  while (true) {
    ExtWatchdogPoll();
    if (UpdateServer(BootConfigGetHardwareType())
        || !boot_failure) {
      boot_failure = !BootApplication();
    }
  }
  return 0;
}
