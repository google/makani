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

#include "avionics/cs/firmware/monitor.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/cs/firmware/output.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/drivers/pca9546a.h"
#include "avionics/firmware/monitors/cs.h"
#include "avionics/firmware/monitors/cs_types.h"
#include "avionics/firmware/monitors/ina219.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/firmware/serial/cs_serial_params.h"
#include "common/macros.h"

// The core switch utilizes a PCA9546A I2C bus multiplexer switch. Here, we
// utilize a state machine to select the appropriate I2C bus, then poll that
// bus until completion. This process iterates through all I2C bus, then
// repeats.

typedef enum {
  kMonStateInit,
  kMonStateSelectVoltageBus,  // INA219, MCP9800 hardware monitors.
  kMonStatePollVoltageBusMonitors,
  kMonStateSelectSfpBus,      // FTLF8519P3BYL SFP transceiver.
  kMonStatePollSfpBus,
  kMonStateSelectSfpAuxBus,   // FTLF8519P3BYL SFP transceiver (aux).
  kMonStatePollSfpAuxBus,
  kMonStateIdle
} MonState;

// See PCA9546A wiring configuration on core switch schematics.
typedef enum {
  kI2cBusMaskVoltage = 1 << 0,
  kI2cBusMaskSfp     = 1 << 1,
  kI2cBusMaskSfpAux  = 1 << 2
} I2cBusMask;

static Pca9546a g_i2c_switch;
static MonState g_mon_state = kMonStateInit;

// PCA9546A bus switch configuration.

static void I2cSwitchReset(void) {
  // nRESET requires a pulse duration (t_WL) of 6 ns.
  GIO.DCLRA.raw = GIO_DCLRA_DCLR7;
  __asm__ __volatile__("nop");
  __asm__ __volatile__("nop");
  GIO.DSETA.raw = GIO_DSETA_DSET7;
}

static const Pca9546aConfig kI2cSwitchConfig = {
  .addr = 0x70,
  .reset_func = I2cSwitchReset
};

// State machine.

static void MonStateInit(MonState next) {
  // Initialize PCA9546A nRESET pin on GIO[A7].
  PeripheralEnable(kPeripheralGio);
  IommSetPinmux(4, 0);  // Select GIOA7.

  // GIO[A7] pin configuration.
  GIO.GCR0.RESET = 1;  // Enable GIO module.
  GIO.DIRA.DIR7 = 1;   // Set as output.
  GIO.PDRA.PDR7 = 1;   // Set as open-drain.
  GIO.DSETA.raw = GIO_DSETA_DSET7;

  // Initialize drivers.
  Pca9546aInit(&g_i2c_switch);
  CsMonitorInit();

  // Done.
  g_mon_state = next;
}

static void MonStateSelectBus(MonState next, uint8_t bus_mask) {
  if (Pca9546aPoll(bus_mask, &kI2cSwitchConfig, &g_i2c_switch)) {
    g_mon_state = next;
  }
}

static void MonStatePollVoltageBusMonitors(MonState next) {
  if (CsMonitorPoll(GetBoardHardwareRevision(), CsOutputGetCsMonitors())) {
    g_mon_state = next;
  }
}

// TODO: Implement SFP monitors.
static void MonStatePollSfpBus(MonState next) {
  g_mon_state = next;
}

// Returns return on completion to enable encapsulation by higher level
// state machines.
bool CsMonPoll(void) {
  switch (g_mon_state) {
    case kMonStateInit:
      MonStateInit(kMonStateSelectVoltageBus);
      break;
    case kMonStateSelectVoltageBus:
      MonStateSelectBus(kMonStatePollVoltageBusMonitors, kI2cBusMaskVoltage);
      break;
    case kMonStatePollVoltageBusMonitors:
      MonStatePollVoltageBusMonitors(kMonStateSelectSfpBus);
      break;
    case kMonStateSelectSfpBus:
      MonStateSelectBus(kMonStatePollSfpBus, kI2cBusMaskSfp);
      break;
    case kMonStatePollSfpBus:
      MonStatePollSfpBus(kMonStateSelectSfpAuxBus);
      break;
    case kMonStateSelectSfpAuxBus:
      MonStateSelectBus(kMonStatePollSfpAuxBus, kI2cBusMaskSfpAux);
      break;
    case kMonStatePollSfpAuxBus:
      MonStatePollSfpBus(kMonStateIdle);
      break;
    case kMonStateIdle:
      g_mon_state = kMonStateSelectVoltageBus;
      break;
    default:
      g_mon_state = kMonStateInit;
      assert(false);
      break;
  }
  return g_mon_state == kMonStateIdle;
}
