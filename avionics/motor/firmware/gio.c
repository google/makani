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

#include "avionics/motor/firmware/gio.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/gio.h"
#include "avionics/firmware/cpu/io.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/serial/motor_serial_params.h"
#include "avionics/motor/firmware/flags.h"
#include "avionics/motor/firmware/isr.h"
#include "avionics/motor/firmware/svpwm.h"

static MotorHardware g_motor_controller_type = -1;

void MotorPositionControlInit(MotorHardware motor_controller_type) {
  if (motor_controller_type == kMotorHardwareOzoneA1) {
    // Set up GPIO to pull nTMS570-POS-CTRL low. This allows the TMS570 to
    // control the position sensor on an Ozone controller.
    IoConfigureAsOutputPushPull(kIoDcan3PinTx, false);
  }
  // On Gin controllers, the TMS570 automatically has control of the position
  // sensor and kIoDcan3PinTx is instead used for power good detection.
}

void MotorGdbPowerGoodInit(MotorHardware motor_controller_type) {
  g_motor_controller_type = motor_controller_type;

  if (motor_controller_type == kMotorHardwareOzoneA1) {
    // Due to IO limitations, the TMS570 on the Ozone only has access to the
    // anded signal from all gate driver boards. However, this signal is
    // latched; set up a GPIO pin as PGOOD latch nreset.
    IoConfigureAsOutputPushPull(kIoDcan3PinRx, true);
    // nDesat pin configuration.
    GioConfigureAsInput(kGioPinA2);
    IoConfigureAsOutputOpenDrain(kIoDcan2PinTx, true);
  } else {
    // With the Gin controllers, individual power good signals from the high and
    // low sides of each gate driver board are fed (more or less) directly into
    // the TMS570. Meanwhile, no latching is present.
    IoConfigureAsInput(kIoDcan2PinTx);
    IoConfigureAsInput(kIoDcan2PinRx);
    IoConfigureAsInput(kIoDcan3PinTx);
    IoConfigureAsInput(kIoDcan3PinRx);
    IoConfigureAsInput(kIoSpi3PinScs4);
    IoConfigureAsInput(kIoSpi3PinMosi0);
  }

  // Set up FIQ interrupt for GDB power loss.
  VimRegisterFiq(kVimChannelGioLowLevel, MotorGdbPowerGoodInterruptHandler);
  VimEnableInterrupt(kVimChannelGioLowLevel);

  // Anded GDB PGOOD signals are connected to GIOA[5].
  GioConfigureAsInput(kGioPinA5);
  GioSetInterruptDetect(kGioPinA5, kGioInterruptDetectFallingEdge);
  GioSetInterruptPriorityAsLow(kGioPinA5);
  GioClearInterruptFlag(kGioPinA5);
  GioEnableInterrupt(kGioPinA5);
}

uint32_t MotorGdbProcessPowerGoodInterrupt(void) {
  GioClearInterruptFlag(kGioPinA5);

  uint32_t warnings = MotorGdbGetPowerGoodStatus();

  // Signal phantom interrupt.
  if (!warnings) {
    warnings = kMotorWarningPowerGoodPhantom;
  }

  return warnings;
}

uint32_t MotorGdbGetPowerGoodStatus(void) {
  uint32_t warnings = kMotorWarningNone;

  assert(g_motor_controller_type >= 0);
  if (g_motor_controller_type != kMotorHardwareOzoneA1) {
    // Poll power good pins.
    if (!IoGetValue(kIoDcan2PinTx))
      warnings |= kMotorWarningPowerGood1Hs;
    if (!IoGetValue(kIoDcan2PinRx))
      warnings |= kMotorWarningPowerGood2Hs;
    if (!IoGetValue(kIoDcan3PinTx))
      warnings |= kMotorWarningPowerGood3Hs;
    if (!IoGetValue(kIoDcan3PinRx))
      warnings |= kMotorWarningPowerGood1Ls;
    if (!IoGetValue(kIoSpi3PinScs4))
      warnings |= kMotorWarningPowerGood2Ls;
    if (!IoGetValue(kIoSpi3PinMosi0))
      warnings |= kMotorWarningPowerGood3Ls;
  }

  // Look for undetected falling edge interrupt via N2HET's HETPINENA.
  if (!SvpwmCheckHetPinEna()) {
    warnings |= kMotorWarningPowerGoodHetPinEna;
  }

  return warnings;
}

void MotorGdbPowerGoodReset(void) {
  assert(g_motor_controller_type >= 0);
  if (g_motor_controller_type == kMotorHardwareOzoneA1) {
    IoSetOutputLow(kIoDcan3PinRx);
    IoSetOutputHigh(kIoDcan3PinRx);
  }
}

uint32_t MotorGdbDesatStatus(void) {
  uint32_t warnings = kMotorWarningNone;

  assert(g_motor_controller_type >= 0);
  if (g_motor_controller_type == kMotorHardwareOzoneA1) {
    // Poll latched nDesat input.
    if (!GioGetValue(kGioPinA2)) {
      warnings |= kMotorWarningDesat;
    }
  }

  return warnings;
}

void MotorGdbDesatReset(void) {
  assert(g_motor_controller_type >= 0);
  if (g_motor_controller_type == kMotorHardwareOzoneA1) {
    IoSetOutputLow(kIoDcan2PinTx);
    IoSetOutputHigh(kIoDcan2PinTx);
  }
}
