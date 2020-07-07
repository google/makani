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

#include "avionics/firmware/cpu/gio.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "common/macros.h"


// GIO pin enumeration must adhere to mathematical relationship between pin
// number and register bits.
COMPILE_ASSERT(kGioPinA0 == 0, kGioPinA0_must_be_0);
COMPILE_ASSERT(kGioPinA7 == 7, kGioPinA7_must_be_7);
COMPILE_ASSERT(kGioPinB0 == 8, kGioPinB0_must_be_8);
COMPILE_ASSERT(kGioPinB7 == 15, kGioPinB7_must_be_15);

static void EnablePinmux(GioPin pin) {
  // See TMS570 TRM "Output Multiplexing and Control".
  static const IommPinmux kPinmux[kNumGioPins] = {
    [kGioPinA2] = {2, 0},
    [kGioPinA3] = {2, 16},
    [kGioPinA5] = {2, 24},
    [kGioPinA6] = {3, 16},
    [kGioPinA7] = {4, 0},
    [kGioPinB2] = {9, 18},
  };
  IommSetPinmuxByIndex(kPinmux, ARRAYSIZE(kPinmux), pin);
}

static void EnableGio(void) {
  PeripheralEnable(kPeripheralGio);
  GIO.GCR0.RESET = 1;
}

void GioInit(void) {
  PeripheralEnable(kPeripheralGio);
  GIO.GCR0.RESET = 0;
  GIO.GCR0.RESET = 1;
}

void GioConfigureAsInputPullDown(GioPin pin) {
  EnableGio();
  GioSetDirectionAsInput(pin);
  GioSetAsPullDown(pin);
  GioEnablePull(pin);
  EnablePinmux(pin);
}

void GioConfigureAsInputPullUp(GioPin pin) {
  EnableGio();
  GioSetDirectionAsInput(pin);
  GioSetAsPullUp(pin);
  GioEnablePull(pin);
  EnablePinmux(pin);
}

void GioConfigureAsInput(GioPin pin) {
  EnableGio();
  GioSetDirectionAsInput(pin);
  GioDisablePull(pin);
  EnablePinmux(pin);
}

void GioConfigureAsOutputPushPull(GioPin pin, bool value) {
  EnableGio();
  GioSetValue(pin, value);
  GioSetAsPushPull(pin);
  GioSetDirectionAsOutput(pin);
  EnablePinmux(pin);
}

void GioConfigureAsOutputOpenDrain(GioPin pin, bool value) {
  EnableGio();
  GioSetValue(pin, value);
  GioSetAsOpenDrain(pin);
  GioSetDirectionAsOutput(pin);
  EnablePinmux(pin);
}

void GioSetInterruptDetect(GioPin pin, GioInterruptDetect detect) {
  uint32_t mask = 1U << pin;
  switch (detect) {
    case kGioInterruptDetectFallingEdge:
      GIO.INTDET.raw &= ~mask;  // Detect a single edge (falling or rising).
      GIO.POL.raw &= ~mask;     // Set to falling edge polarity.
      break;
    case kGioInterruptDetectRisingEdge:
      GIO.INTDET.raw &= ~mask;  // Detect a single edge (falling or rising).
      GIO.POL.raw |= mask;      // Set to rising edge polarity.
      break;
    case kGioInterruptDetectBothEdges:
      GIO.INTDET.raw |= mask;  // Detect both falling and rising edges.
      break;
    default:
      assert(false);
      break;
  }
}

void GioEnableInterrupt(GioPin pin) {
  GIO.ENASET.raw = 1U << pin;
}

void GioDisableInterrupt(GioPin pin) {
  GIO.ENACLR.raw = 1U << pin;
}

void GioSetInterruptPriorityAsHigh(GioPin pin) {
  GIO.LVLSET.raw = 1U << pin;
}

void GioSetInterruptPriorityAsLow(GioPin pin) {
  GIO.LVLCLR.raw = 1U << pin;
}

void GioClearInterruptFlag(GioPin pin) {
  GIO.FLG.raw = 1U << pin;
}

bool GioGetInterruptFlag(GioPin pin) {
  return (GIO.FLG.raw >> pin) & 0x01;
}

void GioSetDirectionAsOutput(GioPin pin) {
  uint32_t mask = 1U << (pin % 8);
  switch (pin / 8) {
    case 0:
      GIO.DIRA.raw |= mask;
      break;
    case 1:
      GIO.DIRB.raw |= mask;
      break;
    default:
      assert(false);
      break;
  }
}

void GioSetDirectionAsInput(GioPin pin) {
  uint32_t mask = 1U << (pin % 8);
  switch (pin / 8) {
    case 0:
      GIO.DIRA.raw &= ~mask;
      break;
    case 1:
      GIO.DIRB.raw &= ~mask;
      break;
    default:
      assert(false);
      break;
  }
}

void GioSetValue(GioPin pin, bool value) {
  if (value) {
    GioSetOutputHigh(pin);
  } else {
    GioSetOutputLow(pin);
  }
}

bool GioGetValue(GioPin pin) {
  uint32_t shift = (pin % 8);
  switch (pin / 8) {
    case 0:
      return (GIO.DINA.raw >> shift) & 0x01;
    case 1:
      return (GIO.DINB.raw >> shift) & 0x01;
    default:
      assert(false);
      return false;
  }
}

void GioSetOutputHigh(GioPin pin) {
  uint32_t mask = 1U << (pin % 8);
  switch (pin / 8) {
    case 0:
      GIO.DSETA.raw = mask;
      break;
    case 1:
      GIO.DSETB.raw = mask;
      break;
    default:
      assert(false);
      break;
  }
}

void GioSetOutputLow(GioPin pin) {
  uint32_t mask = 1U << (pin % 8);
  switch (pin / 8) {
    case 0:
      GIO.DCLRA.raw = mask;
      break;
    case 1:
      GIO.DCLRB.raw = mask;
      break;
    default:
      assert(false);
      break;
  }
}

void GioSetAsPushPull(GioPin pin) {
  uint32_t mask = 1U << (pin % 8);
  switch (pin / 8) {
    case 0:
      GIO.PDRA.raw &= ~mask;
      break;
    case 1:
      GIO.PDRB.raw &= ~mask;
      break;
    default:
      assert(false);
      break;
  }
}

void GioSetAsOpenDrain(GioPin pin) {
  uint32_t mask = 1U << (pin % 8);
  switch (pin / 8) {
    case 0:
      GIO.PDRA.raw |= mask;
      break;
    case 1:
      GIO.PDRB.raw |= mask;
      break;
    default:
      assert(false);
      break;
  }
}

void GioDisablePull(GioPin pin) {
  uint32_t mask = 1U << (pin % 8);
  switch (pin / 8) {
    case 0:
      GIO.PULDISA.raw |= mask;
      break;
    case 1:
      GIO.PULDISB.raw |= mask;
      break;
    default:
      assert(false);
      break;
  }
}

void GioEnablePull(GioPin pin) {
  uint32_t mask = 1U << (pin % 8);
  switch (pin / 8) {
    case 0:
      GIO.PULDISA.raw &= ~mask;
      break;
    case 1:
      GIO.PULDISB.raw &= ~mask;
      break;
    default:
      assert(false);
      break;
  }
}

void GioSetAsPullDown(GioPin pin) {
  uint32_t mask = 1U << (pin % 8);
  switch (pin / 8) {
    case 0:
      GIO.PSLA.raw &= ~mask;
      break;
    case 1:
      GIO.PSLB.raw &= ~mask;
      break;
    default:
      assert(false);
      break;
  }
}

void GioSetAsPullUp(GioPin pin) {
  uint32_t mask = 1U << (pin % 8);
  switch (pin / 8) {
    case 0:
      GIO.PSLA.raw |= mask;
      break;
    case 1:
      GIO.PSLA.raw |= mask;
      break;
    default:
      assert(false);
      break;
  }
}
