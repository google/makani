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

#include "avionics/firmware/cpu/sci_pin.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "common/macros.h"

// See Table 4-21 from TMS570 Reference Manual Rev A (TI P/N SPNU515A).
static const struct {
  SciPin pin;
  int32_t reg;  // PINMMR register number x in PINMMRx[y].
  int32_t bit;  // PINMMR register bit number y in PINMMRx[y].
} kSciPinmux[] = {
  {kSci2PinRx, 7, 17},
  {kSci2PinTx, 8, 1},
};

static void EnablePinmux(SciPin pin) {
  for (int32_t i = 0; i < ARRAYSIZE(kSciPinmux); ++i) {
    if (kSciPinmux[i].pin == pin) {
      IommSetPinmux(kSciPinmux[i].reg, kSciPinmux[i].bit);
      break;
    }
  }
}

static bool GetBusMask(SciPin pin, uint32_t *bus, uint32_t *mask) {
  *bus = (pin / 2) + 1;  // Sci1 or Sci2.
  *mask = 1U << ((pin % 2) + 1);  // Tx or Rx.
  return (pin < kNumSciPins);
}

static void EnableBusForPin(SciPin pin) {
  uint32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SCI(bus).GCR0.RESET = 1;
  }
}

static void EnablePeripheralForPin(SciPin pin) {
  if (pin == kSci1PinRx || pin == kSci1PinTx) {  // Sci1.
    PeripheralEnable(kPeripheralSci1);
  } else if (pin == kSci2PinRx || pin == kSci2PinTx) {  // Sci2.
    PeripheralEnable(kPeripheralSci2);
    // SciInit in sci.c has more actions, like reset and soft reset.
  } else {
    assert(false);
  }
  EnableBusForPin(pin);
}

static void SciPinSetToGioFunction(SciPin pin) {
  uint32_t bus;
  uint32_t mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SCI(bus).PIO0.raw &= ~mask;
  }
}

void SciPinConfigureAsInputPullDown(SciPin pin) {
  EnablePeripheralForPin(pin);
  SciPinSetDirectionAsInput(pin);
  SciPinSetAsPullDown(pin);
  SciPinEnablePull(pin);
  SciPinSetToGioFunction(pin);
  EnablePinmux(pin);
}

void SciPinConfigureAsInputPullUp(SciPin pin) {
  EnablePeripheralForPin(pin);
  SciPinSetDirectionAsInput(pin);
  SciPinSetAsPullUp(pin);
  SciPinEnablePull(pin);
  SciPinSetToGioFunction(pin);
  EnablePinmux(pin);
}

void SciPinConfigureAsInput(SciPin pin) {
  EnablePeripheralForPin(pin);
  SciPinSetDirectionAsInput(pin);
  SciPinDisablePull(pin);
  SciPinSetToGioFunction(pin);
  EnablePinmux(pin);
}

void SciPinConfigureAsOutputPushPull(SciPin pin, bool value) {
  EnablePeripheralForPin(pin);
  SciPinSetValue(pin, value);
  SciPinSetAsPushPull(pin);
  SciPinSetDirectionAsOutput(pin);
  SciPinSetToGioFunction(pin);
  EnablePinmux(pin);
}

void SciPinConfigureAsOutputOpenDrain(SciPin pin, bool value) {
  EnablePeripheralForPin(pin);
  SciPinSetValue(pin, value);
  SciPinSetAsOpenDrain(pin);
  SciPinSetDirectionAsOutput(pin);
  SciPinSetToGioFunction(pin);
  EnablePinmux(pin);
}

void SciPinSetDirectionAsOutput(SciPin pin) {
  uint32_t bus;
  uint32_t mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SCI(bus).PIO1.raw |= mask;
  }
}

void SciPinSetDirectionAsInput(SciPin pin) {
  uint32_t bus;
  uint32_t mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SCI(bus).PIO1.raw &= ~mask;
  }
}

void SciPinSetValue(SciPin pin, bool value) {
  if (value) {
    SciPinSetOutputHigh(pin);
  } else {
    SciPinSetOutputLow(pin);
  }
}

bool SciPinGetValue(SciPin pin) {
  if (pin % 2 == 0) {  // Rx.
    return SCI((pin / 2) + 1).PIO2.RXIN;  // Pin / 2 + 1 gets Sci1 or Sci2.
  } else {  // Tx.
    return SCI((pin / 2) + 1).PIO2.TXIN;
  }
}

void SciPinSetOutputHigh(SciPin pin) {
  uint32_t bus;
  uint32_t mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SCI(bus).PIO4.raw = mask;  // PIO4 is TX SET / RX SET.
  }
}

void SciPinSetOutputLow(SciPin pin) {
  uint32_t bus;
  uint32_t mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SCI(bus).PIO5.raw = mask;  // PIO5 is TX CLR / RX CLR.
  }
}

void SciPinSetAsPushPull(SciPin pin) {
  uint32_t bus;
  uint32_t mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SCI(bus).PIO6.raw &= ~mask;
  }
}

void SciPinSetAsOpenDrain(SciPin pin) {
  uint32_t bus;
  uint32_t mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SCI(bus).PIO6.raw |= mask;
  }
}

void SciPinDisablePull(SciPin pin) {
  uint32_t bus;
  uint32_t mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SCI(bus).PIO7.raw |= mask;
  }
}

void SciPinEnablePull(SciPin pin) {
  uint32_t bus;
  uint32_t mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SCI(bus).PIO7.raw &= ~mask;
  }
}

void SciPinSetAsPullDown(SciPin pin) {
  uint32_t bus;
  uint32_t mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SCI(bus).PIO8.raw &= ~mask;
  }
}

void SciPinSetAsPullUp(SciPin pin) {
  uint32_t bus;
  uint32_t mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SCI(bus).PIO8.raw |= mask;
  }
}
