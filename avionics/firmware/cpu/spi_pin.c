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

#include "avionics/firmware/cpu/spi_pin.h"

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "common/macros.h"

// See Table 4-21 from TMS570 Reference Manual Rev A (TI P/N SPNU515A).
static const struct {
  SpiPin pin;
  int32_t reg;  // PINMMR register number x in PINMMRx[y].
  int32_t bit;  // PINMMR register bit number y in PINMMRx[y].
} kSpiPinmux[] = {
  // kSpi1PinClk does not have a PINMMR register.
  {kSpi1PinEna, 12, 16},
  // kSpi1PinMiso0 does not have a PINMMR register.
  // kSpi1PinMosi0 does not have a PINMMR register.
  {kSpi1PinScs0, 13, 24},
  {kSpi1PinScs1, 20, 16},
  {kSpi1PinScs2, 8, 8},
  {kSpi1PinScs3, 9, 24},
  {kSpi1PinScs4, 8, 17},
  {kSpi1PinScs5, 11, 25},
  {kSpi3PinClk, 33, 24},
  {kSpi3PinMiso0, 33, 8},
  {kSpi3PinMosi0, 33, 16},
  {kSpi3PinScs0, 9, 16},
  {kSpi3PinScs1, 7, 8},
  {kSpi3PinScs2, 0, 24},
  {kSpi3PinScs3, 0, 16},
  {kSpi3PinScs4, 1, 9},
  {kSpi3PinScs5, 9, 9},
  {kSpi4PinClk, 5, 1},
  {kSpi4PinEna, 4, 17},
  {kSpi4PinMiso0, 5, 17},
  {kSpi4PinMosi0, 5, 9},
  {kSpi4PinScs0, 4, 25},
  {kSpi5PinClk, 13, 16},
  {kSpi5PinEna, 12, 24},
  {kSpi5PinMiso0, 13, 0},
  {kSpi5PinMiso1, 12, 28},
  {kSpi5PinMiso2, 13, 12},
  {kSpi5PinMosi0, 13, 8},
  {kSpi5PinScs0, 27, 0},
};

static bool GetBusMask(SpiPin pin, int32_t *bus, int32_t *mask) {
  *bus = SPI_ENUM_TO_BUS(pin);
  *mask = 1U << SPI_ENUM_TO_PIN(pin);
  return SpiBusIsValid(*bus);
}

static void EnablePinmux(SpiPin pin) {
  for (int32_t i = 0; i < ARRAYSIZE(kSpiPinmux); ++i) {
    if (kSpiPinmux[i].pin == pin) {
      IommSetPinmux(kSpiPinmux[i].reg, kSpiPinmux[i].bit);
      break;
    }
  }
}

static void EnableBusForPin(SpiPin pin) {
  int32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SPI(bus).GCR0.NRESET = 1;
  }
}

static void EnablePeripheralForPin(SpiPin pin) {
  const Peripheral kMap[] = {
    [1] = kPeripheralMibSpi1,
    [3] = kPeripheralMibSpi3,
    [4] = kPeripheralSpi4,
    [5] = kPeripheralMibSpi5,
  };
  int32_t bus = SPI_ENUM_TO_BUS(pin);
  if (SpiBusIsValid(bus) && bus < ARRAYSIZE(kMap)) {
    PeripheralEnable(kMap[bus]);
  }
  EnableBusForPin(pin);
}

void SpiPinConfigureCsMaskForSpi(int32_t bus, uint8_t cs_mask) {
  for (int32_t i = 0; i < ARRAYSIZE(kSpiPinmux); ++i) {
    int32_t i_bus, i_mask;
    if (GetBusMask(kSpiPinmux[i].pin, &i_bus, &i_mask) && bus == i_bus
        && (i_mask & cs_mask) != 0) {
      SpiPinConfigureForSpi(kSpiPinmux[i].pin);
    }
  }
}

void SpiPinConfigureForSpi(SpiPin pin) {
  EnablePeripheralForPin(pin);
  SpiPinSetToSpiFunction(pin);
  EnablePinmux(pin);
}

void SpiPinConfigureAsInputPullDown(SpiPin pin) {
  EnablePeripheralForPin(pin);
  SpiPinSetToGioFunction(pin);
  SpiPinSetDirectionAsInput(pin);
  SpiPinSetAsPullDown(pin);
  SpiPinEnablePull(pin);
  EnablePinmux(pin);
}

void SpiPinConfigureAsInputPullUp(SpiPin pin) {
  EnablePeripheralForPin(pin);
  SpiPinSetToGioFunction(pin);
  SpiPinSetDirectionAsInput(pin);
  SpiPinSetAsPullUp(pin);
  SpiPinEnablePull(pin);
  EnablePinmux(pin);
}

void SpiPinConfigureAsInput(SpiPin pin) {
  EnablePeripheralForPin(pin);
  SpiPinSetToGioFunction(pin);
  SpiPinSetDirectionAsInput(pin);
  SpiPinDisablePull(pin);
  EnablePinmux(pin);
}

void SpiPinConfigureAsOutputPushPull(SpiPin pin, bool value) {
  EnablePeripheralForPin(pin);
  SpiPinSetToGioFunction(pin);
  SpiPinSetValue(pin, value);
  SpiPinSetAsPushPull(pin);
  SpiPinSetDirectionAsOutput(pin);
  EnablePinmux(pin);
}

void SpiPinConfigureAsOutputOpenDrain(SpiPin pin, bool value) {
  EnablePeripheralForPin(pin);
  SpiPinSetToGioFunction(pin);
  SpiPinSetValue(pin, value);
  SpiPinSetAsOpenDrain(pin);
  SpiPinSetDirectionAsOutput(pin);
  EnablePinmux(pin);
}

void SpiPinSetToSpiFunction(SpiPin pin) {
  int32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SPI(bus).PC0.raw |= mask;
  }
}

void SpiPinSetToGioFunction(SpiPin pin) {
  int32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SPI(bus).PC0.raw &= ~mask;
  }
}

void SpiPinSetDirectionAsOutput(SpiPin pin) {
  int32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SPI(bus).PC1.raw |= mask;
  }
}

void SpiPinSetDirectionAsInput(SpiPin pin) {
  int32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SPI(bus).PC1.raw &= ~mask;
  }
}

void SpiPinSetValue(SpiPin pin, bool value) {
  if (value) {
    SpiPinSetOutputHigh(pin);
  } else {
    SpiPinSetOutputLow(pin);
  }
}

bool SpiPinGetValue(SpiPin pin) {
  int32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    return (SPI(bus).PC2.raw & mask) != 0;
  }
  return false;
}

void SpiPinSetOutputHigh(SpiPin pin) {
  int32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SPI(bus).PC4.raw = mask;
  }
}

void SpiPinSetOutputLow(SpiPin pin) {
  int32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SPI(bus).PC5.raw = mask;
  }
}

void SpiPinSetAsPushPull(SpiPin pin) {
  int32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SPI(bus).PC6.raw &= ~mask;
  }
}

void SpiPinSetAsOpenDrain(SpiPin pin) {
  int32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SPI(bus).PC6.raw |= mask;
  }
}

void SpiPinDisablePull(SpiPin pin) {
  int32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SPI(bus).PC7.raw |= mask;
  }
}

void SpiPinEnablePull(SpiPin pin) {
  int32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SPI(bus).PC7.raw &= ~mask;
  }
}

void SpiPinSetAsPullDown(SpiPin pin) {
  int32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SPI(bus).PC8.raw &= ~mask;
  }
}

void SpiPinSetAsPullUp(SpiPin pin) {
  int32_t bus, mask;
  if (GetBusMask(pin, &bus, &mask)) {
    SPI(bus).PC8.raw |= mask;
  }
}
