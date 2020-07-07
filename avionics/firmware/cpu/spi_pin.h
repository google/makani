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

#ifndef AVIONICS_FIRMWARE_CPU_SPI_PIN_H_
#define AVIONICS_FIRMWARE_CPU_SPI_PIN_H_

#include <stdbool.h>
#include <stdint.h>

#define SPI_ENUM_BUS_SHIFT 8
#define SPI_BUS_PIN_TO_ENUM(BUS, PIN) ((BUS) << SPI_ENUM_BUS_SHIFT | (PIN))
#define SPI_ENUM_TO_BUS(ENUM) ((ENUM) >> SPI_ENUM_BUS_SHIFT)
#define SPI_ENUM_TO_PIN(ENUM) ((ENUM) & ((1U << SPI_ENUM_BUS_SHIFT) - 1))

typedef enum {
  kSpi1PinClk   = SPI_BUS_PIN_TO_ENUM(1, 9),
  kSpi1PinEna   = SPI_BUS_PIN_TO_ENUM(1, 8),
  kSpi1PinMiso0 = SPI_BUS_PIN_TO_ENUM(1, 11),
  kSpi1PinMosi0 = SPI_BUS_PIN_TO_ENUM(1, 10),
  kSpi1PinScs0  = SPI_BUS_PIN_TO_ENUM(1, 0),
  kSpi1PinScs1  = SPI_BUS_PIN_TO_ENUM(1, 1),
  kSpi1PinScs2  = SPI_BUS_PIN_TO_ENUM(1, 2),
  kSpi1PinScs3  = SPI_BUS_PIN_TO_ENUM(1, 3),
  kSpi1PinScs4  = SPI_BUS_PIN_TO_ENUM(1, 4),
  kSpi1PinScs5  = SPI_BUS_PIN_TO_ENUM(1, 5),
  kSpi3PinClk   = SPI_BUS_PIN_TO_ENUM(3, 9),
  kSpi3PinEna   = SPI_BUS_PIN_TO_ENUM(3, 8),
  kSpi3PinMiso0 = SPI_BUS_PIN_TO_ENUM(3, 11),
  kSpi3PinMosi0 = SPI_BUS_PIN_TO_ENUM(3, 10),
  kSpi3PinScs0  = SPI_BUS_PIN_TO_ENUM(3, 0),
  kSpi3PinScs1  = SPI_BUS_PIN_TO_ENUM(3, 1),
  kSpi3PinScs2  = SPI_BUS_PIN_TO_ENUM(3, 2),
  kSpi3PinScs3  = SPI_BUS_PIN_TO_ENUM(3, 3),
  kSpi3PinScs4  = SPI_BUS_PIN_TO_ENUM(3, 4),
  kSpi3PinScs5  = SPI_BUS_PIN_TO_ENUM(3, 5),
  kSpi4PinClk   = SPI_BUS_PIN_TO_ENUM(4, 9),
  kSpi4PinEna   = SPI_BUS_PIN_TO_ENUM(4, 8),
  kSpi4PinMiso0 = SPI_BUS_PIN_TO_ENUM(4, 11),
  kSpi4PinMosi0 = SPI_BUS_PIN_TO_ENUM(4, 10),
  kSpi4PinScs0  = SPI_BUS_PIN_TO_ENUM(4, 0),
  kSpi5PinClk   = SPI_BUS_PIN_TO_ENUM(5, 9),
  kSpi5PinEna   = SPI_BUS_PIN_TO_ENUM(5, 8),
  kSpi5PinMiso0 = SPI_BUS_PIN_TO_ENUM(5, 11),
  kSpi5PinMiso1 = SPI_BUS_PIN_TO_ENUM(5, 25),
  kSpi5PinMiso2 = SPI_BUS_PIN_TO_ENUM(5, 26),
  kSpi5PinMosi0 = SPI_BUS_PIN_TO_ENUM(5, 10),
  kSpi5PinScs0  = SPI_BUS_PIN_TO_ENUM(5, 0),
} SpiPin;

static inline bool SpiBusIsValid(int32_t bus) {
  return bus == 1 || bus == 3 || bus == 4 || bus == 5;
}

// Use these functions to initialize a GIO pin to a known state.
void SpiPinConfigureCsMaskForSpi(int32_t bus, uint8_t cs_mask);
void SpiPinConfigureForSpi(SpiPin pin);
void SpiPinConfigureAsInputPullDown(SpiPin pin);
void SpiPinConfigureAsInputPullUp(SpiPin pin);
void SpiPinConfigureAsInput(SpiPin pin);
void SpiPinConfigureAsOutputPushPull(SpiPin pin, bool value);
void SpiPinConfigureAsOutputOpenDrain(SpiPin pin, bool value);

// The functions below set individual bits of a GIO pin.

// Function bits.
void SpiPinSetToSpiFunction(SpiPin pin);
void SpiPinSetToGioFunction(SpiPin pin);

// Direction bits.
void SpiPinSetDirectionAsOutput(SpiPin pin);
void SpiPinSetDirectionAsInput(SpiPin pin);

// Get/Set value bits.
void SpiPinSetValue(SpiPin pin, bool value);
bool SpiPinGetValue(SpiPin pin);
void SpiPinSetOutputHigh(SpiPin pin);
void SpiPinSetOutputLow(SpiPin pin);

// Output type bits.
void SpiPinSetAsPushPull(SpiPin pin);
void SpiPinSetAsOpenDrain(SpiPin pin);

// Input type bits.
void SpiPinDisablePull(SpiPin pin);
void SpiPinEnablePull(SpiPin pin);
void SpiPinSetAsPullDown(SpiPin pin);
void SpiPinSetAsPullUp(SpiPin pin);

#endif  // AVIONICS_FIRMWARE_CPU_SPI_PIN_H_
