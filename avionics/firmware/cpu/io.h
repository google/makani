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

#ifndef AVIONICS_FIRMWARE_CPU_IO_H_
#define AVIONICS_FIRMWARE_CPU_IO_H_

#include <stdbool.h>

#include "avionics/firmware/cpu/dcan_pin.h"
#include "avionics/firmware/cpu/gio.h"
#include "avionics/firmware/cpu/n2het.h"
#include "avionics/firmware/cpu/sci_pin.h"
#include "avionics/firmware/cpu/spi_pin.h"

typedef enum {
  kIoInterfaceForceSigned = -1,
  kIoInterfaceUnmapped,
  kIoInterfaceDcan,
  kIoInterfaceGio,
  kIoInterfaceN2het,
  kIoInterfaceSci,
  kIoInterfaceSpi,
  kNumIoInterfaces
} IoInterface;

#define IO_ENUM_TYPE_SHIFT 24
#define IO_TYPE_TO_ENUM(TYPE, PIN) ((TYPE) << IO_ENUM_TYPE_SHIFT | (PIN))
#define IO_ENUM_TO_TYPE(ENUM) ((ENUM) >> IO_ENUM_TYPE_SHIFT)
#define IO_ENUM_TO_PIN(ENUM) ((ENUM) & ((1U << IO_ENUM_TYPE_SHIFT) - 1))

#define DCAN_TO_IO_PIN(PIN) IO_TYPE_TO_ENUM(kIoInterfaceDcan, (PIN))
#define GIO_TO_IO_PIN(PIN) IO_TYPE_TO_ENUM(kIoInterfaceGio, (PIN))
#define N2HET_TO_IO_PIN(PIN) IO_TYPE_TO_ENUM(kIoInterfaceN2het, (PIN))
#define SCI_TO_IO_PIN(PIN) IO_TYPE_TO_ENUM(kIoInterfaceSci, (PIN))
#define SPI_TO_IO_PIN(PIN) IO_TYPE_TO_ENUM(kIoInterfaceSpi, (PIN))

typedef enum {
  // Unmapped pin.
  kIoUnmappedPin = 0,  // Set to zero to match designated initializer default.
  // DCAN pins.
  kIoDcan1PinRx = DCAN_TO_IO_PIN(kDcan1PinRx),
  kIoDcan1PinTx = DCAN_TO_IO_PIN(kDcan1PinTx),
  kIoDcan2PinRx = DCAN_TO_IO_PIN(kDcan2PinRx),
  kIoDcan2PinTx = DCAN_TO_IO_PIN(kDcan2PinTx),
  kIoDcan3PinRx = DCAN_TO_IO_PIN(kDcan3PinRx),
  kIoDcan3PinTx = DCAN_TO_IO_PIN(kDcan3PinTx),
  // GIO pins.
  kIoGioPinA0 = GIO_TO_IO_PIN(kGioPinA0),
  kIoGioPinA1 = GIO_TO_IO_PIN(kGioPinA1),
  kIoGioPinA2 = GIO_TO_IO_PIN(kGioPinA2),
  kIoGioPinA3 = GIO_TO_IO_PIN(kGioPinA3),
  kIoGioPinA4 = GIO_TO_IO_PIN(kGioPinA4),
  kIoGioPinA5 = GIO_TO_IO_PIN(kGioPinA5),
  kIoGioPinA6 = GIO_TO_IO_PIN(kGioPinA6),
  kIoGioPinA7 = GIO_TO_IO_PIN(kGioPinA7),
  kIoGioPinB0 = GIO_TO_IO_PIN(kGioPinB0),
  kIoGioPinB1 = GIO_TO_IO_PIN(kGioPinB1),
  kIoGioPinB2 = GIO_TO_IO_PIN(kGioPinB2),
  kIoGioPinB3 = GIO_TO_IO_PIN(kGioPinB3),
  kIoGioPinB4 = GIO_TO_IO_PIN(kGioPinB4),
  kIoGioPinB5 = GIO_TO_IO_PIN(kGioPinB5),
  kIoGioPinB6 = GIO_TO_IO_PIN(kGioPinB6),
  kIoGioPinB7 = GIO_TO_IO_PIN(kGioPinB7),
  // N2HET1 pins.
  kIoN2het1Pin0  = N2HET_TO_IO_PIN(kN2het1Pin0),
  kIoN2het1Pin1  = N2HET_TO_IO_PIN(kN2het1Pin1),
  kIoN2het1Pin2  = N2HET_TO_IO_PIN(kN2het1Pin2),
  kIoN2het1Pin3  = N2HET_TO_IO_PIN(kN2het1Pin3),
  kIoN2het1Pin4  = N2HET_TO_IO_PIN(kN2het1Pin4),
  kIoN2het1Pin5  = N2HET_TO_IO_PIN(kN2het1Pin5),
  kIoN2het1Pin6  = N2HET_TO_IO_PIN(kN2het1Pin6),
  kIoN2het1Pin7  = N2HET_TO_IO_PIN(kN2het1Pin7),
  kIoN2het1Pin8  = N2HET_TO_IO_PIN(kN2het1Pin8),
  kIoN2het1Pin9  = N2HET_TO_IO_PIN(kN2het1Pin9),
  kIoN2het1Pin10 = N2HET_TO_IO_PIN(kN2het1Pin10),
  kIoN2het1Pin11 = N2HET_TO_IO_PIN(kN2het1Pin11),
  kIoN2het1Pin12 = N2HET_TO_IO_PIN(kN2het1Pin12),
  kIoN2het1Pin13 = N2HET_TO_IO_PIN(kN2het1Pin13),
  kIoN2het1Pin14 = N2HET_TO_IO_PIN(kN2het1Pin14),
  kIoN2het1Pin15 = N2HET_TO_IO_PIN(kN2het1Pin15),
  kIoN2het1Pin16 = N2HET_TO_IO_PIN(kN2het1Pin16),
  kIoN2het1Pin17 = N2HET_TO_IO_PIN(kN2het1Pin17),
  kIoN2het1Pin18 = N2HET_TO_IO_PIN(kN2het1Pin18),
  kIoN2het1Pin19 = N2HET_TO_IO_PIN(kN2het1Pin19),
  kIoN2het1Pin20 = N2HET_TO_IO_PIN(kN2het1Pin20),
  kIoN2het1Pin21 = N2HET_TO_IO_PIN(kN2het1Pin21),
  kIoN2het1Pin22 = N2HET_TO_IO_PIN(kN2het1Pin22),
  kIoN2het1Pin23 = N2HET_TO_IO_PIN(kN2het1Pin23),
  kIoN2het1Pin24 = N2HET_TO_IO_PIN(kN2het1Pin24),
  kIoN2het1Pin25 = N2HET_TO_IO_PIN(kN2het1Pin25),
  kIoN2het1Pin26 = N2HET_TO_IO_PIN(kN2het1Pin26),
  kIoN2het1Pin27 = N2HET_TO_IO_PIN(kN2het1Pin27),
  kIoN2het1Pin28 = N2HET_TO_IO_PIN(kN2het1Pin28),
  kIoN2het1Pin29 = N2HET_TO_IO_PIN(kN2het1Pin29),
  kIoN2het1Pin30 = N2HET_TO_IO_PIN(kN2het1Pin30),
  kIoN2het1Pin31 = N2HET_TO_IO_PIN(kN2het1Pin31),
  // N2HET2 pins.
  kIoN2het2Pin0  = N2HET_TO_IO_PIN(kN2het2Pin0),
  kIoN2het2Pin1  = N2HET_TO_IO_PIN(kN2het2Pin1),
  kIoN2het2Pin2  = N2HET_TO_IO_PIN(kN2het2Pin2),
  kIoN2het2Pin3  = N2HET_TO_IO_PIN(kN2het2Pin3),
  kIoN2het2Pin4  = N2HET_TO_IO_PIN(kN2het2Pin4),
  kIoN2het2Pin5  = N2HET_TO_IO_PIN(kN2het2Pin5),
  kIoN2het2Pin6  = N2HET_TO_IO_PIN(kN2het2Pin6),
  kIoN2het2Pin7  = N2HET_TO_IO_PIN(kN2het2Pin7),
  kIoN2het2Pin8  = N2HET_TO_IO_PIN(kN2het2Pin8),
  kIoN2het2Pin9  = N2HET_TO_IO_PIN(kN2het2Pin9),
  kIoN2het2Pin10 = N2HET_TO_IO_PIN(kN2het2Pin10),
  kIoN2het2Pin11 = N2HET_TO_IO_PIN(kN2het2Pin11),
  kIoN2het2Pin12 = N2HET_TO_IO_PIN(kN2het2Pin12),
  kIoN2het2Pin13 = N2HET_TO_IO_PIN(kN2het2Pin13),
  kIoN2het2Pin14 = N2HET_TO_IO_PIN(kN2het2Pin14),
  kIoN2het2Pin15 = N2HET_TO_IO_PIN(kN2het2Pin15),
  kIoN2het2Pin16 = N2HET_TO_IO_PIN(kN2het2Pin16),
  kIoN2het2Pin17 = N2HET_TO_IO_PIN(kN2het2Pin17),
  kIoN2het2Pin18 = N2HET_TO_IO_PIN(kN2het2Pin18),
  kIoN2het2Pin19 = N2HET_TO_IO_PIN(kN2het2Pin19),
  kIoN2het2Pin20 = N2HET_TO_IO_PIN(kN2het2Pin20),
  kIoN2het2Pin21 = N2HET_TO_IO_PIN(kN2het2Pin21),
  kIoN2het2Pin22 = N2HET_TO_IO_PIN(kN2het2Pin22),
  kIoN2het2Pin23 = N2HET_TO_IO_PIN(kN2het2Pin23),
  kIoN2het2Pin24 = N2HET_TO_IO_PIN(kN2het2Pin24),
  kIoN2het2Pin25 = N2HET_TO_IO_PIN(kN2het2Pin25),
  kIoN2het2Pin26 = N2HET_TO_IO_PIN(kN2het2Pin26),
  kIoN2het2Pin27 = N2HET_TO_IO_PIN(kN2het2Pin27),
  kIoN2het2Pin28 = N2HET_TO_IO_PIN(kN2het2Pin28),
  kIoN2het2Pin29 = N2HET_TO_IO_PIN(kN2het2Pin29),
  kIoN2het2Pin30 = N2HET_TO_IO_PIN(kN2het2Pin30),
  kIoN2het2Pin31 = N2HET_TO_IO_PIN(kN2het2Pin31),
  // SCI pins.
  kIoSci1PinRx = SCI_TO_IO_PIN(kSci1PinRx),
  kIoSci1PinTx = SCI_TO_IO_PIN(kSci1PinTx),
  kIoSci2PinRx = SCI_TO_IO_PIN(kSci2PinRx),
  kIoSci2PinTx = SCI_TO_IO_PIN(kSci2PinTx),
  // SPI1 pins.
  kIoSpi1PinClk   = SPI_TO_IO_PIN(kSpi1PinClk),
  kIoSpi1PinEna   = SPI_TO_IO_PIN(kSpi1PinEna),
  kIoSpi1PinMiso0 = SPI_TO_IO_PIN(kSpi1PinMiso0),
  kIoSpi1PinMosi0 = SPI_TO_IO_PIN(kSpi1PinMosi0),
  kIoSpi1PinScs0  = SPI_TO_IO_PIN(kSpi1PinScs0),
  kIoSpi1PinScs1  = SPI_TO_IO_PIN(kSpi1PinScs1),
  kIoSpi1PinScs2  = SPI_TO_IO_PIN(kSpi1PinScs2),
  kIoSpi1PinScs3  = SPI_TO_IO_PIN(kSpi1PinScs3),
  kIoSpi1PinScs4  = SPI_TO_IO_PIN(kSpi1PinScs4),
  kIoSpi1PinScs5  = SPI_TO_IO_PIN(kSpi1PinScs5),
  // SPI3 pins.
  kIoSpi3PinClk   = SPI_TO_IO_PIN(kSpi3PinClk),
  kIoSpi3PinEna   = SPI_TO_IO_PIN(kSpi3PinEna),
  kIoSpi3PinMiso0 = SPI_TO_IO_PIN(kSpi3PinMiso0),
  kIoSpi3PinMosi0 = SPI_TO_IO_PIN(kSpi3PinMosi0),
  kIoSpi3PinScs0  = SPI_TO_IO_PIN(kSpi3PinScs0),
  kIoSpi3PinScs1  = SPI_TO_IO_PIN(kSpi3PinScs1),
  kIoSpi3PinScs2  = SPI_TO_IO_PIN(kSpi3PinScs2),
  kIoSpi3PinScs3  = SPI_TO_IO_PIN(kSpi3PinScs3),
  kIoSpi3PinScs4  = SPI_TO_IO_PIN(kSpi3PinScs4),
  kIoSpi3PinScs5  = SPI_TO_IO_PIN(kSpi3PinScs5),
  // SPI4 pins.
  kIoSpi4PinClk   = SPI_TO_IO_PIN(kSpi4PinClk),
  kIoSpi4PinEna   = SPI_TO_IO_PIN(kSpi4PinEna),
  kIoSpi4PinMiso0 = SPI_TO_IO_PIN(kSpi4PinMiso0),
  kIoSpi4PinMosi0 = SPI_TO_IO_PIN(kSpi4PinMosi0),
  kIoSpi4PinScs0  = SPI_TO_IO_PIN(kSpi4PinScs0),
  // SPI5 pins.
  kIoSpi5PinClk   = SPI_TO_IO_PIN(kSpi5PinClk),
  kIoSpi5PinEna   = SPI_TO_IO_PIN(kSpi5PinEna),
  kIoSpi5PinMiso0 = SPI_TO_IO_PIN(kSpi5PinMiso0),
  kIoSpi5PinMiso1 = SPI_TO_IO_PIN(kSpi5PinMiso1),
  kIoSpi5PinMiso2 = SPI_TO_IO_PIN(kSpi5PinMiso2),
  kIoSpi5PinMosi0 = SPI_TO_IO_PIN(kSpi5PinMosi0),
  kIoSpi5PinScs0  = SPI_TO_IO_PIN(kSpi5PinScs0),
} IoPin;

void IoInit(void);
bool IoPinIsUnmappedPin(IoPin pin);
bool IoPinIsDcanPin(IoPin pin);
bool IoPinIsGioPin(IoPin pin);
bool IoPinIsN2hetPin(IoPin pin);
bool IoPinIsSciPin(IoPin pin);
bool IoPinIsSpiPin(IoPin pin);
DcanPin IoPinToDcanPin(IoPin pin);
GioPin IoPinToGioPin(IoPin pin);
N2hetPin IoPinToN2hetPin(IoPin pin);
SciPin IoPinToSciPin(IoPin pin);
SpiPin IoPinToSpiPin(IoPin pin);

// Use these functions to initialize a IO pin to a known state.
void IoConfigureAsInputPullDown(IoPin pin);
void IoConfigureAsInputPullUp(IoPin pin);
void IoConfigureAsInput(IoPin pin);
void IoConfigureAsOutputPushPull(IoPin pin, bool value);
void IoConfigureAsOutputOpenDrain(IoPin pin, bool value);

// The functions below set individual bits of a IO pin.

// Direction bits.
void IoSetDirectionAsOutput(IoPin pin);
void IoSetDirectionAsInput(IoPin pin);

// Get/Set value bits.
void IoSetValue(IoPin pin, bool value);
bool IoGetValue(IoPin pin);
void IoSetOutputHigh(IoPin pin);
void IoSetOutputLow(IoPin pin);

// Output type bits.
void IoSetAsPushPull(IoPin pin);
void IoSetAsOpenDrain(IoPin pin);

// Input type bits.
void IoDisablePull(IoPin pin);
void IoEnablePull(IoPin pin);
void IoSetAsPullDown(IoPin pin);
void IoSetAsPullUp(IoPin pin);

#endif  // AVIONICS_FIRMWARE_CPU_IO_H_
