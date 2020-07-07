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

#include "avionics/firmware/cpu/io.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>

#include "avionics/firmware/cpu/dcan_pin.h"
#include "avionics/firmware/cpu/gio.h"
#include "avionics/firmware/cpu/n2het.h"
#include "avionics/firmware/cpu/sci_pin.h"
#include "avionics/firmware/cpu/spi_pin.h"

#define GET_FUNC1(NAME)                         \
  bool Io ## NAME(IoPin pin) {                  \
    return GetFunc1(pin,                        \
                    DcanPin ## NAME,            \
                    Gio ## NAME,                \
                    N2het ## NAME,              \
                    SciPin ## NAME,             \
                    SpiPin ## NAME);            \
  }

#define SET_FUNC1(NAME)                         \
  void Io ## NAME(IoPin pin) {                  \
    SetFunc1(pin,                               \
             DcanPin ## NAME,                   \
             Gio ## NAME,                       \
             N2het ## NAME,                     \
             SciPin ## NAME,                    \
             SpiPin ## NAME);                   \
  }

#define SET_FUNC2(NAME)                         \
  void Io ## NAME(IoPin pin, bool value) {      \
    SetFunc2(pin, value,                        \
             DcanPin ## NAME,                   \
             Gio ## NAME,                       \
             N2het ## NAME,                     \
             SciPin ## NAME,                    \
             SpiPin ## NAME);                   \
  }

static bool GetFunc1(IoPin pin,
                     bool (* const dcan_func)(DcanPin pin),
                     bool (* const gio_func)(GioPin pin),
                     bool (* const n2het_func)(N2hetPin pin),
                     bool (* const sci_func)(SciPin pin),
                     bool (* const spi_func)(SpiPin pin)) {
  if (IoPinIsUnmappedPin(pin)) {
    // No-op.
    return false;
  } else if (IoPinIsDcanPin(pin)) {
    return dcan_func(IoPinToDcanPin(pin));
  } else if (IoPinIsGioPin(pin)) {
    return gio_func(IoPinToGioPin(pin));
  } else if (IoPinIsN2hetPin(pin)) {
    return n2het_func(IoPinToN2hetPin(pin));
  } else if (IoPinIsSciPin(pin)) {
    return sci_func(IoPinToSciPin(pin));
  } else if (IoPinIsSpiPin(pin)) {
    return spi_func(IoPinToSpiPin(pin));
  } else {
    assert(false);
    return false;
  }
}

static void SetFunc1(IoPin pin,
                     void (* const dcan_func)(DcanPin pin),
                     void (* const gio_func)(GioPin pin),
                     void (* const n2het_func)(N2hetPin pin),
                     void (* const sci_func)(SciPin pin),
                     void (* const spi_func)(SpiPin pin)) {
  if (IoPinIsUnmappedPin(pin)) {
    // No-op.
  } else if (IoPinIsDcanPin(pin)) {
    dcan_func(IoPinToDcanPin(pin));
  } else if (IoPinIsGioPin(pin)) {
    gio_func(IoPinToGioPin(pin));
  } else if (IoPinIsN2hetPin(pin)) {
    n2het_func(IoPinToN2hetPin(pin));
  } else if (IoPinIsSciPin(pin)) {
    sci_func(IoPinToSciPin(pin));
  } else if (IoPinIsSpiPin(pin)) {
    spi_func(IoPinToSpiPin(pin));
  } else {
    assert(false);
  }
}

static void SetFunc2(IoPin pin, bool value,
                     void (* const dcan_func)(DcanPin pin, bool value),
                     void (* const gio_func)(GioPin pin, bool value),
                     void (* const n2het_func)(N2hetPin pin, bool value),
                     void (* const sci_func)(SciPin pin, bool value),
                     void (* const spi_func)(SpiPin pin, bool value)) {
  if (IoPinIsUnmappedPin(pin)) {
    // No-op.
  } else if (IoPinIsDcanPin(pin)) {
    dcan_func(IoPinToDcanPin(pin), value);
  } else if (IoPinIsGioPin(pin)) {
    gio_func(IoPinToGioPin(pin), value);
  } else if (IoPinIsN2hetPin(pin)) {
    n2het_func(IoPinToN2hetPin(pin), value);
  } else if (IoPinIsSciPin(pin)) {
    sci_func(IoPinToSciPin(pin), value);
  } else if (IoPinIsSpiPin(pin)) {
    spi_func(IoPinToSpiPin(pin), value);
  } else {
    assert(false);
  }
}

void IoInit(void) {
  GioInit();
  N2hetInit();
}

bool IoPinIsUnmappedPin(IoPin pin) {
  return IO_ENUM_TO_TYPE(pin) == kIoInterfaceUnmapped;
}

bool IoPinIsDcanPin(IoPin pin) {
  return IO_ENUM_TO_TYPE(pin) == kIoInterfaceDcan;
}

bool IoPinIsGioPin(IoPin pin) {
  return IO_ENUM_TO_TYPE(pin) == kIoInterfaceGio;
}

bool IoPinIsN2hetPin(IoPin pin) {
  return IO_ENUM_TO_TYPE(pin) == kIoInterfaceN2het;
}

bool IoPinIsSciPin(IoPin pin) {
  return IO_ENUM_TO_TYPE(pin) == kIoInterfaceSci;
}

bool IoPinIsSpiPin(IoPin pin) {
  return IO_ENUM_TO_TYPE(pin) == kIoInterfaceSpi;
}

DcanPin IoPinToDcanPin(IoPin pin) {
  assert(IO_ENUM_TO_TYPE(pin) == kIoInterfaceDcan);
  return (DcanPin)IO_ENUM_TO_PIN(pin);
}

GioPin IoPinToGioPin(IoPin pin) {
  assert(IO_ENUM_TO_TYPE(pin) == kIoInterfaceGio);
  return (GioPin)IO_ENUM_TO_PIN(pin);
}

N2hetPin IoPinToN2hetPin(IoPin pin) {
  assert(IO_ENUM_TO_TYPE(pin) == kIoInterfaceN2het);
  return (N2hetPin)IO_ENUM_TO_PIN(pin);
}

SciPin IoPinToSciPin(IoPin pin) {
  assert(IO_ENUM_TO_TYPE(pin) == kIoInterfaceSci);
  return (SciPin)IO_ENUM_TO_PIN(pin);
}

SpiPin IoPinToSpiPin(IoPin pin) {
  assert(IO_ENUM_TO_TYPE(pin) == kIoInterfaceSpi);
  return (SpiPin)IO_ENUM_TO_PIN(pin);
}

SET_FUNC1(ConfigureAsInputPullDown);
SET_FUNC1(ConfigureAsInputPullUp);
SET_FUNC1(ConfigureAsInput);
SET_FUNC2(ConfigureAsOutputPushPull);
SET_FUNC2(ConfigureAsOutputOpenDrain);
SET_FUNC1(SetDirectionAsOutput);
SET_FUNC1(SetDirectionAsInput);
SET_FUNC2(SetValue);
GET_FUNC1(GetValue);
SET_FUNC1(SetOutputHigh);
SET_FUNC1(SetOutputLow);
SET_FUNC1(SetAsPushPull);
SET_FUNC1(SetAsOpenDrain);
SET_FUNC1(DisablePull);
SET_FUNC1(EnablePull);
SET_FUNC1(SetAsPullDown);
SET_FUNC1(SetAsPullUp);
