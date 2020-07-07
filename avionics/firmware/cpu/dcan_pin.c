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

#include "avionics/firmware/cpu/dcan_pin.h"

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "common/macros.h"

// Create map from enum to Dcan number and Gpio register.
// DCAN_TIOC union is used for both tx and rx as the bitfields are the same.
const struct {
  uint32_t dcan_num;
  Peripheral peripheral;
  volatile union DCAN_TIOC *ctrl_reg;
} kDcanPinMap[] = {
  [kDcan1PinRx] = {1, kPeripheralDcan1,
                   (volatile union DCAN_TIOC *)&DCAN(1).RIOC},
  [kDcan1PinTx] = {1, kPeripheralDcan1, &DCAN(1).TIOC},
  [kDcan2PinRx] = {2, kPeripheralDcan2,
                   (volatile union DCAN_TIOC *)&DCAN(2).RIOC},
  [kDcan2PinTx] = {2, kPeripheralDcan2, &DCAN(2).TIOC},
  [kDcan3PinRx] = {3, kPeripheralDcan3,
                   (volatile union DCAN_TIOC *)&DCAN(3).RIOC},
  [kDcan3PinTx] = {3, kPeripheralDcan3, &DCAN(3).TIOC}
};

static void EnablePeripheralForPin(DcanPin pin) {
  if (0 <= pin && pin < kNumDcanPins) {
    PeripheralEnable(kDcanPinMap[pin].peripheral);
    DCAN(kDcanPinMap[pin].dcan_num).CTL.INIT = 1;
  }
}

static void DcanPinSetToGioFunction(DcanPin pin) {
  if (0 <= pin && pin < kNumDcanPins) {
    kDcanPinMap[pin].ctrl_reg->FUNC = 0;
  }
}

void DcanPinConfigureAsInputPullDown(DcanPin pin) {
  EnablePeripheralForPin(pin);
  DcanPinSetDirectionAsInput(pin);
  DcanPinSetAsPullDown(pin);
  DcanPinEnablePull(pin);
  DcanPinSetToGioFunction(pin);
}

void DcanPinConfigureAsInputPullUp(DcanPin pin) {
  EnablePeripheralForPin(pin);
  DcanPinSetDirectionAsInput(pin);
  DcanPinSetAsPullUp(pin);
  DcanPinEnablePull(pin);
  DcanPinSetToGioFunction(pin);
}

void DcanPinConfigureAsInput(DcanPin pin) {
  EnablePeripheralForPin(pin);
  DcanPinSetDirectionAsInput(pin);
  DcanPinDisablePull(pin);
  DcanPinSetToGioFunction(pin);
}

void DcanPinConfigureAsOutputPushPull(DcanPin pin, bool value) {
  EnablePeripheralForPin(pin);
  DcanPinSetValue(pin, value);
  DcanPinSetAsPushPull(pin);
  DcanPinSetDirectionAsOutput(pin);
  DcanPinSetToGioFunction(pin);
}

void DcanPinConfigureAsOutputOpenDrain(DcanPin pin, bool value) {
  EnablePeripheralForPin(pin);
  DcanPinSetValue(pin, value);
  DcanPinSetAsOpenDrain(pin);
  DcanPinSetDirectionAsOutput(pin);
  DcanPinSetToGioFunction(pin);
}

void DcanPinSetDirectionAsOutput(DcanPin pin) {
  if (0 <= pin && pin < kNumDcanPins) {
    kDcanPinMap[pin].ctrl_reg->DIR = 1;
  }
}

void DcanPinSetDirectionAsInput(DcanPin pin) {
  if (0 <= pin && pin < kNumDcanPins) {
    kDcanPinMap[pin].ctrl_reg->DIR = 0;
  }
}

void DcanPinSetValue(DcanPin pin, bool value) {
  if (value) {
    DcanPinSetOutputHigh(pin);
  } else {
    DcanPinSetOutputLow(pin);
  }
}

bool DcanPinGetValue(DcanPin pin) {
  if (0 <= pin && pin < kNumDcanPins) {
    return kDcanPinMap[pin].ctrl_reg->IN;
  }
  return false;
}

void DcanPinSetOutputHigh(DcanPin pin) {
  if (0 <= pin && pin < kNumDcanPins) {
    kDcanPinMap[pin].ctrl_reg->OUT = 1;
  }
}

void DcanPinSetOutputLow(DcanPin pin) {
  if (0 <= pin && pin < kNumDcanPins) {
    kDcanPinMap[pin].ctrl_reg->OUT = 0;
  }
}

void DcanPinSetAsPushPull(DcanPin pin) {
  if (0 <= pin && pin < kNumDcanPins) {
    kDcanPinMap[pin].ctrl_reg->OD = 0;
  }
}

void DcanPinSetAsOpenDrain(DcanPin pin) {
  if (0 <= pin && pin < kNumDcanPins) {
    kDcanPinMap[pin].ctrl_reg->OD = 1;
  }
}

void DcanPinDisablePull(DcanPin pin) {
  if (0 <= pin && pin < kNumDcanPins) {
    kDcanPinMap[pin].ctrl_reg->PD = 1;
  }
}

void DcanPinEnablePull(DcanPin pin) {
  if (0 <= pin && pin < kNumDcanPins) {
    kDcanPinMap[pin].ctrl_reg->PD = 0;
  }
}

void DcanPinSetAsPullDown(DcanPin pin) {
  if (0 <= pin && pin < kNumDcanPins) {
    kDcanPinMap[pin].ctrl_reg->PU = 0;
  }
}

void DcanPinSetAsPullUp(DcanPin pin) {
  if (0 <= pin && pin < kNumDcanPins) {
    kDcanPinMap[pin].ctrl_reg->PU = 1;
  }
}
