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

#include "avionics/short_stack/firmware/gpio.h"

#include <assert.h>
#include <stdbool.h>

#include "avionics/firmware/cpu/io.h"
#include "avionics/firmware/serial/short_stack_serial_params.h"


static ShortStackStatus output_pin_to_status[] = {
  [kShortStackGpioOutputPinForceNoTrips] = kShortStackStatusForceNoTrips,
  [kShortStackGpioOutputPinForceTripB0]  = kShortStackStatusForceTripB0,
  [kShortStackGpioOutputPinForceTripB1]  = kShortStackStatusForceTripB1,
  [kShortStackGpioOutputPinForceTripB2]  = kShortStackStatusForceTripB2,
  [kShortStackGpioOutputPinForceTripB3]  = kShortStackStatusForceTripB3,
};

static IoPin InputPinAssignment(ShortStackHardware rev,
                                ShortStackGpioInputPin pin_func) {
  assert(kNumShortStackHardwares == 1);  // Else need to add new revs here.
  const IoPin rev_map[kNumShortStackHardwares][kNumShortStackGpioInputPins] = {
    [kShortStackHardwareRev01] = {
      // "XArmed" tells whether the xilinx has armed, in other words has found
      // the sum of stack levels to be sufficient to enable auto-tripping
      // based on measured stack level voltages.
      [kShortStackGpioInputPinXArmed]  = kIoN2het1Pin14,
      // "XLat" is Xilinx latching status for a given block level.
      // An auto-trip based on measured voltages will set XLat for that
      // block to 1, and prevent the other three levels from auto-tripping.
      // Force trips cannot override the latch. Force-no-trips can clear it.
      [kShortStackGpioInputPinXLatB0]  = kIoSpi3PinClk,   // +MV/2 to +MV.
      [kShortStackGpioInputPinXLatB1]  = kIoN2het1Pin20,  // -MV/2 to MID.
      [kShortStackGpioInputPinXLatB2]  = kIoN2het1Pin18,  // -MV to -MV/2.
      [kShortStackGpioInputPinXLatB3]  = kIoN2het1Pin22,  // MID to +MV/2.
      // "Gate" is the Xilinx output logic pin controlling the relay firing
      // circuits for a given block level.
      [kShortStackGpioInputPinGateB0]  = kIoN2het1Pin4,   // +MV/2 to +MV.
      [kShortStackGpioInputPinGateB1]  = kIoDcan1PinTx,   // -MV/2 to MID.
      [kShortStackGpioInputPinGateB2]  = kIoSpi3PinScs4,  // -MV to -MV/2.
      [kShortStackGpioInputPinGateB3]  = kIoDcan1PinRx,   // MID to +MV/2.
      // "Mon" pins monitor the state of the relay firing coils on a given
      // block level. If MonBx == 1, the short-stack has activated that coil.
      [kShortStackGpioInputPinMonB0]   = kIoDcan3PinRx,   // +MV/2 to +MV.
      [kShortStackGpioInputPinMonB1]   = kIoDcan2PinRx,   // -MV/2 to MID.
      [kShortStackGpioInputPinMonB2]   = kIoDcan2PinTx,   // -MV to -MV/2.
      [kShortStackGpioInputPinMonB3]   = kIoDcan3PinTx,   // MID to +MV/2.
    },
  };
  return rev_map[rev][pin_func];
}

static IoPin OutputPinAssignment(ShortStackHardware rev,
                                 ShortStackGpioOutputPin pin_func) {
  assert(kNumShortStackHardwares == 1);  // Else need to add new revs here.
  const IoPin rev_map[kNumShortStackHardwares][kNumShortStackGpioOutputPins] = {
    [kShortStackHardwareRev01] = {
      // Activating this pin clears force-trips and disallow auto-trips.
      [kShortStackGpioOutputPinForceNoTrips] = kIoSci1PinRx,
      // Send force-trip signal to the respective voltage block.
      [kShortStackGpioOutputPinForceTripB0]  = kIoSpi3PinMiso0,  // Sbo/Pto.
      [kShortStackGpioOutputPinForceTripB1]  = kIoSpi3PinScs0,   // Sbi/Pti.
      [kShortStackGpioOutputPinForceTripB2]  = kIoSpi3PinMosi0,  // Pbi/Sti.
      [kShortStackGpioOutputPinForceTripB3]  = kIoSpi3PinScs5,   // Pbo/Sto.
    },
  };
  return rev_map[rev][pin_func];
}

// Set one short-stack output control pin and clear all other output pins.
void ShortStackGpioSetCtrlPins(ShortStackHardware rev,
                               ShortStackGpioOutputPin pin_requested,
                               ShortStackMonitorData *mon) {
  for (int pin=0; pin < kNumShortStackGpioOutputPins; ++pin) {
    if (pin == pin_requested) {
      IoSetValue(OutputPinAssignment(rev, pin), 1);
      SetStatus(output_pin_to_status[pin], 1, &mon->flags);
    } else {
      IoSetValue(OutputPinAssignment(rev, pin), 0);
      SetStatus(output_pin_to_status[pin], 0, &mon->flags);
    }
  }
}

// Return-to-default command received -- clear all control pins.
void ShortStackGpioClearCtrlPins(ShortStackHardware rev,
                                 ShortStackMonitorData *mon) {
  for (int pin=0; pin < kNumShortStackGpioOutputPins; ++pin) {
    IoSetValue(OutputPinAssignment(rev, pin), 0);
    SetStatus(output_pin_to_status[pin], 0, &mon->flags);
  }
}

void ShortStackGpioInit(ShortStackHardware rev) {
  IoInit();
  // Initialize input GPIO pins.
  for (int pin_func = 0; pin_func < kNumShortStackGpioInputPins; ++pin_func) {
    IoConfigureAsInput(InputPinAssignment(rev, pin_func));
  }
  // Initialize output GPIO pins and set to 0 (they are all active high).
  for (int pin_func = 0; pin_func < kNumShortStackGpioOutputPins; ++pin_func) {
    IoConfigureAsOutputPushPull(OutputPinAssignment(rev, pin_func), 0);
  }
}

bool ShortStackGpioPollInputPin(ShortStackHardware rev,
                                ShortStackGpioInputPin pin_func) {
  return IoGetValue(InputPinAssignment(rev, pin_func));
}
