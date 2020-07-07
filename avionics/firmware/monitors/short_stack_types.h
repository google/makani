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

#ifndef AVIONICS_FIRMWARE_MONITORS_SHORT_STACK_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_SHORT_STACK_TYPES_H_

#include <stdint.h>

#include "avionics/common/faults.h"
#include "avionics/firmware/monitors/short_stack_analog_types.h"
#include "avionics/firmware/monitors/short_stack_mcp342x_types.h"
#include "avionics/network/aio_labels.h"

typedef enum {
  kShortStackStatusForceNoTrips = (1 << 0),
  kShortStackStatusForceTripB0  = (1 << 1),
  kShortStackStatusForceTripB1  = (1 << 2),
  kShortStackStatusForceTripB2  = (1 << 3),
  kShortStackStatusForceTripB3  = (1 << 4),
  kShortStackStatusTrippedB0    = (1 << 5),
  kShortStackStatusTrippedB1    = (1 << 6),
  kShortStackStatusTrippedB2    = (1 << 7),
  kShortStackStatusTrippedB3    = (1 << 8),
} ShortStackStatus;

#define SHORT_STACK_STATUS_TRIPPED (kShortStackStatusTrippedB0 |        \
                                    kShortStackStatusTrippedB1 |        \
                                    kShortStackStatusTrippedB2 |        \
                                    kShortStackStatusTrippedB3)

typedef enum {
  kShortStackMonitorWarning72vfire = 1 << 0,
  kShortStackMonitorWarning5v      = 1 << 1,
  kShortStackMonitorWarning3v3     = 1 << 2,
} ShortStackMonitorWarning;

typedef enum {
  kShortStackMonitorErrorNone = 0,
} ShortStackMonitorError;

typedef enum {
  kShortStackGpioInputPinXArmed,  // Xilinx arming latch.
  kShortStackGpioInputPinXLatB0,  // Xilinx trip latch, block 0.
  kShortStackGpioInputPinXLatB1,  // Xilinx trip latch, block 1.
  kShortStackGpioInputPinXLatB2,  // Xilinx trip latch, block 2.
  kShortStackGpioInputPinXLatB3,  // Xilinx trip latch, block 3.
  kShortStackGpioInputPinGateB0,  // Xilinx block 0 relay fire control pin.
  kShortStackGpioInputPinGateB1,  // Xilinx block 1 relay fire control pin.
  kShortStackGpioInputPinGateB2,  // Xilinx block 2 relay fire control pin.
  kShortStackGpioInputPinGateB3,  // Xilinx block 3 relay fire control pin.
  kShortStackGpioInputPinMonB0,  // Block 0 relay control coil low-side state.
  kShortStackGpioInputPinMonB1,  // Block 1 relay control coil low-side state.
  kShortStackGpioInputPinMonB2,  // Block 2 relay control coil low-side state.
  kShortStackGpioInputPinMonB3,  // Block 3 relay control coil low-side state.
  kNumShortStackGpioInputPins,
} ShortStackGpioInputPin;

typedef enum {
  kShortStackGpioOutputPinForceNoTrips,  // Clear force-trips & ban auto-trips.
  kShortStackGpioOutputPinForceTripB0,  // Force-trip signal to Sbo/Pto block.
  kShortStackGpioOutputPinForceTripB1,  // Force-trip signal to Sbi/Pti block.
  kShortStackGpioOutputPinForceTripB2,  // Force-trip signal to Pbi/Sti block.
  kShortStackGpioOutputPinForceTripB3,  // Force-trip signal to Pbo/Sto block.
  kNumShortStackGpioOutputPins,
} ShortStackGpioOutputPin;

typedef struct {
  StatusFlags flags;

  // Tms570 ADC channels.
  uint32_t analog_populated;
  float analog_data[kNumShortStackAnalogVoltages];

  // Gpio input pin values.
  uint32_t gpio_inputs;

  // External ADC channels measuring hardware trip threshold settings.
  float mcp342x_data[kNumShortStackMcp342xMonitors];

  // Including motor voltages to verify receipt of stacking messages.
  float motor_voltage[kNumMotors];
} ShortStackMonitorData;

#endif  // AVIONICS_FIRMWARE_MONITORS_SHORT_STACK_TYPES_H_
