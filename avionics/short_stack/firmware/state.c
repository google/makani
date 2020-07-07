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

#include "avionics/short_stack/firmware/state.h"

#include <assert.h>
#include <stdbool.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/safety_codes.h"
#include "avionics/common/short_stack_types.h"
#include "avionics/firmware/drivers/log.h"
#include "avionics/firmware/serial/carrier_serial.h"
#include "avionics/network/aio_labels.h"
#include "avionics/short_stack/firmware/gpio.h"
#include "avionics/short_stack/firmware/output.h"
#include "common/macros.h"


typedef enum {
  kShortStackStateIdle,
  kShortStackStateRecvCmd,
  kShortStackStateRecvMotorMsg,
  kShortStackStateSetCtrlPins,
  kShortStackStateMonGpio,
  kShortStackStateInit
} ShortStackState;

static struct {
  ShortStackState state;
  bool force_no_trips;  // When high, clear force-trips & prevent default trips.
  bool force_trip;  // When high, force block indicated below to trip.
  int force_trip_block;  // 0 through 3 for the four stack level blocks.
} g_short_stack;

static MotorStackingMessage g_stacking_messages[kNumMotors] = {{0}};

static bool CommandSignalValid(const ShortStackCommandMessage *msg) {
  switch (msg->command_value) {
    case kShortStackCommandValueForceNoTrips:
      return msg->command_signal == SHORT_STACK_FORCE_NO_TRIPS_SIGNAL;
    case kShortStackCommandValueForceTripB0:
      return msg->command_signal == SHORT_STACK_FORCE_TRIP_B0_SIGNAL;
    case kShortStackCommandValueForceTripB1:
      return msg->command_signal == SHORT_STACK_FORCE_TRIP_B1_SIGNAL;
    case kShortStackCommandValueForceTripB2:
      return msg->command_signal == SHORT_STACK_FORCE_TRIP_B2_SIGNAL;
    case kShortStackCommandValueForceTripB3:
      return msg->command_signal == SHORT_STACK_FORCE_TRIP_B3_SIGNAL;
    case kShortStackCommandValueReturnToDefault:
      return msg->command_signal == SHORT_STACK_RETURN_TO_DEFAULT_SIGNAL;
    default:
      return false;
  }
}

static ShortStackCommandValue QueryShortStackCommand(AioNode op) {
  ShortStackCommandMessage msg;
  if (CvtGetShortStackCommandMessage(op, &msg, NULL, NULL)
      && CommandSignalValid(&msg)) {
    return msg.command_value;
  }
  return kShortStackCommandValueNone;
}

static void ReceiveCommand(void) {
  ShortStackCommandValue cmd_value = QueryShortStackCommand(kAioNodeOperator);
  switch (cmd_value) {
    case kShortStackCommandValueForceNoTrips:
      LOG_PRINTF("Received Short Stack Force-No-Trips command.\n");
      // Store variable indicating force-no-trips mode.
      g_short_stack.force_no_trips = true;
      // If we had any blocks force-tripped, force-no-trip command undoes that.
      g_short_stack.force_trip = false;
      break;
    case kShortStackCommandValueReturnToDefault:
      LOG_PRINTF("Received Short Stack Return-to-Default command.\n");
      // In force-trip, you must command force-no-trip before return-to-default.
      if (g_short_stack.force_trip) {
        LOG_PRINTF("You must issue force-no-trip cmd to exit force-trip.\n");
        // TODO: This ordering should be clear on the webmonitor.
      } else {
        // Enter or remain in default operation by exiting force-no-trips mode.
        g_short_stack.force_no_trips = false;
      }
      break;
    case kShortStackCommandValueForceTripB0:
      LOG_PRINTF("Received Short Stack Force-Trip-B0 command.\n");
      if (g_short_stack.force_trip) {
        LOG_PRINTF("You cannot force-trip when already in force-trip mode.\n");
        // TODO: This ordering should be clear on the webmonitor.
      } else {
        // Exit force-no-trips mode, enter force-trip, and specify the block.
        g_short_stack.force_no_trips = false;
        g_short_stack.force_trip = true;
        g_short_stack.force_trip_block = 0;
      }
      break;
    case kShortStackCommandValueForceTripB1:
      LOG_PRINTF("Received Short Stack Force-Trip-B1 command.\n");
      if (g_short_stack.force_trip) {
        LOG_PRINTF("You cannot force-trip when already in force-trip mode.\n");
        // TODO: This ordering should be clear on the webmonitor.
      } else {
        // Exit force-no-trips mode, enter force-trip, and specify the block.
        g_short_stack.force_no_trips = false;
        g_short_stack.force_trip = true;
        g_short_stack.force_trip_block = 1;
      }
      break;
    case kShortStackCommandValueForceTripB2:
      LOG_PRINTF("Received Short Stack Force-Trip-B2 command.\n");
      if (g_short_stack.force_trip) {
        LOG_PRINTF("You cannot force-trip when already in force-trip mode.\n");
        // TODO: This ordering should be clear on the webmonitor.
      } else {
        // Exit force-no-trips mode, enter force-trip, and specify the block.
        g_short_stack.force_no_trips = false;
        g_short_stack.force_trip = true;
        g_short_stack.force_trip_block = 2;
      }
      break;
    case kShortStackCommandValueForceTripB3:
      LOG_PRINTF("Received Short Stack Force-Trip-B3 command.\n");
      if (g_short_stack.force_trip) {
        LOG_PRINTF("You cannot force-trip when already in force-trip mode.\n");
        // TODO: This ordering should be clear on the webmonitor.
      } else {
        // Exit force-no-trips mode, enter force-trip, and specify the block.
        g_short_stack.force_no_trips = false;
        g_short_stack.force_trip = true;
        g_short_stack.force_trip_block = 3;
      }
      break;
    default:
      break;
  }
}

// State Handling Functions.

// Wakeup state handling function for the short stack carrier board.
static void ShortStackHandleInit(ShortStackState next) {
  // Default mode when short stack awakes.
  g_short_stack.force_no_trips = false;
  g_short_stack.force_trip = false;
  g_short_stack.force_trip_block = 0;

  // Initialize input and output I/O pins.
  ShortStackGpioInit(GetCarrierHardwareRevision());

  g_short_stack.state = next;
}

// Idle state handling function for the short stack carrier board.
static void ShortStackHandleIdle(ShortStackState next) {
  g_short_stack.state = next;
}

// State handling function for receiving commands from the operator.
static void ShortStackHandleRecvCmd(ShortStackState next) {
  ReceiveCommand();
  g_short_stack.state = next;
}

// State handling func. for receiving stacking messages from motor controllers.
static void ShortStackHandleRecvMotorMsg(ShortStackState next) {
  // TODO: Flesh out rest of handling function.
  ShortStackMonitorData *mon = ShortStackOutputGetShortStackMonitors();
  for (int32_t i = 0; i < kNumMotors; ++i) {
    MotorStackingMessage message;
    if (CvtGetMotorStackingMessage((AioNode)(i + kAioNodeMotorSbo),
                                   &message, NULL, NULL)) {
      g_stacking_messages[i] = message;
      mon->motor_voltage[i] = message.bus_voltage;
      // TODO: receive motor errors to determine whether to fire.
    }
  }

  g_short_stack.state = next;
}

// State handling to set I/O pins that control trip/no-trip/default modes.
static void ShortStackHandleSetCtrlPins(ShortStackState next) {
  ShortStackHardware rev = GetCarrierHardwareRevision();
  ShortStackMonitorData *mon = ShortStackOutputGetShortStackMonitors();
  if (g_short_stack.force_no_trips) {
    // Clear all force-trips and set force_no_trips.
    ShortStackGpioSetCtrlPins(rev, kShortStackGpioOutputPinForceNoTrips, mon);
  } else if (g_short_stack.force_trip) {
    // Clear all other control pins and set force_trip for indicated block.
    ShortStackGpioSetCtrlPins(rev, kShortStackGpioOutputPinForceTripB0
                              + g_short_stack.force_trip_block, mon);
  } else {
    // Default operation: clear all control pins.
    ShortStackGpioClearCtrlPins(rev, mon);
  }
  g_short_stack.state = next;
}

// State handling func. for checking input values on tms570 gpio pins.
static void ShortStackHandleMonGpio(ShortStackState next) {
  ShortStackMonitorData *mon = ShortStackOutputGetShortStackMonitors();
  ShortStackHardware rev = GetCarrierHardwareRevision();
  // Store pin value in output structure for status message.
  for (int pin_func = 0; pin_func < kNumShortStackGpioInputPins; ++pin_func) {
    bool pin_value = ShortStackGpioPollInputPin(rev, pin_func);
    // Set status based on which level (if any) has been triggered to short out.
    if (pin_func >= kShortStackGpioInputPinXLatB0 &&
        pin_func <= kShortStackGpioInputPinXLatB3) {  // Xilinx trip latch.
      int i_block = pin_func - kShortStackGpioInputPinXLatB0;  // B0 -> 0 etc.
      SetStatus(kShortStackStatusTrippedB0 << i_block, pin_value, &mon->flags);
    }
    // Report pin values in short stack AIO status message.
    if (pin_value) {
      mon->gpio_inputs |= (uint32_t)(1 << pin_func);
    } else {
      mon->gpio_inputs &= (uint32_t)(~(1 << pin_func));
    }
  }

  g_short_stack.state = next;
}

// State machine init function accessed from main short stack code.
void ShortStackStateInit(void) {
  memset(&g_short_stack, 0, sizeof(g_short_stack));
  g_short_stack.state = kShortStackStateInit;
}

// State-machine-advancing function called from main short stack code loop.
void ShortStackStatePoll(void) {
  ShortStackState current_state = g_short_stack.state;
  switch (current_state) {
    case kShortStackStateInit:
      ShortStackHandleInit(kShortStackStateIdle);
      break;
    case kShortStackStateIdle:
      ShortStackHandleIdle(kShortStackStateRecvCmd);
      break;
    case kShortStackStateRecvCmd:
      ShortStackHandleRecvCmd(kShortStackStateRecvMotorMsg);
      break;
    case kShortStackStateRecvMotorMsg:
      ShortStackHandleRecvMotorMsg(kShortStackStateSetCtrlPins);
      break;
    case kShortStackStateSetCtrlPins:
      ShortStackHandleSetCtrlPins(kShortStackStateMonGpio);
      break;
    case kShortStackStateMonGpio:
      ShortStackHandleMonGpio(kShortStackStateIdle);
      break;
    default:
      g_short_stack.state = kShortStackStateIdle;
      break;
  }
}
