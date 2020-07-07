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

#include "avionics/servo/firmware/init.h"

#include <stdbool.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/sci.h"
#include "avionics/firmware/drivers/log.h"
#include "avionics/servo/firmware/config_params.h"
#include "avionics/servo/firmware/output.h"
#include "avionics/servo/firmware/r22.h"
#include "avionics/servo/firmware/r22_can.h"
#include "avionics/servo/firmware/r22_def.h"
#include "avionics/servo/firmware/r22_param.h"
#include "avionics/servo/firmware/r22_serial.h"

#define SERVO_FAILURE_LIMIT       5
#define SERVO_PARAMETER_SET_TRIES 3
#define SERVO_PING_TIME_US        (100 * 1000)

typedef enum {
  kServoIdle,
  kServoInitInterface,
  kServoSerialBreak,
  kServoReset,
  kServoRetrieveOperatingMode,
  kServoSwapOperatingModes,
  kServoSetBaudRate,
  kServoSetBaudRateWait,
  kServoGetParameter,
  kServoSetParameter,
  kServoGetErrorLog,
  kServoConfigureCanPdo,
} ServoInitState;

static struct {
  ServoInitState state;
  int64_t timeout;
  bool first_entry;
  bool clear_entry;
  bool r22_reset;
  int32_t num_failures;
} g_init;

static struct {
  R22Parameter param;
  const R22ParamInfo *info;
  R22Memory mem;
  int32_t num_tries;
  const uint8_t *data;
} g_param;

// Handle timeouts common to most states.
static bool ServoBusyTimeout(ServoInitState timeout_state, int64_t now) {
  bool timeout = (now > g_init.timeout);
  if (timeout) {
    ++g_init.num_failures;
    if (g_init.num_failures > SERVO_FAILURE_LIMIT) {
      g_init.num_failures = 0;
      g_init.state = kServoInitInterface;
    } else {
      // Flag clear_entry enables reinitializing the current state.
      g_init.clear_entry = true;
      g_init.state = timeout_state;
    }
  }
  return timeout;
}

// Initialize the R22 interface.
static void ServoInitInterface(int64_t now) {
  LOG_PRINTF("R22 initialize.\n");
  R22Init();
  g_init.num_failures = 0;
  g_init.timeout = now + R22_POWER_ON_TIME_US;
  g_init.r22_reset = false;

  // Apply flash param limits to saved R22 params.
  R22ParamSetValue(kR22ParamNegativeSoftwareLimit,
                   ServoInvertAngleCal(kServoConfigParams->servo_min_limit));
  R22ParamSetValue(kR22ParamPositiveSoftwareLimit,
                   ServoInvertAngleCal(kServoConfigParams->servo_max_limit));
  g_init.state = kServoSerialBreak;
}

// Perform serial break to force baud rate to 9600.
static void ServoSerialBreak(int64_t now) {
  if (g_init.first_entry) {
    SciInit(&R22_SCI, R22_DEFAULT_BAUD_RATE);
    SciSetBreak(&R22_SCI);
    g_init.timeout = now + R22_BREAK_TIME_US;
  } else if (now > g_init.timeout) {
    SciClearBreak(&R22_SCI);
    // If the R22 has not been reset yet, reset it before proceeding.
    if (!g_init.r22_reset) {
      g_init.state = kServoReset;
    } else {
      g_init.state = kServoRetrieveOperatingMode;
    }
  }
}

// Perform a reset of the R22.
static void ServoReset(int64_t now) {
  if (g_init.first_entry) {
    R22SerialWriteReset();
    g_init.timeout = now + R22_POWER_ON_TIME_US;
  } else if (now > g_init.timeout) {
    g_init.r22_reset = true;
    g_init.state = kServoSerialBreak;
  } else {
    R22SerialTransfer();
  }
}

// Determine if we need to swap operating modes.
static void ServoRetrieveOperatingMode(int64_t now) {
  if (g_init.first_entry) {
    R22SerialWriteRetrieveOperatingMode();
    g_init.timeout = now + SERVO_PING_TIME_US;
  } else if (R22SerialTransfer()) {
    uint16_t boot_mode;
    if (R22SerialReadUint16(1, &boot_mode) && boot_mode) {
      g_init.state = kServoSwapOperatingModes;
    } else {
      g_init.state = kServoSetBaudRate;
    }
    g_init.num_failures = 0;
  } else {
    ServoBusyTimeout(kServoSerialBreak, now);
  }
}

// Swap between boot mode and normal mode.
static void ServoSwapOperatingModes(int64_t now) {
  if (g_init.first_entry) {
    R22SerialWriteSwapOperatingModes();
    g_init.timeout = now + R22_SWAP_OPERATING_MODES_TIME_US;
  } else if (R22SerialTransfer()) {
    g_init.num_failures = 0;
    g_init.state = kServoRetrieveOperatingMode;
  } else {
    ServoBusyTimeout(kServoSerialBreak, now);
  }
}

// Increase baud rate to meet control loop timing requirements. R22 sets
// baud rate upon reception of command then replies at new baud rate. This
// function sends the command and then sets the baud rate. Function
// ServoSetBaudRateWait() handles the reply.
static void ServoSetBaudRate(int64_t now) {
  if (g_init.first_entry) {
    uint32_t baud_rate = R22_DESIRED_BAUD_RATE;
    R22SerialWriteSetVariableUint32(kR22ParamBaudRate, kR22MemoryRam,
                                    1, &baud_rate);
    g_init.timeout = now + R22_TRANSFER_TIMEOUT_US;
  } else if (R22SerialSend()) {
    SciInit(&R22_SCI, R22_DESIRED_BAUD_RATE);
    g_init.num_failures = 0;
    g_init.state = kServoSetBaudRateWait;
  } else if (ServoBusyTimeout(kServoSerialBreak, now)) {
    SciInit(&R22_SCI, R22_DEFAULT_BAUD_RATE);
  }
}

// Wait for SetBaudRate reply, occurs at new baud rate.
static void ServoSetBaudRateWait(int64_t now) {
  if (g_init.first_entry) {
    g_init.timeout = now + R22_TRANSFER_TIMEOUT_US;
  } else if (R22SerialReceive()) {
    g_init.num_failures = 0;
    // Initialize GetParameter loop.
    g_init.state = kServoGetParameter;
    g_param.num_tries = 0;
    g_param.mem = kR22MemoryFlash;
    g_param.param = R22ParamGetNextConfigParam(kR22ParamNone, g_param.mem);
  } else if (ServoBusyTimeout(kServoSerialBreak, now)) {
    SciInit(&R22_SCI, R22_DEFAULT_BAUD_RATE);
  }
}

// Loop through all parameters in flash and ram and verify that their value
// matches our desired configuration.
static void ServoGetParameter(int64_t now) {
  bool get_next = g_init.first_entry;

  if (g_init.first_entry) {
    // Get first parameter.
  } else if (R22SerialTransfer()) {
    R22Error error = R22SerialReadError();
    g_init.num_failures = 0;

    // Check parameter against stored database.
    if (g_param.num_tries >= SERVO_PARAMETER_SET_TRIES
        || (error == kR22ErrorNone
            && R22SerialCompare(g_param.info->addr_length, g_param.data))) {
      // Get next parameter.
      R22Parameter next_param = R22ParamGetNextConfigParam(g_param.param,
                                                           g_param.mem);
      if ((int32_t)next_param <= (int32_t)g_param.param) {
        if (g_param.mem == kR22MemoryFlash) {
          g_param.mem = kR22MemoryRam;
        } else {
          g_init.state = kServoGetErrorLog;
        }
      } else {
        get_next = true;
      }
      g_param.num_tries = 0;
      g_param.param = next_param;

    } else {
      // Parameter does not match desired value.
      ++g_param.num_tries;
      g_init.state = kServoSetParameter;
    }
  } else {
    ServoBusyTimeout(kServoGetParameter, now);
  }
  if (get_next) {
    g_init.timeout = now + R22_TRANSFER_TIMEOUT_US;
    g_param.info = R22ParamGetInfo(g_param.param);
    R22ParamGetValuePtr(g_param.param, &g_param.data);
    R22SerialWriteGetVariableValue(g_param.param, g_param.mem);
  }
}

// Called when parameter does not match desired value.
static void ServoSetParameter(int64_t now) {
  if (g_init.first_entry) {
    LOG_PRINTF("R22 set parameter=%s, memory=%d.\n",
               R22ParamName(g_param.param), g_param.mem);
    R22SerialWriteSetVariableUint8(g_param.param, g_param.mem,
                                   g_param.info->addr_length, g_param.data);
  } else if (R22SerialTransfer()) {
    g_init.num_failures = 0;
    g_init.state = kServoGetParameter;
  } else {
    ServoBusyTimeout(kServoSetParameter, now);
  }
}

// Get the R22's stored error log.
static void ServoGetErrorLog(int64_t now) {
  if (g_init.first_entry) {
    R22SerialWriteErrorLog(kR22ErrorLogGetData, SERVO_ERROR_LOG_ENTRIES);
    g_init.timeout = now + R22_TRANSFER_TIMEOUT_US;
  } else if (R22SerialTransfer()) {
    uint32_t data[2*SERVO_ERROR_LOG_ENTRIES];
    if (R22SerialReadUint32(2*SERVO_ERROR_LOG_ENTRIES, data)) {
      ServoOutputErrorLog(2*SERVO_ERROR_LOG_ENTRIES, data);
    }
    g_init.num_failures = 0;
    g_init.state = kServoConfigureCanPdo;
  } else {
    ServoBusyTimeout(kServoGetErrorLog, now);
  }
}

static void ServoConfigureCanPdo(int64_t now) {
  if (R22CanPdoConfigured()) {
    // Bring the R22 into the Operational state so we can use PDOs.
    R22CanCommandOperational();
    g_init.state = kServoIdle;
  } else {
    R22CanPdoInitPoll();
    ServoBusyTimeout(kServoConfigureCanPdo, now);
  }
}

void ServoInit(void) {
  memset(&g_init, 0, sizeof(g_init));
  memset(&g_param, 0, sizeof(g_param));
  g_init.state = kServoInitInterface;
  g_init.first_entry = true;
  g_param.mem = kR22MemoryRam;
}

bool ServoInitPoll(int64_t now) {
  ServoInitState current;
  do {
    current = g_init.state;
    switch (current) {
      case kServoInitInterface:
        ServoInitInterface(now);
        break;
      case kServoSerialBreak:
        ServoSerialBreak(now);
        break;
      case kServoReset:
        ServoReset(now);
        break;
      case kServoRetrieveOperatingMode:
        ServoRetrieveOperatingMode(now);
        break;
      case kServoSwapOperatingModes:
        ServoSwapOperatingModes(now);
        break;
      case kServoSetBaudRate:
        ServoSetBaudRate(now);
        break;
      case kServoSetBaudRateWait:
        ServoSetBaudRateWait(now);
        break;
      case kServoGetParameter:
        ServoGetParameter(now);
        break;
      case kServoSetParameter:
        ServoSetParameter(now);
        break;
      case kServoGetErrorLog:
        ServoGetErrorLog(now);
        break;
      case kServoConfigureCanPdo:
        ServoConfigureCanPdo(now);
        break;
      case kServoIdle:
        break;
      default:
        g_init.state = kServoIdle;
        break;
    }
    // Flag first_entry allows states to perform initialization routines on
    // first execution, then poll until completion. Flag clear_entry allows
    // states to reset and run the initialization routines again as if the
    // state were just entered.
    g_init.first_entry = !(current == g_init.state && !g_init.clear_entry);
    g_init.clear_entry = false;
  } while (current != g_init.state);

  return (g_init.state == kServoIdle);
}
