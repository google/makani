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

#include "avionics/firmware/util/selftest.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/build_info.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/drivers/bcm_unified.h"
#include "avionics/firmware/drivers/eeprom24.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/firmware/params/params.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/firmware/serial/carrier_serial.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/switch_types.h"
#include "common/macros.h"

#define FAILURE_MESSAGE_PERIOD_CYCLES CLOCK32_MSEC_TO_CYCLES(1000)

static void HandleSerialParams(const SerialParams *in_params,
                               SerialParams *out_params) {
  if (in_params != NULL) {
    *out_params = *in_params;
  } else {
    memset(out_params, 0x0, sizeof(*out_params));
  }
}

static void WriteSelfTestMessage(
    uint32_t code, const char *reason, va_list ap, SelfTestMessage *out)
    __attribute__((format(printf, 2, 0)));
static void WriteSelfTestMessage(
    uint32_t code, const char *reason, va_list ap, SelfTestMessage *out) {
  // Needed for checking carrier params.  OK to call multiple times.
  Eeprom24Init();

  memset(out, 0, sizeof(*out));
  GetBuildInfo(&out->build_info);
  HandleSerialParams(GetBoardSerialParams(), &out->serial_params);
  HandleSerialParams(GetCarrierSerialParams(), &out->carrier_serial_params);
  out->failure = code;
  vsnprintf(out->text, ARRAYSIZE(out->text) - 1, reason, ap);
}

static void FailureLoop(const SelfTestMessage *message) {
  uint32_t wakeup = Clock32GetCycles() + FAILURE_MESSAGE_PERIOD_CYCLES;
  BcmUnifiedInit(true);
  while (true) {
    if (CLOCK32_GE(Clock32GetCycles(), wakeup)) {
      wakeup += FAILURE_MESSAGE_PERIOD_CYCLES;
      NetSendAioSelfTestMessage(message);
    } else {
      NetPoll();
      BcmUnifiedPoll(NULL);
      ExtWatchdogPoll();
    }
  }
}

void SelfTestCompatibleHardware(int32_t num, const HardwareSpec *possible) {
  HardwareType my_type = BootConfigGetHardwareType();
  int32_t my_rev = GetBoardHardwareRevision();

  for (int32_t i = 0; i < num; ++i) {
    const HardwareSpec *valid = &possible[i];
    if (my_type == valid->hardware_type && my_rev == valid->hardware_revision) {
      return;
    }
  }
  SelfTestFailedLoop(kSelfTestFailureIncompatibleHardware,
                     "Incompatible hardware detected.");
}

void SelfTestNetworkIdentity(void) {
  if (!AppConfigIsAioNodeValid()) {
    SelfTestFailedLoop(kSelfTestFailureInvalidNetworkIdentity,
                       "Network identity not permitted.");
  }
}

void SelfTestBootloader(void) {
  if (!BootConfigIsValid()) {
    SelfTestFailedLoop(kSelfTestFailureInvalidBootloaderConfig,
                       "Bootloader configuration not valid.");
  }
}

void SelfTestCommon(int32_t num, const HardwareSpec *possible) {
  SelfTestSerialParameters();
  SelfTestCompatibleHardware(num, possible);
  SelfTestNetworkIdentity();
  SelfTestBootloader();
}

void SelfTestCarrierCommon(int32_t num, const CarrierHardwareSpec *possible) {
  Eeprom24Init();
  CarrierSerialParamsInit();
  const SerialParams *serial = GetCarrierSerialParams();
  CarrierHardwareType type = GetCarrierHardwareType();
  if (serial == NULL) {
    SelfTestFailedLoop(kSelfTestFailureInvalidCarrierSerialParams,
                       "Carrier serial parameters not programmed or invalid.");
  }

  for (int32_t i = 0; i < num; ++i) {
    const CarrierHardwareSpec *valid = &possible[i];
    if (type == valid->hardware_type
        && serial->hardware_revision == valid->hardware_revision) {
      return;
    }
  }
  SelfTestFailedLoop(kSelfTestFailureIncompatibleHardware,
                     "Incompatible carrier board detected.");
}

void SelfTestCalibParameters(uint32_t possible) {
  SelfTestCalibParametersMany(1, &possible);
}

void SelfTestCalibParametersMany(int32_t num, const uint32_t *possible) {
  uint32_t my_version;
  if (!GetCalibParamsRaw(&my_version)) {
    SelfTestFailedLoop(kSelfTestFailureInvalidCalibParams,
                       "Calibration parameters not programmed or invalid.");
  }
  for (int32_t i = 0; i < num; ++i) {
    if (my_version == possible[i]) {
      return;
    }
  }
  SelfTestFailedLoop(kSelfTestFailureInvalidCalibParams,
                     "Calibration parameters version not supported.");
}

void SelfTestConfigParameters(uint32_t possible) {
  SelfTestConfigParametersMany(1, &possible);
}

void SelfTestConfigParametersMany(int32_t num, const uint32_t *possible) {
  uint32_t my_version;
  if (!GetConfigParamsRaw(&my_version)) {
    SelfTestFailedLoop(kSelfTestFailureInvalidConfigParams,
                       "Configuration parameters not programmed or invalid.");
  }
  for (int32_t i = 0; i < num; ++i) {
    if (my_version == possible[i]) {
      return;
    }
  }
  SelfTestFailedLoop(kSelfTestFailureInvalidConfigParams,
                     "Configuration parameters version not supported.");
}

void SelfTestSerialParameters(void) {
  BoardSerialParamsInit();
  if (GetBoardSerialParams() == NULL) {
    SelfTestFailedLoop(kSelfTestFailureInvalidSerialParams,
                       "Serial parameters not programmed or invalid.");
  }
}

void SelfTestFailedLoop(uint32_t code, const char *reason, ...) {
  SelfTestMessage message;
  va_list ap;
  va_start(ap, reason);
  WriteSelfTestMessage(code, reason, ap, &message);
  va_end(ap);
  FailureLoop(&message);
}
