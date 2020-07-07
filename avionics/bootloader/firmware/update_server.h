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

#ifndef AVIONICS_BOOTLOADER_FIRMWARE_UPDATE_SERVER_H_
#define AVIONICS_BOOTLOADER_FIRMWARE_UPDATE_SERVER_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/identity/identity_types.h"

// Returns true iff it burned an image successfully.
bool UpdateServer(HardwareType hardware_type);

// Exposed for use by the bootloader client.
typedef enum {
  kPacketIdStart        = 0,
  kPacketIdReady        = 1,
  kPacketIdData         = 2,
  kPacketIdDataReceived = 3
} PacketId;

// Exposed for use by the bootloader client.
typedef enum {
  kUpdateTypeBootloader          = 0,
  kUpdateTypeApplication         = 1,
  kUpdateTypeConfigParams        = 2,  // Renamed from kUpdateTypeFlashParams.
  kUpdateTypeCalibParams         = 3,  // Renamed from kUpdateTypeOtpParams.
  kUpdateTypeSerialParams        = 4,
  kUpdateTypeCarrierSerialParams = 5,
} UpdateType;

#endif  // AVIONICS_BOOTLOADER_FIRMWARE_UPDATE_SERVER_H_
