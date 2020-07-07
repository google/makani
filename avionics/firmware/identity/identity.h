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

#ifndef AVIONICS_FIRMWARE_IDENTITY_IDENTITY_H_
#define AVIONICS_FIRMWARE_IDENTITY_IDENTITY_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/network_addresses.h"
#include "avionics/firmware/identity/identity_types.h"
#include "avionics/network/aio_node.h"

// Specified in BootConfig. See ldscript.h.
HardwareType BootConfigGetHardwareType(void);
IpAddress BootConfigGetIpAddress(void);
EthernetAddress BootConfigGetMacAddress(void);
bool BootConfigIsValid(void);

// Specified in AppConfig. See ldscript.h. Do not call from the bootloader.
int32_t AppConfigGetIndex(void);
AioNode AppConfigGetAioNode(void);
bool AppConfigIsAioNodeValid(void);

CarrierHardwareType GetCarrierHardwareType(void);

#endif  // AVIONICS_FIRMWARE_IDENTITY_IDENTITY_H_
