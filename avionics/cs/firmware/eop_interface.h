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

#ifndef AVIONICS_CS_FIRMWARE_EOP_INTERFACE_H_
#define AVIONICS_CS_FIRMWARE_EOP_INTERFACE_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/network_addresses.h"

bool HandleEopMessage(const IpAddress *source_ip, uint16_t source_port,
                      uint16_t dest_port, int32_t length, const uint8_t *data);

#endif  // AVIONICS_CS_FIRMWARE_EOP_INTERFACE_H_
