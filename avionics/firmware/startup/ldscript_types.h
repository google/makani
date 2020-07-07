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

// Symbols exposed by linker scripts.

#ifndef AVIONICS_FIRMWARE_STARTUP_LDSCRIPT_TYPES_H_
#define AVIONICS_FIRMWARE_STARTUP_LDSCRIPT_TYPES_H_

#include <stdint.h>

// Application configuration. This configuration does not need to be forwards
// and backwards compatible because we generate a new configuration for each
// new binary.
typedef struct {
  int32_t aio_node;         // AioNode type.
  int32_t node_index;       // AioNode label index (moved from BootConfig).
  uint32_t crc_app_only;    // CRC32 of application only.
  uint32_t crc_app_config;  // CRC32 of application + AppConfig bytes preceding.
} __attribute__((__packed__)) AppConfig;

// Board identification. This configuration needs to be forwards and backwards
// compatible between the bootloader and application.
typedef struct {
  int32_t version;         // Version field to help handle changes.
  uint8_t ip_address[4];   // Node IP address.
  uint8_t mac_address[6];  // Node MAC address.
  int16_t hardware_type;   // See HardwareType enum.
  // TODO: Replace the following two fields with a CRC32 of the
  // bootloader only (similar to AppConfig).
  int16_t unused;          // Previously used as AppType.
  int16_t unused_2;        // Previously used as AioNode label index.
  uint32_t crc;            // CRC32 of bootloader + BootConfig bytes preceding.
} __attribute__((__packed__)) BootConfig;

#endif  // AVIONICS_FIRMWARE_STARTUP_LDSCRIPT_TYPES_H_
