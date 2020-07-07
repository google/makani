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

#ifndef AVIONICS_BOOTLOADER_FIRMWARE_FLASH_H_
#define AVIONICS_BOOTLOADER_FIRMWARE_FLASH_H_

#include <stdbool.h>
#include <stdint.h>

#include <F021.h>

#define FLASH_BANK_WIDTH 16

typedef enum {
  kFlashUpdateTypeInvalid = -1,
  kFlashUpdateTypeBootloader,
  kFlashUpdateTypeApplication,
  kFlashUpdateTypeConfigParams,
  kFlashUpdateTypeCalibParams,
  kFlashUpdateTypeSerialParams,
  kFlashUpdateTypeCarrierSerialParams,
  kNumFlashUpdateTypes
} FlashUpdateType;

typedef enum {
  kFlashTypeForceSigned = -1,
  kFlashTypeInternal = 0,
  kFlashTypeCarrierEeprom = 1,
} FlashType;

typedef struct {
  FlashType type;
  Fapi_FlashBankType bank;
  uint32_t begin_address;
  uint32_t end_address;
} FlashSegment;

void FlashInit(FlashUpdateType update_type);
bool FlashEraseSectors(FlashUpdateType update_type);
uint32_t GetFlashWriteAddress(FlashUpdateType update_type, int32_t offset);
int32_t GetFlashAddressLength(FlashUpdateType update_type);
bool FlashWrite(FlashUpdateType update_type, int32_t offset, int32_t data_len,
                uint8_t *data);

#endif  // AVIONICS_BOOTLOADER_FIRMWARE_FLASH_H_
