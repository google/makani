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

#ifndef AVIONICS_BOOTLOADER_FIRMWARE_INTERNAL_FLASH_H_
#define AVIONICS_BOOTLOADER_FIRMWARE_INTERNAL_FLASH_H_

#include "avionics/bootloader/firmware/flash.h"

void InternalFlashInit(const FlashSegment *segment);
bool InternalFlashEraseSectors(const FlashSegment *segment);
bool InternalFlashWrite(const FlashSegment *segment, int32_t offset,
                        int32_t data_len, uint8_t *data);

#endif  // AVIONICS_BOOTLOADER_FIRMWARE_INTERNAL_FLASH_H_
