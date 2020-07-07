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

#include "avionics/bootloader/firmware/flash.h"

#include <assert.h>
#include <stdio.h>

#include "avionics/bootloader/firmware/eeprom_flash.h"
#include "avionics/bootloader/firmware/internal_flash.h"
#include "avionics/firmware/startup/ldscript.h"
#include "common/macros.h"

// Define shorthand macros to avoid lint errors.
#define BOOT_BEGIN   ((uint32_t)&ldscript_boot_flash_begin)
#define BOOT_END     ((int32_t)&ldscript_boot_flash_end)
#define APP_BEGIN    ((uint32_t)&ldscript_app_flash_begin)
#define APP_END      ((int32_t)&ldscript_app_flash_end)
#define CONFIG_BEGIN ((uint32_t)&ldscript_config_param_begin)
#define CONFIG_END   ((int32_t)&ldscript_config_param_end)
#define CALIB_BEGIN  ((uint32_t)&ldscript_calib_param_begin)
#define CALIB_END    ((int32_t)&ldscript_calib_param_end)
#define SERIAL_BEGIN ((uint32_t)&ldscript_serial_param_begin)
#define SERIAL_END   ((int32_t)&ldscript_serial_param_end)

static const FlashSegment kFlashSegment[kNumFlashUpdateTypes] = {
  [kFlashUpdateTypeBootloader] = {
    kFlashTypeInternal, Fapi_FlashBank0, BOOT_BEGIN, BOOT_END,
  },
  [kFlashUpdateTypeApplication] = {
    kFlashTypeInternal, Fapi_FlashBank0, APP_BEGIN, APP_END,
  },
  [kFlashUpdateTypeConfigParams] = {
    kFlashTypeInternal, Fapi_FlashBank7, CONFIG_BEGIN, CONFIG_END,
  },
  [kFlashUpdateTypeCalibParams] = {
    kFlashTypeInternal, Fapi_FlashBank7, CALIB_BEGIN, CALIB_END,
  },
  [kFlashUpdateTypeSerialParams] = {
    kFlashTypeInternal, Fapi_FlashBank7, SERIAL_BEGIN, SERIAL_END,
  },
  [kFlashUpdateTypeCarrierSerialParams] = {
    kFlashTypeCarrierEeprom, 0, 0x0, 2048,
  },
};

static const FlashSegment *GetFlashSegment(FlashUpdateType update_type) {
  if (update_type < 0 || ARRAYSIZE(kFlashSegment) <= update_type) {
    printf("Invalid update type.\n");
    return NULL;
  }

  const FlashSegment *segment = &kFlashSegment[update_type];
  if (segment->begin_address >= segment->end_address) {
    printf("Invalid flash segment.\n");
    return NULL;
  }
  return segment;
}

void FlashInit(FlashUpdateType update_type) {
  const FlashSegment *segment = GetFlashSegment(update_type);
  if (segment == NULL) {
    return;
  }

  if (segment->type == kFlashTypeInternal) {
    InternalFlashInit(segment);
  } else if (segment->type == kFlashTypeCarrierEeprom) {
    EepromFlashInit(segment);
  }
}

uint32_t GetFlashWriteAddress(FlashUpdateType update_type, int32_t offset) {
  const FlashSegment *segment = GetFlashSegment(update_type);
  if (segment != NULL) {
    return segment->begin_address + offset;
  }
  return 0x80000000;  // Return used EMIF memory space address (does not exist).
}

int32_t GetFlashAddressLength(FlashUpdateType update_type) {
  const FlashSegment *segment = GetFlashSegment(update_type);
  if (segment != NULL) {
    return (int32_t)(segment->end_address - segment->begin_address);
  }
  return 0;
}

bool FlashEraseSectors(FlashUpdateType update_type) {
  const FlashSegment *segment = GetFlashSegment(update_type);

  if (segment == NULL) {
    return false;
  }

  if (segment->type == kFlashTypeInternal) {
    return InternalFlashEraseSectors(segment);
  } else if (segment->type == kFlashTypeCarrierEeprom) {
    return EepromFlashEraseSectors(segment);
  }

  return false;
}

bool FlashWrite(FlashUpdateType update_type, int32_t offset, int32_t data_len,
                uint8_t *data) {
  const FlashSegment *segment = GetFlashSegment(update_type);

  if (segment == NULL) {
    return false;
  }

  if (segment->type == kFlashTypeInternal) {
    return InternalFlashWrite(segment, offset, data_len, data);
  } else if (segment->type == kFlashTypeCarrierEeprom) {
    return EepromFlashWrite(segment, offset, data_len, data);
  }

  return false;
}
