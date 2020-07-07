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

#include "avionics/bootloader/firmware/internal_flash.h"

#include <assert.h>
#include <stdio.h>

#include "avionics/bootloader/firmware/flash.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/startup/flash_tms570.h"

static bool AddressToSector(const Fapi_FlashBankSectorsType *info,
                            uint32_t address, uint32_t *sector) {
  uint32_t a = info->u32BankStartAddress;
  for (*sector = 0; *sector < info->u32NumberOfSectors && a < address;
       ++(*sector)) {
    a += info->au16SectorSizes[*sector] * 1024UL;  // Scale KBytes to Bytes.
  }
  return a == address;
}

static bool GetSectorRange(const FlashSegment *segment,
                           Fapi_FlashBankSectorsType *info,
                           uint32_t *start_sector, uint32_t *end_sector) {
  assert(segment != NULL);
  assert(info != NULL);
  assert(start_sector != NULL && end_sector != NULL);
  Fapi_StatusType status = Fapi_getBankSectors(segment->bank, info);
  if (status != Fapi_Status_Success) {
    printf("Failed to get sector info for bank %d (status=%d).\n",
           segment->bank, status);
    return false;
  }
  return AddressToSector(info, segment->begin_address, start_sector)
      && AddressToSector(info, segment->end_address, end_sector)
      && *start_sector < *end_sector;
}

static void WaitForFlashOperation(void) {
  while (FAPI_CHECK_FSM_READY_BUSY == Fapi_Status_FsmBusy) {
    ExtWatchdogPoll();
  }
}

static bool EraseSector(uint32_t address, uint32_t sector, int32_t length) {
  WaitForFlashOperation();
  Fapi_StatusType status =
      Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector, (uint32_t *)address);
  if (status != Fapi_Status_Success) {
    Fapi_flushPipeline();
    printf("Failed to initiate erase for sector %lu with status %d.\n",
           sector, status);
    return false;
  }

  WaitForFlashOperation();
  uint32_t reg = FAPI_GET_FSM_STATUS;
  if (reg) {
    Fapi_flushPipeline();
    printf("Failed to erase sector %lu with status 0x%04lX.\n", sector, reg);
    return false;
  }

  Fapi_FlashStatusWordType status_word;
  status = Fapi_doBlankCheck((uint32_t *)address, length / 4 /* words */,
                             &status_word);
  if (status != Fapi_Status_Success) {
    Fapi_flushPipeline();
    uint32_t *word = &status_word.au32StatusWord[0];
    printf("Erase verification failed for address 0x%08lX with status %d,"
           " status_word 0x%08lX 0x%08lX 0x%08lX 0x%08lX.\n", address,
           status, word[0], word[1], word[2], word[3]);
    return false;
  }
  return true;
}

static void EraseSectors(const Fapi_FlashBankSectorsType *info,
                         uint32_t start_address, uint32_t start_sector,
                         uint32_t end_sector) {
  for (uint32_t sector = start_sector, address = start_address;
       sector < end_sector; ++sector) {
    // Field au16SectorSizes[] represents size in KBytes.
    int32_t sector_size = info->au16SectorSizes[sector] * 1024UL;
    EraseSector(address, sector, sector_size);
    address += sector_size;
  }
  Fapi_flushPipeline();
}

static bool WriteOneChunk(uint32_t *address, uint8_t *data, int32_t len) {
  assert(len > 0);
  WaitForFlashOperation();
  uint32_t status = Fapi_issueProgrammingCommand(
      address, data, (uint8_t)len, NULL, 0U, Fapi_AutoEccGeneration);
  if (status != Fapi_Status_Success) {
    Fapi_flushPipeline();
    printf("Failed to initiate write for address %p.\n", address);
    return false;
  }

  WaitForFlashOperation();
  uint32_t reg = FAPI_GET_FSM_STATUS;
  if (reg) {
    Fapi_flushPipeline();
    printf("Failed to write address %p with status 0x%04lX.\n", address, reg);
    return false;
  }

  Fapi_FlashStatusWordType status_word;
  status = Fapi_doVerifyByByte((uint8_t *)address, len, data, &status_word);
  if (status != Fapi_Status_Success) {
    Fapi_flushPipeline();
    printf("Write verification failed for address %p.\n", address);
    return false;
  }
  return true;
}

static bool WriteData(const FlashSegment *seg, int32_t offset, int32_t data_len,
                      uint8_t *data) {
  bool success = true;
  uint32_t address = seg->begin_address + offset;
  int32_t count;
  for (int32_t bytes_written = 0; bytes_written < data_len && success;
       bytes_written += count, address += count, data += count) {
    count = data_len - bytes_written;
    if (count > FLASH_BANK_WIDTH) {
      count = FLASH_BANK_WIDTH;
    }
    success = WriteOneChunk((uint32_t *)address, data, count);
  }
  Fapi_flushPipeline();
  return success;
}

static bool ValidateWriteRange(const FlashSegment *segment, int32_t offset,
                               int32_t length) {
  uint32_t start = segment->begin_address + offset;
  uint32_t end = start + length;
  if (!(start < end && end <= segment->end_address && offset >= 0
        && (start & (FLASH_BANK_WIDTH - 1)) == 0
        && (end & (FLASH_BANK_WIDTH - 1)) == 0)) {
    printf("Invalid write range begin=0x%08lX offset=0x%04lX, length=0x%04lX\n",
           segment->begin_address, offset, length);
    return false;
  }
  return true;
}

void InternalFlashInit(const FlashSegment *segment) {
  assert(segment != NULL);

  Fapi_issueAsyncCommand(Fapi_ClearStatus);
  Fapi_initializeFlashBanks(100);
  Fapi_setActiveFlashBank(segment->bank);
  Fapi_enableMainBankSectors(0xFFFF);
}

bool InternalFlashEraseSectors(const FlashSegment *segment) {
  assert(segment != NULL);

  Fapi_FlashBankSectorsType info;
  uint32_t start, end;
  if (GetSectorRange(segment, &info, &start, &end)) {
    // Disable flash ECC protection while erasing to avoid ESM errors.
    StartupFlashDisableEcc();
    EraseSectors(&info, segment->begin_address, start, end);
    StartupFlashEnableEcc();
    return true;
  }
  return false;
}

bool InternalFlashWrite(const FlashSegment *segment, int32_t offset,
                        int32_t data_len, uint8_t *data) {
  assert(segment != NULL);
  if (ValidateWriteRange(segment, offset, data_len)) {
    // Disable flash ECC protection while writing to avoid ESM errors.
    StartupFlashDisableEcc();
    bool success = WriteData(segment, offset, data_len, data);
    StartupFlashEnableEcc();
    return success;
  }
  return false;
}

// This function is required by the F021 API. It's called during Blank, Read,
// and Verify functions.
Fapi_StatusType Fapi_serviceWatchdogTimer(void) {
  ExtWatchdogPoll();
  return Fapi_Status_Success;
}
