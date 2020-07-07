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

#ifndef AVIONICS_FIRMWARE_DRIVERS_SPI_FLASH_H_
#define AVIONICS_FIRMWARE_DRIVERS_SPI_FLASH_H_

// This module implements a driver for Micron's N25Q512A13GSF40G NOR SPI
// flash chip, though should be generic to all SPI flash chips. See
// http://www.micron.com/products/nor-flash/serial-nor-flash/512Mb#/
// for more information.

#include <stdbool.h>
#include <stdint.h>

// Size in bytes.
#define SPI_FLASH_DIE_SIZE       (256 * 1024 * 1024 / 8)
#define SPI_FLASH_PAGE_SIZE      256
#define SPI_FLASH_SUBSECTOR_SIZE 4096
#define SPI_FLASH_SECTOR_SIZE    (SPI_FLASH_SUBSECTOR_SIZE * 16)

// Value of erased data.
#define SPI_FLASH_ERASE_VALUE 0xFF

typedef enum {
  // Reset operations.
  kSpiFlashCommandResetEnable = 0x66,
  kSpiFlashCommandResetMemory = 0x99,
  // Identification operations.
  kSpiFlashCommandReadId = 0x9E,
  // Read operations.
  kSpiFlashCommandRead = 0x03,
  // Write operations.
  kSpiFlashCommandWriteEnable  = 0x06,
  kSpiFlashCommandWriteDisable = 0x04,
  // Register operations.
  kSpiFlashCommandReadStatus                  = 0x05,
  kSpiFlashCommandWriteStatus                 = 0x01,
  kSpiFlashCommandReadLock                    = 0xE8,
  kSpiFlashCommandWriteLock                   = 0xE5,
  kSpiFlashCommandReadFlagStatus              = 0x70,
  kSpiFlashCommandClearFlagStatus             = 0x50,
  kSpiFlashCommandReadNonvolatileConfig       = 0xB5,
  kSpiFlashCommandWriteNonvolatileConfig      = 0xB1,
  kSpiFlashCommandReadVolatileConfig          = 0x85,
  kSpiFlashCommandWriteVolatileConfig         = 0x81,
  kSpiFlashCommandReadEnhancedVolatileConfig  = 0x65,
  kSpiFlashCommandWriteEnhancedVolatileConfig = 0x61,
  kSpiFlashCommandReadExtendedAddress         = 0xC8,
  kSpiFlashCommandWriteExtendedAddress        = 0xC5,
  // Program/erase operations.
  kSpiFlashCommandPageProgram    = 0x02,
  kSpiFlashCommandSubsectorErase = 0x20,
  kSpiFlashCommandSectorErase    = 0xD8,
  kSpiFlashCommandDieErase       = 0xC4,
  // OTP operations.
  kSpiFlashCommandReadOtpArray    = 0x4B,
  kSpiFlashCommandProgramOtpArray = 0x42,
  // 4-byte address mode operations.
  kSpiFlashCommandEnter4ByteAddressMode = 0xB7,
  kSpiFlashCommandExit4ByteAddressMode  = 0xE9,
} SpiFlashCommand;

// Status register bits.
typedef enum {
  kSpiFlashStatusWriteInProgress  = 1 << 0,
  kSpiFlashStatusWriteEnableLatch = 1 << 1,
  kSpiFlashStatusBlockProtect0    = 1 << 2,
  kSpiFlashStatusBlockProtect1    = 1 << 3,
  kSpiFlashStatusBlockProtect2    = 1 << 4,
  kSpiFlashStatusBottom           = 1 << 5,
  kSpiFlashStatusBlockProtect3    = 1 << 6,
  kSpiFlashStatusWriteDisabled    = 1 << 7
} SpiFlashStatus;

// Flag Status register bits.
typedef enum {
  kSpiFlashFlagStatus4ByteAddressing   = 1 << 0,
  kSpiFlashFlagStatusProtectionError   = 1 << 1,
  kSpiFlashFlagStatusProgramSuspended  = 1 << 2,
  kSpiFlashFlagStatusVppError          = 1 << 3,
  kSpiFlashFlagStatusProgramError      = 1 << 4,
  kSpiFlashFlagStatusEraseError        = 1 << 5,
  kSpiFlashFlagStatusEraseSuspended    = 1 << 6,
  kSpiFlashFlagStatusProgramEraseReady = 1 << 7
} SpiFlashFlagStatus;

// Nonvolatile configuration register bits.
typedef enum {
  kSpiFlashNonvolatileConfig3ByteAddress  = 1 << 0,
  kSpiFlashNonvolatileConfigLower128Mb    = 1 << 1,
  kSpiFlashNonvolatileConfigDisableDualIo = 1 << 2,
  kSpiFlashNonvolatileConfigDisableQuadIo = 1 << 3,
  kSpiFlashNonvolatileConfigResetHold     = 1 << 4,
  kSpiFlashNonvolatileConfigReservedMask  = 1 << 5,
  kSpiFlashNonvolatileConfig30OhmDrive    = 0x07 << 6,   // 3 bit field.
  kSpiFlashNonvolatileConfigDisableXip    = 0x07 << 9,   // 3 bit field.
  kSpiFlashNonvolatileConfig15DummyCycles = 0x00 << 12,  // 4 bit field.
} SpiFlashNonvolatileConfig;

typedef struct {
  uint8_t manufacture_id;
  uint16_t device_id;
  uint16_t extended_id;
  int32_t capacity;  // [bytes]
} SpiFlashId;

void SpiFlashInit(void);
bool SpiFlashPoll(void);
bool SpiFlashIsIdle(void);
bool SpiFlashReadAsync(uint32_t addr, int32_t length, uint8_t *data);
bool SpiFlashProgramAsync(uint32_t addr, int32_t length, const uint8_t *data);
bool SpiFlashEraseSectorAsync(uint32_t addr, int32_t length);
bool SpiFlashEraseSubsectorAsync(uint32_t addr, int32_t length);
bool SpiFlashSetAddressAsync(uint32_t addr);

const SpiFlashId *SpiFlashGetId(void);
int32_t SpiFlashGetCapacity(void);
int32_t SpiFlashGetDies(void);
int32_t SpiFlashGetPages(void);
int32_t SpiFlashGetSectors(void);
int32_t SpiFlashGetSubsectors(void);

uint32_t SpiFlashDieToAddress(uint32_t die);
uint32_t SpiFlashPageToAddress(uint32_t page);
uint32_t SpiFlashSectorToAddress(uint32_t sector);
uint32_t SpiFlashSectorToSubsector(uint32_t sector);
uint32_t SpiFlashSubsectorToAddress(uint32_t subsector);

int32_t SpiFlashSaturateAtDie(uint32_t addr, int32_t length);
int32_t SpiFlashSaturateAtPage(uint32_t addr, int32_t length);
int32_t SpiFlashSaturateAtSector(uint32_t addr, int32_t length);
int32_t SpiFlashSaturateAtSubsector(uint32_t addr, int32_t length);

void SpiFlashPrintData(uint32_t addr, int32_t length, const uint8_t *data);

// Blocking functions for testing. Do not use in flight code.
void SpiFlashRead(uint32_t addr, int32_t length, uint8_t *data);
void SpiFlashProgram(uint32_t addr, int32_t length, const uint8_t *data);
void SpiFlashEraseSector(uint32_t addr, int32_t length);
void SpiFlashEraseSubsector(uint32_t addr, int32_t length);
void SpiFlashSetAddress(uint32_t addr);

#endif  // AVIONICS_FIRMWARE_DRIVERS_SPI_FLASH_H_
