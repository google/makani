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

#include "avionics/firmware/drivers/spi_flash.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/cpu/spi.h"
#include "common/macros.h"

#define SPI_BUS      4
#define SPI_CS       0x01
#define SPI_FREQ     80000000
#define SPI_BITS     8
#define SPI_POLARITY 0
#define SPI_PHASE    0
#define RECOVERY_BIT_CYCLES CLOCK32_USEC_TO_CYCLES(100)

// This driver communicates using the 3-byte addressing protocol because our
// device does not support the 4-byte addressing protocol for all commands.

typedef enum {
  // Independent states.
  kSpiFlashStateIdle,
  kSpiFlashStatePowerLossRecoverySequence,
  kSpiFlashStateClearFlagStatus,
  kSpiFlashStateReadExtendedAddress,
  kSpiFlashStateReadFlagStatus,
  kSpiFlashStateReadId,
  kSpiFlashStateReadNonvolatileConfig,
  kSpiFlashStateReadStatus,
  // Read enable -> set address -> verify address -> read loop.
  kSpiFlashStateReadEnable,
  kSpiFlashStateReadSetAddress,
  kSpiFlashStateReadVerifyAddress,
  kSpiFlashStateReadExecute,
  // Write enable -> set address -> verify address -> execute -> wait loop.
  kSpiFlashStateWriteEnable,
  kSpiFlashStateWriteSetAddress,
  kSpiFlashStateWriteVerifyAddress,
  kSpiFlashStateWriteExecute,
  kSpiFlashStateWriteWaitForFlagStatus,
  kSpiFlashStateWriteWaitForStatus,
} SpiFlashState;

// Status flags indicating validity for each piece of information.
typedef enum {
  kSpiFlashValidExtendedAddress   = 1 << 0,
  kSpiFlashValidFlagStatus        = 1 << 1,
  kSpiFlashValidId                = 1 << 2,
  kSpiFlashValidNonvolatileConfig = 1 << 3,
  kSpiFlashValidStatus            = 1 << 4,
} SpiFlashValid;

// SPI bus device index.
static int32_t g_device = -1;

// Cached state of device.
static struct {
  SpiFlashId id;
  uint8_t status;               // See SpiFlashStatus enum.
  uint8_t extended_address;     // Address extended beyond 3-bytes.
  uint8_t flag_status;          // See SpiFlashFlagStatus enum.
  int32_t flag_status_count;    // Read the flag status register for each die.
  uint16_t nonvolatile_config;  // See SpiFlashNonvolatileConfig enum.
  uint32_t valid;               // See SpiFlashValid enum.
} g_cache;

// State machine state.
static struct {
  SpiFlashState next;
  SpiFlashCommand command;
  bool first_entry;  // Set true when transitioning to a new state.
  bool repeat;       // Set to re-enter a state; automatically cleared.
  bool error;        // Set to abort current command and re-initialize.
  bool wrote_data;   // Used in page program and read operations.
  int32_t recover_seq;   // Recovery sequence index.
  int32_t recover_iter;  // Recovery iteration within current sequence.
  uint32_t timeout;  // Timeout in cycles, used for bit width timer.
} g_state;

// Transfer object.
static struct {
  uint8_t mosi[16];  // Command master-out-slave-in SPI data.
  uint8_t miso[16];  // Command master-in-slave-out SPI data.
  uint32_t addr;     // Chip address for program, erase, and read.
  int32_t length;    // Length of address for program, erase, and read.
  uint8_t *data_in;         // TMS570 memory location to store data to.
  const uint8_t *data_out;  // TMS570 memory location to load data from.
} g_xfer;


static uint16_t GetDesiredNonvolatileConfig(void) {
  return (g_cache.nonvolatile_config & kSpiFlashNonvolatileConfigReservedMask)
      | kSpiFlashNonvolatileConfig3ByteAddress
      | kSpiFlashNonvolatileConfigLower128Mb
      | kSpiFlashNonvolatileConfigDisableDualIo
      | kSpiFlashNonvolatileConfigDisableQuadIo
      | kSpiFlashNonvolatileConfigResetHold
      | kSpiFlashNonvolatileConfig30OhmDrive
      | kSpiFlashNonvolatileConfigDisableXip
      | kSpiFlashNonvolatileConfig15DummyCycles;
}

static bool IsFlagStatusValid(void) {
  return g_cache.valid & kSpiFlashValidFlagStatus;
}

static bool IsExtendedAddressValid(void) {
  return g_cache.valid & kSpiFlashValidExtendedAddress;
}

static bool IsIdValid(void) {
  return g_cache.valid & kSpiFlashValidId;
}

static bool IsNonvolatileConfigValid(void) {
  return g_cache.valid & kSpiFlashValidNonvolatileConfig;
}

static bool IsStatusValid(void) {
  return g_cache.valid & kSpiFlashValidStatus;
}

static bool IsFlagStatusReady(void) {
  return g_cache.flag_status & kSpiFlashFlagStatusProgramEraseReady;
}

static bool IsFlagStatusError(void) {
  uint32_t errors =
      kSpiFlashFlagStatusProtectionError
      | kSpiFlashFlagStatusVppError
      | kSpiFlashFlagStatusProgramError
      | kSpiFlashFlagStatusEraseError;
  return (g_cache.flag_status & errors) != 0x0;
}

static bool IsFlagStatus4ByteAddressing(void) {
  return g_cache.flag_status & kSpiFlashFlagStatus4ByteAddressing;
}

static bool IsStatusReady(void) {
  return !(g_cache.status & kSpiFlashStatusWriteInProgress);
}

static bool CompareExtendedAddressWithCache(uint32_t addr) {
  return (g_cache.valid & kSpiFlashValidExtendedAddress)
      && (g_cache.extended_address == addr >> 24);
}

static bool CompareNonvolatileConfigWithCache(void) {
  return g_cache.nonvolatile_config == GetDesiredNonvolatileConfig();
}

// The Read and Program commands operate on the current memory segment. This
// function determines the length remaining starting at 'addr' and going to
// the end of the segment, then saturates input 'length' accordingly.
static int32_t SaturateLength(uint32_t segment_size, uint32_t addr,
                              int32_t length) {
  uint32_t segment_start = addr & ~(segment_size - 1);
  uint32_t segment_end = segment_start + segment_size;
  uint32_t end_addr = addr + length;
  if (end_addr > segment_end) {
    end_addr = segment_end;
  }
  return end_addr - addr;
}

static bool SpiTransfer(bool cs_hold, int32_t length, const uint8_t *mosi,
                        uint8_t *miso) {
  return SpiTransferUint8Async(SPI_BUS, g_device, SPI_CS, cs_hold, 1, length,
                               mosi, miso);
}

static uint32_t ReadId(const uint8_t *miso) {
  ReadUint8Be(&miso[1], &g_cache.id.manufacture_id);
  ReadUint16Be(&miso[2], &g_cache.id.device_id);
  ReadUint16Be(&miso[5], &g_cache.id.extended_id);
  // Compute capacity in bytes.
  g_cache.id.capacity = (int32_t)(g_cache.id.device_id & 0xFF) * 2 * 1048576;
  // Pull-up causes transfer to return 0xFF for all bytes on a failed device.
  if (g_cache.id.capacity > 0 && g_cache.id.device_id != 0xFFFF) {
    g_cache.valid |= kSpiFlashValidId;
  } else {
    g_cache.valid &= ~kSpiFlashValidId;
  }
  return 0x0;
}

static uint32_t ReadStatus(const uint8_t *miso) {
  g_cache.status = miso[1];
  g_cache.valid |= kSpiFlashValidStatus;
  return g_cache.status;
}

static uint32_t ReadFlagStatus(const uint8_t *miso) {
  g_cache.flag_status = miso[1];
  // See note from "READ STATUS REGISTER or FLAG STATUS REGISTER Command."
  // Poll the flag status register once for each die following an operation.
  ++g_cache.flag_status_count;
  int32_t num_dies = g_cache.id.capacity / SPI_FLASH_DIE_SIZE;
  if (g_cache.flag_status_count >= num_dies) {
    g_cache.flag_status_count = num_dies;
    g_cache.valid |= kSpiFlashValidFlagStatus;
  } else {
    g_cache.valid &= ~kSpiFlashValidFlagStatus;
  }
  return g_cache.flag_status;
}

static uint32_t ReadExtendedAddress(const uint8_t *miso) {
  g_cache.extended_address = miso[1];
  g_cache.valid |= kSpiFlashValidExtendedAddress;
  return g_cache.extended_address;
}

static uint32_t ReadNonvolatileConfig(const uint8_t *miso) {
  ReadUint16Be(&miso[1], &g_cache.nonvolatile_config);
  g_cache.valid |= kSpiFlashValidNonvolatileConfig;
  return g_cache.nonvolatile_config;
}

// All CommandX functions return true on complete.

static bool CommandReadRegister(SpiFlashCommand command,
                                uint32_t (* const read_func)(const uint8_t *),
                                int32_t length) {
  if (g_state.first_entry || g_state.error) {
    g_xfer.mosi[0] = command;
    memset(&g_xfer.mosi[1], 0x0, length - 1);
    g_state.error = !SpiTransfer(false, length, g_xfer.mosi, g_xfer.miso);
    return false;
  }
  read_func(g_xfer.miso);
  return true;
}

static bool CommandReadId(void) {
  return CommandReadRegister(kSpiFlashCommandReadId, ReadId, 7);
}

static bool CommandReadStatus(void) {
  return CommandReadRegister(kSpiFlashCommandReadStatus, ReadStatus, 2);
}

static bool CommandReadFlagStatus(void) {
  return CommandReadRegister(kSpiFlashCommandReadFlagStatus, ReadFlagStatus, 2);
}

static bool CommandReadExtendedAddress(void) {
  return CommandReadRegister(kSpiFlashCommandReadExtendedAddress,
                             ReadExtendedAddress, 2);
}

static bool CommandReadNonvolatileConfig(void) {
  return CommandReadRegister(kSpiFlashCommandReadNonvolatileConfig,
                             ReadNonvolatileConfig, 3);
}

static bool CommandRead(void) {
  if (g_state.first_entry || g_state.error) {
    g_xfer.mosi[0] = kSpiFlashCommandRead;
    WriteUint24Be(g_xfer.addr, &g_xfer.mosi[1]);
    g_state.error = !SpiTransfer(true, 4, g_xfer.mosi, NULL);
    g_state.wrote_data = false;
  } else if (!g_state.wrote_data) {
    // Reads are limited to current die.
    int32_t length = SpiFlashSaturateAtDie(g_xfer.addr, g_xfer.length);
    g_state.error = !SpiTransfer(false, length, NULL, g_xfer.data_in);
    g_state.wrote_data = true;
    g_xfer.addr += length;
    g_xfer.length -= length;
    g_xfer.data_in += length;
  } else {
    // State machine needs to change the extended address register before
    // calling this command again.
    return true;
  }
  return false;
}

static bool Command1Byte(SpiFlashCommand command) {
  if (g_state.first_entry || g_state.error) {
    g_xfer.mosi[0] = command;
    g_state.error = !SpiTransfer(false, 1, g_xfer.mosi, NULL);
    return false;
  }
  return true;
}

static bool CommandWriteEnable(void) {
  return Command1Byte(kSpiFlashCommandWriteEnable);
}

static bool CommandClearFlagStatus(void) {
  return Command1Byte(kSpiFlashCommandClearFlagStatus);
}

static bool CommandExit4ByteAddressMode(void) {
  return Command1Byte(kSpiFlashCommandExit4ByteAddressMode);
}

static bool CommandPageProgram(void) {
  if (g_state.first_entry || g_state.error) {
    g_xfer.mosi[0] = kSpiFlashCommandPageProgram;
    WriteUint24Be(g_xfer.addr, &g_xfer.mosi[1]);
    g_state.error = !SpiTransfer(true, 4, g_xfer.mosi, NULL);
    g_state.wrote_data = false;
  } else if (!g_state.wrote_data) {
    // Writes are limited to current page.
    int32_t length = SpiFlashSaturateAtPage(g_xfer.addr, g_xfer.length);
    g_state.error = !SpiTransfer(false, length, g_xfer.data_out, NULL);
    g_state.wrote_data = true;
    g_xfer.addr += length;
    g_xfer.length -= length;
    g_xfer.data_out += length;
  } else {
    // State machine needs to change the extended address register and perform
    // a write enable before calling this command again.
    return true;
  }
  return false;
}

static bool CommandErase(SpiFlashCommand command, int32_t block_size) {
  if (g_state.first_entry || g_state.error) {
    // Align to sector or subsector boundary.
    g_xfer.length += g_xfer.addr & (block_size - 1);
    g_xfer.addr &= ~(block_size - 1);
    g_xfer.mosi[0] = command;
    WriteUint24Be(g_xfer.addr, &g_xfer.mosi[1]);
    g_state.error = !SpiTransfer(false, 4, g_xfer.mosi, NULL);
    return false;
  }

  // State machine needs to change the extended address register and perform
  // a write enable before calling this command again.
  g_xfer.addr += block_size;
  g_xfer.length -= block_size;
  return true;
}

static bool CommandWriteExtendedAddress(uint32_t addr) {
  if (g_state.first_entry || g_state.error) {
    g_xfer.mosi[0] = kSpiFlashCommandWriteExtendedAddress;
    g_xfer.mosi[1] = (uint8_t)(addr >> 24);
    g_state.error = !SpiTransfer(false, 2, g_xfer.mosi, NULL);
    return false;
  }
  return true;
}

static bool CommandWriteNonvolatileConfig(void) {
  if (g_state.first_entry || g_state.error) {
    g_xfer.mosi[0] = kSpiFlashCommandWriteNonvolatileConfig;
    WriteUint16Be(GetDesiredNonvolatileConfig(), &g_xfer.mosi[1]);
    g_state.error = !SpiTransfer(false, 3, g_xfer.mosi, NULL);
    return false;
  }
  return true;
}

static void SetupForBitBang(void) {
  SPI(SPI_BUS).PC1.CLKDIR = 1;
  SPI(SPI_BUS).PC3.CLKDOUT = 1;
  SPI(SPI_BUS).PC0.CLKFUN = 0;

  SPI(SPI_BUS).PC1.SCSDIR = 1;
  SPI(SPI_BUS).PC3.SCSDOUT = 1;
  SPI(SPI_BUS).PC0.SCSFUN = 0;

  SPI(SPI_BUS).PC1.SOMIDIR0 = 0;
  SPI(SPI_BUS).PC3.SOMIDOUT0 = 1;
  SPI(SPI_BUS).PC0.SOMIFUN0 = 0;

  SPI(SPI_BUS).PC1.SIMODIR0 = 1;
  SPI(SPI_BUS).PC3.SIMODOUT0 = 1;
  SPI(SPI_BUS).PC0.SIMOFUN0 = 0;
}

static void SetupForSpi(void) {
  SPI(SPI_BUS).PC0.CLKFUN = 1;
  SPI(SPI_BUS).PC0.SCSFUN = 1;
  SPI(SPI_BUS).PC0.SOMIFUN0 = 1;
  SPI(SPI_BUS).PC0.SIMOFUN0 = 1;
}

static void StatePowerLossRecoverySequence(SpiFlashState next) {
  // See n25q_512mb_1ce_3V_65nm (Rev. U 1/15 EN): Power Loss Recovery Sequence.
  const int32_t kLowPulses[] = {7, 9, 13, 17, 25, 33};
  if (g_state.first_entry) {
    SetupForBitBang();
    g_state.recover_seq = 0;
    g_state.recover_iter = -1;
  }

  if (g_state.first_entry || CLOCK32_LE(g_state.timeout, Clock32GetCycles())) {
    // Bit-bang the following sequence.
    // SCSDOUT -_______________-
    // CLKDOUT --_-_-_-_-_-_-_-- (n=7 low pulses, 16 iterations)
    // iter -1: SCSDOUT = 1, CLKDOUT = 1
    // iter  0: SCSDOUT = 0, CLKDOUT = 1
    // iter  1: SCSDOUT = 0, CLKDOUT = 0 (bit count = 1)
    // iter  2: SCSDOUT = 0, CLKDOUT = 1
    // iter  3: SCSDOUT = 0, CLKDOUT = 0 (bit count = 2)
    // iter  4: SCSDOUT = 0, CLKDOUT = 1
    // iter  5: SCSDOUT = 0, CLKDOUT = 0 (bit count = 3)
    // iter  6: SCSDOUT = 0, CLKDOUT = 1
    // iter  7: SCSDOUT = 0, CLKDOUT = 0 (bit count = 4)
    // iter  8: SCSDOUT = 0, CLKDOUT = 1
    // iter  9: SCSDOUT = 0, CLKDOUT = 0 (bit count = 5)
    // iter 10: SCSDOUT = 0, CLKDOUT = 1
    // iter 11: SCSDOUT = 0, CLKDOUT = 0 (bit count = 6)
    // iter 12: SCSDOUT = 0, CLKDOUT = 1
    // iter 13: SCSDOUT = 0, CLKDOUT = 0 (bit count = 7)
    // iter 14: SCSDOUT = 0, CLKDOUT = 1
    // iter 15: SCSDOUT = 1, CLKDOUT = 1

    int32_t two_n = 2 * kLowPulses[g_state.recover_seq];
    if (g_state.recover_iter < 0) {
      SPI(SPI_BUS).PC3.CLKDOUT = 1;
      SPI(SPI_BUS).PC3.SCSDOUT = 1;
    } else if (g_state.recover_iter == 0) {
      SPI(SPI_BUS).PC3.SCSDOUT = 0;
    } else if (g_state.recover_iter == two_n) {
      SPI(SPI_BUS).PC3.CLKDOUT = 1;
    } else if (g_state.recover_iter == two_n + 1) {
      SPI(SPI_BUS).PC3.SCSDOUT = 1;
    } else if (g_state.recover_iter > two_n + 1) {
      ++g_state.recover_seq;
      g_state.recover_iter = -1;
      if (g_state.recover_seq >= ARRAYSIZE(kLowPulses)) {
        SetupForSpi();
        g_state.next = next;
      }
    } else {
      SPI(SPI_BUS).PC3.CLKDOUT = (g_state.recover_iter % 2 == 0);
    }
    ++g_state.recover_iter;
    g_state.timeout = Clock32GetCycles() + RECOVERY_BIT_CYCLES;
  }
}

static void StateReadId(SpiFlashState failure, SpiFlashState success) {
  if (CommandReadId()) {
    if (g_cache.valid & kSpiFlashValidId) {
      g_state.next = success;
    } else {
      g_state.next = failure;
    }
  }
}

static void StateSimpleCommand(bool (* const func)(void), SpiFlashState next) {
  if (func()) {
    g_state.next = next;
  }
}

static void StateReadEnable(SpiFlashState set_address, SpiFlashState execute) {
  if (CompareExtendedAddressWithCache(g_xfer.addr)) {
    g_state.next = execute;
  } else if (CommandWriteEnable()) {
    g_state.next = set_address;
  }
}

static void StateSetAddress(SpiFlashState next) {
  if (CommandWriteExtendedAddress(g_xfer.addr)) {
    g_state.next = next;
  }
}

static void StateVerifyAddress(SpiFlashState next) {
  if (CommandReadExtendedAddress()) {
    if (CompareExtendedAddressWithCache(g_xfer.addr)) {
      g_state.next = next;
    } else {
      g_state.error = true;
    }
  }
}

static void StateReadExecute(SpiFlashState again, SpiFlashState done) {
  if (CommandRead()) {
    if (g_xfer.length > 0) {
      g_state.next = again;
    } else {
      g_state.next = done;
    }
  }
}

static void StateWriteEnable(SpiFlashState set_address, SpiFlashState execute) {
  if (CommandWriteEnable()) {
    if (CompareExtendedAddressWithCache(g_xfer.addr)) {
      g_state.next = execute;
    } else {
      g_state.next = set_address;
    }
  }
}

static void StateWriteExecute(SpiFlashState next) {
  bool complete = false;
  switch (g_state.command) {
    case kSpiFlashCommandPageProgram:
      complete = CommandPageProgram();
      break;
    case kSpiFlashCommandSectorErase:
      complete = CommandErase(g_state.command, SPI_FLASH_SECTOR_SIZE);
      break;
    case kSpiFlashCommandSubsectorErase:
      complete = CommandErase(g_state.command, SPI_FLASH_SUBSECTOR_SIZE);
      break;
    case kSpiFlashCommandExit4ByteAddressMode:
      complete = CommandExit4ByteAddressMode();
      break;
    case kSpiFlashCommandWriteNonvolatileConfig:
      complete = CommandWriteNonvolatileConfig();
      break;
    case kSpiFlashCommandWriteExtendedAddress:
      complete = CommandWriteExtendedAddress(g_xfer.addr);
      break;
    default:
      assert(false);
      g_state.error = true;  // Unsupported command.
      break;
  }
  if (complete) {
    // Poll these registers until write operation completes.
    g_cache.valid &= ~(kSpiFlashValidStatus | kSpiFlashValidFlagStatus);
    g_cache.flag_status_count = 0;
    g_state.next = next;
  }
}

static void StateWriteWaitForFlagStatus(SpiFlashState next) {
  if (CommandReadFlagStatus()) {
    if (IsFlagStatusValid() && IsFlagStatusReady()) {
      g_state.next = next;
    } else {
      g_state.repeat = true;
    }
  }
}

static void StateWriteWaitForStatus(SpiFlashState again, SpiFlashState done) {
  if (CommandReadStatus()) {
    if (!IsStatusValid() || !IsStatusReady()) {
      g_state.repeat = true;
    } else if (g_xfer.length > 0) {
      g_state.next = again;
    } else {
      g_state.next = done;
    }
  }
}

// This state machine only runs after the SPI bus returns to idle.
static bool StateProcess(void) {
  // Advance state machine, where the resulting g_state.next specifies the
  // state for the next iteration.
  SpiFlashState current = g_state.next;
  switch (current) {
    case kSpiFlashStatePowerLossRecoverySequence:
      StatePowerLossRecoverySequence(kSpiFlashStateIdle);
      break;
    case kSpiFlashStateClearFlagStatus:
      StateSimpleCommand(CommandClearFlagStatus, kSpiFlashStateIdle);
      break;
    case kSpiFlashStateReadId:
      StateReadId(kSpiFlashStatePowerLossRecoverySequence, kSpiFlashStateIdle);
      break;
    case kSpiFlashStateReadFlagStatus:
      StateSimpleCommand(CommandReadFlagStatus, kSpiFlashStateIdle);
      break;
    case kSpiFlashStateReadStatus:
      StateSimpleCommand(CommandReadStatus, kSpiFlashStateIdle);
      break;
    case kSpiFlashStateReadExtendedAddress:
      StateSimpleCommand(CommandReadExtendedAddress, kSpiFlashStateIdle);
      break;
    case kSpiFlashStateReadNonvolatileConfig:
      StateSimpleCommand(CommandReadNonvolatileConfig, kSpiFlashStateIdle);
      break;
    case kSpiFlashStateReadEnable:
      StateReadEnable(kSpiFlashStateReadSetAddress, kSpiFlashStateReadExecute);
      break;
    case kSpiFlashStateReadSetAddress:
      StateSetAddress(kSpiFlashStateReadVerifyAddress);
      break;
    case kSpiFlashStateReadVerifyAddress:
      StateVerifyAddress(kSpiFlashStateReadExecute);
      break;
    case kSpiFlashStateReadExecute:
      StateReadExecute(kSpiFlashStateReadEnable, kSpiFlashStateIdle);
      break;
    case kSpiFlashStateWriteEnable:
      StateWriteEnable(kSpiFlashStateWriteSetAddress,
                       kSpiFlashStateWriteExecute);
      break;
    case kSpiFlashStateWriteSetAddress:
      StateSetAddress(kSpiFlashStateWriteVerifyAddress);
      break;
    case kSpiFlashStateWriteVerifyAddress:
      StateVerifyAddress(kSpiFlashStateWriteEnable);
      break;
    case kSpiFlashStateWriteExecute:
      StateWriteExecute(kSpiFlashStateWriteWaitForFlagStatus);
      break;
    case kSpiFlashStateWriteWaitForFlagStatus:
      StateWriteWaitForFlagStatus(kSpiFlashStateWriteWaitForStatus);
      break;
    case kSpiFlashStateWriteWaitForStatus:
      StateWriteWaitForStatus(kSpiFlashStateWriteEnable, kSpiFlashStateIdle);
      break;
    case kSpiFlashStateIdle:
      break;
    default:
      g_state.next = kSpiFlashStateIdle;
      assert(false);
      break;
  }
  g_state.first_entry = (g_state.repeat || current != g_state.next);
  g_state.repeat = false;
  return g_state.next == kSpiFlashStateIdle;
}

static void ScheduleNow(SpiFlashState state, SpiFlashCommand command,
                        uint32_t addr, int32_t length,
                        const uint8_t *data_out, uint8_t *data_in) {
  g_state.next = state;
  g_state.command = command;
  g_state.first_entry = true;
  g_state.repeat = false;
  g_xfer.addr = addr;
  g_xfer.length = length;
  g_xfer.data_in = data_in;
  g_xfer.data_out = data_out;
}

static void ScheduleSimpleNow(SpiFlashState state, SpiFlashCommand command) {
  // Prevent the state machine from changing the extended address register
  // during write operations that do not require an address.
  uint32_t dummy_addr = (uint32_t)g_cache.extended_address << 24;
  ScheduleNow(state, command, dummy_addr, 0, NULL, NULL);
}

static bool ScheduleWhenIdle(SpiFlashState state, SpiFlashCommand command,
                             uint32_t addr, int32_t length,
                             const uint8_t *data_out, uint8_t *data_in) {
  if (SpiFlashIsIdle()) {
    ScheduleNow(state, command, addr, length, data_out, data_in);
    return true;
  }
  return false;
}

static bool HandleSpiError(void) {
  if (g_state.error) {
    g_state.next = kSpiFlashStatePowerLossRecoverySequence;
    g_state.first_entry = true;
    g_state.repeat = false;
    g_state.error = false;
    g_cache.valid = 0x0;
    g_cache.flag_status_count = 0;
    return false;
  }
  return true;
}

// This function executes after the state machine returns to idle.
static bool IsValid(void) {
  if (!IsIdValid()) {
    ScheduleSimpleNow(kSpiFlashStateReadId, kSpiFlashCommandReadId);
  } else if (!IsFlagStatusValid() || !IsFlagStatusReady()) {
    ScheduleSimpleNow(kSpiFlashStateReadFlagStatus,
                      kSpiFlashCommandReadFlagStatus);
  } else if (IsFlagStatusError()) {
    ScheduleSimpleNow(kSpiFlashStateClearFlagStatus,
                      kSpiFlashCommandClearFlagStatus);
  } else if (IsFlagStatus4ByteAddressing()) {
    ScheduleSimpleNow(kSpiFlashStateWriteEnable,
                      kSpiFlashCommandExit4ByteAddressMode);
  } else if (!IsStatusValid() || !IsStatusReady()) {
    ScheduleSimpleNow(kSpiFlashStateReadStatus, kSpiFlashCommandReadStatus);
  } else if (!IsExtendedAddressValid()) {
    ScheduleSimpleNow(kSpiFlashStateReadExtendedAddress,
                      kSpiFlashCommandReadExtendedAddress);
  } else if (!IsNonvolatileConfigValid()) {
    ScheduleSimpleNow(kSpiFlashStateReadNonvolatileConfig,
                      kSpiFlashCommandReadNonvolatileConfig);
  } else if (!CompareNonvolatileConfigWithCache()) {
    ScheduleSimpleNow(kSpiFlashStateWriteEnable,
                      kSpiFlashCommandWriteNonvolatileConfig);
  }
  return g_state.next == kSpiFlashStateIdle;
}

void SpiFlashInit(void) {
  // Initialize globals.
  memset(&g_cache, 0x0, sizeof(g_cache));
  memset(&g_state, 0x0, sizeof(g_state));
  memset(&g_xfer, 0x0, sizeof(g_xfer));
  g_state.next = kSpiFlashStateIdle;

  // Initialize SPI hardware.
  SpiInit(SPI_BUS, kSpiPinmuxAll);
  g_device = SpiAddDevice(SPI_BUS, SPI_CS, SPI_FREQ, 50, SPI_BITS, SPI_POLARITY,
                          SPI_PHASE);

  // Disable hardware write protect.
  SPI(SPI_BUS).PC0.ENAFUN = 0;
  SPI(SPI_BUS).PC1.ENADIR = 1;
  SPI(SPI_BUS).PC3.ENADOUT = 1;

  // Schedule initialization sequence.
  ScheduleSimpleNow(kSpiFlashStateReadFlagStatus,
                    kSpiFlashCommandReadFlagStatus);
}

bool SpiFlashPoll(void) {
  return SpiPoll(SPI_BUS)  // Wait for SPI operation.
      && HandleSpiError()  // Allow one sleep cycle on error.
      && StateProcess()    // Wait for state machine.
      && IsValid();        // Wait for valid configuration/cache.
}

bool SpiFlashIsIdle(void) {
  return (g_state.next == kSpiFlashStateIdle) || SpiFlashPoll();
}

bool SpiFlashReadAsync(uint32_t addr, int32_t length, uint8_t *data) {
  return ScheduleWhenIdle(kSpiFlashStateReadEnable, kSpiFlashCommandRead,
                          addr, length, NULL, data);
}

bool SpiFlashProgramAsync(uint32_t addr, int32_t length, const uint8_t *data) {
  return ScheduleWhenIdle(kSpiFlashStateWriteEnable,
                          kSpiFlashCommandPageProgram,
                          addr, length, data, NULL);
}

bool SpiFlashEraseSectorAsync(uint32_t addr, int32_t length) {
  return ScheduleWhenIdle(kSpiFlashStateWriteEnable,
                          kSpiFlashCommandSectorErase,
                          addr, length, NULL, NULL);
}

bool SpiFlashEraseSubsectorAsync(uint32_t addr, int32_t length) {
  return ScheduleWhenIdle(kSpiFlashStateWriteEnable,
                          kSpiFlashCommandSubsectorErase,
                          addr, length, NULL, NULL);
}

bool SpiFlashSetAddressAsync(uint32_t addr) {
  return ScheduleWhenIdle(kSpiFlashStateWriteEnable,
                          kSpiFlashCommandWriteExtendedAddress,
                          addr, 0, NULL, NULL);
}

const SpiFlashId *SpiFlashGetId(void) {
  return (g_cache.valid & kSpiFlashValidId) ? &g_cache.id : NULL;
}

int32_t SpiFlashGetCapacity(void) {
  return g_cache.id.capacity;
}

int32_t SpiFlashGetDies(void) {
  return g_cache.id.capacity / SPI_FLASH_DIE_SIZE;
}

int32_t SpiFlashGetPages(void) {
  return g_cache.id.capacity / SPI_FLASH_PAGE_SIZE;
}

int32_t SpiFlashGetSectors(void) {
  return g_cache.id.capacity / SPI_FLASH_SECTOR_SIZE;
}

int32_t SpiFlashGetSubsectors(void) {
  return g_cache.id.capacity / SPI_FLASH_SUBSECTOR_SIZE;
}

uint32_t SpiFlashDieToAddress(uint32_t die) {
  return die * SPI_FLASH_DIE_SIZE;
}

uint32_t SpiFlashPageToAddress(uint32_t page) {
  return page * SPI_FLASH_PAGE_SIZE;
}

uint32_t SpiFlashSectorToAddress(uint32_t sector) {
  return sector * SPI_FLASH_SECTOR_SIZE;
}

uint32_t SpiFlashSectorToSubsector(uint32_t sector) {
  return sector * (SPI_FLASH_SECTOR_SIZE / SPI_FLASH_SUBSECTOR_SIZE);
}

uint32_t SpiFlashSubsectorToAddress(uint32_t subsector) {
  return subsector * SPI_FLASH_SUBSECTOR_SIZE;
}

int32_t SpiFlashSaturateAtDie(uint32_t addr, int32_t length) {
  return SaturateLength(SPI_FLASH_DIE_SIZE, addr, length);
}

int32_t SpiFlashSaturateAtPage(uint32_t addr, int32_t length) {
  return SaturateLength(SPI_FLASH_PAGE_SIZE, addr, length);
}

int32_t SpiFlashSaturateAtSector(uint32_t addr, int32_t length) {
  return SaturateLength(SPI_FLASH_SECTOR_SIZE, addr, length);
}

int32_t SpiFlashSaturateAtSubsector(uint32_t addr, int32_t length) {
  return SaturateLength(SPI_FLASH_SUBSECTOR_SIZE, addr, length);
}

void SpiFlashPrintData(uint32_t addr, int32_t length, const uint8_t *data) {
  for (int32_t i = 0; i < length; ++i) {
    uint32_t offset = (uint32_t)i;
    if (i % 16 == 0) {
      if (i != 0) {
        printf("\n");
      }
      printf("0x%04lX:", addr + offset);
    }
    if (i % 4 == 0) {
      printf(" ");
    }
    printf("%02X", data[i]);
  }
  printf("\n");
}

// Blocking functions for testing. Do not use in flight code.

void SpiFlashRead(uint32_t addr, int32_t length, uint8_t *data) {
  while (!SpiFlashReadAsync(addr, length, data)) {}
  while (!SpiFlashPoll()) {}
}

void SpiFlashProgram(uint32_t addr, int32_t length, const uint8_t *data) {
  while (!SpiFlashProgramAsync(addr, length, data)) {}
  while (!SpiFlashPoll()) {}
}

void SpiFlashEraseSector(uint32_t addr, int32_t length) {
  while (!SpiFlashEraseSectorAsync(addr, length)) {}
  while (!SpiFlashPoll()) {}
}

void SpiFlashEraseSubsector(uint32_t addr, int32_t length) {
  while (!SpiFlashEraseSubsectorAsync(addr, length)) {}
  while (!SpiFlashPoll()) {}
}

void SpiFlashSetAddress(uint32_t addr) {
  while (!SpiFlashSetAddressAsync(addr)) {}
  while (!SpiFlashPoll()) {}
}
