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

#include "avionics/firmware/drivers/eeprom24.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/cpu/memcpy.h"
#include "common/macros.h"

#define EEPROM24_PAGE_SIZE 16

// 24LC16 datasheet indicates 5 ms write cycle time (byte or page).
// http://ww1.microchip.com/downloads/en/devicedoc/21703j.pdf
#define EEPROM24_WRITE_TIMEOUT_CYCLES CLOCK32_MSEC_TO_CYCLES(10)

typedef enum {
  kDeviceStateIdle,
  kDeviceStateFailure,
  kDeviceStateRead,
  kDeviceStateWrite,
  kDeviceStateWriteBusy
} DeviceState;

static struct {
  DeviceState state;
  bool first_entry;
  int32_t addr;
  int32_t length;
  uint8_t *read_ptr;
  const uint8_t *write_ptr;
  uint32_t timeout;  // [cycles]
} g_device;

// Length = address byte + maximum write length.
static uint8_t g_write[1 + EEPROM24_PAGE_SIZE];
static bool g_eeprom24_initialized = false;

// Convert the on chip memory address to an I2C address using the upper 3 bits
// of the 11 bit memory address.
static uint8_t GetI2cAddress(int32_t addr) {
  assert(0 <= addr && addr <= 0x07FF);
  return (uint8_t)(0x50 | ((addr >> 8) & 0x07));
}

static int32_t GetWriteLength(int32_t length) {
  return (length > EEPROM24_PAGE_SIZE) ? EEPROM24_PAGE_SIZE : length;
}

static bool InitTransfer(DeviceState state, int32_t addr, int32_t length,
                         const uint8_t *write_ptr, uint8_t *read_ptr) {
  if (g_device.state == kDeviceStateIdle
      || g_device.state == kDeviceStateFailure) {
    g_device.state = state;
    g_device.first_entry = true;
    g_device.addr = addr;
    g_device.length = length;
    g_device.write_ptr = write_ptr;
    g_device.read_ptr = read_ptr;
    return true;
  }
  return false;
}

static void StateRead(void) {
  if (g_device.first_entry && g_device.length > 0) {
    // Perform a "random read" operation by specifying the address to read from.
    g_write[0] = (uint8_t)(g_device.addr & 0xFF);
    if (!I2cWriteReadAsync(GetI2cAddress(g_device.addr), 1, g_write,
                           g_device.length, g_device.read_ptr)) {
      g_device.state = kDeviceStateFailure;
    }
  } else {
    g_device.state = kDeviceStateIdle;
  }
}

static void StateWrite(void) {
  g_device.state = kDeviceStateIdle;
  if (g_device.length > 0) {
    // Limit write length to one page (16 bytes).
    int32_t length = GetWriteLength(g_device.length);
    assert(0 < length && length < ARRAYSIZE(g_write));
    // Write address + data.
    g_write[0] = (uint8_t)(g_device.addr & 0xFF);
    WordCopy(length, g_device.write_ptr, &g_write[1]);
    if (I2cWriteAsync(GetI2cAddress(g_device.addr), 1 + length, g_write)) {
      // Increment position within transfer.
      g_device.addr += length;
      g_device.length -= length;
      g_device.write_ptr += length;
      g_device.state = kDeviceStateWriteBusy;
    } else {
      g_device.state = kDeviceStateFailure;
    }
  }
}

static void StateWriteBusy(bool i2c_error) {
  // This device does not respond while a write operation is in process. Poll
  // the device by attempting to read 1 byte from the current address until
  // we receive an I2C acknowledgment (i2c_error is false).
  if (g_device.first_entry) {
    g_device.timeout = Clock32GetCycles() + EEPROM24_WRITE_TIMEOUT_CYCLES;
  }

  if (g_device.first_entry ||i2c_error) {
    if (CLOCK32_GE(Clock32GetCycles(), g_device.timeout)
        || !I2cReadAsync(GetI2cAddress(0), 1, g_write)) {
      g_device.state = kDeviceStateFailure;
    }
  } else {
    g_device.state = kDeviceStateWrite;
  }
}

// This state machine only runs after the I2C bus returns to idle.
static bool StateProcess(bool i2c_error) {
  DeviceState current = g_device.state;

  // Device does not respond while a write operation is in progress.
  if (i2c_error && g_device.state != kDeviceStateWriteBusy) {
    g_device.state = kDeviceStateFailure;
  }
  // Advance state machine, where the resulting g_device.state specifies the
  // state for the next iteration.
  switch (g_device.state) {
    case kDeviceStateRead:
      StateRead();
      break;
    case kDeviceStateWrite:
      StateWrite();
      break;
    case kDeviceStateWriteBusy:
      StateWriteBusy(i2c_error);
      break;
    case kDeviceStateIdle:
      break;
    case kDeviceStateFailure:
      break;
    default:
      g_device.state = kDeviceStateFailure;
      assert(false);
      break;
  }

  g_device.first_entry = (current != g_device.state);

  return g_device.state == kDeviceStateIdle;
}

void Eeprom24Init(void) {
  if (g_eeprom24_initialized) {
    return;
  }

  I2cInit(400e3);

  memset(&g_device, 0, sizeof(g_device));
  g_device.state = kDeviceStateIdle;
  g_device.first_entry = true;

  g_eeprom24_initialized = true;
}

bool Eeprom24Poll(void) {
  bool i2c_error;
  return I2cPoll(&i2c_error) && StateProcess(i2c_error);
}

bool Eeprom24HasError(void) {
  return g_device.state == kDeviceStateFailure;
}

bool Eeprom24ReadAsync(int32_t addr, int32_t length, uint8_t *data) {
  assert(addr >= 0 && length > 0 && data != NULL);
  return InitTransfer(kDeviceStateRead, addr, length, NULL, data);
}

bool Eeprom24WriteAsync(int32_t addr, int32_t length, const uint8_t *data) {
  assert(addr >= 0 && length > 0 && data != NULL);
  return InitTransfer(kDeviceStateWrite, addr, length, data, NULL);
}

bool Eeprom24ReadSync(int32_t addr, int32_t data_len, uint8_t *data) {
  while (!Eeprom24Poll() && !Eeprom24HasError()) {}

  if (!Eeprom24ReadAsync(addr, data_len, data)) {
    return false;
  }

  while (!Eeprom24Poll() && !Eeprom24HasError()) {}

  return !Eeprom24HasError();
}

bool Eeprom24WriteSync(int32_t addr, int32_t data_len, const uint8_t *data) {
  while (!Eeprom24Poll() && !Eeprom24HasError()) {}

  if (!Eeprom24WriteAsync(addr, data_len, data)) {
    return false;
  }

  while (!Eeprom24Poll() && !Eeprom24HasError()) {}

  return !Eeprom24HasError();
}
