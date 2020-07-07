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

#include "avionics/firmware/drivers/spi_flash_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/firmware/drivers/spi_flash.h"
#include "avionics/firmware/test/test.h"
#include "common/macros.h"

static void TestSetup(void) {
  SpiFlashInit();

  // Wait for initialization to complete (required for device identification).
  while (!SpiFlashPoll()) {}
}

static void TestTeardown(void) {
}

static void TestGetId(void) {
  const SpiFlashId *id = SpiFlashGetId();
  ASSERT_NE(id, NULL);
  EXPECT_EQ(id->manufacture_id, 0x20);
  EXPECT_EQ(id->device_id, 0xBA20);
  EXPECT_EQ(id->extended_id, 0);
  EXPECT_EQ(id->capacity, 64 * 1024 * 1024);
}

static bool IsErased(uint32_t addr, int32_t length, uint8_t *data) {
  SpiFlashRead(addr, length, data);
  for (int32_t i = 0; i < length; ++i) {
    if (data[i] != SPI_FLASH_ERASE_VALUE) {
      return false;
    }
  }
  return true;
}

static void TestEraseSubsector(uint32_t addr, int32_t length) {
  uint8_t data[SPI_FLASH_SUBSECTOR_SIZE];

  for (int32_t offset = 0; offset < length; offset += ARRAYSIZE(data)) {
    uint32_t seg_addr = addr + offset;

    // Corrupt memory segment.
    for (int32_t i = 0; i < ARRAYSIZE(data); ++i) {
      data[i] = i;
    }
    SpiFlashProgram(seg_addr, ARRAYSIZE(data), data);

    // Verify corrupted memory segment.
    EXPECT_FALSE(IsErased(seg_addr, ARRAYSIZE(data), data));
  }

  // Erase memory segment.
  SpiFlashEraseSubsector(addr, length);

  // Verify erased memory segment.
  for (int32_t offset = 0; offset < length; offset += ARRAYSIZE(data)) {
    uint32_t seg_addr = addr + offset;

    EXPECT_TRUE(IsErased(seg_addr, ARRAYSIZE(data), data));
  }
}

static void TestEraseOneSubsector(void) {
  uint32_t addr;

  // Erase first subsector.
  addr = 0U;
  TestEraseSubsector(addr, SPI_FLASH_SUBSECTOR_SIZE);

  // Erase last subsector.
  addr = SpiFlashSubsectorToAddress(SpiFlashGetSubsectors() - 1U);
  TestEraseSubsector(addr, SPI_FLASH_SUBSECTOR_SIZE);
}

static void TestEraseMultipleSubsectors(void) {
  uint32_t addr;

  // Erase first.
  addr = 0U;
  TestEraseSubsector(addr, 2 * SPI_FLASH_SUBSECTOR_SIZE);

  // Erase across sector boundary.
  if (SpiFlashGetSectors() > 1) {
    addr = SpiFlashSubsectorToAddress(SpiFlashSectorToSubsector(1U) - 1U);
    TestEraseSubsector(addr, 2 * SPI_FLASH_SUBSECTOR_SIZE);
  }

  // Erase last.
  if (SpiFlashGetSectors() > 1) {
    addr = SpiFlashSubsectorToAddress(SpiFlashGetSubsectors() - 2U);
    TestEraseSubsector(addr, 2 * SPI_FLASH_SUBSECTOR_SIZE);
  }
}

static void TestProgram(uint32_t addr, int32_t length, uint8_t *data) {
  // Erase.
  SpiFlashEraseSubsector(addr, length);
  EXPECT_TRUE(IsErased(addr, length, data));

  // Program.
  for (int32_t i = 0; i < length; ++i) {
    data[i] = (uint8_t)i;
  }
  SpiFlashProgram(addr, length, data);

  // Verify.
  for (int32_t i = 0; i < length; ++i) {
    data[i] = 0;
  }
  SpiFlashRead(addr, length, data);
  for (int32_t i = 0; i < length; ++i) {
    if (!EXPECT_EQ(data[i], (uint8_t)i)) {
      break;
    }
  }
}

static void TestProgramOnePage(void) {
  uint32_t addr;
  uint8_t data[SPI_FLASH_PAGE_SIZE];

  // Program first page.
  addr = 0U;
  TestProgram(addr, ARRAYSIZE(data), data);

  // Program last page.
  addr = SpiFlashPageToAddress(SpiFlashGetPages()) - ARRAYSIZE(data);
  TestProgram(addr, ARRAYSIZE(data), data);
}

static void TestProgramMultiplePages(void) {
  uint32_t addr;
  uint8_t data[2 * SPI_FLASH_PAGE_SIZE];

  // Program first page.
  addr = 0U;
  TestProgram(addr, ARRAYSIZE(data), data);

  // Program across die boundary.
  if (SpiFlashGetDies() > 1) {
    addr = SpiFlashDieToAddress(1U) - ARRAYSIZE(data) / 2;
    TestProgram(addr, ARRAYSIZE(data), data);
  }

  // Program last pages.
  addr = SpiFlashPageToAddress(SpiFlashGetPages()) - ARRAYSIZE(data);
  TestProgram(addr, ARRAYSIZE(data), data);
}

static const TestConfig kSpiFlashTests[] = {
  TEST_CONFIG_INIT(TestGetId, 1000000),
  TEST_CONFIG_INIT(TestEraseOneSubsector, 5000000),
  TEST_CONFIG_INIT(TestEraseMultipleSubsectors, 5000000),
  TEST_CONFIG_INIT(TestProgramOnePage, 5000000),
  TEST_CONFIG_INIT(TestProgramMultiplePages, 5000000),
};

const TestSuite kSpiFlashTest =
    TEST_SUITE_INIT(kSpiFlashTests, TestSetup, TestTeardown);
