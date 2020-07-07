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

#include "avionics/firmware/cpu/memcpy_test.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/memcpy.h"
#include "avionics/firmware/test/test.h"

#define PROFILE_ITERATIONS 10000

#define MAX_COPY_LENGTH 128
#define MAX_TEST_RUNTIME_US 1000000
#define MAX_PROFILE_RUNTIME_US 10000000
#define COPY_BUFFER_LENGTH (MAX_COPY_LENGTH + 4)

typedef void (* const CopyFunction)(int32_t length, const void *src, void *dst);

// Wrap memcpy to match FastCopy/WordCopy prototypes.
static void MemcpyWrapper(int32_t length, const void *src, void *dst) {
  if (length >= 0) {
    // memcpy does not support negative lengths.
    memcpy(dst, src, length);
  }
}

static uint8_t GenerateSourceData(int32_t i) {
  // Each source element should be unique.
  return 1 + i;
}

static uint8_t GenerateDestData(int32_t i) {
  // Corrupt destination data such that the original destination data does
  // not match the source data.
  return ~GenerateSourceData(i);
}

static void InitCopyData(int32_t length, uint8_t *source, uint8_t *dest) {
  for (int32_t i = 0; i < length; ++i) {
    source[i] = GenerateSourceData(i);
    dest[i] = GenerateDestData(i);
  }
}

static void VerifyCopyData(int32_t buffer_length, int32_t copy_length,
                           int32_t source_offset, const uint8_t *source,
                           int32_t dest_offset, const uint8_t *dest) {
  for (int32_t i = 0; i < buffer_length; ++i) {
    EXPECT_EQ(source[i], GenerateSourceData(i));
    if (dest_offset <= i && i < dest_offset + copy_length) {
      EXPECT_EQ(source[i - dest_offset + source_offset], dest[i]);
    } else {
      EXPECT_EQ(dest[i], GenerateDestData(i));
    }
  }
}

static void TestUnalignedCopy(CopyFunction copy_func) {
  // Test all combinations of source/destination alignment for all copy
  // lengths up to MAX_COPY_LENGTH, including -1 and 0.
  for (int32_t copy_length = -1; copy_length < MAX_COPY_LENGTH; ++copy_length) {
    for (int32_t source_offset = 0; source_offset < 4; ++source_offset) {
      for (int32_t dest_offset = 0; dest_offset < 4; ++dest_offset) {
        uint8_t source[COPY_BUFFER_LENGTH];
        uint8_t dest[COPY_BUFFER_LENGTH];

        InitCopyData(COPY_BUFFER_LENGTH, source, dest);
        copy_func(copy_length, &source[source_offset], &dest[dest_offset]);
        VerifyCopyData(COPY_BUFFER_LENGTH, copy_length,
                       source_offset, source, dest_offset, dest);
      }
    }
  }
}

static float ProfileCopy(CopyFunction copy_func, int32_t copy_length,
                         const uint8_t *source, uint8_t *dest) {
  int32_t iterations = PROFILE_ITERATIONS + 1;
  uint32_t start_cycles = Clock32GetCycles();
  while (--iterations > 0) {
    copy_func(copy_length, source, dest);
  }
  uint32_t end_cycles = Clock32GetCycles();
  uint32_t total_cycles = end_cycles - start_cycles;
  return (float)total_cycles / (float)PROFILE_ITERATIONS;
}

static void ProfileCopyFunctions(const uint8_t *source, uint8_t *dest) {
  printf("%% length, source, dest, memcpy, fast_copy, word_copy\n");
  for (int32_t length = 0; length < MAX_COPY_LENGTH; length += 5) {
    float memcpy_avg = ProfileCopy(MemcpyWrapper, length, source, dest);
    float fast_copy_avg = ProfileCopy(FastCopy, length, source, dest);
    float word_copy_avg = ProfileCopy(WordCopy, length, source, dest);
    printf("%ld, %ld, %ld, %.2f, %.2f, %.2f\n", length,
           (uint32_t)source % 4, (uint32_t)dest % 4,
           memcpy_avg, fast_copy_avg, word_copy_avg);
  }
}

static void TestMemcpy(void) {
  TestUnalignedCopy(MemcpyWrapper);
}

static void TestFastCopy(void) {
  TestUnalignedCopy(FastCopy);
}

static void TestWordCopy(void) {
  TestUnalignedCopy(WordCopy);
}

static void ProfileAligned(void) {
  // Profile coping data from source 32-bit aligned to destination 32-bit
  // aligned.
  uint8_t source[MAX_COPY_LENGTH];
  uint8_t dest[MAX_COPY_LENGTH];
  ProfileCopyFunctions(source, dest);
}

static void ProfileUnaligned02(void) {
  // Profile coping data from source 32-bit aligned to destination 32-bit
  // misalignment(+2 bytes).
  uint8_t source[COPY_BUFFER_LENGTH];
  uint8_t dest[COPY_BUFFER_LENGTH];
  ProfileCopyFunctions(&source[0], &dest[2]);
}

static void ProfileUnaligned13(void) {
  // Profile coping data from source 32-bit misalignment(+1 bytes) to
  // destination 32-bit misalignment(+3 bytes).
  uint8_t source[COPY_BUFFER_LENGTH];
  uint8_t dest[COPY_BUFFER_LENGTH];
  ProfileCopyFunctions(&source[1], &dest[3]);
}

static const TestConfig kMemcpyTests[] = {
  TEST_CONFIG_INIT(TestMemcpy, MAX_TEST_RUNTIME_US),
  TEST_CONFIG_INIT(TestFastCopy, MAX_TEST_RUNTIME_US),
  TEST_CONFIG_INIT(TestWordCopy, MAX_TEST_RUNTIME_US),
  TEST_CONFIG_INIT(ProfileAligned, MAX_PROFILE_RUNTIME_US),
  TEST_CONFIG_INIT(ProfileUnaligned02, MAX_PROFILE_RUNTIME_US),
  TEST_CONFIG_INIT(ProfileUnaligned13, MAX_PROFILE_RUNTIME_US),
};

const TestSuite kMemcpyTest = TEST_SUITE_INIT(kMemcpyTests, NULL, NULL);
