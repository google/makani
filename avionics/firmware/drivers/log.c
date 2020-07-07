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

#include "avionics/firmware/drivers/log.h"

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/startup/ldscript.h"

#define MAGIC_VALUE 0xDEAD1234

static bool g_disable = true;

typedef struct {
  const char *format;  // Printf style format in .const memory section.
  uint32_t cycles;
  size_t arg0;  // Must be size_t type to work properly on 32-bit and 64-bit.
  size_t arg1;
  size_t arg2;
  size_t arg3;
} LogData;

extern uint8_t ldscript_log_begin[];
extern uint8_t ldscript_log_end[];
extern uint32_t ldscript_log_magic;
extern int32_t ldscript_log_size;
extern int32_t ldscript_log_head;
extern int32_t ldscript_log_tail;
extern LogData ldscript_log_data[];

#define MAX_STRING_LENGTH 128

static bool ValidateString(const char *str) {
  // Validate format length (find NULL termination).
  bool valid = false;
  const uint8_t *tmp = (const uint8_t *)str;
  if (ldscript_app_flash_begin <= tmp && tmp < ldscript_app_flash_end) {
    const uint8_t *end_addr = tmp + MAX_STRING_LENGTH;
    if (end_addr > ldscript_app_flash_end) {
      end_addr = ldscript_app_flash_end;
    }
    for (const uint8_t *addr = tmp; addr < end_addr && !valid; ++addr) {
      valid = (*addr == 0x0);
    }
  }
  return valid;
}

void LogInit(void) {
  // Upon reboot, we cannot make any assumptions regarding the state of the
  // circular buffer. Attempt to validate all possible fields to determine if
  // the circular buffer contains valid data.

  // Validate existing log to aid debugging after a reset or lockup.
  int32_t length = (uint32_t)ldscript_log_end - (uint32_t)ldscript_log_data;
  int32_t size = length / sizeof(LogData);
  bool valid = (ldscript_log_magic == MAGIC_VALUE && ldscript_log_size == size
                && 0 <= ldscript_log_head && ldscript_log_head < size
                && 0 <= ldscript_log_tail && ldscript_log_tail < size);

  // Validate format address.
  for (int32_t i = ldscript_log_tail; i != ldscript_log_head && valid;
       i = (i + 1) % size) {
    valid = ValidateString(ldscript_log_data[i].format);
  }

  // Initialize.
  LogDisable();
  if (valid) {
    // TODO: Evaluate other options to communicate the trace log.
    for (int32_t i = 0; i < ldscript_log_size && LogToStdout(); ++i) {}
  } else {
    // Initialize circular buffer to a known state.
    memset(ldscript_log_begin, 0x0,
           (size_t)(ldscript_log_end - ldscript_log_begin));
    ldscript_log_magic = MAGIC_VALUE;
    ldscript_log_size = size;
    ldscript_log_head = 0;
    ldscript_log_tail = 0;
  }
  LogEnable();
}

void LogDisable(void) {
  g_disable = true;
}

void LogEnable(void) {
  g_disable = false;
}

void LogPut(const char *format, ...) {
  if (g_disable) {
    return;
  }
  assert(format != NULL);

  uint32_t state = VimDisableIrq();  // Disable interrupts (thread safe).
  LogData *entry = &ldscript_log_data[ldscript_log_head];
  entry->format = format;
  entry->cycles = Clock32GetCycles();

  // Copy sufficient data from the stack to reproduce two 64-bit arguments
  // in a printf() statement on a 32-bit machine. The underlying data will
  // align according to size_t boundaries. A 32-bit machine can store four
  // char/int types aligned to 32-bit boundaries. A 64-bit machine will store
  // two char, int, float, or double.
  va_list ap;
  va_start(ap, format);
  entry->arg0 = va_arg(ap, size_t);  // 4 bytes on 32-bit, 8 bytes on 64-bit.
  entry->arg1 = va_arg(ap, size_t);  // 4 bytes on 32-bit, 8 bytes on 64-bit.
  entry->arg2 = va_arg(ap, size_t);  // 4 bytes on 32-bit, 8 bytes on 64-bit.
  entry->arg3 = va_arg(ap, size_t);  // 4 bytes on 32-bit, 8 bytes on 64-bit.
  va_end(ap);
  ldscript_log_head = (ldscript_log_head + 1) % ldscript_log_size;
  if (ldscript_log_head == ldscript_log_tail) {
    ldscript_log_tail = (ldscript_log_tail + 1) % ldscript_log_size;
  }
  VimRestoreIrq(state);  // Restore interrupts to previous state.
}

bool LogGet(int32_t *index, uint32_t *cycles, const char **format,
            size_t *arg0, size_t *arg1, size_t *arg2, size_t *arg3) {
  assert(index != NULL);
  assert(cycles != NULL);
  assert(format != NULL);
  assert(arg0 != NULL && arg1 != NULL && arg2 != NULL && arg3 != NULL);

  uint32_t state = VimDisableIrq();  // Disable interrupts (thread safe).
  bool success = (ldscript_log_tail != ldscript_log_head);
  if (success) {
    LogData *entry = &ldscript_log_data[ldscript_log_tail];
    *index = ldscript_log_tail;
    *cycles = entry->cycles;
    *format = entry->format;
    // Argument data corresponds to the stack, not the specific arguments
    // passed to LogPut().
    *arg0 = entry->arg0;
    *arg1 = entry->arg1;
    *arg2 = entry->arg2;
    *arg3 = entry->arg3;
    ldscript_log_tail = (ldscript_log_tail + 1) % ldscript_log_size;
  }
  VimRestoreIrq(state);  // Restore interrupts to previous state.
  return success;
}

int32_t LogRead(int32_t max_length, char *out) {
  assert(max_length >= 0);
  assert(out != NULL);

  int32_t count;
  int32_t index;
  uint32_t cycles;
  const char *format;
  size_t arg0, arg1, arg2, arg3, zero = 0U;
  if (LogGet(&index, &cycles, &format, &arg0, &arg1, &arg2, &arg3)) {
    // Copy stored data back to stack using va_arg.
    count = snprintf(out, max_length, "[%ld @ %lu] ", index, cycles);
    count += snprintf(&out[count], max_length - count, format, arg0, &arg1,
                      arg2, arg3, zero, zero);
  } else {
    count = 0;
  }
  return count;  // Number of bytes written to output buffer.
}

bool LogToStdout(void) {
#ifndef NDEBUG
  int32_t index;
  uint32_t cycles;
  const char *format;
  size_t arg0, arg1, arg2, arg3, zero = 0U;
  if (LogGet(&index, &cycles, &format, &arg0, &arg1, &arg2, &arg3)) {
    // Copy stored data back to stack using va_arg.
    printf("[%ld @ %lu] ", index, cycles);
    printf(format, arg0, arg1, arg2, arg3, zero, zero);
    return true;
  }
#endif  // NDEBUG
  return false;
}
