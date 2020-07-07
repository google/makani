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

#include "avionics/common/faults.h"

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/bits.h"

void SetStatus(uint16_t mask, bool value, StatusFlags *flags) {
  SetBitByValue16(mask, value, &flags->status);
}

void SignalWarning(uint16_t mask, bool value, StatusFlags *flags) {
  SetBitByValue16(mask, value, &flags->warning);
}

void SignalError(uint16_t mask, bool value, StatusFlags *flags) {
  if (value) {
    flags->error |= mask;
  }
}

bool CheckStatus(const StatusFlags *flags, uint16_t mask) {
  return (flags->status & mask) == mask;
}

bool CheckWarning(const StatusFlags *flags, uint16_t mask) {
  return (flags->warning & mask) == mask;
}

bool CheckError(const StatusFlags *flags, uint16_t mask) {
  return (flags->error & mask) == mask;
}

bool HasWarning(const StatusFlags *flags) {
  return flags->warning != 0;
}

bool HasError(const StatusFlags *flags) {
  return flags->error != 0;
}

void ClearErrors(StatusFlags *flags) {
  flags->error = 0;
}

void ClearStatus(StatusFlags *flags) {
  flags->status = 0;
}

void ClearWarnings(StatusFlags *flags) {
  flags->warning = 0;
}
