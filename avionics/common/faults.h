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

#ifndef AVIONICS_COMMON_FAULTS_H_
#define AVIONICS_COMMON_FAULTS_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// StatusFlags is a set of bitfields corresponding to the status of a subsystem.
// The status field is purely informational and should include information that
// would be useful primarily for monitoring the subsystem.  The warning field
// represents statuses which the subsystem believes to be problems, but ones
// under which it can continue to operate.  Any warning flag values should be
// investigated to determine if they represent a true problem or if they are
// incorrectly being signalled.  The error field should be reserved for statuses
// under which operation of the system cannot continue.  In these cases the
// subsystem has given up normal operation and it is the responsibility of a
// higher system such as the flight computer or the operator to mitigate the
// extent of the problem.
typedef struct {
  uint16_t status;
  uint16_t warning;
  uint16_t error;
} StatusFlags;

// Set the status flags which 'mask' has set to 'value'.  Bits where mask is not
// set will remain unchanged.
void SetStatus(uint16_t mask, bool value, StatusFlags *flags);
// Set the warning flags which 'mask' has set to 'value'.  Bits where mask is
// not set will remain unchanged.
void SignalWarning(uint16_t mask, bool value, StatusFlags *flags);
// Set the error flags which 'mask' has set if 'value' is true.  Any set error
// flags will remain set regardless of the value of 'value' (they are sticky).
void SignalError(uint16_t mask, bool value, StatusFlags *flags);
// Return true if all of the bits set in 'mask' are set in the status flags.
bool CheckStatus(const StatusFlags *flags, uint16_t mask);
// Return true if all of the bits set in 'mask' are set in the warning flags.
bool CheckWarning(const StatusFlags *flags, uint16_t mask);
// Return true if all of the bits set in 'mask' are set in the error flags.
bool CheckError(const StatusFlags *flags, uint16_t mask);
// Return true if any warning flag is set.
bool HasWarning(const StatusFlags *flags);
// Return true if any error flag is set.
bool HasError(const StatusFlags *flags);
// Unset all error flags in 'flags'.
void ClearErrors(StatusFlags *flags);
// Unset all status flags in 'flags'.
void ClearStatus(StatusFlags *flags);
// Unset all warning flags in 'flags'.
void ClearWarnings(StatusFlags *flags);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_FAULTS_H_
