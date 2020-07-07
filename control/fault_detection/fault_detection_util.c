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

#include "control/fault_detection/fault_detection_util.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>

#include "common/c_math/util.h"
#include "control/fault_detection/fault_detection_types.h"

void ClearAllFaults(FaultMask *fault) {
  assert(fault != NULL);
  fault->code = 0;
}

void SetFault(FaultType fault_type, bool on, FaultMask *fault) {
  assert(fault != NULL);
  if (on) {
    fault->code |= (1 << fault_type);
  } else {
    fault->code &= ~(1 << fault_type);
  }
}
