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

#include "control/avionics/avionics_interface_types.h"

#include <assert.h>

#include "avionics/common/loadcell_types.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "common/macros.h"

static const int32_t kNumStrainsPerDevice =
    ARRAYSIZE(((LoadcellData *)0)->strain);

float GetStrain(const LoadcellMessage messages[], const StrainLocation *loc) {
  assert(0 <= loc->i_msg && loc->i_msg < kNumLoadcellNodes);
  assert(0 <= loc->i_strain && loc->i_strain < kNumStrainsPerDevice);
  return messages[loc->i_msg].loadcell_data.strain[loc->i_strain].value;
}

float *GetMutableStrain(LoadcellMessage messages[], const StrainLocation *loc) {
  assert(0 <= loc->i_msg && loc->i_msg < kNumLoadcellNodes);
  assert(0 <= loc->i_strain && loc->i_strain < kNumStrainsPerDevice);
  return &messages[loc->i_msg].loadcell_data.strain[loc->i_strain].value;
}
