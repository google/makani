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

#include "avionics/ground_power/q7/ground_power_types.h"

#include <assert.h>
#include <stdbool.h>

const char *GroundPowerStateToString(GroundPowerState gp_state) {
  switch (gp_state) {
    case kInitializationStateModbusTcpConnected:
      return "InitializationStateModbusTCPConnected";
    case kInitializationStateQueryThreadsRunning:
      return "InitializationStateQueryThreadsRunning";
    default:
      assert(false);
      return "<Unknown>";
  }
}
