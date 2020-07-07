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

#include "gs/monitor/indicators/indicators_util.h"

#include <stdint.h>

#include "avionics/common/winch_messages.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/indicator.h"

void UpdateEmpty(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "");
    indicator_set_value(ind, "");
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else {
    // The window's background is not set until after the
    // initialization stage, so the else statement and switching from
    // NONE to EMPTY is necessary to force a color change.
    indicator_set_state(ind, INDICATOR_STATE_EMPTY);
  }
}
