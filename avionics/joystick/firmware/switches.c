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

#include "avionics/joystick/firmware/switches.h"

#include "avionics/common/debounce.h"
#include "avionics/common/safety_codes.h"
#include "avionics/firmware/cpu/io.h"
#include "avionics/joystick/firmware/output.h"

#define SCUTTLE_SWITCH_PIN kIoGioPinA2
#define TETHER_RELEASE_INTERLOCK_SWITCH_PIN kIoGioPinA7

static DebounceState g_scuttle_state;
static DebounceState g_tether_release_interlock_state;

static void SwitchesConfigurePins(void) {
  // Switches are active low so we pull them up to default to inactive.
  IoConfigureAsInputPullUp(SCUTTLE_SWITCH_PIN);
  IoConfigureAsInputPullUp(TETHER_RELEASE_INTERLOCK_SWITCH_PIN);
}

void SwitchesInit(void) {
  IoInit();
  SwitchesConfigurePins();
  DebounceStateInit(false, &g_scuttle_state);
  DebounceStateInit(false, &g_tether_release_interlock_state);
}

void SwitchesPoll(void) {
  // Reconfigure input on every read to protect against register corruption.
  SwitchesConfigurePins();

  // Invert pin value as switch is active low.
  bool scuttle_switch = !IoGetValue(SCUTTLE_SWITCH_PIN);
  bool tether_release_interlock_switch =
      !IoGetValue(TETHER_RELEASE_INTERLOCK_SWITCH_PIN);

  // 100 cycles at 10us poll period is 1 second of debounce on active.
  if (Debounce(scuttle_switch, 100, 0, &g_scuttle_state)) {
    JoystickOutputSetScuttleCode(SCUTTLE_SAFETY_CODE);
  } else {
    JoystickOutputSetScuttleCode(0);
  }

  // 10 cycles at 10us poll period is 100ms of debounce on active.
  if (Debounce(tether_release_interlock_switch, 10, 0,
               &g_tether_release_interlock_state)) {
    JoystickOutputSetTetherReleaseInterlockCode(
        TETHER_RELEASE_INTERLOCK_SAFETY_CODE);
  } else {
    JoystickOutputSetTetherReleaseInterlockCode(0);
  }
}
