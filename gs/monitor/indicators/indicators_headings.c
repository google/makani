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

#include "gs/monitor/indicators/indicators_headings.h"

#include <stdint.h>

#include "gs/monitor/monitor.h"

static void UpdateHeading(const char *heading, Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, heading);
    indicator_set_value(ind, "");
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else {
    // The window's background is not set until after the
    // initialization stage, so the else statement and switching from
    // NONE to EMPTY is necessary to force a color change.
    indicator_set_state(ind, INDICATOR_STATE_EMPTY);
  }
}

void UpdateHeadingAio(Indicator *ind, int32_t init) {
  UpdateHeading("AIO Update", ind, init);
}

void UpdateHeadingArbitration(Indicator *ind, int32_t init) {
  UpdateHeading("Leader Arbitration", ind, init);
}

void UpdateHeadingAvionics(Indicator *ind, int32_t init) {
  UpdateHeading("Avionics", ind, init);
}

void UpdateHeadingBottomMotors(Indicator *ind, int32_t init) {
  UpdateHeading("Bottom Motors", ind, init);
}

void UpdateHeadingFlightController(Indicator *ind, int32_t init) {
  UpdateHeading("Flight Controller", ind, init);
}

void UpdateHeadingGroundStation(Indicator *ind, int32_t init) {
  UpdateHeading("Ground Station", ind, init);
}

void UpdateHeadingGsCompass(Indicator *ind, int32_t init) {
  UpdateHeading("GS Compass", ind, init);
}

void UpdateHeadingGsGps(Indicator *ind, int32_t init) {
  UpdateHeading("GS GPS", ind, init);
}

void UpdateHeadingImu(Indicator *ind, int32_t init) {
  UpdateHeading("IMU", ind, init);
}

void UpdateHeadingPlatform(Indicator *ind, int32_t init) {
  UpdateHeading("Platform", ind, init);
}

void UpdateHeadingRedundantImus(Indicator *ind, int32_t init) {
  UpdateHeading("Redundant IMUs", ind, init);
}

void UpdateHeadingServos(Indicator *ind, int32_t init) {
  UpdateHeading("Servos", ind, init);
}

void UpdateHeadingTopMotors(Indicator *ind, int32_t init) {
  UpdateHeading("Top Motors", ind, init);
}

void UpdateHeadingWinch(Indicator *ind, int32_t init) {
  UpdateHeading("Winch", ind, init);
}

void UpdateHeadingWind(Indicator *ind, int32_t init) {
  UpdateHeading("Wind", ind, init);
}

void UpdateHeadingWing(Indicator *ind, int32_t init) {
  UpdateHeading("Wing", ind, init);
}

void UpdateHeadingWingGpsA(Indicator *ind, int32_t init) {
  UpdateHeading("Wing GPS A", ind, init);
}

void UpdateHeadingWingGpsB(Indicator *ind, int32_t init) {
  UpdateHeading("Wing GPS B", ind, init);
}

void UpdateHeadingWingSensors(Indicator *ind, int32_t init) {
  UpdateHeading("Wing Sensors", ind, init);
}

void UpdateHeadingWingState(Indicator *ind, int32_t init) {
  UpdateHeading("Wing State", ind, init);
}
