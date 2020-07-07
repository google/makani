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

#include "control/experiments/crosswind_experiment.h"

double CrosswindSpoilerExperimentOnRate(double target_deflection_angle) {
  double expected_loop_duration_s = 20.0;
  return target_deflection_angle / expected_loop_duration_s;
}

double CrosswindSpoilerExperimentOffRate(void) {
  // Offline analysis shows this rate results in acceptable response and is
  // relatively fast (the spoiler can be retracted from 20 deg in 5 seconds).
  return 0.07;
}
