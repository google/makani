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

#ifndef AVIONICS_COMMON_ACTUATOR_TYPES_H_
#define AVIONICS_COMMON_ACTUATOR_TYPES_H_

// These states and commands are hard-coded on the winch PLC.  The PLC must be
// modified if these IDs are changed.

typedef enum {
  kActuatorStateInit      = 0,
  kActuatorStateReady     = 1,
  kActuatorStateArmed     = 2,
  kActuatorStateRunning   = 3,
  kActuatorStateError     = 4,
  kActuatorStateTest      = 5,
} ActuatorState;

// Actuator command for SetState messages.
typedef enum {
  kActuatorStateCommandNone        = 0,
  kActuatorStateCommandDisarm      = 1,
  kActuatorStateCommandArm         = 2,
  kActuatorStateCommandClearErrors = 3,
  kActuatorStateCommandTest        = 4,
} ActuatorStateCommand;

#endif  // AVIONICS_COMMON_ACTUATOR_TYPES_H_
