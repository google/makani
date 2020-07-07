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

#ifndef AVIONICS_SERVO_FIRMWARE_DEF_H_
#define AVIONICS_SERVO_FIRMWARE_DEF_H_

#define SERVO_CONTROL_PERIOD_US     (10 * 1000)
#define SERVO_ENCODER_COUNTS        16384
#define SERVO_CONTROLLER_TIMEOUT_US (50 * 1000)
#define SERVO_NET_TIMEOUT_US        2000
#define SERVO_PAIR_TIMEOUT_US       (1000*1000)

#endif  // AVIONICS_SERVO_FIRMWARE_DEF_H_
