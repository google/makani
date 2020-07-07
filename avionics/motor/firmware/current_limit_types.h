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

#ifndef AVIONICS_MOTOR_FIRMWARE_CURRENT_LIMIT_TYPES_H_
#define AVIONICS_MOTOR_FIRMWARE_CURRENT_LIMIT_TYPES_H_

typedef struct {
  float iq_upper_limit;
  float iq_lower_limit;
  float iq_cmd_raw;
} CurrentLimit;

typedef struct {
  float iq_upper_limit;  // Maximum available local bus current.
  float iq_lower_limit;  // Minimum available local bus current.
} CurrentLimitInput;

typedef struct {
  float p_i2r;
  float p_bus;
  float p_mech;
  float v_bus;
  float i_bus;
  float omega;
  float iq_cmd_raw;
} CurrentLimitData;

typedef struct {
  float iq_upper_limit_bus;
  float iq_lower_limit_bus;
  float iq_cmd_residual;
  float kt_scale;
} CurrentLimitNetOutput;

#endif  // AVIONICS_MOTOR_FIRMWARE_CURRENT_LIMIT_TYPES_H_
