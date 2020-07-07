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

#ifndef AVIONICS_COMMON_IMU_TYPES_H_
#define AVIONICS_COMMON_IMU_TYPES_H_

#include <stdint.h>

// Latency measurements represent the difference between the sampling time
// and network transmission time, all represented in microseconds. Do not
// use measurements when saturated.

// See coning_sculling.c for coning and sculling algorithm.
typedef struct {
  uint32_t timestamp;  // 32-bit local processor timestamp of sample [usec].
  int32_t latency;  // Time since time(m) sample point [usec].
  float dt;         // Integration period from time(m-1) to time(m) [s].
  float phi[3];     // Compensated angular rate integral [rad].
  float dvsf[3];    // Compensated specific force integral [m/s].
  float alpha[3];   // Uncompensated angular rate integral [rad].
  float nu[3];      // Uncompensated specific force integral [m/s].
} ImuConingScullingData;

typedef struct {
  int32_t latency;  // Time since last update [usec].
  float acc[3];     // Accelerometer specific force [m/s^2].
  float gyro[3];    // Gyro angular rate [rad/s].
} ImuRawData;

typedef struct {
  int32_t mag_latency;       // Time since last update [usec].
  int32_t pressure_latency;  // Time since last update [usec].
  float mag[3];    // Magnetic field [gauss].
  float pressure;  // Pressure [bar].
  float temp;      // Temperature [C].
} ImuAuxSensorData;

#endif  // AVIONICS_COMMON_IMU_TYPES_H_
