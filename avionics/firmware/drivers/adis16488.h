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

#ifndef AVIONICS_FIRMWARE_DRIVERS_ADIS16488_H_
#define AVIONICS_FIRMWARE_DRIVERS_ADIS16488_H_

#include <stdbool.h>
#include <stdint.h>

#define ADIS16488_SYNC_FREQ_HZ 2400

typedef struct {
  uint16_t seq_cnt;
  uint16_t sys_e_flag;
  uint16_t diag_sts;
  float dt;
  float temp;
  float accel[3];
  float gyro[3];
  float mag[3];
  float baro;
} Adis16488OutputData;

void Adis16488Init(void);
void Adis16488ResetRelease(void);
void Adis16488WaitForSyncPulse(void);
bool Adis16488PollOutputData(int64_t now, Adis16488OutputData *out);
bool Adis16488IsReady(void);

#endif  // AVIONICS_FIRMWARE_DRIVERS_ADIS16488_H_
