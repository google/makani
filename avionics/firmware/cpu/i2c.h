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

#ifndef AVIONICS_FIRMWARE_CPU_I2C_H_
#define AVIONICS_FIRMWARE_CPU_I2C_H_

#include <stdbool.h>
#include <stdint.h>

void I2cInit(double freq);

// Reads or writes a byte if they are ready.  Returns true if the
// transfer is complete.
bool I2cPoll(bool *error);

bool I2cIsBusIdle(void);
bool I2cRead(uint8_t addr, int32_t len, void *data);
bool I2cReadAsync(uint8_t addr, int32_t len, void *data);
bool I2cWrite(uint8_t addr, int32_t len, const void *data);
bool I2cWriteAsync(uint8_t addr, int32_t len, const void *data);
bool I2cWriteRead(uint8_t addr, int32_t wr_len, const void *wr_data,
                  int32_t rd_len, void *rd_data);
bool I2cWriteReadAsync(uint8_t addr, int32_t wr_len, const void *wr_data,
                       int32_t rd_len, void *rd_data);

#endif  // AVIONICS_FIRMWARE_CPU_I2C_H_
