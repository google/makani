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

#ifndef AVIONICS_FIRMWARE_DRIVERS_PCA9546A_H_
#define AVIONICS_FIRMWARE_DRIVERS_PCA9546A_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  kPca9546aStateReset,
  kPca9546aStateResetRecovery,
  kPca9546aStateWriteControl,
  kPca9546aStateReadControl,
  kPca9546aStateIdle
} Pca9546aState;

typedef struct {
  uint8_t addr;  // I2C 7-bit address.
  void (* const reset_func)(void);  // Function to pulse nRESET low.
} Pca9546aConfig;

typedef struct {
  Pca9546aState state;
  bool error;
  uint32_t recovery_timeout;
  uint8_t config;  // Desired bus selection mask.
  uint8_t status;  // Current bus selection mask.
} Pca9546a;

void Pca9546aInit(Pca9546a *device);
bool Pca9546aPoll(uint8_t bus_mask, const Pca9546aConfig *config,
                  Pca9546a *device);

#endif  // AVIONICS_FIRMWARE_DRIVERS_PCA9546A_H_
