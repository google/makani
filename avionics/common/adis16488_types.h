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

#ifndef AVIONICS_COMMON_ADIS16488_TYPES_H_
#define AVIONICS_COMMON_ADIS16488_TYPES_H_

typedef enum {
  kAdis16488StatusAlarm               = (1 << 0),
  kAdis16488StatusSpiError            = (1 << 3),
  kAdis16488StatusSensorOverrange     = (1 << 4),
  kAdis16488StatusInertialTestFailure = (1 << 5),
  kAdis16488StatusFlashUpdateResult   = (1 << 6),
  kAdis16488StatusProcessingOverrun   = (1 << 7),
  kAdis16488StatusNewMagnetometer     = (1 << 8),
  kAdis16488StatusNewBarometer        = (1 << 9),
  kAdis16488StatusWatchdogTimer       = (1 << 15)
} Adis16488StatusFlag;

typedef enum {
  kAdis16488ErrorGyroX  = (1 << 0),
  kAdis16488ErrorGyroY  = (1 << 1),
  kAdis16488ErrorGyroZ  = (1 << 2),
  kAdis16488ErrorAccelX = (1 << 3),
  kAdis16488ErrorAccelY = (1 << 4),
  kAdis16488ErrorAccelZ = (1 << 5),
  kAdis16488ErrorMagX   = (1 << 8),
  kAdis16488ErrorMagY   = (1 << 9),
  kAdis16488ErrorMagZ   = (1 << 10),
  kAdis16488ErrorBaro   = (1 << 11)  // Only tested at start-up.
} Adis16488ErrorFlag;

#endif  // AVIONICS_COMMON_ADIS16488_TYPES_H_
