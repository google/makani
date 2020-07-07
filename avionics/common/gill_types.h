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

#ifndef AVIONICS_COMMON_GILL_TYPES_H_
#define AVIONICS_COMMON_GILL_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  kGillDataIdMetPakFull,
  kGillDataIdMetPakCrossDeadReckoning,
  kGillDataIdMetPakMeanWindVelocity,
  kGillDataIdWindmasterPolar,
  kGillDataIdWindmasterUvw,
} GillDataId;

typedef enum {
  kGillMetPakFieldInvalid = -1,
  kGillMetPakFieldNode,
  kGillMetPakFieldDirection,
  kGillMetPakFieldSpeed,
  kGillMetPakFieldWAxis,
  kGillMetPakFieldPressure,
  kGillMetPakFieldHumidity,
  kGillMetPakFieldTemperature,
  kGillMetPakFieldDewpoint,
  kGillMetPakFieldVoltage,
  kGillMetPakFieldStatus,
  kNumGillMetPakFields
} GillMetPakField;

typedef enum {
  kGillMetPakStatusOk = 0x0,
  kGillMetPakStatusWindAxis1Failed = 0x01,
  kGillMetPakStatusWindAxis2Failed = 0x02,
  kGillMetPakStatusWindAxis1And2Failed = 0x04,
  kGillMetPakStatusWindNvmChecksumFailed = 0x08,
  kGillMetPakStatusWindRomChecksumFailed = 0x09,
  kGillMetPakStatusAcceptableData = 0x0A,
  kGillMetPakStatusWindSensorFailed = 0x0B,
  kGillMetPakStatusHygroClipError = 0x10,
  kGillMetPakStatusDewpointError = 0x20,
  kGillMetPakStatusHumidityError = 0x40,
  kGillMetPakStatusWindPowerFailure = 0x66,
  kGillMetPakStatusWindCommsFailure = 0x67,
  kGillMetPakStatusPressureError = 0x80,
} GillMetPakStatus;

typedef enum {
  kGillWindmasterFieldInvalid = -1,
  kGillWindmasterFieldNode,
  kGillWindmasterFieldUVelocity,
  kGillWindmasterFieldVVelocity,
  kGillWindmasterFieldWVelocity,
  kGillWindmasterFieldUnits,
  kGillWindmasterFieldSpeedOfSound,
  kGillWindmasterFieldStatus,
  kNumGillWindmasterFields
} GillWindmasterField;

typedef enum {
  kGillWindmasterStatusOk = 0x0,
  kGillWindmasterStatusSampleFailurePair1 = 0x01,
  kGillWindmasterStatusSampleFailurePair2 = 0x02,
  kGillWindmasterStatusSampleFailurePair3 = 0x03,
  kGillWindmasterStatusSampleFailurePairs1And2 = 0x04,
  kGillWindmasterStatusSampleFailurePairs1And3 = 0x05,
  kGillWindmasterStatusSampleFailurePairs2And3 = 0x06,
  kGillWindmasterStatusSampleFailureAllPairs = 0x07,
  kGillWindmasterStatusNvmChecksumFailed = 0x08,
  kGillWindmasterStatusRomChecksumFailed = 0x09,
  kGillWindmasterStatusAtMaxGain = 0x0A,
  kGillWindmasterStatusRetriesUsed = 0x0B,
} GillWindmasterStatus;

typedef struct {
  char node;
  float wind_direction;
  float wind_speed;
  float w_axis;
  float pressure;
  float humidity;
  float temperature;
  float dewpoint;
  float voltage;
  uint8_t status;  // See GillMetPakStatus.
} GillDataMetPakFull;

typedef struct {
  float temperature;
  float pressure;
  float humidity;
} GillDataMetPakCrossDeadReckoning;

typedef struct {
  float wind_direction;
  float wind_speed;
  bool acceptable;
} GillDataMetPakMeanWindVelocity;

typedef struct {
  float wind_direction;
  float wind_speed;
  float w_axis;
  float speed_of_sound;
  uint8_t status;  // See GillWindmasterStatus.
} GillDataWindmasterPolar;

typedef struct {
  float wind_velocity[3];
  float speed_of_sound;
  uint8_t status;  // See GillWindmasterStatus.
} GillDataWindmasterUvw;

typedef struct {
  GillDataId id;
  union {
    GillDataMetPakFull metpak_full;
    GillDataMetPakCrossDeadReckoning metpak_cross_dead_reckoning;
    GillDataMetPakMeanWindVelocity metpak_mean_wind_velocity;
    GillDataWindmasterPolar windmaster_polar;
    GillDataWindmasterUvw windmaster_uvw;
  } u;
} GillData;

#endif  // AVIONICS_COMMON_GILL_TYPES_H_
