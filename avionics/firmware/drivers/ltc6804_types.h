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

#ifndef AVIONICS_FIRMWARE_DRIVERS_LTC6804_TYPES_H_
#define AVIONICS_FIRMWARE_DRIVERS_LTC6804_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  kLtc6804ForceSigned = -1,
  kLtc6804Rate27kHz = 0,
  kLtc6804Rate14kHz = 1,
  kLtc6804Rate7kHz = 2,
  kLtc6804Rate3kHz = 3,
  kLtc6804Rate2kHz = 4,
  kLtc6804Rate26Hz = 5
} Ltc6804Rate;

typedef enum {
  kLtc6804CellChAll = 0,
  kLtc6804CellCh1And7 = 1,
  kLtc6804CellCh2And8 = 2,
  kLtc6804CellCh3And9 = 3,
  kLtc6804CellCh4And10 = 4,
  kLtc6804CellCh5And11 = 5,
  kLtc6804CellCh6And12 = 6
} Ltc6804CellCh;

typedef enum {
  kLtc6804AuxChAll = 0,
  kLtc6804AuxChGpio1 = 1,
  kLtc6804AuxChGpio2 = 2,
  kLtc6804AuxChGpio3 = 3,
  kLtc6804AuxChGpio4 = 4,
  kLtc6804AuxChGpio5 = 5,
  kLtc6804AuxChVref2 = 6
} Ltc6804AuxCh;

typedef enum {
  kLtc6804StatChAll = 0,
  kLtc6804StatChSoc = 1,
  kLtc6804StatChItmp = 2,
  kLtc6804StatChVa = 3,
  kLtc6804StatChVd = 4
} Ltc6804StatCh;

typedef enum {
  kLtc6804DctoDisable = 0,
  kLtc6804Dcto30sec = 1,
  kLtc6804Dcto1min = 2,
  kLtc6804Dcto2min = 3,
  kLtc6804Dcto3min = 4,
  kLtc6804Dcto4min = 5,
  kLtc6804Dcto5min = 6,
  kLtc6804Dcto10min = 7,
  kLtc6804Dcto15min = 8,
  kLtc6804Dcto20min = 9,
  kLtc6804Dcto30min = 10,
  kLtc6804Dcto40min = 11,
  kLtc6804Dcto60min = 12,
  kLtc6804Dcto75min = 13,
  kLtc6804Dcto90min = 14,
  kLtc6804Dcto120min = 15
} Ltc6804Dcto;

typedef enum {
  kLtc6804SelfTest1 = 1,
  kLtc6804SelfTest2 = 2
} Ltc6804SelfTest;

typedef struct {
  float under_volt_thres;
  float over_volt_thres;
  bool reference_on;
  bool discharge_permitted;
  Ltc6804Rate rate;
  Ltc6804CellCh cell_channels;
  Ltc6804AuxCh aux_channels;
  Ltc6804StatCh stat_channels;
  Ltc6804Dcto discharge_timeout;
  Ltc6804SelfTest self_test_mode;
} Ltc6804Control;

typedef struct {
  int32_t index;
  int32_t bms_chip;
  int32_t bms_channel;
  float voltage;
} Ltc6804CellIndex;

typedef struct {
  float min_cell_v;
  float max_cell_v;
  int32_t error_count;
  float stack_voltage;
} Ltc6804OutputData;

#endif  // AVIONICS_FIRMWARE_DRIVERS_LTC6804_TYPES_H_
