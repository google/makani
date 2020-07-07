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

#ifndef AVIONICS_FIRMWARE_MONITORS_BATT_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_BATT_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/faults.h"
#include "avionics/firmware/drivers/bq34z100_types.h"
#include "avionics/firmware/drivers/ltc4151_types.h"
#include "avionics/firmware/drivers/ltc6804_types.h"
#include "avionics/firmware/monitors/batt_analog_types.h"
#include "avionics/firmware/monitors/batt_bq34z100_types.h"
#include "avionics/firmware/monitors/batt_ltc4151_types.h"
#include "avionics/firmware/monitors/batt_mcp342x_types.h"

// TODO: Add BattMonitorError and complete BattMonitorWarning.

typedef enum {
  kBattMonitorWarning12v            = 1 << 0,
  kBattMonitorWarning5v             = 1 << 1,
  kBattMonitorWarningLvA            = 1 << 2,
  kBattMonitorWarningLvB            = 1 << 3,
  kBattMonitorWarningVLvOr          = 1 << 4,
  kBattMonitorWarningILvOr          = 1 << 5,
  kBattMonitorWarningIHall          = 1 << 6,
  kBattMonitorWarningChargerOutput  = 1 << 7,
  kBattMonitorWarningTempReadErrors = 1 << 8,
  kBattMonitorWarningIChg           = 1 << 9,
  kBattMonitorWarningBalancer       = 1 << 10,
  kBattMonitorWarningLowCharge      = 1 << 11,
  kBattMonitorWarningOCProtect      = 1 << 12,
  kBattMonitorWarningMisconfigured  = 1 << 13,
} BattMonitorWarning;

typedef enum {
  kBattMonitorErrorNone       = 0,
  kBattMonitorErrorVLvOr      = 1 << 0,
  kBattMonitorErrorHeatPlate1 = 1 << 1,
  kBattMonitorErrorHeatPlate2 = 1 << 2,
  kBattMonitorErrorBatteries1 = 1 << 3,
  kBattMonitorErrorBatteries2 = 1 << 4,
} BattMonitorError;

typedef enum {
  kBattMonitorStatusConnected = 1 << 0,
  kBattMonitorStatusCharging  = 1 << 1,
  kBattMonitorStatusDualBig   = 1 << 2,
} BattMonitorStatus;

typedef struct {
  StatusFlags flags;

  // Tms570 ADC channels.
  uint32_t analog_populated;
  float analog_data[kNumBattAnalogVoltages];

  // BQ34Z100 pack capacity, voltage, and current monitors.
  uint32_t bq34z100_populated;
  Bq34z100OutputData bq34z100_data[kNumBattBq34z100Monitors];

  // LTC4151 voltage and current monitors.
  uint32_t ltc4151_populated;
  Ltc4151OutputData ltc4151_data[kNumBattLtc4151Monitors];

  // LTC6804 cell voltage monitor and balancer.
  Ltc6804OutputData ltc6804_data;

  // Charger output data, sourced from ltc4151 or tms570 adc depending on rev.
  float charger_current;

  // MCP342X temperature monitors.
  uint32_t mcp342x_populated;
  float mcp342x_data[kNumBattMcp342xMonitors];

  // Ltc6804 stack voltage from the other battery box on the kite.
  float paired_stack_voltage;
} BattMonitorData;

#endif  // AVIONICS_FIRMWARE_MONITORS_BATT_TYPES_H_
