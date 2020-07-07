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

#include "avionics/firmware/monitors/batt.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/faults.h"
#include "avionics/firmware/drivers/bq34z100.h"
#include "avionics/firmware/drivers/ltc4151.h"
#include "avionics/firmware/drivers/ltc6804.h"
#include "avionics/firmware/drivers/mcp342x.h"
#include "avionics/firmware/monitors/analog.h"
#include "avionics/firmware/monitors/batt_analog_types.h"
#include "avionics/firmware/monitors/batt_bq34z100_types.h"
#include "avionics/firmware/monitors/batt_ltc4151_types.h"
#include "avionics/firmware/monitors/batt_ltc6804_types.h"
#include "avionics/firmware/monitors/batt_mcp342x_types.h"
#include "avionics/firmware/monitors/batt_types.h"
#include "avionics/firmware/monitors/bq34z100.h"
#include "avionics/firmware/monitors/ltc4151.h"
#include "avionics/firmware/monitors/ltc6804.h"
#include "avionics/firmware/monitors/mcp342x.h"
#include "avionics/firmware/monitors/thermal_types.h"
#include "avionics/firmware/serial/batt_serial_params.h"
#include "common/macros.h"

static BattMonitorData *g_monitors = NULL;
static Bq34z100 g_bq34z100[MAX_BATT_BQ34Z100_DEVICES];
static Ltc4151 g_ltc4151[MAX_BATT_LTC4151_DEVICES];
static Mcp342x g_mcp342x[MAX_BATT_MCP342X_DEVICES];

static Ltc6804 g_ltc6804;
static Ltc6804CellIndex g_ltc6804_cells[LTC6804_NUM_CHANNELS *
                                        MAX_BATT_LTC6804_DEVICES];

// Thermal limit [C] for each temperature sensor, which should signal
// an immediate landing of the wing.
static float g_batt_thermal_limits[kNumBattMcp342xMonitors] = {
  [kBattMcp342xMonitorHeatPlate1] = 90.0f,
  [kBattMcp342xMonitorHeatPlate2] = 90.0f,
  [kBattMcp342xMonitorBatteries1] = 80.0f,
  [kBattMcp342xMonitorBatteries2] = 80.0f,
};

static const int32_t kOverTempThreshold = 3;
static const int32_t kTempReadErrorThreshold = 3;

// Hysteresis for reporting over-temp errors.
static int32_t g_over_temp_count[kNumBattMcp342xMonitors] = {0};
static int32_t g_temp_read_errors = 0;

// Heat plate (B57703M0502G040) calibration data.
static const ThermalCalData kHeatPlateCal[] = {
  {-40.0f, 32612, -0.1282051282f},
  {-30.0f, 32473, -0.0719424460f},
  {-20.0f, 32234, -0.0418410042f},
  {-10.0f, 31843, -0.0255754476f},
  {0.0f,   31230, -0.0163132137f},
  {10.0f,  30318, -0.0109649123f},
  {20.0f,  29031, -0.0077700078f},
  {30.0f,  27316, -0.0058309038f},
  {40.0f,  25170, -0.0046598322f},
  {50.0f,  22657, -0.0039793076f},
  {60.0f,  19904, -0.0036324010f},
  {70.0f,  17086, -0.0035486160f},
  {80.0f,  14383, -0.0036995930f},
  {90.0f,  11906, -0.0040371417f},
  {100.0f, 9739,  -0.0046146747f},
  {110.0f, 7904,  -0.0054495913f},
  {120.0f, 6387,  -0.0065919578f},
  {130.0f, 5165,  -0.0081833061f},
  {140.0f, 4175,  -0.0101010101f},
  {150.0f, 3386,  -0.0126742712f}
};

static const ThermalSensor kHeatPlate = {
  kHeatPlateCal,
  ARRAYSIZE(kHeatPlateCal)
};

static void SetTemperature(BattMcp342xMonitor ch, float temp, bool valid) {
  const uint16_t error_map[kNumBattMcp342xMonitors] = {
    [kBattMcp342xMonitorHeatPlate1] = kBattMonitorErrorHeatPlate1,
    [kBattMcp342xMonitorHeatPlate2] = kBattMonitorErrorHeatPlate2,
    [kBattMcp342xMonitorBatteries1] = kBattMonitorErrorBatteries1,
    [kBattMcp342xMonitorBatteries2] = kBattMonitorErrorBatteries2,
  };

  // Read hysteresis to mitigate errors due to read glitches.
  if (valid) {
    --g_temp_read_errors;
  } else {
    ++g_temp_read_errors;
  }
  if (g_temp_read_errors >= kTempReadErrorThreshold) {
    g_temp_read_errors = kTempReadErrorThreshold;
    SignalWarning(kBattMonitorWarningTempReadErrors, true, &g_monitors->flags);
  } else if (g_temp_read_errors <= 0) {
    g_temp_read_errors = 0;
    SignalWarning(kBattMonitorWarningTempReadErrors, false, &g_monitors->flags);
  }

  if (valid) {
    // Insert data and any errors into output structure.
    g_monitors->mcp342x_populated |= 1U << ch;
    g_monitors->mcp342x_data[ch] = temp;

    // Over-temp hysteresis to mitigate errors due to erroneous measurements.
    if (temp >= g_batt_thermal_limits[ch]) {
      ++g_over_temp_count[ch];
    } else {
      --g_over_temp_count[ch];
    }
    if (g_over_temp_count[ch] >= kOverTempThreshold) {
      g_over_temp_count[ch] = kOverTempThreshold;
      SignalError(error_map[ch], 1, &g_monitors->flags);
    } else if (g_over_temp_count[ch] <= 0) {
      g_over_temp_count[ch] = 0;
    }
  }
}

static void OutputAnalog(const AnalogMonitor *config, float in,
                         uint32_t flags) {
  assert(g_monitors != NULL);

  // Map the analog input channels to a common flags field output.
  // The warn_map[] array defines a bitmask to set or clear in SignalWarning.
  const uint16_t warn_map[kNumBattAnalogInputs] = {
    [kBattAnalogInput12v]   = kBattMonitorWarning12v,
    [kBattAnalogInput5v]    = kBattMonitorWarning5v,
    [kBattAnalogInputIHall] = kBattMonitorWarningIHall,
    [kBattAnalogInputILvOr] = kBattMonitorWarningILvOr,
    [kBattAnalogInputIChg]  = kBattMonitorWarningIChg,
    [kBattAnalogInputLvA]   = kBattMonitorWarningLvA,
    [kBattAnalogInputLvB]   = kBattMonitorWarningLvB,
    [kBattAnalogInputVLvOr] = kBattMonitorWarningVLvOr,
  };

  const uint16_t error_map[kNumBattAnalogInputs] = {
    [kBattAnalogInput12v]   = kBattMonitorErrorNone,
    [kBattAnalogInput5v]    = kBattMonitorErrorNone,
    [kBattAnalogInputIHall] = kBattMonitorErrorNone,
    [kBattAnalogInputILvOr] = kBattMonitorErrorNone,
    [kBattAnalogInputIChg]  = kBattMonitorErrorNone,
    [kBattAnalogInputLvA]   = kBattMonitorErrorNone,
    [kBattAnalogInputLvB]   = kBattMonitorErrorNone,
    [kBattAnalogInputVLvOr] = kBattMonitorErrorNone,
  };

  if (config->type == kAnalogTypeVoltage) {
    assert(0 <= config->voltage
           && config->voltage < ARRAYSIZE(g_monitors->analog_data));
    g_monitors->analog_populated |= 1U << config->voltage;
    g_monitors->analog_data[config->voltage] = in;
  }

  SignalWarning(warn_map[config->input], flags & ANALOG_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);
  SignalError(error_map[config->input], flags & ANALOG_MONITOR_ERROR_FLAGS,
              &g_monitors->flags);
}

static void OutputBq34z100(int32_t device, const Bq34z100OutputData *data,
                           uint32_t flags) {
  assert(g_monitors != NULL);
  assert(0 <= device && device < ARRAYSIZE(g_monitors->bq34z100_data));

  // Map the BQ34Z100 voltage monitoring to a common flags field output.
  // The warn_map array defines a bitmask to set or clear in SignalWarning.
  const uint16_t warn_map[kNumBattBq34z100Monitors] = {
    [kBattBq34z100MonitorCoulCount] = kBattMonitorWarningLowCharge,
  };
  SignalWarning(warn_map[device], flags & BQ34Z100_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);

  g_monitors->bq34z100_populated |= 1U << device;
  g_monitors->bq34z100_data[device] = *data;
}

static void OutputLtc4151(int32_t device, const Ltc4151OutputData *data,
                          uint32_t flags) {
  assert(g_monitors != NULL);
  assert(0 <= device && device < ARRAYSIZE(g_monitors->ltc4151_data));

  // Map the LTC4151 voltage monitoring to a common flags field output.
  // The warn_map array defines a bitmask to set or clear in SignalWarning.
  const uint16_t warn_map[kNumBattLtc4151Monitors] = {
    [kBattLtc4151MonitorChargerOutput] = kBattMonitorWarningChargerOutput,
  };
  SignalWarning(warn_map[device], flags & LTC4151_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);

  g_monitors->ltc4151_populated |= 1U << device;
  g_monitors->ltc4151_data[device] = *data;
}

static void OutputLtc6804(const Ltc6804OutputData *data) {
  assert(g_monitors != NULL);
  assert(data != NULL);
  g_monitors->ltc6804_data = *data;

  if (data->min_cell_v < 2.5f ||
      data->max_cell_v > 5.0f ||
      data->error_count > 3) {
    SignalWarning(kBattMonitorWarningBalancer, 1, &g_monitors->flags);
  } else {
    // Clear warning if chips responsive and cell voltages in normal range.
    SignalWarning(kBattMonitorWarningBalancer, 0, &g_monitors->flags);
  }
}

static void OutputMcp342x(int32_t monitor, int32_t t_raw, bool valid) {
  assert(0 <= monitor && monitor < ARRAYSIZE(g_monitors->mcp342x_data));

  const ThermalSensor *kCalibMap[kNumBattMcp342xMonitors] = {
    [kBattMcp342xMonitorBatteries1] = &kHeatPlate,
    [kBattMcp342xMonitorBatteries2] = &kHeatPlate,
    [kBattMcp342xMonitorHeatPlate1] = &kHeatPlate,
    [kBattMcp342xMonitorHeatPlate2] = &kHeatPlate,
  };
  const ThermalSensor *calib = kCalibMap[monitor];

  assert(calib != NULL);
  SetTemperature(monitor, ThermalCodeToTemp(t_raw, calib), valid);
}

static bool PollAnalog(BattHardware rev) {
  return AnalogMonitorPoll(BattAnalogGetConfig(rev), OutputAnalog);
}

static bool PollBq34z100(BattHardware rev) {
  static uint32_t index = 0U;
  return Bq34z100MonitorPoll(BattBq34z100GetConfig(rev), OutputBq34z100,
                             &index, g_bq34z100);
}

static bool PollLtc4151(BattHardware rev) {
  static uint32_t index = 0U;
  return Ltc4151MonitorPoll(BattLtc4151GetConfig(rev), OutputLtc4151,
                            &index, g_ltc4151);
}

static bool PollLtc6804(BattHardware rev, bool reduced_ltc6804_read_rate) {
  return Ltc6804MonitorPoll(BattLtc6804GetConfig(rev),
                            reduced_ltc6804_read_rate,
                            g_ltc6804_cells, OutputLtc6804, &g_ltc6804);
}

static bool PollMcp342x(BattHardware rev) {
  static uint32_t device_index = 0U;
  static uint32_t config_indices[MAX_BATT_MCP342X_DEVICES] = {0U};
  return Mcp342xMonitorPoll(BattMcp342xGetConfig(rev), OutputMcp342x,
                            &device_index, config_indices, g_mcp342x);
}

void BattMonitorInit(BattHardware rev) {
  Bq34z100MonitorInit(ARRAYSIZE(g_bq34z100), g_bq34z100);
  Ltc4151MonitorInit(ARRAYSIZE(g_ltc4151), g_ltc4151);
  Mcp342xMonitorInit(ARRAYSIZE(g_mcp342x), g_mcp342x);
  Ltc6804MonitorInit(BattLtc6804GetConfig(rev), ARRAYSIZE(g_ltc6804_cells),
                     g_ltc6804_cells, &g_ltc6804);
}

bool BattMonitorPoll(BattHardware rev, bool reduced_ltc6804_read_rate,
                     BattMonitorData *monitors) {
  static bool (*const poll[])(BattHardware rev) = {
    PollAnalog,
    PollBq34z100,
    PollLtc4151,
    PollMcp342x,
  };
  static uint32_t index = 0U;

  // Set for output functions.
  g_monitors = monitors;

  // Make certain that device_index is always valid.
  index %= (ARRAYSIZE(poll) + 1);
  if ((index < ARRAYSIZE(poll) && poll[index](rev)) ||
      (index == ARRAYSIZE(poll) &&
       PollLtc6804(rev, reduced_ltc6804_read_rate))) {
    ++index;
    return true;
  }
  return false;
}
