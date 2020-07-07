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

#include "avionics/firmware/monitors/mvlv.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/faults.h"
#include "avionics/firmware/drivers/ltc2309.h"
#include "avionics/firmware/drivers/mcp342x.h"
#include "avionics/firmware/monitors/analog.h"
#include "avionics/firmware/monitors/ltc2309.h"
#include "avionics/firmware/monitors/ltc2309_types.h"
#include "avionics/firmware/monitors/mcp342x.h"
#include "avionics/firmware/monitors/mvlv_analog_types.h"
#include "avionics/firmware/monitors/mvlv_ltc2309_types.h"
#include "avionics/firmware/monitors/mvlv_mcp342x_types.h"
#include "avionics/firmware/monitors/mvlv_types.h"
#include "avionics/firmware/monitors/thermal_types.h"
#include "avionics/firmware/serial/mvlv_serial_params.h"
#include "common/macros.h"

static MvlvMonitorData *g_monitors = NULL;
static Ltc2309 g_ltc2309[MAX_MVLV_LTC2309_DEVICES];
static Mcp342x g_mcp342x[MAX_MVLV_MCP342X_DEVICES];

// Thermal limit [C] for each temperature sensor, which should signal
// a MVLV failure. Operator should switch to battery as soon as possible.
static float g_mvlv_thermal_limits[kNumMvlvMcp342xMonitors] = {
  // Thermal limits set based on 2017-09-12 experiment.
  // Test condition: 600W load inside Thermotron at 55C.
  // https://goo.gl/F5mots.
  [kMvlvMcp342xMonitorFilterCap]          =  95.0f,
  [kMvlvMcp342xMonitorOutputSwitch]       = 100.0f,
  [kMvlvMcp342xMonitorSyncRectMosfetSide] =  90.0f,
  [kMvlvMcp342xMonitorSyncRectPcb]        =  90.0f,
  [kMvlvMcp342xMonitorSyncRectMosfetTop]  =  90.0f,
  [kMvlvMcp342xMonitorHvResonantCap]      =  90.0f,
  [kMvlvMcp342xMonitorIgbt]               =  95.0f,
  [kMvlvMcp342xMonitorEnclosureAir]       =  75.0f,
};

static const int32_t kOverTempThreshold = 3;
static const int32_t kTempReadErrorThreshold = 3;

// Hysteresis for reporting over-temp errors.
static int32_t g_over_temp_count[kNumMvlvMcp342xMonitors] = {0};
static int32_t g_temp_read_errors = 0;

// Thermistor (NCP18XH103F03RB) calibration data.
// Datasheet: https://goo.gl/wGPmgG.
// Calculation spread sheet: https://goo.gl/UjaZQs.
static const ThermalCalData kThermistorCal[] = {
  {-40.0f, 32500, -.0978875859f},
  {-30.0f, 32309, -.0521615546f},
  {-20.0f, 32012, -.0336984583f},
  {-10.0f, 31571, -.0226732050f},
  {  0.0f, 30936, -.0157513283f},
  { 10.0f, 30064, -.0114748519f},
  { 20.0f, 28910, -.0086646745f},
  { 30.0f, 27447, -.0068325624f},
  { 40.0f, 25674, -.0056399028f},
  { 50.0f, 23618, -.0048647520f},
  { 60.0f, 21350, -.0044096175f},
  { 70.0f, 19010, -.0042739471f},
  { 80.0f, 16666, -.0042656767f},
  { 90.0f, 14427, -.0044659621f},
  {100.0f, 12340, -.0047921431f},
  {110.0f, 10480, -.0053762179f},
  {120.0f,  8849, -.0061315427f},
  {125.0f,  8121, -.0068605988f},
};

static const ThermalSensor kThermistor = {
  kThermistorCal,
  ARRAYSIZE(kThermistorCal)
};

// Thermistor (B57703M0502G040) calibration data.
// Datasheet: https://goo.gl/9x3ooC.
// Calculation spread sheet: https://goo.gl/UjaZQs.
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

static void SetTemperature(MvlvMcp342xMonitor ch, float temp, bool valid) {
  assert(0 <= ch && ch < kNumMvlvMcp342xMonitors);

  const uint16_t error_map[kNumMvlvMcp342xMonitors] = {
    [kMvlvMcp342xMonitorSyncRectMosfetSide]
    = kMvlvMonitorErrorSyncRectMosfetSide,
    [kMvlvMcp342xMonitorSyncRectPcb]   = kMvlvMonitorErrorSyncRectPcb,
    [kMvlvMcp342xMonitorFilterCap]     = kMvlvMonitorErrorFilterCap,
    [kMvlvMcp342xMonitorOutputSwitch]  = kMvlvMonitorErrorOutputSwitch,
    [kMvlvMcp342xMonitorSyncRectMosfetTop]
    = kMvlvMonitorErrorSyncRectMosfetTop,
    [kMvlvMcp342xMonitorHvResonantCap] = kMvlvMonitorErrorHvResonantCap,
    [kMvlvMcp342xMonitorIgbt]          = kMvlvMonitorErrorIgbt,
    [kMvlvMcp342xMonitorEnclosureAir]  = kMvlvMonitorErrorEnclosureAir,
  };

  // TODO: Implement code to hadle error from multiple chips.
  // Each chip should have its own g_temp_read_error. SetTemperature will
  // find the device used for the ch and process accordingly.

  if (valid) {
    // Read hysteresis to mitigate errors due to read glitches.
    --g_temp_read_errors;

    // Insert data and any errors into output structure.
    g_monitors->mcp342x_populated |= 1U << ch;
    g_monitors->mcp342x_data[ch] = temp;

    // Over-temp hysteresis to mitigate errors due to erroneous measurements.
    if (temp >= g_mvlv_thermal_limits[ch]) {
      ++g_over_temp_count[ch];
    } else {
      --g_over_temp_count[ch];
    }
    if (g_over_temp_count[ch] >= kOverTempThreshold) {
      g_over_temp_count[ch] = kOverTempThreshold;
      SignalError(error_map[ch], true, &g_monitors->flags);
    } else if (g_over_temp_count[ch] <= 0) {
      g_over_temp_count[ch] = 0;
    }
  } else {
    ++g_temp_read_errors;
  }

  if (g_temp_read_errors >= kTempReadErrorThreshold) {
    g_temp_read_errors = kTempReadErrorThreshold;
    SignalWarning(kMvlvMonitorWarningTempReadErrors, true, &g_monitors->flags);
  } else if (g_temp_read_errors <= 0) {
    g_temp_read_errors = 0;
    SignalWarning(kMvlvMonitorWarningTempReadErrors, false, &g_monitors->flags);
  }
}

static void OutputAnalog(const AnalogMonitor *config, float in,
                         uint32_t flags) {
  assert(g_monitors != NULL);

  // Map the analog input channels to a common flags field output.
  // The warn_map[] array defines a bitmask to set or clear in SignalWarning.
  const uint16_t warn_map[kNumMvlvAnalogInputs] = {
    [kMvlvAnalogInput12v]    = kMvlvMonitorWarning12v,
    [kMvlvAnalogInput3v3]    = kMvlvMonitorWarning3v3,
    [kMvlvAnalogInput5v]     = kMvlvMonitorWarning5v,
    [kMvlvAnalogInputIHall]  = kMvlvMonitorWarningIHall,
    [kMvlvAnalogInputVExt]   = kMvlvMonitorWarningVExt,
    [kMvlvAnalogInputVLv]    = kMvlvMonitorWarningVLv,
    [kMvlvAnalogInputVLvOr]  = kMvlvMonitorWarningVLvOr,
    [kMvlvAnalogInputVLvPri] = kMvlvMonitorWarningVLvPri,
    [kMvlvAnalogInputVLvSec] = kMvlvMonitorWarningVLvSec,
  };

  const uint16_t error_map[kNumMvlvAnalogInputs] = {
    [kMvlvAnalogInput12v]    = kMvlvMonitorErrorNone,
    [kMvlvAnalogInput3v3]    = kMvlvMonitorErrorNone,
    [kMvlvAnalogInput5v]     = kMvlvMonitorErrorNone,
    [kMvlvAnalogInputIHall]  = kMvlvMonitorErrorNone,
    [kMvlvAnalogInputVExt]   = kMvlvMonitorErrorNone,
    [kMvlvAnalogInputVLv]    = kMvlvMonitorErrorNone,
    [kMvlvAnalogInputVLvOr]  = kMvlvMonitorErrorNone,
    [kMvlvAnalogInputVLvPri] = kMvlvMonitorErrorNone,
    [kMvlvAnalogInputVLvSec] = kMvlvMonitorErrorNone,
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

static void OutputLtc2309(int32_t input, float data, uint32_t flags) {
  assert(g_monitors != NULL);

  // Map the Ltc2309 input channels to a common flags field output.
  // TODO: Implement warning handling.
  const uint16_t warn_map[kNumMvlvLtc2309Monitors] = {};
  g_monitors->ltc2309_populated |= 1U << input;
  g_monitors->ltc2309_data[input] = data;
  SignalWarning(warn_map[input], flags & LTC2309_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);
}

static void OutputMcp342x(int32_t monitor, int32_t t_raw, bool valid) {
  assert(0 <= monitor && monitor < ARRAYSIZE(g_monitors->mcp342x_data));

  const ThermalSensor *kCalibMap[kNumMvlvMcp342xMonitors] = {
    [kMvlvMcp342xMonitorSyncRectMosfetSide] = &kThermistor,
    [kMvlvMcp342xMonitorSyncRectPcb]        = &kThermistor,
    [kMvlvMcp342xMonitorFilterCap]          = &kThermistor,
    [kMvlvMcp342xMonitorOutputSwitch]       = &kThermistor,
    [kMvlvMcp342xMonitorSyncRectMosfetTop]  = &kHeatPlate,
    [kMvlvMcp342xMonitorHvResonantCap]      = &kHeatPlate,
    [kMvlvMcp342xMonitorIgbt]               = &kHeatPlate,
    [kMvlvMcp342xMonitorEnclosureAir]       = &kHeatPlate,
  };
  const ThermalSensor *calib = kCalibMap[monitor];

  assert(calib != NULL);
  SetTemperature(monitor, ThermalCodeToTemp(t_raw, calib), valid);
}

static bool PollAnalog(MvlvHardware rev) {
  return AnalogMonitorPoll(MvlvAnalogGetConfig(rev), OutputAnalog);
}

static bool PollLtc2309(MvlvHardware rev) {
  static uint32_t device_index = 0U;
  static uint32_t config_indices[MAX_MVLV_LTC2309_DEVICES] = {0U};
  return Ltc2309MonitorPoll(MvlvLtc2309GetConfig(rev), OutputLtc2309,
                            &device_index, config_indices, g_ltc2309);
}

static bool PollMcp342x(MvlvHardware rev) {
  static uint32_t device_index = 0U;
  static uint32_t config_indices[MAX_MVLV_MCP342X_DEVICES] = {0U};
  return Mcp342xMonitorPoll(MvlvMcp342xGetConfig(rev), OutputMcp342x,
                            &device_index, config_indices, g_mcp342x);
}

void MvlvMonitorInit(void) {
  Ltc2309MonitorInit(ARRAYSIZE(g_ltc2309), g_ltc2309);
  Mcp342xMonitorInit(ARRAYSIZE(g_mcp342x), g_mcp342x);
}

bool MvlvMonitorPoll(MvlvHardware rev,
                     MvlvMonitorData *monitors) {
  static bool (*const poll[])(MvlvHardware rev) = {
    PollAnalog,
    PollLtc2309,
    PollMcp342x,
  };
  static uint32_t index = 0U;

  // Set for output functions.
  g_monitors = monitors;

  // Make certain that device_index is always valid.
  index %= ARRAYSIZE(poll);
  if (poll[index](rev)) {
    ++index;
    return true;
  }
  return false;
}
