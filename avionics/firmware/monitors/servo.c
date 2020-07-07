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

#include "avionics/firmware/monitors/servo.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/mcp342x.h"
#include "avionics/firmware/drivers/mcp9800.h"
#include "avionics/firmware/monitors/analog.h"
#include "avionics/firmware/monitors/mcp342x.h"
#include "avionics/firmware/monitors/mcp9800.h"
#include "avionics/firmware/monitors/servo_analog_types.h"
#include "avionics/firmware/monitors/servo_mcp9800_types.h"
#include "avionics/firmware/serial/servo_serial_params.h"
#include "avionics/firmware/util/timer.h"
#include "common/macros.h"

#define CLAMP_SETTLING_TIME_MS 100

static ServoMonitorData *g_monitors = NULL;
static Mcp342x g_mcp342x[MAX_SERVO_MCP342X_DEVICES];
static Mcp9800 g_mcp9800[MAX_SERVO_MCP9800_DEVICES];

static float AdcMidpoint(float v_start, float v_end) {
  return v_start + (v_end - v_start) / 2.0f;
}

typedef enum {
  kClampStatusActive,
  kClampStatusFuseBlown,
  kClampStatusNormal,
  kClampStatusResistorDisconnected,
} ClampStatus;

static ClampStatus GetCurrentClampStatus(void) {
  // Compare v_clamp sampling point across v_servo voltage dividers. The
  // expected value for v_clamp is:
  //   0 V when clamp is active (0.73242 mV / #).
  //   v_servo / 93.8 when external R disconnected.
  //   v_servo / 70.6 when fuse blown.
  //   v_servo / 47.4 when R connected, fuse normal.
  float v_servo = g_monitors->analog_data[kServoAnalogVoltageVServo];
  float v_clamp = g_monitors->analog_data[kServoAnalogVoltageClampResistor];
  float v_disconnected = v_servo / 93.8f;
  float v_fuse_blown = v_servo / 70.6f;
  float v_normal = v_servo / 47.4f;

  // Compare v_clamp to midpoint between expected values.
  // Output space: [ (active) .. (disconnected) .. (fuse blown) .. (normal) ].
  if (v_clamp < AdcMidpoint(0.0f, v_disconnected)) {
    return kClampStatusActive;
  } else if (v_clamp < AdcMidpoint(v_disconnected, v_fuse_blown)) {
    return kClampStatusResistorDisconnected;
  } else if (v_clamp < AdcMidpoint(v_fuse_blown, v_normal)) {
    return kClampStatusFuseBlown;
  } else {
    return kClampStatusNormal;
  }
}

static void SetClampStatus(void) {
  assert(g_monitors != NULL);

  static Timer clamp_status_timer;
  static ClampStatus last_clamp_status = kClampStatusNormal;
  ClampStatus clamp_status = GetCurrentClampStatus();

  if (clamp_status != last_clamp_status) {
    last_clamp_status = clamp_status;
    TimerStartMsec(CLAMP_SETTLING_TIME_MS, &clamp_status_timer);
  }

  // Clear all flags, then set those that are active.
  SetStatus(kServoMonitorStatusClampNormal | kServoMonitorStatusClampActive,
            false, &g_monitors->flags);
  SignalWarning(kServoMonitorWarningClampResistorDisconnected, false,
                &g_monitors->flags);
  SignalError(kServoMonitorErrorClampFuseBlown, false, &g_monitors->flags);

  switch (clamp_status) {
    case kClampStatusActive:
      SetStatus(kServoMonitorStatusClampActive, true, &g_monitors->flags);
      break;
    case kClampStatusNormal:
      SetStatus(kServoMonitorStatusClampNormal, true, &g_monitors->flags);
      break;
    case kClampStatusResistorDisconnected:
      if (TimerExpired(&clamp_status_timer)) {
        SignalWarning(kServoMonitorWarningClampResistorDisconnected, true,
                      &g_monitors->flags);
      }
      break;
    case kClampStatusFuseBlown:
      if (TimerExpired(&clamp_status_timer)) {
        SignalError(kServoMonitorErrorClampFuseBlown, true,
                    &g_monitors->flags);
      }
      break;
    default:
      assert(false);
  }
}

static void OutputAnalog(const AnalogMonitor *config, float in,
                         uint32_t flags) {
  assert(g_monitors != NULL);

  // Map the analog input channels to a common flags field output. The
  // warn_map[] array defines a bitmask to set or clear in SignalWarning.
  // Unspecified entries in the map default to zero, thus calling
  // SignalWarning does not alter the flags field output.
  const uint16_t warn_map[kNumServoAnalogInputs] = {
    [kServoAnalogInput12v] = kServoMonitorWarning12v,
    [kServoAnalogInput5v] = kServoMonitorWarning5v,
    [kServoAnalogInputIServo] = kServoMonitorWarningServoCurrent,
    [kServoAnalogInputLvA] = kServoMonitorWarningLvA,
    [kServoAnalogInputLvB] = kServoMonitorWarningLvB,
    [kServoAnalogInputVServo] = kServoMonitorWarningServoVoltage,
  };

  if (config->type == kAnalogTypeVoltage) {
    assert(0 <= config->voltage
           && config->voltage < ARRAYSIZE(g_monitors->analog_data));
    g_monitors->analog_populated |= 1U << config->voltage;
    g_monitors->analog_data[config->voltage] = in;
  }
  SignalWarning(warn_map[config->input], flags & ANALOG_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);
}

static bool PollAnalog(ServoHardware rev) {
  if (AnalogMonitorPoll(ServoAnalogGetConfig(rev), OutputAnalog)) {
    if (rev >= kServoHardwareRevBa) {
      SetClampStatus();
    }
    return true;
  }
  return false;
}

static void OutputMcp342x(int32_t monitor, int32_t value, bool valid) {
  assert(0 <= monitor && monitor < ARRAYSIZE(g_monitors->mcp342x_data));
  if (valid) {
    g_monitors->mcp342x_data[monitor] = value;
  }
}

static bool PollMcp342x(ServoHardware rev) {
  static uint32_t device_index = 0U;
  static uint32_t config_indices[MAX_SERVO_MCP342X_DEVICES] = {0U};
  return Mcp342xMonitorPoll(ServoMcp342xGetConfig(rev), OutputMcp342x,
                            &device_index, config_indices, g_mcp342x);
}

static void OutputMcp9800(int32_t device, float data) {
  assert(g_monitors != NULL);
  assert(0 <= device && device < ARRAYSIZE(g_monitors->mcp9800_data));
  g_monitors->mcp9800_data[device] = data;
}

static bool PollMcp9800(ServoHardware rev) {
  static uint32_t index = 0U;
  return Mcp9800MonitorPoll(ServoMcp9800GetConfig(rev), OutputMcp9800,
                            &index, g_mcp9800);
}

void ServoMonitorInit(void) {
  AnalogMonitorInit();
  Mcp342xMonitorInit(ARRAYSIZE(g_mcp342x), g_mcp342x);
  Mcp9800MonitorInit(ARRAYSIZE(g_mcp9800), g_mcp9800);
}

bool ServoMonitorPoll(ServoHardware rev, ServoMonitorData *monitors) {
  static bool (* const poll[])(ServoHardware rev) = {
    PollAnalog,
    PollMcp342x,
    PollMcp9800,
  };
  static uint32_t index = 0U;

  // Set for output functions.
  g_monitors = monitors;
  g_monitors->revision = rev;

  // Make certain that device_index is always valid.
  index %= ARRAYSIZE(poll);
  if (poll[index](rev)) {
    ++index;      // Process all devices concurrently.
    return true;  // Set true when done polling. Data may not update.
  }
  return false;
}
