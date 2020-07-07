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

#include "avionics/servo/firmware/output.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/bits.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/drivers/ina219_types.h"
#include "avionics/firmware/drivers/log.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"
#include "avionics/servo/firmware/control.h"
#include "avionics/servo/firmware/input.h"
#include "common/c_math/util.h"
#include "common/macros.h"

static ServoStatusMessage g_status;
static ServoPairedStatusMessage g_paired_status;
static int32_t g_servo_status_count;

void ServoOutputInit(void) {
  memset(&g_status, 0, sizeof(g_status));
  memset(&g_paired_status, 0, sizeof(g_paired_status));
  g_servo_status_count = 0;
}

AioModuleMonitorData *ServoOutputGetAioModuleMonitors(void) {
  return &g_status.aio_mon;
}

ServoMonitorData *ServoOutputGetServoMonitors(void) {
  return &g_status.servo_mon;
}

static void ServoOutputR22StatusBits(uint32_t status_bits) {
  g_status.r22.status_bits = status_bits;
}

static void ServoOutputR22Current(const ServoMeasurement *current) {
  assert(current != NULL);
  g_status.r22.current = current->raw;
}

static void ServoOutputR22Angle(float angle, int32_t raw) {
  g_status.r22.angle = raw;
  g_status.angle_measured = angle;
}

static void ServoOutputR22Velocity(const ServoMeasurement *velocity) {
  assert(velocity != NULL);
  g_status.r22.angular_velocity = velocity->raw;
  g_status.angular_velocity = velocity->value;
}

static void ServoOutputR22Temperature(int16_t temperature) {
  g_status.r22.temperature = temperature;
}

static void ServoOutputControlState(const ServoControlState *control) {
  assert(control != NULL);
  g_status.angle_estimate = control->angle_estimate;
  g_status.angle_variance = control->angle_variance;
  g_status.angle_bias = control->angle_bias;
  g_status.angle_feedback = control->angle_feedback;

  g_status.flags.error = control->flags.error;
  g_status.flags.warning = control->flags.warning;
  g_status.flags.status = control->flags.status;
}

static void ServoOutputControlOutput(const ServoControlState *control) {
  assert(control != NULL);
  g_status.r22.current_limit = ServoInvertCurrentCal(control->current_limit);
  g_status.angle_desired = control->desired_angle;
}

void ServoOutputErrorLog(int32_t data_length, const uint32_t *data) {
  static ServoErrorLogMessage msg;

  memset(&msg, 0, sizeof(msg));
  int32_t entries = SERVO_ERROR_LOG_ENTRIES;
  if (entries > data_length/2) {
    entries = data_length/2;
  }
  for (int32_t i = 0; i < entries; ++i) {
    msg.data[i].event = (data[2*i + 0] >> 26);
    msg.data[i].seconds = (data[2*i + 0] & 0x3FFFFFF);
    msg.data[i].error_bits = data[2*i + 1];
    LOG_PRINTF("R22 log event=%d errors=0x%08lX.\n",
               msg.data[i].event, msg.data[i].error_bits);
  }
  NetSendAioServoErrorLogMessage(&msg);
}

static void NetSendAioServoPairedStatusMessage(ServoPairedStatusMessage *msg) {
  switch (AppConfigGetAioNode()) {
    case kAioNodeServoE1:
    case kAioNodeServoE2:
      NetSendAioServoPairedStatusElevatorMessage(msg);
      return;
    case kAioNodeServoR1:
    case kAioNodeServoR2:
      NetSendAioServoPairedStatusRudderMessage(msg);
      return;
    default:
      return;
  }
}

void ServoOutputSendStatusMessage(ServoState *state) {
  ServoOutputR22StatusBits(state->input.r22.r22_status_bits);
  ServoOutputR22Current(&state->input.r22.current);
  ServoOutputR22Angle(state->input.r22.angle, state->input.r22.angle_raw);
  ServoOutputR22Velocity(&state->input.r22.velocity);
  ServoOutputR22Temperature(state->input.r22.temperature);
  ServoOutputControlState(&state->control);
  ServoOutputControlOutput(&state->control);
  g_paired_status.input = state->input;
  g_paired_status.control_state = state->control;
  g_paired_status.state = (uint8_t)state->state;
  g_status.state = (uint8_t)state->state;
  g_status.cmd_arbiter.controllers_used = state->input.controllers_used;
  g_paired_status.latency_usec = SaturateInt32(
      (int32_t)(ClockGetUs() - state->sync_timestamp), -1e6, 1e6);
  NetSendAioServoPairedStatusMessage(&g_paired_status);
  NetSendAioServoDebugMessage(&g_status);
  if (++g_servo_status_count >= 10) {
    g_servo_status_count = 0;
    NetSendAioServoStatusMessage(&g_status);
  }
  state->input.controllers_used = 0;
}
