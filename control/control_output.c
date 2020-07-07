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

#include "control/control_output.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/util.h"
#include "common/macros.h"
#include "control/actuator_util.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/system_params.h"
#include "system/labels.h"

void ControlOutputInit(const ControlOutputParams *params,
                       ControlOutputState *state) {
  assert(params != NULL && state != NULL);
  (void)params;

  state->run_motors_latched = false;
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    state->flaps_z1[i] = 0.0;
  }

  state->last_gs_azi_cmd_valid = false;
  state->gs_azi_cmd_z1.target = 0.0;
  state->gs_azi_cmd_z1.dead_zone = 0.0;
}

const double *ControlOutputGetLastFlapCommand(const ControlOutputState *state) {
  return state->flaps_z1;
}

static void LimitOutput(const ControlOutput *raw_output,
                        const JoystickData *joystick,
                        const ControlOutputParams *params,
                        ControlOutputState *state,
                        ControlOutput *final_output) {
  const JoystickControlParams *joystick_params = g_cont.joystick_control;
  const RotorControlParams *rotor_params = g_cont.rotor_control;

  // Final saturations on motors and flaps.  There may be other places
  // in the code where tighter limits are set, but these hard-stops
  // are to ensure the actuator limits are never exceeded.
  for (int32_t i = 0; i < kNumMotors; ++i) {
    final_output->motor_speed_upper_limit[i] =
        Saturate(raw_output->motor_speed_upper_limit[i],
                 rotor_params->idle_speed, rotor_params->max_speeds[i]);
    final_output->motor_speed_lower_limit[i] =
        Saturate(raw_output->motor_speed_lower_limit[i],
                 rotor_params->idle_speed, rotor_params->max_speeds[i]);
    // TODO: Come up with reasonable saturation limits to apply to
    // motor_torques.  Should be +/- controller maximum.
    final_output->motor_torque[i] =
        Saturate(raw_output->motor_torque[i], rotor_params->min_torque_command,
                 rotor_params->max_torque_command);
  }

  for (int32_t i = 0; i < kNumFlaps; ++i) {
    final_output->flaps[i] = Saturate(
        raw_output->flaps[i], params->flaps_min[i], params->flaps_max[i]);
    final_output->flaps[i] =
        RateLimit(final_output->flaps[i], -params->rate_limit,
                  params->rate_limit, *g_sys.ts, &state->flaps_z1[i]);
  }

  // TODO: Remove the joystick_params->e_stop_throttle parameter
  // entirely. To do so we will have to rework this logic for setting
  // run_motors_latched at the start of flight.
  if (joystick->throttle > joystick_params->e_stop_throttle) {
    // The motors go into an error state if they receive a run command
    // when not armed.  At the same time, we want the controller to be
    // running before we arm the motors.  This code latches the
    // run_motors flag to high after the first time the software
    // E-STOP is released.
    //
    // TODO: Update this code when the final motor arming
    // sequence is decided.
    state->run_motors_latched = true;
  }

  // If stop_motors from the inner loop is true, it will override run_motors
  // when the motor command is generated.
  final_output->stop_motors = raw_output->stop_motors;
  final_output->run_motors = state->run_motors_latched;
}

void ControlOutputStep(const ControlOutput *raw_control_output,
                       const JoystickEstimate *joystick,
                       const PerchAziEstimate *perch_azi,
                       const ControlOutputParams *params,
                       ControlOutputState *state,
                       ControlOutput *control_output) {
  *control_output = *raw_control_output;

  // Apply hard limits to outputs.
  //
  // TODO: Check joystick->valid flag.
  LimitOutput(raw_control_output, &joystick->data, params, state,
              control_output);

  if (control_output->hold_gs_azi_cmd) {
    // If we are not actively commanding GS azi command, and there hasn't been
    // a valid one yet, use the GS position.
    if (!state->last_gs_azi_cmd_valid) {
      // Make the dead zone a large value to avoid moving the ground station.
      control_output->gs_azi_cmd.dead_zone = GS_AZI_DEAD_ZONE_MAX;
      if (perch_azi->valid) {
        control_output->gs_azi_cmd.target =
            perch_azi->angle -
            GetSystemParams()->ground_station.gs02.reel_azi_offset_from_wing;
      } else {
        // TODO: We can't avoid commanding the gs if the perch azi
        // estimate is invalid, so 0.0 is the best we can do.  The GS will slew
        // at this point.
        control_output->gs_azi_cmd.target = 0.0;
      }
    } else {
      control_output->gs_azi_cmd = state->gs_azi_cmd_z1;
    }
  } else {
    state->last_gs_azi_cmd_valid = true;
  }
  control_output->gs_azi_cmd.target =
      Wrap(control_output->gs_azi_cmd.target, 0.0, 2.0 * PI);
  state->gs_azi_cmd_z1 = control_output->gs_azi_cmd;
}
