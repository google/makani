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

#include "control/manual/manual.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/linalg.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/manual/manual_output.h"
#include "control/manual/manual_types.h"
#include "control/system_params.h"
#include "system/labels.h"

void ManualInit(const StateEstimate *state_est, const ManualParams *params,
                ManualState *state) {
  assert(state_est != NULL && params != NULL && state != NULL);
  assert(ManualValidateParams(params));

  memset(state, 0, sizeof(*state));
  state->release_latched = false;
}

bool ManualValidateParams(const ManualParams *params) {
  assert(params != NULL);

  if ((params->auto_glide.pitch_pid.ki != 0.0) ||
      (params->auto_glide.roll_pid.ki != 0.0)) {
    assert(!(
        bool)"auto_glide.pitch_pid.ki and auto_glide.roll_pid.ki must be "
             "zero.");
    return false;
  }
  return true;
}

// Compute the kite's attitude with respect to an auto-glide frame
// defined by wings-level attitude and a fixed pitch attitude with
// respect to the ground.
//
// TODO: Write a test for this function.
static void CalcAutoGlideAngles(const Mat3 *dcm_g2b,
                                const ManualAutoGlideParams *params,
                                double *pitch_error, double *roll_error) {
  double unused_yaw, pitch, roll;
  DcmToAngle(dcm_g2b, kRotationOrderZyx, &unused_yaw, &pitch, &roll);
  *pitch_error = params->pitch_angle - pitch;
  *roll_error = params->roll_angle - roll;
}

// Using feedback to the elevator and ailerons, attempt to maintain a
// constant glide-slope.
static void AddAutoGlideDeltas(const Mat3 *dcm_g2b, const Vec3 *pqr_f,
                               const ManualAutoGlideParams *params,
                               Deltas *deltas) {
  double pitch_error, roll_error;
  CalcAutoGlideAngles(dcm_g2b, params, &pitch_error, &roll_error);

  double unused_int_output = 0.0;
  deltas->elevator +=
      Pid(pitch_error, -pqr_f->y, *g_sys.ts, kIntegratorModeReset,
          &params->pitch_pid, &unused_int_output);
  deltas->aileron += Pid(roll_error, -pqr_f->x, *g_sys.ts, kIntegratorModeReset,
                         &params->roll_pid, &unused_int_output);
  deltas->rudder += params->yaw_rate_gain *
                    (pqr_f->z - params->roll_crossfeed_gain * roll_error);
  deltas->inboard_flap = 0.0;
  deltas->midboard_flap = 0.0;
  deltas->outboard_flap = 0.0;

  // Update telemetry.
  GetManualTelemetry()->pitch_error = pitch_error;
  GetManualTelemetry()->roll_error = roll_error;
}

// Compute control surfaces deflections based on joystick inputs.
static void AddPilotDeltas(const JoystickEstimate *joystick,
                           const Deltas *flap_gains, Deltas *deltas) {
  if (joystick->valid) {
    deltas->aileron += flap_gains->aileron * joystick->data.roll;
    deltas->elevator += flap_gains->elevator * joystick->data.pitch;
    deltas->rudder += flap_gains->rudder * joystick->data.yaw;
    deltas->inboard_flap = 0.0;
    deltas->midboard_flap = 0.0;
    deltas->outboard_flap = 0.0;
  } else {
    deltas->aileron = 0.0;
    deltas->elevator = 0.0;
    deltas->rudder = 0.0;
    deltas->inboard_flap = 0.0;
    deltas->midboard_flap = 0.0;
    deltas->outboard_flap = 0.0;
  }
}

// Off-tether manual mode offers pilot control of the flight control
// surfaces via the joystick and a stability-enhancement system called
// auto-glide.
void ManualStep(const FlightStatus *flight_status,
                const StateEstimate *state_est, const ManualParams *params,
                ManualState *state, ControlOutput *control_output) {
  assert(flight_status != NULL && state_est != NULL && params != NULL &&
         state != NULL && control_output != NULL);

  // This flight mode should only run when the tether is released.
  state->release_latched = true;

  // Auto-glide is active if tether was released a while ago and the
  // tether release button is pushed.
  bool auto_glide_active =
      state->release_latched &&
      (flight_status->flight_mode == kFlightModeOffTether) &&
      state_est->joystick.data.engage_auto_glide;

  Deltas deltas = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  ThrustMoment thrust_moment = {0.0, {0.0, 0.0, 0.0}};
  if (auto_glide_active) {
    AddAutoGlideDeltas(&state_est->dcm_g2b, &state_est->pqr_f,
                       &params->auto_glide, &deltas);
  }
  AddPilotDeltas(&state_est->joystick, &params->joystick_flap_gains, &deltas);

  ManualOutputStep(&deltas, &thrust_moment, &params->output,
                   &flight_status->flight_mode_time, control_output);

  // Update telemetry.
  GetManualTelemetry()->auto_glide_active = auto_glide_active;
  GetManualTelemetry()->release_latched = state->release_latched;
}
