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

#include "control/control_system.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/build_info.h"
#include "avionics/linux/clock.h"
#include "common/c_math/filter.h"
#include "common/c_math/util.h"
#include "common/macros.h"
#include "control/actuator_util.h"
#include "control/avionics/avionics_faults.h"
#include "control/avionics/avionics_interface.h"
#include "control/avionics/avionics_interface_types.h"
#include "control/common.h"
#include "control/control_output.h"
#include "control/control_params.h"
#include "control/control_planner.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind.h"
#include "control/crosswind/crosswind_types.h"
#include "control/estimator/estimator.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_disabled.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "control/hover/hover.h"
#include "control/manual/manual.h"
#include "control/sensor_types.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "control/trans_in/trans_in.h"
#include "experiments/hover_experiment_types.h"
#include "system/labels.h"

const ControllerType kReadyForModeOwner[] =
    {[kFlightModePilotHover] = kControllerNone,
     [kFlightModePerched] = kControllerHover,
     [kFlightModeHoverAscend] = kControllerHover,
     [kFlightModeHoverPayOut] = kControllerHover,
     [kFlightModeHoverFullLength] = kControllerHover,
     [kFlightModeHoverPrepTransformGsUp] = kControllerHover,
     [kFlightModeHoverTransformGsUp] = kControllerHover,
     [kFlightModeHoverAccel] = kControllerHover,
     [kFlightModeTransIn] = kControllerTransIn,
     [kFlightModeCrosswindNormal] = kControllerCrosswind,
     [kFlightModeCrosswindPrepTransOut] = kControllerCrosswind,
     [kFlightModeHoverTransOut] = kControllerCrosswind,
     [kFlightModeHoverPrepTransformGsDown] = kControllerHover,
     [kFlightModeHoverTransformGsDown] = kControllerHover,
     [kFlightModeHoverReelIn] = kControllerHover,
     [kFlightModeHoverDescend] = kControllerHover,
     [kFlightModeOffTether] = kControllerNone};

COMPILE_ASSERT(ARRAYSIZE(kReadyForModeOwner) == kNumFlightModes,
               kFlightModeOwner_must_have_length_kNumFlightModes);

void ControlSystemInit(const ControlParams *params, ControlState *state) {
  assert(params != NULL && state != NULL);
  assert(ActuatorUtilValidateParams(&params->rotor_control));

  memset(state, 0, sizeof(*state));

  state->init_state = kInitializationStateFirstEntry;
  memset(&state->input_messages, 0, sizeof(state->input_messages));
  state->time = 0.0;
  for (int32_t i = 0; i < kNumSubsystems; ++i) {
    ClearAllFaults(&state->faults[i]);
  }

  PlannerInit(&state->planner);
  ControlOutputInit(&params->control_output, &state->control_output);
}

// Compute the next initialization state from the previous state.
static InitializationState GetNextInitState(FlightPlan flight_plan,
                                            const FaultMask faults[],
                                            InitializationState init_state) {
  // Update init_state.
  switch (init_state) {
    case kInitializationStateFirstEntry:
      return kInitializationStateWaitForValidData;

    case kInitializationStateWaitForValidData:
      return EstimatorIsDataReady(flight_plan, faults)
                 ? kInitializationStateFirstLoop
                 : init_state;

    case kInitializationStateFirstLoop:
      return kInitializationStateRunning;

    default:
      assert(false);  // Fall-through intentional.
    case kInitializationStateRunning:
      return init_state;
  }
}

static void QueryReadyStates(const FlightStatus *flight_status,
                             const StateEstimate *state_est,
                             const ControlParams *params,
                             const ControlState *state, bool ready_states[]) {
  for (int32_t i = 0; i < kNumFlightModes; ++i) {
    switch (kReadyForModeOwner[i]) {
      case kControllerHover:
        ready_states[i] = HoverIsReadyForMode(i, flight_status, state_est,
                                              &params->hover, &state->hover);
        break;
      case kControllerCrosswind:
        ready_states[i] = CrosswindIsReadyForMode(
            i, flight_status, state_est, &params->crosswind, &state->crosswind);
        break;
      case kControllerTransIn:
        ready_states[i] = TransInIsReadyForMode(
            i, flight_status, state_est, &params->trans_in, &state->trans_in);
        break;

      default:
      case kControllerForceSigned:
        assert(false);  // Fall-through intentional.
      case kControllerManual:
      case kControllerNone:
        ready_states[i] = false;
        break;
    }
  }
}

static bool IsEnteringHover(const FlightStatus *flight_status) {
  return (flight_status->flight_mode == kFlightModePerched ||
          AnyHoverFlightMode(flight_status->flight_mode)) &&
         !(flight_status->last_flight_mode == kFlightModePerched ||
           AnyHoverFlightMode(flight_status->last_flight_mode)) &&
         flight_status->flight_mode_first_entry;
}

static bool IsEnteringTransIn(const FlightStatus *flight_status) {
  return flight_status->flight_mode == kFlightModeTransIn &&
         flight_status->flight_mode_first_entry;
}

static bool IsEnteringCrosswind(const FlightStatus *flight_status) {
  return AnyCrosswindFlightMode(flight_status->flight_mode) &&
         !AnyCrosswindFlightMode(flight_status->last_flight_mode) &&
         flight_status->flight_mode_first_entry;
}

// Runs the controller after data has been converted to SI format.
// TODO: This function does not check state_est->joystick.valid.
static void RunController(const FlightStatus *flight_status,
                          const StateEstimate *state_est,
                          const ControlParams *params, ControlState *state,
                          bool ready_states[], ControlOutput *control_output) {
  const double *flaps_z1 =
      ControlOutputGetLastFlapCommand(&state->control_output);

  if (state->init_state == kInitializationStateFirstLoop) {
    HoverInit(state_est, NULL, 0.0, 0.0, flaps_z1, &kVec3Zero, &params->hover,
              &state->hover);
    TransInInit(state_est, &kVec3Zero, &params->trans_in, &state->trans_in);
    CrosswindInit(state_est, flaps_z1, 0.0, 0, 0.0, &params->crosswind,
                  &state->hover, &state->crosswind);
    ManualInit(state_est, &params->manual, &state->manual);
  }

  if (IsEnteringHover(flight_status)) {
    Vec3 previous_perched_pos_g = state->hover.path.perched_pos_g;
    Vec3 previous_loop_center_pos_v =
        state->crosswind.output.path_center_v_f_z1;
    double previous_int_yaw_angle = state->hover.position.int_angles.z;

    double previous_thrust_cmd;
    if (AnyCrosswindFlightMode(flight_status->last_flight_mode)) {
      previous_thrust_cmd = state->crosswind.inner.thrust_cmd_z1;
    } else if (flight_status->last_flight_mode == kFlightModeTransIn) {
      previous_thrust_cmd = params->trans_in.longitudinal.thrust_cmd;
    } else {
      // This should never happen.
      previous_thrust_cmd = g_sys.wing->m * g_sys.phys->g;
      assert(false);
    }

    HoverInit(state_est, &previous_perched_pos_g, previous_int_yaw_angle,
              previous_thrust_cmd, flaps_z1, &previous_loop_center_pos_v,
              &params->hover, &state->hover);
  }
  if (IsEnteringTransIn(flight_status)) {
    TransInInit(state_est, &HoverGetLastThrustMomentCmd(&state->hover)->moment,
                &params->trans_in, &state->trans_in);
  }
  if (IsEnteringCrosswind(flight_status)) {
    // The detwist loop angle and revolution count are preserved from the last
    // time the crosswind controller ran.
    CrosswindInit(state_est, flaps_z1,
                  state->crosswind.output.detwist_loop_angle,
                  state->crosswind.output.detwist_rev_count,
                  state->crosswind.inner.accumulated_loop_angle,
                  &params->crosswind, &state->hover, &state->crosswind);
  }

  // Select controller based on flight mode.
  ControlOutput raw_control_output;
  memset(&raw_control_output, 0, sizeof(raw_control_output));

  switch (flight_status->flight_mode) {
    default:
    case kFlightModeForceSigned:
    case kNumFlightModes:
      assert(false);  // Fall-through intentional.
    case kFlightModePerched:
    case kFlightModePilotHover:
    case kFlightModeHoverAscend:
    case kFlightModeHoverPayOut:
    case kFlightModeHoverPrepTransformGsUp:
    case kFlightModeHoverTransformGsUp:
    case kFlightModeHoverFullLength:
    case kFlightModeHoverAccel:
    case kFlightModeHoverTransOut:
    case kFlightModeHoverPrepTransformGsDown:
    case kFlightModeHoverTransformGsDown:
    case kFlightModeHoverReelIn:
    case kFlightModeHoverDescend:
      HoverStep(flight_status, state_est, &params->crosswind.playbook,
                &params->hover, &state->hover, &raw_control_output);
      break;

    case kFlightModeTransIn:
      TransInStep(flight_status, state_est, &params->trans_in, &state->trans_in,
                  &raw_control_output);
      break;

    case kFlightModeCrosswindNormal:
    case kFlightModeCrosswindPrepTransOut:
      CrosswindStep(flight_status, state_est, &params->crosswind,
                    &state->crosswind, &raw_control_output);
      break;

    case kFlightModeOffTether:
      ManualStep(flight_status, state_est, &params->manual, &state->manual,
                 &raw_control_output);
      break;
  }

  ControlOutputStep(&raw_control_output, &state_est->joystick,
                    &state_est->perch_azi, &params->control_output,
                    &state->control_output, control_output);

  // Query controllers for readiness for each flight mode.
  QueryReadyStates(flight_status, state_est, params, state, ready_states);
}

// Run a single loop of the entire control system, which includes the
// conversion to and from the avionics packets, the controller itself,
// and the fault detection module.
bool ControlSystemStep(ControllerLabel controller_label,
                       const SystemParams *system_params,
                       const ControlParams *control_params, ControlState *state,
                       ControllerCommandMessage *command_message,
                       ControllerSyncMessage *sync_message) {
  if (controller_label < 0 || kNumControllers <= controller_label) {
    assert(false);
    return false;
  }

  // Log the loop start.
  int64_t start_usec = ClockGetUs();

  // Convert avionics messages to control input.
  if (state->init_state == kInitializationStateFirstEntry) {
    AvionicsInterfaceInit(&state->avionics_interface);
  }
  ControlInput control_input;
  ConvertAvionicsToControl(system_params, &control_params->sensor_limits,
                           &control_params->fault_detection,
                           &state->avionics_interface, &state->input_messages,
                           &control_input, state->faults);

  SetDisabledFaults(&control_params->fault_detection.disabled, state->faults);

  // Update initialization state.
  InitializationState next_init_state = GetNextInitState(
      system_params->flight_plan, state->faults, state->init_state);
  bool send_messages = IsControlSystemRunning(state->init_state);

  // Do not run the estimator and controller until a valid message has
  // been received from all critical sensors.  However, continue to
  // send telemetry indicating the current initialization state.
  if (IsControlSystemRunning(state->init_state)) {
    const FlightStatus *flight_status = PlannerGetFlightStatus(&state->planner);

    if (state->init_state == kInitializationStateFirstLoop) {
      EstimatorInit(system_params, &control_params->estimator,
                    &state->estimator);
    }

    StateEstimate state_est;
    EstimatorStep(flight_status, &control_input, system_params, state->faults,
                  &control_params->estimator, &state->estimator, &state_est);

    bool ready_states[kNumFlightModes];
    ControlOutput control_output;
    RunController(flight_status, &state_est, control_params, state,
                  ready_states, &control_output);

    PlannerStep(controller_label, control_input.sync, &state_est, state->faults,
                ready_states, control_input.force_hover_accel,
                &control_params->planner, &control_params->joystick_control,
                &control_output, &state->planner, &control_output.sync);

    // Convert the control output back to avionics messages.
    ConvertControlToAvionics(&control_output, system_params, command_message,
                             sync_message);

    // Update telemetry.
    ControlTelemetry *ct = GetControlTelemetry();
    ct->state_est = state_est;
    ct->control_output = control_output;
    ct->command_message = *command_message;
    ct->sync_message = *sync_message;
    ct->detwist_loop_count =
        state_est.tether_ground_angles.accumulated_detwist_angle / 2.0 / PI;
  }

  // Log the completion time.
  int64_t finish_usec = ClockGetUs();

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->controller_label = controller_label;
  ct->init_state = state->init_state;
  ct->time = state->time;
  for (int32_t i = 0; i < kNumSubsystems; ++i) {
    ct->faults[i] = state->faults[i];
  }
  ct->start_usec = start_usec;
  ct->finish_usec = finish_usec;
  ct->sequence_numbers = state->avionics_interface.sequence_numbers;
  ct->control_input = control_input;

  // Update slow telemetry.
  ControlSlowTelemetry *cst = GetControlSlowTelemetry();
  cst->controller_label = controller_label;
  cst->flight_plan = *g_cont.flight_plan;
  cst->gs_model = system_params->gs_model;
  cst->test_site = system_params->test_site;
  cst->wing_serial = system_params->wing_serial;
  cst->control_opt = GetControlParams()->control_opt;
  GetBuildInfo(&cst->build_info);

  // Update small telemetry.
  ControlTelemetryToSmallControlTelemetryMessage(GetControlTelemetry(),
                                                 GetSmallControlTelemetry());

  // Update persistent state.
  state->init_state = next_init_state;
  state->time += *g_sys.ts;

  return send_messages;
}
