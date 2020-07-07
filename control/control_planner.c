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

#include "control/control_planner.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/util.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"
#include "control/system_params.h"
#include "system/labels.h"

void PlannerInit(PlannerState *state) {
  assert(state != NULL);

  memset(state, 0, sizeof(*state));

  state->flight_status.flight_mode = kFlightModePilotHover;
  state->flight_status.last_flight_mode = kFlightModePilotHover;
  state->flight_status.flight_mode_first_entry = true;
  state->flight_status.flight_mode_time = 0.0;

  FlightPlan flight_plan = *g_cont.flight_plan;
  if (flight_plan == kFlightPlanManual) {
    state->autonomous_flight_mode = kFlightModeOffTether;
  } else if (flight_plan == kFlightPlanHighHover ||
             flight_plan == kFlightPlanLaunchPerch ||
             flight_plan == kFlightPlanTurnKey) {
    state->autonomous_flight_mode = kFlightModePerched;
  } else if (flight_plan == kFlightPlanDisengageEngage) {
    state->autonomous_flight_mode = kFlightModeHoverPayOut;
  } else {
    state->autonomous_flight_mode = kFlightModeHoverFullLength;
  }
  state->flight_mode_zero_throttle_timer = 0.0;
  state->controller_sync_sequence = 0U;
  state->force_hover_accel_latched = false;

  state->desired_flight_mode = kFlightModePerched;
  state->throttle_gate_satisfied = false;
  state->enable_autonomous_operation = false;
  state->flight_status.fully_autonomous_mode = false;
}

const FlightStatus *PlannerGetFlightStatus(const PlannerState *state) {
  assert(state != NULL);
  return &state->flight_status;
}

static bool IsInWindWindow(const WindWindow *window,
                           const WindEstimate *wind_est) {
  assert(window != NULL && wind_est != NULL);
  assert(InsideRange(window->wind_azi_min, 0.0, 2.0 * PI));
  assert(InsideRange(window->wind_azi_max, 0.0, 2.0 * PI));
  assert(window->wind_speed_min >= 0.0 && window->wind_speed_max >= 0.0);

  // Convert [-pi, pi) TO wind direction to [0, 2pi) FROM wind direction by
  // adding a half revolution.
  double wind_dir = wind_est->dir_f_playbook + PI;
  assert(InsideRange(wind_dir, 0.0, 2.0 * PI));

  if (!InsideRangeWrapped(wind_dir, 0, 2 * PI, window->wind_azi_min,
                          window->wind_azi_max)) {
    return false;
  }

  return InsideRange(wind_est->speed_f_playbook, window->wind_speed_min,
                     window->wind_speed_max);
}

static bool IsLandingFaultDetected(const FaultMask faults[]) {
  // TODO(b/120917437): Support operation with multiple flight controllers.
  // TODO(b/146355187): Respond to subsystem faults to make landing decision.
  return HasAnyFault(&faults[SUBSYS_CONTROLLERS + kControllerA]);
}

static void UpdateAutonomousFlightCriteria(const StateEstimate *state_est,
                                           const FaultMask faults[],
                                           const PlannerParams *planner_params,
                                           PlannerState *state) {
  state->inside_launch_window =
      IsInWindWindow(&planner_params->takeoff, &state_est->wind_g);
  state->inside_landing_window =
      IsInWindWindow(&planner_params->landing, &state_est->wind_g);
  // In crosswind, use the playbook wind estimate for landing as well.
  if (AnyCrosswindFlightMode(state->flight_status.flight_mode)) {
    state->inside_landing_window &=
        IsInWindWindow(&planner_params->landing, &state_est->wind_aloft_g);
  }
  state->landing_fault_detected = IsLandingFaultDetected(faults);
}

static FlightMode GetDesiredFlightModeAutonomous(
    bool first_entry, const PlannerParams *planner_params,
    PlannerState *state) {
  // Reset the takeoff timer whenever we are outside the launch window.
  if (!state->inside_launch_window || first_entry) {
    state->autonomous_takeoff_countdown_timer = planner_params->takeoff_delay;
  }

  if (state->landing_fault_detected) {
    return kFlightModePerched;
  }
  // The landing window is the range we must stay in to continue flight.  If
  // we exit the landing window, we return to the perch.
  if (!state->inside_landing_window) {
    return kFlightModePerched;
  }
  // If we are inside the launch window and on the perch, countdown using the
  // takeoff timer and then launch.  Otherwise, go back to crosswind, in case we
  // were landing.
  if (state->inside_launch_window) {
    if (state->autonomous_flight_mode == kFlightModePerched) {
      state->autonomous_takeoff_countdown_timer -= *g_sys.ts;
      if (state->autonomous_takeoff_countdown_timer <= 0) {
        state->autonomous_takeoff_countdown_timer = 0.0;
        return kFlightModeCrosswindNormal;
      } else {
        return kFlightModePerched;
      }

    } else {
      return kFlightModeCrosswindNormal;
    }
  }

  // No change to wind window -- do nothing.
  return state->desired_flight_mode;
}

static FlightMode UpdateDesiredFlightMode(
    const StateEstimate *state_est, const FaultMask faults[],
    const PlannerParams *planner_params,
    const JoystickControlParams *joystick_params, PlannerState *state) {
  const JoystickData *joystick = &state_est->joystick.data;
  bool throttle_gate_satisfied = false;

  // Determine autonomous launch/land criteria so they are updated even if the
  // autonomous controller is not running.
  UpdateAutonomousFlightCriteria(state_est, faults, planner_params, state);

  if (joystick->switch_position == kJoystickSwitchPositionUp) {
    // Pilot hover mode: Joystick controls hover pitch/yaw/throttle.
    state->desired_flight_mode = kFlightModePilotHover;
    state->enable_autonomous_operation = false;
  } else if (joystick->switch_position == kJoystickSwitchPositionMiddle) {
    // Semi-autonomous mode: Use throttle to direct the kite state machine.
    if (joystick->throttle > joystick_params->ascend_payout_throttle) {
      state->desired_flight_mode = kFlightModeCrosswindNormal;
      throttle_gate_satisfied = true;
    } else if (joystick->throttle < joystick_params->descend_reel_in_throttle) {
      state->desired_flight_mode = kFlightModePerched;
      throttle_gate_satisfied = true;
    } else if (joystick->throttle < joystick_params->prep_trans_out_throttle) {
      state->desired_flight_mode = kFlightModeHoverTransOut;
    }
    state->enable_autonomous_operation = false;
  } else if (joystick->switch_position == kJoystickSwitchPositionDown) {
    // Fully autonomous mode: Use throttle to enable flight or request landing.
    bool autonomous_first_entry = false;
    throttle_gate_satisfied = true;
    if (joystick->throttle > joystick_params->ascend_payout_throttle) {
      if (!state->enable_autonomous_operation) {
        autonomous_first_entry = true;
      }
      state->enable_autonomous_operation = true;
    } else if (joystick->throttle < joystick_params->descend_reel_in_throttle) {
      state->enable_autonomous_operation = false;
    }
    if (state->enable_autonomous_operation) {
      state->desired_flight_mode = GetDesiredFlightModeAutonomous(
          autonomous_first_entry, planner_params, state);
    } else {
      state->desired_flight_mode = kFlightModePerched;
    }
  } else {
    assert(false);
  }

  state->throttle_gate_satisfied = throttle_gate_satisfied;
  return state->desired_flight_mode;
}

static FlightMode ProposeNextFlightMode(
    FlightMode desired_flight_mode, const StateEstimate *state_est,
    const GroundStationMode gs_mode_request, const bool ready_states[],
    bool force_hover_accel, const JoystickControlParams *joystick_params,
    PlannerState *state) {
  FlightPlan flight_plan = *g_cont.flight_plan;
  const JoystickData *joystick = &state_est->joystick.data;

  const bool actually_force_hover_accel =
      force_hover_accel || state->force_hover_accel_latched;

  // Prevent momentary throttle drop from causing inadvertent switch
  // to the perched state.
  if (joystick->throttle < joystick_params->e_stop_throttle) {
    state->flight_mode_zero_throttle_timer += *g_sys.ts;
  } else {
    state->flight_mode_zero_throttle_timer = 0.0;
  }

  FlightMode flight_mode;

  if (flight_plan == kFlightPlanManual) {
    // For kFlightPlanManual, we always stay in kFlightPlanOffTether.
    flight_mode = kFlightModeOffTether;
    state->autonomous_flight_mode = kFlightModeOffTether;
  } else if (desired_flight_mode == kFlightModePilotHover &&
             !AnyCrosswindFlightMode(state->autonomous_flight_mode)) {
    // Piloted hover mode.
    flight_mode = kFlightModePilotHover;

    // Initialize the autonomous state machine.
    switch (flight_plan) {
      case kFlightPlanHighHover:
      case kFlightPlanLaunchPerch:
      case kFlightPlanTurnKey:
        if (state->flight_mode_zero_throttle_timer >
            joystick_params->e_stop_throttle_latch_time) {
          state->autonomous_flight_mode = kFlightModePerched;
        } else if (actually_force_hover_accel) {
          // Latch the hover accel command in pilot hover, so it remains active
          // until we return to autonomous.
          state->force_hover_accel_latched = true;
          state->autonomous_flight_mode = kFlightModeHoverAccel;
        } else {
          // We debated what this should switch back to.  Here are our
          // thoughts (We chose option 3):
          // Option 1: Switch to previous auto state.  This will
          //   cause a sudden decrease in throttle if we're in
          //   kFlightModePerched mode.  If we're in
          //   kFlightModeHoverFullLength, we won't be able to
          //   switch back to kFlightModeHoverReelIn.
          //
          // Option 2: Switch to kFlightModeHoverPayOut.  If we were
          //   previously in kFlightModeHoverFullLength, this will
          //   instantly switch back to
          //   kFlightModeHoverFullLength, and we won't be able to
          //   switch back to kFlightModeHoverReelIn.
          //
          // Option 3: Switch to kFlightModeHoverReelIn.  This seems
          //   to work.  If you need to switch back to
          //   kFlightModeHoverPayOut, just bring the throttle above
          //   ascend_payout_throttle.

          // TODO: Update the comment above.

          // Addendum to the above: With the addition of GSv2, we need
          // to end up in a flight mode compatible with the current
          // ground station state.
          if (state_est->gs_mode == kGroundStationModeTransform) {
            if (gs_mode_request == kGroundStationModeHighTension) {
              state->autonomous_flight_mode = kFlightModeHoverTransformGsUp;
            } else {
              assert(gs_mode_request == kGroundStationModeReel);
              state->autonomous_flight_mode = kFlightModeHoverTransformGsDown;
            }
          } else if (state_est->gs_mode == kGroundStationModeHighTension) {
            state->autonomous_flight_mode = kFlightModeHoverFullLength;
          } else {
            state->autonomous_flight_mode = kFlightModeHoverReelIn;
          }
        }
        break;
      case kFlightPlanDisengageEngage:
        state->autonomous_flight_mode = kFlightModeHoverReelIn;
        break;
      case kFlightPlanHoverInPlace:
      case kFlightPlanStartDownwind:
        // TODO: This might not be the right choice. Not sure.
        state->autonomous_flight_mode = kFlightModeHoverFullLength;
        break;
      default:
      case kFlightPlanManual:
      case kFlightPlanForceSigned:
      case kNumFlightPlans:
        assert(false);
        break;
    }
  } else {
    // If the desired flight mode is PilotHover, we must be in crosswind.
    // This will cause us to try and exit crosswind when possible, and
    // then switch to PilotHover.
    bool progress_to_crosswind =
        (desired_flight_mode == kFlightModeCrosswindNormal);
    bool progress_to_perch = (desired_flight_mode == kFlightModePerched);

    // Clear the hover accel latch, and force hover accel if appropriate.
    state->force_hover_accel_latched = false;
    if (actually_force_hover_accel &&
        AnyHoverFlightMode(state->autonomous_flight_mode)) {
      state->autonomous_flight_mode = kFlightModeHoverAccel;
    } else {
      // Autonomous control.
      switch (state->autonomous_flight_mode) {
        case kFlightModePerched:
          if (progress_to_crosswind && state->throttle_gate_satisfied &&
              ready_states[kFlightModeHoverAscend]) {
            state->autonomous_flight_mode = kFlightModeHoverAscend;
          }
          break;
        case kFlightModeHoverAscend:
          if (progress_to_perch) {
            // This is not in the normal transition order so we do not
            // check the ready state.
            state->autonomous_flight_mode = kFlightModeHoverDescend;
          } else if (ready_states[kFlightModeHoverPayOut]) {
            state->autonomous_flight_mode = kFlightModeHoverPayOut;
          }
          break;
        case kFlightModeHoverPayOut:
          if (progress_to_perch) {
            // This is not in the normal transition order so we do not
            // check the ready state.
            state->autonomous_flight_mode = kFlightModeHoverReelIn;
          } else if (ready_states[kFlightModeHoverPrepTransformGsUp]) {
            state->autonomous_flight_mode = kFlightModeHoverPrepTransformGsUp;
          }
          break;
        case kFlightModeHoverPrepTransformGsUp:
          if (progress_to_perch && state->throttle_gate_satisfied &&
              ready_states[kFlightModeHoverReelIn]) {
            state->autonomous_flight_mode = kFlightModeHoverReelIn;
          } else if ((progress_to_crosswind && state->throttle_gate_satisfied &&
                      ready_states[kFlightModeHoverTransformGsUp]) ||
                     state_est->force_high_tension) {
            state->autonomous_flight_mode = kFlightModeHoverTransformGsUp;
          }
          break;
        case kFlightModeHoverTransformGsUp:
          if (progress_to_crosswind && state->throttle_gate_satisfied &&
              ready_states[kFlightModeHoverFullLength]) {
            state->autonomous_flight_mode = kFlightModeHoverFullLength;
          } else if (progress_to_perch) {
            state->autonomous_flight_mode = kFlightModeHoverTransformGsDown;
          }
          break;
        case kFlightModeHoverFullLength:
          if (progress_to_perch && state->throttle_gate_satisfied &&
              ready_states[kFlightModeHoverPrepTransformGsDown]) {
            state->autonomous_flight_mode = kFlightModeHoverPrepTransformGsDown;
          } else if (progress_to_crosswind && state->throttle_gate_satisfied &&
                     ready_states[kFlightModeHoverAccel]) {
            state->autonomous_flight_mode = kFlightModeHoverAccel;
          }
          break;
        case kFlightModeHoverAccel:
          if (ready_states[kFlightModeHoverTransOut]) {
            state->autonomous_flight_mode = kFlightModeHoverTransOut;
          } else if (ready_states[kFlightModeTransIn]) {
            state->autonomous_flight_mode = kFlightModeTransIn;
          }
          break;
        case kFlightModeTransIn:
          if (ready_states[kFlightModeCrosswindNormal]) {
            state->autonomous_flight_mode = kFlightModeCrosswindNormal;
          }
          break;
        case kFlightModeCrosswindNormal:
          if (!progress_to_crosswind &&
              ready_states[kFlightModeCrosswindPrepTransOut]) {
            state->autonomous_flight_mode = kFlightModeCrosswindPrepTransOut;
          }
          break;
        case kFlightModeCrosswindPrepTransOut:
          if (progress_to_crosswind && state->throttle_gate_satisfied &&
              ready_states[kFlightModeCrosswindNormal]) {
            state->autonomous_flight_mode = kFlightModeCrosswindNormal;
          } else if (ready_states[kFlightModeHoverTransOut]) {
            state->autonomous_flight_mode = kFlightModeHoverTransOut;
          }
          break;
        case kFlightModeHoverTransOut:
          if (progress_to_crosswind && state->throttle_gate_satisfied &&
              ready_states[kFlightModeHoverFullLength]) {
            state->autonomous_flight_mode = kFlightModeHoverFullLength;
          } else if (progress_to_perch && state->throttle_gate_satisfied &&
                     ready_states[kFlightModeHoverPrepTransformGsDown]) {
            state->autonomous_flight_mode = kFlightModeHoverPrepTransformGsDown;
          }
          break;
        case kFlightModeHoverPrepTransformGsDown:
          if (progress_to_crosswind && state->throttle_gate_satisfied &&
              ready_states[kFlightModeHoverFullLength]) {
            // Before the GS transform is initiated, allow the pilot to
            // command a return to FullLength.
            state->autonomous_flight_mode = kFlightModeHoverFullLength;
          } else if ((progress_to_perch && state->throttle_gate_satisfied &&
                      ready_states[kFlightModeHoverTransformGsDown]) ||
                     state_est->force_reel) {
            state->autonomous_flight_mode = kFlightModeHoverTransformGsDown;
          }
          break;
        case kFlightModeHoverTransformGsDown:
          if (progress_to_perch && state->throttle_gate_satisfied &&
              ready_states[kFlightModeHoverReelIn]) {
            state->autonomous_flight_mode = kFlightModeHoverReelIn;
          } else if (progress_to_crosswind && state->throttle_gate_satisfied) {
            state->autonomous_flight_mode = kFlightModeHoverTransformGsUp;
          }
          break;
        case kFlightModeHoverReelIn:
          if (progress_to_crosswind) {
            // This is not in the normal transition order so we do not
            // check the ready state.
            state->autonomous_flight_mode = kFlightModeHoverPayOut;
          } else if (progress_to_perch &&
                     ready_states[kFlightModeHoverDescend]) {
            state->autonomous_flight_mode = kFlightModeHoverDescend;
          }
          break;
        case kFlightModeHoverDescend:
          if (progress_to_crosswind) {
            // This is not in the normal transition order so we do not
            // check the ready state.
            state->autonomous_flight_mode = kFlightModeHoverAscend;
          } else if (ready_states[kFlightModePerched]) {
            state->autonomous_flight_mode = kFlightModePerched;
          }
          break;
        default:
        case kFlightModeForceSigned:
        case kNumFlightModes:
          assert(false);
        // Intentionally fall through to other cases for default.
        case kFlightModePilotHover:
        case kFlightModeOffTether:
          // TODO: Is this the right thing for these modes?
          break;
      }  // switch (state->autonomous_flight_mode)
    }
    flight_mode = state->autonomous_flight_mode;
  }

  // Switch to full manual mode instantly.
  if (state_est->tether_released) flight_mode = kFlightModeOffTether;

  // Update telemetry.
  GetControlTelemetry()->autonomous_flight_mode = state->autonomous_flight_mode;

  return flight_mode;
}

// Implements a preliminary flight mode arbitration between
// controllers.  Currently selects the first flight controller in the
// list (A, B, C) that does not have any faults or the current
// controller as the "leader" and copies their flight_mode.  Note that
// the controller ignores its own index in control_sync_in and faults.
// Instead it uses proposed_flight_mode and assumes itself to not be
// faulted.  This scheme does not guard against the "leader" behaving
// erratically, or flight controllers rejoining the network.
//
// Args:
//   controller_label: This flight controller's label.
//   cont_sync_in: Controller synchronization input.
//   proposed_flight_mode: This flight controller's proposed mode.
//   faults: The system fault array.
//   cont_sync_out: Message indicating the selected flight mode.
static FlightMode ArbitrateFlightMode(ControllerLabel controller_label,
                                      const ControlSyncData cont_sync_in[],
                                      FlightMode proposed_flight_mode,
                                      const FaultMask faults[],
                                      PlannerState *state,
                                      ControlSyncData *cont_sync_out) {
  ControllerLabel leader = controller_label;
  for (int32_t i = 0; i < kNumControllers; ++i) {
    // Currently a flight computer does not declare faults for itself.
    if (i == controller_label ||
        !HasAnyFault(&faults[SUBSYS_CONTROLLERS + i])) {
      leader = i;
      break;
    }
  }

  FlightMode decided_flight_mode;
  if (leader == controller_label) {
    decided_flight_mode = proposed_flight_mode;
  } else {
    decided_flight_mode = cont_sync_in[leader].flight_mode;
  }

  // These sequence numbers are used to allow for simple "no update" checks
  // on received ControlSyncMessages.
  //
  // TODO: Remove these when better network communication fault
  // detection is in place.
  state->controller_sync_sequence++;

  cont_sync_out->sequence = state->controller_sync_sequence;
  cont_sync_out->flight_mode = decided_flight_mode;

  // Update telemetry.
  SyncTelemetry *st = GetSyncTelemetry();
  st->proposed_flight_mode = proposed_flight_mode;
  st->leader = leader;

  return decided_flight_mode;
}

void PlannerStep(ControllerLabel controller_label,
                 const ControlSyncData sync_data_in[],
                 const StateEstimate *state_est, const FaultMask faults[],
                 const bool ready_states[], bool force_hover_accel,
                 const PlannerParams *planner_params,
                 const JoystickControlParams *joystick_params,
                 const ControlOutput *output, PlannerState *state,
                 ControlSyncData *sync_data_out) {
  assert(0 <= controller_label && controller_label < kNumControllers);
  assert(sync_data_in != NULL && state_est != NULL && faults != NULL &&
         ready_states != NULL && joystick_params != NULL && state != NULL &&
         sync_data_out != NULL);

  FlightMode desired_flight_mode = UpdateDesiredFlightMode(
      state_est, faults, planner_params, joystick_params, state);

  FlightMode proposed_flight_mode = ProposeNextFlightMode(
      desired_flight_mode, state_est, output->gs_mode_request, ready_states,
      force_hover_accel, joystick_params, state);

  FlightMode next_flight_mode =
      ArbitrateFlightMode(controller_label, sync_data_in, proposed_flight_mode,
                          faults, state, sync_data_out);

  // Save out current flight mode for telemetry.
  // This is more relevant for the telemetry as the planner is run
  // after the controllers.
  FlightMode current_flight_mode = state->flight_status.flight_mode;

  state->flight_status.fully_autonomous_mode =
      (state_est->joystick.data.switch_position == kJoystickSwitchPositionDown);

  // Update persistent state.
  if (next_flight_mode != state->flight_status.flight_mode) {
    state->flight_status.flight_mode_first_entry = true;
    state->flight_status.last_flight_mode = state->flight_status.flight_mode;
    state->flight_status.flight_mode = next_flight_mode;
    state->flight_status.flight_mode_time = 0.0;
  } else {
    state->flight_status.flight_mode_first_entry = false;
    state->flight_status.flight_mode_time += *g_sys.ts;
  }

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_time = state->flight_status.flight_mode_time;
  ct->flight_mode = current_flight_mode;
  ct->planner.autonomous_flight_enabled = state->enable_autonomous_operation;
  ct->planner.takeoff_countdown_timer =
      state->autonomous_takeoff_countdown_timer;
  ct->planner.inside_launch_window = state->inside_launch_window;
  ct->planner.inside_landing_window = state->inside_landing_window;
  ct->planner.landing_fault_detected = state->landing_fault_detected;
  ct->planner.desired_flight_mode = state->desired_flight_mode;
}
