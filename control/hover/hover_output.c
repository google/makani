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

#include "control/hover/hover_output.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/plc_messages.h"
#include "avionics/network/aio_labels.h"
#include "common/c_math/filter.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/actuator_util.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/ground_frame.h"
#include "control/hover/hover_types.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/vessel_frame.h"
#include "system/labels.h"

static bool ValidateState(const HoverOutputParams *params,
                          const HoverOutputState *state) {
  if (!(0.0 <= state->gain_ramp_latch_timer &&
        state->gain_ramp_latch_timer <= params->gain_ramp_time + 1.0)) {
    assert(!(
        bool)"gain_ramp_latch_timer must be positive and approximately less"
             " than gain_ramp_time.");
    return false;
  }

  if (!(kGainStateEStopped <= state->gain_ramp_state &&
        state->gain_ramp_state < kNumGainStates)) {
    assert(!(bool)"gain_ramp_state has an invalid value.");
    return false;
  }

  if (!(-PI < state->delta_ele_ff_z1 && state->delta_ele_ff_z1 < PI)) {
    assert(!(bool)"delta_ele_ff_z1 must be in (-pi, pi).");
    return false;
  }

  return true;
}

void HoverOutputInit(double joystick_throttle, double delta_elevator,
                     const Vec3 *cw_loop_center_v,
                     const HoverOutputParams *params, HoverOutputState *state) {
  assert(0.0 <= joystick_throttle && joystick_throttle <= 1.0);
  assert(params != NULL && cw_loop_center_v != NULL && state != NULL);
  assert(HoverOutputValidateParams(params));

  memset(state, 0, sizeof(*state));
  state->gain_ramp_latch_timer = 0.0;
  state->gain_ramp_scale = 0.0;
  // Starting this e-stopped, rather than latched, makes it possible
  // to program joystick schedules that skip the gain ramp.  This is
  // important for tests where the wing starts mid-air.
  state->gain_ramp_state =
      (joystick_throttle <= g_cont.joystick_control->e_stop_throttle)
          ? kGainStateEStopped
          : kGainStateFull;
  state->align_with_propwash = true;
  state->delta_ele_ff_z1 = delta_elevator;
  state->thrust_moment_out.thrust = 0.0;
  state->thrust_moment_out.moment = kVec3Zero;
  state->gs_mode_request_z1 = kGroundStationModeReel;
  state->use_high_tension_gs_azi_cmd = false;
  state->forcing_detwist_turn = false;
  state->detwist_cmd_frozen = 0.0;
  state->force_detwist_t0 = 0.0;

  state->cw_loop_center_v_f_z1 = *cw_loop_center_v;

  if (!ValidateState(params, state)) assert(false);
}

// These checks are intentionally loose, so that they may apply to a
// wide range of systems.  They are only meant to catch serious errors
// in the parameters.
bool HoverOutputValidateParams(const HoverOutputParams *params) {
  if (!(params->weights.thrust > 0.0 && params->weights.moment.x > 0.0 &&
        params->weights.moment.y > 0.0 && params->weights.moment.z > 0.0)) {
    assert(!(bool)"weights to least-squares solver must be positive.");
    return false;
  }

  if (!(params->gain_ramp_time > 0.0)) {
    assert(!(bool)"gain_ramp_time must be positive.");
    return false;
  }

  if (!(params->propwash_b.x < 0.0)) {
    assert(!(bool)"propwash_b must be in the negative x direction.");
    return false;
  }

  if (!(params->zero_propwash_wind_speed < params->full_propwash_wind_speed &&
        params->full_propwash_wind_speed <
            params->center_propwash_wind_speed)) {
    assert(!(
        bool)"zero_propwash_wind_speed must less than full_propwash_wind_speed,"
             " which must be less than center_propwash_wind_speed.");
    return false;
  }

  if (!(params->delta_ele_trans_out <= 0.0)) {
    assert(!(bool)"delta_ele_trans_out must be non-positive.");
    return false;
  }

  if (!(params->delta_elevator_per_pitch_moment < 0.0)) {
    assert(!(bool)"delta_elevator_per_pitch_moment must be negative.");
    return false;
  }

  if (!(params->min_delta_elevator_fb <= params->max_delta_elevator_fb)) {
    assert(!(
        bool)"min_delta_elevator_fb must be less than max_delta_elevator_fb.");
    return false;
  }

  if (!(params->elevator_cutoff_freq > 0.0)) {
    assert(!(bool)"elevator_cutoff_freq must be positive.");
    return false;
  }

  if (!(0.0 < params->no_aileron_rudder_speed &&
        params->no_aileron_rudder_speed < params->full_aileron_rudder_speed)) {
    assert(!(
        bool)"no_aileron_rudder_speed must be positive and"
             " full_aileron_rudder_speed must be greater than"
             " no_aileron_rudder_speed.");
    return false;
  }

  if (!(0.0 <= params->full_blown_flaps_forward_speed &&
        params->zero_blown_flaps_forward_speed >
            params->full_blown_flaps_forward_speed)) {
    assert(!(
        bool)"zero_blown_flaps_forward_speed must be positive and "
             "less than full_blown_flaps_forward_speed.");
    return false;
  }

  if (!(params->cl_da < 0.0 && params->cn_dr < 0.0)) {
    assert(!(bool)"Control derivatives have the wrong sign.");
    return false;
  }

  if (!(params->delta_blown_aileron_per_roll_moment <= 0.0)) {
    assert(!(bool)"delta_blown_aileron_per_roll_moment has the wrong sign.");
    return false;
  }

  for (int32_t i = 0; i < kNumFlaps; ++i) {
    if (!(params->lower_flap_limits[i] <= params->flap_offsets[i] &&
          params->flap_offsets[i] <= params->upper_flap_limits[i])) {
      assert(!(
          bool)"lower_flap_limits must be less than flap_offsets, which must"
               " be less than upper_flap_limits.");
      return false;
    }
  }

  return true;
}

bool HoverOutputIsGainRampDone(const HoverOutputState *state) {
  return state->gain_ramp_state == kGainStateFull;
}

// Whether the operator has just commanded one extra detwist rotation.
static bool EnteringForceDetwist(const HoverOutputState *state,
                                 const StateEstimate *state_est,
                                 FlightMode flight_mode) {
  return (state_est->force_detwist_turn_once && !state->forcing_detwist_turn &&
          (flight_mode == kFlightModeHoverFullLength ||
           flight_mode == kFlightModeHoverPrepTransformGsDown));
}

static void ForceDetwistInit(double now, const StateEstimate *state_est,
                             HoverOutputState *state) {
  state->forcing_detwist_turn = true;
  state->detwist_cmd_frozen =
      state_est->tether_ground_angles.departure_detwist_angle;
  state->force_detwist_t0 = now;
}

// Perform forced-detwist rotation at rate of pi/3 rad/s.
static double ForceDetwistStep(double now, HoverOutputState *state) {
  double delta = (now - state->force_detwist_t0) * PI * 0.33;
  // TODO: saturate delta to prevent commanding change > PI.
  if (delta > 2 * PI) {  // Revolution complete.
    state->forcing_detwist_turn = false;
  }
  delta = Saturate(delta, 0.0, 2.0 * PI);
  return Wrap(state->detwist_cmd_frozen - delta, 0.0, 2.0 * PI);
}

double HoverOutputGetLastThrust(const HoverOutputState *state) {
  return state->thrust_moment_out.thrust;
}

// Updates the gain ramp state machine and returns the overall scale
// factor to apply to the rotor thrust-moment.
//
// After the joystick throttle has been below the software e-stop
// threshold for a set period of time, we switch to a state where the
// overall gains will slowly ramp to their full values once the
// joystick goes above the software e-stop threshold.
static double UpdateGainRampScale(FlightMode flight_mode,
                                  const JoystickEstimate *joystick,
                                  bool below_gain_ramp_down_thrust_threshold,
                                  const HoverOutputParams *params,
                                  HoverOutputState *state) {
  // There should always be a soft ramp from the perched position.
  if (flight_mode == kFlightModePerched) {
    state->gain_ramp_state = kGainStateZeroLatched;
  }

  // If the joystick faults and the hover throttle e-stop is enabled,
  // then enter the faulted state and slowly decrease the thrust.
  if ((GetControlParams()->control_opt & kControlOptHoverThrottleEStop) &&
      !joystick->valid) {
    state->gain_ramp_state = kGainStateRampDown;
  }

  // When descending onto the perch, ramp down the motors completely
  // after the controller requests less than a pre-defined thrust threshold.
  if (flight_mode == kFlightModeHoverDescend &&
      below_gain_ramp_down_thrust_threshold) {
    state->gain_ramp_state = kGainStateRampDown;
  }

  switch (state->gain_ramp_state) {
    default:
    case kGainStateForceSigned:
    case kNumGainStates:
      assert(false);  // Fall-through intentional.
    case kGainStateEStopped:
      state->gain_ramp_latch_timer += *g_sys.ts;
      if (state->gain_ramp_latch_timer >
          g_cont.joystick_control->e_stop_throttle_latch_time) {
        state->gain_ramp_state = kGainStateZeroLatched;
      } else if (joystick->data.throttle >
                 g_cont.joystick_control->e_stop_throttle) {
        state->gain_ramp_state = kGainStateFull;
      }
      state->gain_ramp_scale = 0.0;
      break;

    case kGainStateZeroLatched:
      if (joystick->data.throttle > g_cont.joystick_control->e_stop_throttle) {
        state->gain_ramp_state = kGainStateRampUp;
      }
      state->gain_ramp_scale = 0.0;
      break;

    case kGainStateRampUp:
      state->gain_ramp_scale += *g_sys.ts / params->gain_ramp_time;
      if (joystick->data.throttle <= g_cont.joystick_control->e_stop_throttle) {
        state->gain_ramp_state = kGainStateZeroLatched;
      } else if (state->gain_ramp_scale >= 1.0) {
        state->gain_ramp_state = kGainStateFull;
      }
      state->gain_ramp_scale = Saturate(state->gain_ramp_scale, 0.0, 1.0);
      break;

    case kGainStateFull:
      if ((joystick->data.throttle <=
           g_cont.joystick_control->e_stop_throttle) &&
          ((GetControlParams()->control_opt & kControlOptHoverThrottleEStop) ||
           (flight_mode == kFlightModeHoverAscend) ||
           (flight_mode == kFlightModeHoverDescend))) {
        state->gain_ramp_latch_timer = 0.0;
        state->gain_ramp_state = kGainStateEStopped;
      }
      state->gain_ramp_scale = 1.0;
      break;

    case kGainStateRampDown:
      state->gain_ramp_scale -= *g_sys.ts / params->gain_ramp_time;
      state->gain_ramp_scale = Saturate(state->gain_ramp_scale, 0.0, 1.0);
      break;
  }

  // Update telemetry.
  GetHoverTelemetry()->gain_ramp_scale = state->gain_ramp_scale;

  return state->gain_ramp_scale;
}

// Scales a thrust-moment vector.
static void ScaleThrustMoment(const ThrustMoment *thrust_moment_in,
                              double scale, ThrustMoment *thrust_moment_out) {
  thrust_moment_out->thrust = scale * thrust_moment_in->thrust;
  Vec3Scale(&thrust_moment_in->moment, scale, &thrust_moment_out->moment);
}

// Returns the angle-of-attack of the airflow, in body coordinates, at
// the elevator including the combined apparent wind and propwash
// velocities.  This does not take into account the added apparent
// wind due to pitch rate because we would like to keep its damping
// effect.
static double CalcAlphaAtElevator(const Vec3 *apparent_wind_b,
                                  const HoverOutputParams *params) {
  // Remove the effect of the propwash at low wind speeds and very
  // high wind speeds when the propwash should miss the elevator.
  //
  // TODO: We can make the fading more rigorous using the
  // convection model in Drela, ASWING 5.86 Technical Description.
  double propwash_scale = Crossfade(
      1.0, 0.0, fabs(params->center_propwash_wind_speed + apparent_wind_b->z),
      params->center_propwash_wind_speed - params->full_propwash_wind_speed,
      params->center_propwash_wind_speed - params->zero_propwash_wind_speed);

  Vec3 propwash_b;
  Vec3Scale(&params->propwash_b, propwash_scale, &propwash_b);

  Vec3 apparent_wind_and_propwash_b;
  Vec3Add(apparent_wind_b, &propwash_b, &apparent_wind_and_propwash_b);

  ApparentWindSph apparent_wind_and_propwash_sph;
  ApparentWindCartToSph(&apparent_wind_and_propwash_b,
                        &apparent_wind_and_propwash_sph);

  return apparent_wind_and_propwash_sph.alpha;
}

// Calculates an elevator deflection that meets a pitch moment command
// taking into account the propwash and apparent wind speed.
//
// TODO: Check ApparentWindEstimate for faults.  This is
// waiting on the Estimator to provide a valid flag for the apparent
// wind estimate, when it is using the wind sensor.
static double CalcElevatorDeflection(double elevator_pitch_moment,
                                     const ApparentWindEstimate *apparent_wind,
                                     const FlightStatus *flight_status,
                                     const HoverOutputParams *params,
                                     HoverOutputState *state) {
  // Calculate the zero pitch moment elevator deflection.
  double delta_ele_ff = -CalcAlphaAtElevator(&apparent_wind->vector, params);

  // For low apparent wind speeds, leave the elevator aligned with the
  // propwash direction.  This is useful so the elevator doesn't have
  // to quickly adjust if there is a gust and the propwash begins to
  // interact with it.
  double propwash_angle = atan2(-params->propwash_b.z, -params->propwash_b.x);

  // Align the elevator with the propwash. In the future, for some tail
  // configurations and wind speeds, we might choose to align the elevator
  // with the estimated angle-of-attack at the tail.
  state->align_with_propwash = true;

  if (state->align_with_propwash) {
    delta_ele_ff = -propwash_angle;
  }

  // Intentionally stall the wing during transition-out to slow it
  // down quickly.  This relies on roll bridling to maintain roll
  // control.
  if (flight_status->flight_mode == kFlightModeHoverTransOut) {
    delta_ele_ff += Crossfade(params->delta_ele_trans_out, 0.0,
                              flight_status->flight_mode_time, 0.0, 10.0);
  }

  // Coerce the feed-forward component within the allowed range of the
  // elevator before filtering.  This prevents a slow return from an
  // extreme value, and also prevents exceeding the reasonable (-pi,
  // pi) range check on delta_ele_ff_z1.
  delta_ele_ff = Saturate(
      delta_ele_ff,
      params->lower_flap_limits[kFlapEle] - params->flap_offsets[kFlapEle],
      params->upper_flap_limits[kFlapEle] - params->flap_offsets[kFlapEle]);

  // Filter the feed-forward component of the elevator command.  This
  // prevents an unwanted feedback loop from the elevator movement, to
  // the rate gyros, back to the pitch command.  We do not filter the
  // feedback component both because it already has a slow response,
  // and also because we don't want to add unnecessary phase lag.
  delta_ele_ff = Lpf(delta_ele_ff, params->elevator_cutoff_freq, *g_sys.ts,
                     &state->delta_ele_ff_z1);

  // Convert the elevator pitch moment feedback, calculated in the
  // angle loop, to an elevator deflection.  Here we make the
  // simplifying assumption that the airspeed at the elevator is
  // entirely determined by the propwash, and that the elevator is
  // acting as a lifting surface, i.e. it is aligned with the apparent
  // wind.  The maximum feedback deflection is limited to prevent
  // stall.
  //
  // NOTE: I've tried making this more precise by using
  // the estimated apparent wind but ran into issues with an unwanted
  // feedback loop between radial velocity and pitch.
  double delta_ele_fb =
      Saturate(params->delta_elevator_per_pitch_moment * elevator_pitch_moment,
               params->min_delta_elevator_fb, params->max_delta_elevator_fb);

  // Set elevator command based on flight mode.
  double delta_ele_cmd;
  if (flight_status->flight_mode == kFlightModePerched) {
    // Do not move the elevator while perched, so set it to the
    // nominal propwash angle.
    delta_ele_cmd = -propwash_angle;
  } else if (flight_status->flight_mode == kFlightModePilotHover) {
    // Until we have more experience with the elevator feedback
    // command, turn off elevator feedback in pilot hover.
    delta_ele_cmd = delta_ele_ff;
  } else if (flight_status->flight_mode == kFlightModeHoverAccel) {
    // Set zero angle-of-attack at the elevator during acceleration to
    // minimize elevator drag.
    delta_ele_cmd = delta_ele_ff;
  } else {
    delta_ele_cmd = delta_ele_ff + delta_ele_fb;
  }

  // Update telemetry.
  HoverTelemetry *ht = GetHoverTelemetry();
  ht->delta_ele_ff = delta_ele_ff;
  ht->delta_ele_fb = delta_ele_fb;

  return delta_ele_cmd;
}

// Sets aileron, flap, elevator, and rudder commands based on the
// moment commands, forward velocity, and the angle of the apparent
// wind.  The elevator deflection is handled in its own sub-function.
// For the ailerons and rudder, a gain matrix is scheduled based on
// forward velocity.  This matrix is then applied to the moment vector
// to get the deltas.
static void ConvertMomentsToDeltas(const Vec3 *moment_cmd,
                                   double elevator_pitch_moment,
                                   double rudder_yaw_moment,
                                   const ApparentWindEstimate *apparent_wind_b,
                                   const FlightStatus *flight_status,
                                   const HoverOutputParams *params,
                                   HoverOutputState *state, Deltas *deltas) {
  deltas->elevator = CalcElevatorDeflection(
      elevator_pitch_moment, apparent_wind_b, flight_status, params, state);

  // Scale the aileron and rudder deflections with forward speed.
  double scale = Crossfade(0.0, 1.0, -apparent_wind_b->vector.x,
                           params->no_aileron_rudder_speed,
                           params->full_aileron_rudder_speed);

  double dynamic_pressure =
      0.5 * g_sys.phys->rho * apparent_wind_b->sph.v * apparent_wind_b->sph.v;
  double aileron_per_roll_moment =
      1.0 / (fmax(dynamic_pressure, 1.0) * g_sys.wing->A * g_sys.wing->b *
             params->cl_da);
  deltas->aileron = scale * moment_cmd->x * aileron_per_roll_moment;

  // The rudder typically experiences an increased dynamic pressure
  // because it is in the propwash.
  Vec3 local_apparent_wind_b;
  Vec3Add(&apparent_wind_b->vector, &params->propwash_b,
          &local_apparent_wind_b);
  double rudder_dynamic_pressure =
      0.5 * g_sys.phys->rho * Vec3NormSquared(&local_apparent_wind_b);
  double rudder_per_yaw_moment =
      1.0 / (fmax(rudder_dynamic_pressure, 1.0) * g_sys.wing->A *
             g_sys.wing->b * params->cn_dr);
  // Even though the rudder is usually in the propwash and thus has
  // significant dynamic pressure, we scale the fast component of its
  // deflection, which mirrors the motor moments, with forward
  // velocity similar to the ailerons because we would prefer to not
  // rely on the rudder for yaw control in hover.  However, we do
  // always use the slow component of its deflection, which is
  // designed to zero the integrated yaw moment.
  deltas->rudder =
      (scale * moment_cmd->z + rudder_yaw_moment) * rudder_per_yaw_moment;

  // Do nothing with the center flap here.  This is handled later with
  // the blown flap control.
  deltas->inboard_flap = 0.0;
  // Leave the common mode aileron commands fixed.
  deltas->midboard_flap = 0.0;
  deltas->outboard_flap = 0.0;
}

// Calculate the ground station targeting data.
//
// When perched, the target is the downwind direction. In other reel
// modes the target is the wing position. During the transform, and
// when the GS is in high tension mode, we point the gs at the
// crosswind loop center.
static double CalcHoverGsTargetAzimuth(
    FlightMode flight_mode, GroundStationMode gs_mode,
    uint8_t gs_transform_stage, const WindEstimate *wind_g,
    const Vec3 *wing_pos_g, const Vec3 crosswind_loop_center_g,
    const VesselEstimate *vessel, const Gs02Params *params,
    const EstimatorWindParams *estimator_wind_params, bool *target_valid,
    bool *use_high_tension_gs_azi_cmd, HoverOutputState *state,
    double *target_raw) {
  // This state machine is used to latch the high-tension ground
  // station azimuth command at a particular point in the transform
  // from reel to high-tension. The state machine is necessary avoid
  // issues when gs_mode and gs_transform_mode are not perfectly
  // synchronized.
  if (gs_mode == kGroundStationModeHighTension) {
    *use_high_tension_gs_azi_cmd = true;
  }

  if (gs_mode == kGroundStationModeReel) {
    *use_high_tension_gs_azi_cmd = false;
  }

  if (gs_mode == kGroundStationModeTransform) {
    // During the transform from Reel to HighTension, slew the azimuth command
    // from pointing at the kite to pointing at the crosswind loop center.  This
    // takes advantage of a narrow window in which we still have active azimuth
    // control but have the freedom to allow an azimuth offset between the
    // ground station and the kite.  See b2/78529394#comment28.
    // TODO: This depends on the details of the GS transform
    // stages. Encapsulate this with a suitable degree of abstraction.
    if (flight_mode == kFlightModeHoverTransformGsUp &&
        (gs_transform_stage == 2U || gs_transform_stage == 1U)) {
      *use_high_tension_gs_azi_cmd = true;
    }

    // In TransformGsDown, point the perch at the kite once active
    // azimuth control is restored.
    // TODO: We may need to command the kite to move to a
    // position that matches the GS azimuth after trans-out.
    if (flight_mode == kFlightModeHoverTransformGsDown) {
      *use_high_tension_gs_azi_cmd = false;
    }
  }

  if (flight_mode == kFlightModePerched) {
    // Track downwind when perched.
    // TODO(b/138810031): Replace the call to AziGToV using wind_g->dir_f by a
    // call to VecGToAzimuth using wing_g->vector_f.
    assert(!*use_high_tension_gs_azi_cmd);
    *target_valid = wind_g->valid;
    *target_raw = AziGToV(wind_g->dir_f, &vessel->dcm_g2v);
    return *target_raw;
  } else if (*use_high_tension_gs_azi_cmd) {
    // Track the crosswind loop center when in high-tension mode. Low-pass
    // filter it, similarly to how the wind direction is filtered in low-tension
    // mode.
    Vec3 crosswind_loop_center_v;
    *target_valid = wind_g->valid;
    Mat3Vec3Mult(&vessel->dcm_g2v, &crosswind_loop_center_g,
                 &crosswind_loop_center_v);
    *target_raw = Wrap(VecVToAzimuth(&crosswind_loop_center_v), 0.0, 2.0 * PI);
    LpfVec3(&crosswind_loop_center_v, estimator_wind_params->fc_dir, *g_sys.ts,
            &state->cw_loop_center_v_f_z1);
    return Wrap(VecVToAzimuth(&state->cw_loop_center_v_f_z1), 0.0, 2.0 * PI);
  } else {
    // Track the kite position when in reel mode.
    // TODO: Check vessel->valid.
    *target_raw = CalcHoverGsTargetAzimuthReel(
        &vessel->pos_g, &vessel->dcm_g2v, wing_pos_g, params, target_valid);
    return *target_raw;
  }
}

static GroundStationMode CalcHoverGsModeRequest(
    const FlightStatus *flight_status, const StateEstimate *state_est,
    GroundStationMode *gs_mode_request_z1) {
  GroundStationMode gs_mode_request;

  switch (flight_status->flight_mode) {
    case kFlightModePilotHover:
    case kFlightModeOffTether:
      // Maintain the current ground station mode, unless overridden by operator
      // command.
      if (state_est->force_high_tension) {
        gs_mode_request = kGroundStationModeHighTension;
      } else if (state_est->force_reel) {
        gs_mode_request = kGroundStationModeReel;
      } else {
        gs_mode_request = *gs_mode_request_z1;
      }
      break;
    case kFlightModeForceSigned:
    case kFlightModePerched:
    case kFlightModeHoverAscend:
    case kFlightModeHoverPayOut:
    case kFlightModeHoverPrepTransformGsUp:
    case kFlightModeHoverTransformGsDown:
    case kFlightModeHoverReelIn:
    case kFlightModeHoverDescend:
      gs_mode_request = kGroundStationModeReel;
      break;
    case kFlightModeHoverTransformGsUp:
    case kFlightModeHoverFullLength:
    case kFlightModeHoverAccel:
    case kFlightModeTransIn:
    case kFlightModeCrosswindNormal:
    case kFlightModeCrosswindPrepTransOut:
    case kFlightModeHoverTransOut:
    case kFlightModeHoverPrepTransformGsDown:
      gs_mode_request = kGroundStationModeHighTension;
      break;
    case kNumFlightModes:
    default:
      assert(false);
      // Maintain the current ground station mode.
      gs_mode_request = *gs_mode_request_z1;
      break;
  }

  *gs_mode_request_z1 = gs_mode_request;
  return gs_mode_request;
}

void HoverOutputStep(const FlightStatus *flight_status,
                     const StateEstimate *state_est,
                     const Vec3 crosswind_loop_center_g,
                     const ThrustMoment *thrust_moment,
                     double blown_flaps_roll_moment,
                     double elevator_pitch_moment, double rudder_yaw_moment,
                     double winch_vel_cmd, const HoverOutputParams *params,
                     HoverOutputState *state, ControlOutput *control_output) {
  assert(IsValidFlightMode(flight_status->flight_mode));
  assert(state_est != NULL);
  assert(thrust_moment != NULL && thrust_moment->thrust >= 0.0);
  assert(params != NULL && state != NULL && ValidateState(params, state));
  assert(control_output != NULL);

  // Calculate overall gain scale.  This is used to gracefully ramp
  // the motors once the throttle is brought above the software e-stop
  // threshold, and to ramp down motors after descending onto the perch.
  bool below_gain_ramp_down_thrust_threshold =
      thrust_moment->thrust < 0.7 * g_sys.wing->m * g_sys.phys->g;
  double gain_ramp_scale =
      UpdateGainRampScale(flight_status->flight_mode, &state_est->joystick,
                          below_gain_ramp_down_thrust_threshold, params, state);

  control_output->gs_mode_request = CalcHoverGsModeRequest(
      flight_status, state_est, &state->gs_mode_request_z1);

  // Calculate the ground station targeting data.
  double gs_azi_target_raw;
  {
    bool target_valid;
    control_output->gs_azi_cmd.target = CalcHoverGsTargetAzimuth(
        flight_status->flight_mode, state_est->gs_mode,
        state_est->gs_transform_stage, &state_est->wind_g, &state_est->Xg,
        crosswind_loop_center_g, &state_est->vessel,
        &GetSystemParams()->ground_station.gs02,
        &GetControlParams()->estimator.wind, &target_valid,
        &state->use_high_tension_gs_azi_cmd, state, &gs_azi_target_raw);

    // This can occur for a few cycles during initialization.
    control_output->hold_gs_azi_cmd = !target_valid;
  }

  // Output the GS unpause transform signal
  {
    const JoystickControlParams *joystick_params =
        &GetControlParams()->joystick_control;
    if (state_est->gs_unpause_transform ||
        flight_status->fully_autonomous_mode) {
      control_output->gs_unpause_transform = true;
    } else if (flight_status->flight_mode == kFlightModeHoverTransformGsUp) {
      control_output->gs_unpause_transform =
          (state_est->joystick.throttle_f >
           joystick_params->ascend_payout_throttle);
    } else if (flight_status->flight_mode == kFlightModeHoverTransformGsDown) {
      control_output->gs_unpause_transform =
          state_est->joystick.throttle_f <
          joystick_params->descend_reel_in_throttle;
    } else {
      control_output->gs_unpause_transform = false;
    }
  }

  // Avoid "chasing the carrot" when the gain ramp is less than 1.0 and
  // the kite is not in Perch mode.
  if (!HoverOutputIsGainRampDone(state) &&
      flight_status->flight_mode != kFlightModePerched) {
    control_output->hold_gs_azi_cmd = true;
  }

  // Wrap GS02 azimuth target between 0 and 2*pi radians.
  control_output->gs_azi_cmd.target =
      Wrap(control_output->gs_azi_cmd.target, 0.0, 2.0 * PI);
  gs_azi_target_raw = Wrap(gs_azi_target_raw, 0.0, 2.0 * PI);

  // GS02 azimuth axis deadzone commands. The deadzone causes the azimuth
  // motion on GS02 to stop by setting the error to zero.
  if (flight_status->flight_mode == kFlightModePerched) {
    control_output->gs_azi_cmd.dead_zone = params->gs02_deadzone_while_perched;
  } else {
    control_output->gs_azi_cmd.dead_zone = params->gs02_deadzone_during_flight;
  }

  bool force_zero_advance_ratio = true;
  double airspeed = 0.0;
  // In kFlightModeHoverAccel we desire higher thrust levels and have
  // a non-negligible amount of airflow into the propeller disks.  We
  // use the body velocity to approximate this airflow so that
  // MixRotors knows to command higher speeds.
  if (flight_status->flight_mode == kFlightModeHoverAccel ||
      flight_status->flight_mode == kFlightModeHoverTransOut) {
    force_zero_advance_ratio = false;
    airspeed = fmax(0.0, state_est->Vb_f.x);
  }

  // Calculate rotor angular velocities.
  ThrustMoment thrust_moment_motor;
  double v_app_locals[kNumMotors];
  double rotors[kNumMotors];
  ScaleThrustMoment(thrust_moment, gain_ramp_scale, &thrust_moment_motor);
  MixRotors(&thrust_moment_motor, &params->weights, airspeed, &kVec3Zero,
            (StackingState)state_est->stacking_state, force_zero_advance_ratio,
            g_sys.phys->rho, g_sys.rotors, g_cont.rotor_control, rotors,
            &state->thrust_moment_out, v_app_locals);

  // Set the motors to the idle speed while perched.
  if (flight_status->flight_mode == kFlightModePerched) {
    for (int32_t i = 0; i < kNumMotors; ++i) {
      rotors[i] = g_cont.rotor_control->idle_speed;
    }
  }

  // Deal speed command into motor representation.
  // TODO: Make common function for generating these across modes.
  for (int i = 0; i < kNumMotors; ++i) {
    control_output->motor_speed_upper_limit[i] = rotors[i];
    control_output->motor_speed_lower_limit[i] = rotors[i];
    control_output->motor_torque[i] = 0.0;
  }

  // Calculate flap positions.
  Deltas deltas, deltas_available;
  ConvertMomentsToDeltas(&thrust_moment->moment, elevator_pitch_moment,
                         rudder_yaw_moment, &state_est->apparent_wind,
                         flight_status, params, state, &deltas);
  MixFlaps(&deltas, params->flap_offsets, params->lower_flap_limits,
           params->upper_flap_limits, control_output->flaps, &deltas_available);

  // Use the blown center flaps as a source of roll authority in
  // hover.  Crossfade this actuation to zero at higher absolute
  // forward speeds.
  double aileron_roll_moment =
      blown_flaps_roll_moment *
      Crossfade(1.0, 0.0, fabs(state_est->Vb_f.x),
                params->full_blown_flaps_forward_speed,
                params->zero_blown_flaps_forward_speed);

  control_output->flaps[kFlapA4] = Saturate(
      -params->delta_blown_aileron_per_roll_moment * aileron_roll_moment +
          params->flap_offsets[kFlapA4],
      params->lower_flap_limits[kFlapA4], params->upper_flap_limits[kFlapA4]);
  control_output->flaps[kFlapA5] = Saturate(
      params->delta_blown_aileron_per_roll_moment * aileron_roll_moment +
          params->flap_offsets[kFlapA5],
      params->lower_flap_limits[kFlapA5], params->upper_flap_limits[kFlapA5]);

  // Set the winch velocity command.
  control_output->winch_vel_cmd = winch_vel_cmd;

  // Set the detwist servo command.
  if (EnteringForceDetwist(state, state_est, flight_status->flight_mode)) {
    // Operator commanded one rotation. If in correct modes, initiate turn.
    ForceDetwistInit(flight_status->flight_mode_time, state_est, state);
  } else if (state->forcing_detwist_turn) {
    control_output->detwist_cmd =
        ForceDetwistStep(flight_status->flight_mode_time, state);
  } else if (state_est->gs_mode == kGroundStationModeHighTension) {
    control_output->detwist_cmd =
        state_est->tether_ground_angles.departure_detwist_angle;
  } else {
    control_output->detwist_cmd = 0.0;
  }

  // Determine tether release from the joystick.
  control_output->tether_release =
      state_est->joystick.valid && state_est->joystick.data.release;

  // When the throttle is below e_stop_throttle in flight mode Pilot Hover,
  // spoil the flaps and stop the winch. This is used while lifting and lowering
  // the kite.
  if ((flight_status->flight_mode == kFlightModePilotHover) &&
      state_est->joystick.valid && (state_est->joystick.data.throttle <
                                    g_cont.joystick_control->e_stop_throttle)) {
    const FlapLabel aileron_labels[] = {kFlapA1, kFlapA2, kFlapA4,
                                        kFlapA5, kFlapA7, kFlapA8};
    for (int32_t i = 0; i < ARRAYSIZE(aileron_labels); ++i) {
      control_output->flaps[aileron_labels[i]] = params->spoiled_aileron_angle;
    }

    control_output->winch_vel_cmd = 0.0;
  }

  // Don't stop the motors.
  control_output->stop_motors = false;

  // These values are set by the outer control system.
  control_output->run_motors = false;
  control_output->sync.sequence = 0U;
  control_output->sync.flight_mode = -1;

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->deltas = deltas;
  ct->deltas_avail = deltas_available;
  ct->thrust_moment = thrust_moment_motor;
  ct->thrust_moment_avail = state->thrust_moment_out;
  for (int32_t i = 0; i < kNumMotors; ++i) {
    ct->v_app_locals[i] = v_app_locals[i];
  }
  ct->gs_azi_target_raw = gs_azi_target_raw;
}
