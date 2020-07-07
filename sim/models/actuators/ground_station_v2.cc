// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sim/models/actuators/ground_station_v2.h"

#include <math.h>

#include <algorithm>
#include <limits>

#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/vessel_frame.h"
#include "sim/sim_telemetry.h"

namespace {
// Attenuate a velocity command and apply a dead zone as the position target is
// approached.  This corresponds to the McLaren Simulink block "Attenuate
// angular velocity demand as angular target is approached".
static double AttenuateAndApplyDeadZone(double vel_cmd, double pos_error,
                                        double max_accel,
                                        double dead_zone_half_width) {
  // The attenuation strategy is based on the standard relationship for a system
  // under constant acceleration:
  //     2 * a * (x - x_0) = v^2 - v_0^2.
  // Our choice of a maximum velocity corresponds to choosing v_0 such that
  // pos_error = (x - x_0) will be zero when the velocity is zero if the system
  // is decelerated as quickly as allowed.
  double v_max = Sqrt(fabs(pos_error) * 2.0 * max_accel);
  double modified_cmd = Saturate(vel_cmd, -v_max, v_max);

  // This is equivalent to a standard Simulink dead zone:
  // https://www.mathworks.com/help/simulink/slref/deadzone.html.
  double dead_zone_upper = Sqrt(fabs(dead_zone_half_width) * 2.0 * max_accel);
  if (modified_cmd < -dead_zone_upper) {
    modified_cmd += dead_zone_upper;
  } else if (modified_cmd < dead_zone_upper) {
    modified_cmd = 0.0;
  } else {
    modified_cmd -= dead_zone_upper;
  }

  return modified_cmd;
}

}  // namespace

Gs02ModeController::Gs02ModeController(const std::string &name__, double ts)
    : Model(name__, ts),
      azi_pos_(new_discrete_state(), "azi_pos", ts_, 0.0),
      azi_vel_(new_discrete_state(), "azi_vel", ts_, 0.0),
      drum_pos_(new_discrete_state(), "drum_pos", ts_, 0.0),
      drum_vel_(new_discrete_state(), "drum_pos", ts_, 0.0),
      wing_azi_(new_discrete_state(), "wing_azi", ts_, 0.0),
      enabled_(new_discrete_state(), "enabled", ts_, false),
      azi_vel_cmd_(new_discrete_state(), "azi_vel_cmd", ts_, 0.0),
      drum_vel_cmd_(new_discrete_state(), "drum_vel_cmd", ts_, 0.0) {}

void Gs02ModeController::SetCommonInputs(double t, double azi_pos,
                                         double azi_vel, double drum_pos,
                                         double drum_vel, double wing_azi) {
  azi_pos_.DiscreteUpdate(t, azi_pos);
  azi_vel_.DiscreteUpdate(t, azi_vel);
  drum_pos_.DiscreteUpdate(t, drum_pos);
  drum_pos_.DiscreteUpdate(t, drum_vel);
  wing_azi_.DiscreteUpdate(t, wing_azi);
}

ReelController::ReelController(const Gs02SimMcLarenControllerParams &params,
                               double drum_radius)
    : Gs02ModeController("Reel controller", params.ts),
      params_(params),
      drum_radius_(drum_radius),
      drum_linear_vel_cmd_from_wing_(new_discrete_state(),
                                     "drum_linear_vel_cmd_from_wing", params.ts,
                                     0.0),
      azi_cmd_z_(new_discrete_state(), "azi_cmd_z", params.ts, {{0.0, 0.0}}),
      azi_cmd_delay_state_(new_discrete_state(), "azi_cmd_delay_state",
                           params.ts, 0.0),
      azi_cmd_dead_zone_(new_discrete_state(), "azi_cmd_dead_zone", params.ts,
                         0.0),
      azi_in_dead_zone_z1_(new_discrete_state(), "azi_in_dead_zone_z1",
                           params.ts, false),
      azi_delay_state_(new_discrete_state(), "azi_delay_state", params.ts, 0.0),
      azi_rate_limit_state_(new_discrete_state(), "azi_rate_limit_state",
                            params.ts, 0.0),
      drum_vel_rate_limit_state_(new_discrete_state(),
                                 "drum_vel_rate_limit_state", params.ts, 0.0),
      levelwind_engaged_(new_discrete_state(), "levelwind_engaged", params.ts,
                         true) {
  SetupDone();
}

void ReelController::DiscreteStepHelper(double t) {
  if (!enabled_.val()) {
    azi_cmd_z_.DiscreteUpdate(t, azi_cmd_z_.val());
    azi_cmd_delay_state_.DiscreteUpdate(t, azi_cmd_delay_state_.val());
    azi_delay_state_.DiscreteUpdate(t, azi_vel_.val());
    azi_rate_limit_state_.DiscreteUpdate(t, azi_vel_.val());
    drum_vel_rate_limit_state_.DiscreteUpdate(t, drum_vel_.val());
    return;
  }

  {  // Generate azimuth velocity command.
    const double target_pos =
        wing_azi_.val() + params_.reel.azi_offset_from_wing;

    // Unwrap the target position toward the state of the command filter.
    // This prevents the perch from taking the long way around the circle when
    // the target azimuth goes through a 2*pi discontinuity.
    double unwrapped_target_pos = azi_cmd_delay_state_.val();
    double twopimoddiff =
        fmod(azi_cmd_delay_state_.val() - target_pos, 2.0 * PI);
    if (twopimoddiff > PI) {
      unwrapped_target_pos -= twopimoddiff - 2.0 * PI;
    } else {
      unwrapped_target_pos -= twopimoddiff;
    }

    // Apply the 2nd order command filter.
    std::array<double, 2> azi_cmd_z = azi_cmd_z_.val();
    double azi_cmd =
        Lpf2(unwrapped_target_pos, params_.reel.azi_cmd_filter_omega / 2.0 / PI,
             params_.reel.azi_cmd_filter_zeta, params_.ts, azi_cmd_z.data());
    azi_cmd_z_.DiscreteUpdate(t, azi_cmd_z);

    // Acquire the delayed value, and then update it.
    double output_cmd = azi_delay_state_.val();
    double azi_vel_cmd = (azi_cmd - azi_cmd_delay_state_.val()) / params_.ts;
    azi_cmd_delay_state_.DiscreteUpdate(t, azi_cmd);
    double pos_error = Wrap(azi_cmd - azi_pos_.val(), -PI, PI);

    // Apply dead zone. The "effective dead zone" controls the azimuth to within
    // a tight envelope around the target (as a fraction of the commanded dead
    // zone), then expands the envelope to the full specified dead zone. Doing
    // this prevents jitter at the edges of the dead zone.
    double effective_dead_zone =
        azi_in_dead_zone_z1_.val()
            ? azi_cmd_dead_zone_.val()
            : azi_cmd_dead_zone_.val() * params_.reel.little_dead_zone;
    if (fabs(pos_error) <= effective_dead_zone) {
      azi_in_dead_zone_z1_.DiscreteUpdate(t, true);
      pos_error = 0.0;
    } else {
      azi_in_dead_zone_z1_.DiscreteUpdate(t, false);
    }

    double pos_error_saturated = Saturate(
        pos_error, -params_.reel.azi_error_max, params_.reel.azi_error_max);
    azi_delay_state_.DiscreteUpdate(
        t, params_.reel.azi_vel_cmd_kp * pos_error_saturated +
               params_.reel.azi_vel_cmd_ff_gain * azi_vel_cmd);

    // Apply the rate limit.
    double cmd_z1 = azi_rate_limit_state_.val();
    output_cmd =
        RateLimit(output_cmd, -params_.reel.azi_vel_cmd_rate_limit,
                  params_.reel.azi_vel_cmd_rate_limit, params_.ts, &cmd_z1);
    azi_rate_limit_state_.DiscreteUpdate(t, cmd_z1);

    azi_vel_cmd_.DiscreteUpdate(t, output_cmd);
  }

  {  // Generate drum velocity command.
    double cmd = drum_linear_vel_cmd_from_wing_.val() / drum_radius_;

    if (!levelwind_engaged_.val()) {
      // Stop the winch if the levelwind disengages.
      cmd = 0.0;
    }

    double pos_error;
    if (cmd > 0.0) {
      pos_error = params_.reel.drum_angle_upper_limit - drum_pos_.val();
    } else {
      pos_error = -std::numeric_limits<double>::infinity();
    }

    // Clip the velocity command if it's in the wrong direction.
    if (cmd * pos_error < 0) cmd = 0.0;

    cmd = AttenuateAndApplyDeadZone(
        cmd, params_.reel.drum_angle_upper_limit - drum_pos_.val(),
        params_.reel.max_drum_accel, 0.0);

    // Apply the rate limit.
    double cmd_z1 = drum_vel_rate_limit_state_.val();
    cmd = RateLimit(cmd, -params_.reel.max_drum_accel,
                    params_.reel.max_drum_accel, params_.ts, &cmd_z1);
    drum_vel_rate_limit_state_.DiscreteUpdate(t, cmd_z1);

    drum_vel_cmd_.DiscreteUpdate(t, cmd);
  }
}

TransformController::TransformController(
    const Gs02SimMcLarenControllerParams &params)
    : Gs02ModeController("Transform controller", params.ts),
      params_(params.transform),
      azi_rate_limit_state_(new_discrete_state(), "azi_rate_limit_state",
                            params.ts, 0.0),
      winch_rate_limit_state_(new_discrete_state(), "winch_rate_limit_state",
                              params.ts, 0.0),
      detwist_pos_cmd_(new_discrete_state(), "detwist_pos_cmd", ts_, 0.0),
      unpause_transform_(new_discrete_state(), "unpause_transform", params.ts,
                         false),
      transform_stage_(new_discrete_state(), "transform_stage", params.ts, 0),
      ht2reel_(new_discrete_state(), "ht2reel", params.ts, false),
      transform_complete_(new_discrete_state(), "transform_complete", params.ts,
                          false) {
  SetupDone();
}

void TransformController::Publish() const {
  sim_telem.gs02.transform_stage = transform_stage_.val();
}

void TransformController::DiscreteStepHelper(double t) {
  if (!enabled_.val()) {
    azi_rate_limit_state_.DiscreteUpdate(t, azi_vel_.val());
    winch_rate_limit_state_.DiscreteUpdate(t, drum_vel_.val());
    transform_complete_.DiscreteUpdate(t, false);
    transform_stage_.DiscreteUpdate(t, 0);
    return;
  }

  const int32_t s = transform_stage_.val();

  bool azi_target_satisfied;
  {  // Azimuth
    double target = wing_azi_.val() + params_.azi_offset_from_wing;
    double tol;
    if (ht2reel_.val()) {
      target += params_.azi_targets_ht2reel[s];
      tol = params_.azi_tols_ht2reel[s];
    } else {
      target += params_.azi_targets_reel2ht[s];
      tol = params_.azi_tols_reel2ht[s];
    }

    double pos_error = Wrap(target - azi_pos_.val(), -PI, PI);
    double vel_cmd = Sign(pos_error) * params_.azi_nominal_vel;
    vel_cmd = AttenuateAndApplyDeadZone(
        vel_cmd, pos_error, params_.azi_decel_ratio * params_.azi_max_accel,
        params_.azi_dead_zone_half_width);
    double cmd_z1 = azi_rate_limit_state_.val();
    vel_cmd = RateLimit(vel_cmd, -params_.azi_max_accel, params_.azi_max_accel,
                        ts_, &cmd_z1);
    azi_rate_limit_state_.DiscreteUpdate(t, vel_cmd);

    azi_vel_cmd_.DiscreteUpdate(t, vel_cmd);

    azi_target_satisfied = -tol < pos_error && pos_error < tol;
  }

  bool winch_target_satisfied;
  {  // Winch
    double target, tol;
    if (ht2reel_.val()) {
      target = params_.winch_targets_ht2reel[transform_stage_.val()];
      tol = params_.winch_tols_ht2reel[transform_stage_.val()];
    } else {
      target = params_.winch_targets_reel2ht[transform_stage_.val()];
      tol = params_.winch_tols_reel2ht[transform_stage_.val()];
    }

    double pos_error = target - drum_pos_.val();
    double vel_cmd = Sign(pos_error) * params_.winch_nominal_vel;
    vel_cmd = AttenuateAndApplyDeadZone(
        vel_cmd, pos_error, params_.winch_decel_ratio * params_.winch_max_accel,
        params_.winch_dead_zone_half_width);
    double cmd_z1 = winch_rate_limit_state_.val();
    vel_cmd = RateLimit(vel_cmd, -params_.winch_max_accel,
                        params_.winch_max_accel, ts_, &cmd_z1);
    winch_rate_limit_state_.DiscreteUpdate(t, vel_cmd);

    drum_vel_cmd_.DiscreteUpdate(t, vel_cmd);

    winch_target_satisfied =
        target - tol < drum_pos_.val() && drum_pos_.val() < target + tol;
  }

  {  // Generate detwist position command.
    double target;
    if (ht2reel_.val()) {
      target = params_.detwist_targets_ht2reel[transform_stage_.val()];
    } else {
      target = params_.detwist_targets_reel2ht[transform_stage_.val()];
    }
    detwist_pos_cmd_.DiscreteUpdate(t, target);
  }

  // Update the transform stage and determine if the transform is complete.
  bool stage_complete = azi_target_satisfied && winch_target_satisfied;
  int32_t next_stage, last_stage;
  if (ht2reel_.val()) {
    // HighTension to Reel: Stage progression is 0, 1, 2, 3, 4.
    last_stage = 4;
    next_stage = std::min(s + 1, last_stage);
    // Wait at the end of stages 1 and 2 unless unpaused.
    if (!unpause_transform() && (s == 1 || s == 2)) {
      stage_complete = false;
    }
  } else {
    // Reel to HighTension: Stage progression is 0, 4, 3, 2, 1.
    last_stage = 1;
    next_stage = s == 0 ? 4 : std::max(s - 1, last_stage);
    // Wait at the end of stage 3 unless unpaused.
    if (!unpause_transform() && s == 3) {
      stage_complete = false;
    }
  }
  transform_complete_.DiscreteUpdate(t, s == last_stage && stage_complete);
  transform_stage_.DiscreteUpdate(t, stage_complete ? next_stage : s);
}

void TransformController::StartTransform(double t, bool ht2reel) {
  transform_stage_.DiscreteUpdate(t, 0);
  ht2reel_.DiscreteUpdate(t, ht2reel);
  transform_complete_.DiscreteUpdate(t, false);
}

HighTensionController::HighTensionController(
    const Gs02SimMcLarenControllerParams &params)
    : Gs02ModeController("High tension controller", params.ts),
      params_(params.high_tension),
      active_braking_first_entry_(true),
      active_braking_t0_(0.0),
      angular_rate_tol_(1.7e-4),
      hpu_cmd_change_t0_(0.0),
      hpu_delay_(0.4),
      ht_ts_(params.ts),
      brake_torque_(new_discrete_state(), "brake_torque", params.ts, 0.0),
      tether_torque_(new_discrete_state(), "tether_torque", params.ts, 0.0),
      brake_torque_cmd_(new_discrete_state(), "brake_torque_cmd", params.ts,
                        0.0),
      n_hpu_mode_demand_(new_discrete_state(), "n_hpu_mode_demand", params.ts,
                         kBrakesOn),
      azi_velocity_dir_(new_discrete_state(), "azi_velocity_dir", params.ts,
                        0.0),
      n_state_machine_(new_discrete_state(), "n_state_machine", params.ts,
                       kBaseCase),
      total_torque_(new_discrete_state(), "total_torque", params.ts, 0.0),
      n_hpu_mode_(new_discrete_state(), "n_hpu_mode", params.ts, kBrakesOn),
      n_hpu_mode_last_(new_discrete_state(), "n_hpu_mode_last", params.ts,
                       kBrakesOn),
      a_error_(new_discrete_state(), "azimuth_error", params.ts, 0.0),
      detwist_pos_cmd_(new_discrete_state(), "detwist_pos_cmd", ts_, 0.0),
      detwist_pos_cmd_from_wing_(new_discrete_state(),
                                 "detwist_pos_cmd_from_wing", params.ts, 0.0) {
  SetupDone();
}

// The high-tension controller uses a brake plus the tether torque to
// semi-actively control the ground station aziumth in the high-tension mode
// i.e. when the kite is in crosswind flight.
void HighTensionController::DiscreteStepHelper(double t) {
  // Run the controller state logic which emulates the GS02 Simulink code.
  RunControlSwitchingLogic(t);

  // Run the GS02 azimuth control laws and update the HPU and brake models.
  RunControlLaw(t);

  // Generate detwist position command.
  detwist_pos_cmd_.DiscreteUpdate(t, detwist_pos_cmd_from_wing_.val());
}

// The high-tension controller switching logic as implemented in the GS02
// Simulink state-flow code.
void HighTensionController::RunControlSwitchingLogic(double t) {
  // n_state_machine_val keeps track of the current state machine case.
  static SwitchingLogicState n_state_machine_val = kBaseCase;

  // a_error is the error between the flight circle center azimuth and the GS02
  // azimuth less Pi i.e. the GS needs to be pointed -Pi from its reference
  // azimuth in crosswind.
  a_error_.DiscreteUpdate(
      t, Wrap(wing_azi_.val() - azi_pos_.val() - M_PI, -PI, PI));

  switch (n_state_machine()) {
    case kBaseCase:
      azi_velocity_dir_.DiscreteUpdate(t, 0.0);
      n_hpu_mode_demand_.DiscreteUpdate(t, kBrakesOn);
      if ((tether_torque() > params_.m_max_azi_ht) &&
          (a_error() > params_.a_control_threshold_azi_ht)) {
        n_state_machine_val = kWaitForPositiveTetherOverload;
      }
      if ((tether_torque() < -params_.m_max_azi_ht) &&
          (a_error() < -params_.a_control_threshold_azi_ht)) {
        n_state_machine_val = kWaitForNegativeTetherOverload;
      }
      n_state_machine_.DiscreteUpdate(t, n_state_machine_val);
      break;

    case kWaitForPositiveTetherOverload:
      azi_velocity_dir_.DiscreteUpdate(t, 0.0);
      n_hpu_mode_demand_.DiscreteUpdate(t, kBrakesOn);
      if (tether_torque() > params_.m_control_threshold_azi_ht) {
        n_state_machine_val = kWaitForHpuPositive;
      }
      n_state_machine_.DiscreteUpdate(t, n_state_machine_val);
      break;

    case kWaitForHpuPositive:
      azi_velocity_dir_.DiscreteUpdate(t, 0.0);
      n_hpu_mode_demand_.DiscreteUpdate(t, kBrakesRegulated);
      if (n_hpu_mode() == kBrakesRegulated) {
        n_state_machine_val = kAzimuthPositiveRotation;
      }
      if (tether_torque() < params_.m_control_threshold_azi_ht) {
        n_state_machine_val = kWaitForPositiveTetherOverload;
      }
      n_state_machine_.DiscreteUpdate(t, n_state_machine_val);
      break;

    case kAzimuthPositiveRotation:
      azi_velocity_dir_.DiscreteUpdate(t, params_.n_demand_azi_ht);

      // Command zero azimuth rate when close to the commanded azimuth.
      if ((fabs(a_error()) < params_.a_control_tolerance_azi_ht) ||
          a_error() < 0.0) {
        n_state_machine_val = kActiveBraking;
        active_braking_first_entry_ = true;
      } else if (a_error() < 0.0) {
        n_state_machine_val = kAzimuthNegativeRotation;
      }
      n_state_machine_.DiscreteUpdate(t, n_state_machine_val);
      break;

    case kActiveBraking:
      azi_velocity_dir_.DiscreteUpdate(t, 0.0);
      if (active_braking_first_entry_) {
        active_braking_first_entry_ = false;
        active_braking_t0_ = t;
      }

      // Command the brakes fully on if the azimuth rate is small enough.
      if (fabs(azi_vel_.val()) < params_.n_control_tolerance_azi_ht) {
        n_state_machine_val = kBaseCase;
      }

      // Command the brakes fully on if time in this mode is large enough.
      if (t > (active_braking_t0_ + params_.t_threshold_wait_azi_ht)) {
        n_state_machine_val = kBaseCase;
      }
      n_state_machine_.DiscreteUpdate(t, n_state_machine_val);
      break;

    case kWaitForNegativeTetherOverload:
      azi_velocity_dir_.DiscreteUpdate(t, 0.0);
      n_hpu_mode_demand_.DiscreteUpdate(t, kBrakesOn);
      if (tether_torque() < -params_.m_control_threshold_azi_ht) {
        n_state_machine_val = kWaitForHpuNegative;
      }
      n_state_machine_.DiscreteUpdate(t, n_state_machine_val);
      break;

    case kWaitForHpuNegative:
      azi_velocity_dir_.DiscreteUpdate(t, 0.0);
      n_hpu_mode_demand_.DiscreteUpdate(t, kBrakesRegulated);
      if (tether_torque() > -params_.m_control_threshold_azi_ht) {
        n_state_machine_val = kWaitForNegativeTetherOverload;
      }
      if (n_hpu_mode() == kBrakesRegulated) {
        n_state_machine_val = kAzimuthNegativeRotation;
      }
      n_state_machine_.DiscreteUpdate(t, n_state_machine_val);
      break;

    case kAzimuthNegativeRotation:
      azi_velocity_dir_.DiscreteUpdate(t, -params_.n_demand_azi_ht);

      // Command zero azimuth rate when close to the commanded azimuth.
      if ((fabs(a_error()) < params_.a_control_tolerance_azi_ht) ||
          a_error() > 0.0) {
        n_state_machine_val = kActiveBraking;
        active_braking_first_entry_ = true;
      } else if (a_error() > 0.0) {
        n_state_machine_val = kAzimuthPositiveRotation;
      }
      n_state_machine_.DiscreteUpdate(t, n_state_machine_val);
      break;

    default: {};
  }
}

void HighTensionController::RunBrakeModel(double t) {
  double brake_torque_val = brake_torque_cmd();
  double total_torque_val = 0.0;

  // Check if the brake torque is the same sign as the tether torque
  // (this is not physical). If so, then set the brake torque to zero. The
  // controller sign will eventually change.
  // Additionally, check that the brake torque is not higher than the tether
  // torque. This would not be a phisically possible scenario.
  if (tether_torque() * brake_torque_val > 0.0) {
    brake_torque_val = 0.0;
  } else if (fabs(brake_torque_val) > fabs(tether_torque())) {
    brake_torque_val = -tether_torque();
  }

  // Keep the brake torque on for a few hundred milliseconds after
  // n_state_machine goes to zero to kill the residual angular rate (we cannot
  // instantaneously set it to zero because we only have finite brake torque).
  if ((n_state_machine() == kBaseCase) && (n_hpu_mode() == kBrakesRegulated)) {
    // Still have some residual rate so hold the last total torque.
    if (fabs(azi_vel_.val()) > angular_rate_tol_) {
      brake_torque_val = total_torque() - tether_torque();
    } else {  // Residual rate is small but n_hpu_mode != 0 yet so lock the
              // brake.
      brake_torque_val = -tether_torque();
    }
  } else {  // Update last total torque value.
    total_torque_val = tether_torque() + brake_torque_val;
  }
  brake_torque_.DiscreteUpdate(t, brake_torque_val);
  total_torque_.DiscreteUpdate(t, total_torque_val);
}

// This function is a simple model of the ground station HPU controller. The
// model is a time delay between the command and state.
void HighTensionController::RunHpuModel(double t) {
  if (n_hpu_mode_last_.val() != n_hpu_mode_demand()) {  // HPU demand changed.
    hpu_cmd_change_t0_ = t;
  }

  if (t > hpu_cmd_change_t0_ + hpu_delay_) {  // Update HPU mode after delay.
    n_hpu_mode_.DiscreteUpdate(t, n_hpu_mode_demand());
  }
  n_hpu_mode_last_.DiscreteUpdate(t, n_hpu_mode_demand());
}

// Run the high-tension closed-loop controller.
void HighTensionController::RunControlLaw(double t) {
  double brake_torque_cmd_val = 0.0;  // Brake torque command [N-m].
  double azi_omega_cmd = 0.0;         // Azimuth angular rate command [rad/s].
  double k_omega = 0.0;               // Angular rate loop gain [Nm/(rad/s)].
  double total_torque_cmd = 0.0;      // Total torque command [N-m].

  // Brakes reguated mode i.e. closed loop control on speed.
  if (n_hpu_mode_demand() == kBrakesRegulated) {
    if (fabs(azi_velocity_dir()) < params_.test_threshold) {
      azi_omega_cmd = 0.0;
      k_omega = params_.k_stop;
    } else if (fabs(azi_velocity_dir() - params_.n_demand_azi_ht) <
               params_.test_threshold) {
      azi_omega_cmd = params_.omega_nom;
      k_omega = params_.k_spin;
    } else if (fabs(azi_velocity_dir() + params_.n_demand_azi_ht) <
               params_.test_threshold) {
      azi_omega_cmd = -params_.omega_nom;
      k_omega = params_.k_spin;
    } else {
      azi_omega_cmd = 0.0;
      k_omega = params_.k_stop;
    }
    // Compute the brake torque command based on this control law.
    total_torque_cmd = k_omega * (azi_omega_cmd - azi_vel_.val());
    brake_torque_cmd_val = total_torque_cmd - tether_torque();
  } else {  // Do not allow net torque on the azimuth as brakes are fully on.
    brake_torque_cmd_val = -tether_torque();
  }

  // Update brake torque command.
  brake_torque_cmd_.DiscreteUpdate(t, brake_torque_cmd_val);

  // Run a model of the GS02 brake.
  RunBrakeModel(t);

  // Run a simple model of the GS02 HPU (Hydraulic Power Unit).
  RunHpuModel(t);

  // Update azimuth and drum velocity commands.
  azi_vel_cmd_.DiscreteUpdate(
      t, total_torque() / mclarenparams().Iz_gndstation * ht_ts_);
  drum_vel_cmd_.DiscreteUpdate(t, 0.0);
}

GroundStationV2Base::GroundStationV2Base(const ReferenceFrame &ned_frame,
                                         const ReferenceFrame &parent_frame,
                                         const Gs02Params &params,
                                         const Gs02SimParams &sim_params__)
    : Actuator("GS02"),
      params_(params),
      sim_params_(sim_params__),
      racetrack_integrator_(params, sim_params__),
      ned_frame_(ned_frame),
      parent_frame_(parent_frame),
      azi_cmd_target_(new_derived_value(), "azi_cmd_target"),
      azi_cmd_dead_zone_(new_derived_value(), "azi_cmd_dead_zone"),
      drum_linear_vel_cmd_from_wing_(new_derived_value(),
                                     "drum_linear_vel_cmd_from_wing"),
      detwist_pos_cmd_from_wing_(new_derived_value(),
                                 "detwist_pos_cmd_from_wing"),
      mode_cmd_(new_derived_value(), "mode_cmd"),
      tether_force_g_(new_derived_value(), "tether_force_g"),
      unpause_transform_(new_derived_value(), "unpause_transform"),
      levelwind_engaged_(new_derived_value(), "levelwind_engaged"),
      tether_force_p_(kVec3Zero),
      gsg_pos_p_(kVec3Zero),
      mode_(new_discrete_state(), "mode", 0.0, kGroundStationModeReel),
      transform_stage_(new_discrete_state(), "transform_stage", 0.0, 0),
      platform_frame_(new_derived_value(), "platform_frame"),
      dcm_v2p_(new_derived_value(), "dcm_v2p"),
      wd0_frame_(new_derived_value(), "wd_frame"),
      wd_frame_(new_derived_value(), "wd_frame"),
      gsg0_frame_(new_derived_value(), "gsg0_frame"),
      panel_frame_(new_derived_value(), "panel_frame"),
      panel_surface_(this->panel_frame_.val_unsafe(), nullptr) {
  panel_surface_.set_collision_func(
      [this](const Vec3 &contactor_pos_panel, Vec3 *collision_pos) -> bool {
        return ContactPerchPanel(contactor_pos_panel, platform_frame_.val(),
                                 panel_frame_.val(), sim_params().panel,
                                 collision_pos);
      });
}

void GroundStationV2::Init(double azimuth, double drum_angle__) {
  platform_azi_.Clear();
  platform_azi_.set_val(azimuth);
  drum_angle_.Clear();
  drum_angle_.set_val(drum_angle__);
  ClearDerivedValues();
  UpdateDerivedStates();
}

void GroundStationV2Base::SetLevelwindEngaged(double tether_elevation) {
  bool is_engaged = false;
  bool levelwind_in_use = false;
  switch (mode()) {
    case kGroundStationModeReel:
    case kGroundStationModeManual:
      levelwind_in_use = true;
      break;
    case kGroundStationModeTransform:
      // TODO: It is also in use when transform stage == 0 during
      // transform up.
      levelwind_in_use = transform_stage() == 4;
      break;
    case kGroundStationModeHighTension:
      break;
    case kNumGroundStationModes:
    default:
      LOG(FATAL) << "Bad GS02 mode: " << static_cast<int32_t>(mode());
  }
  if (levelwind_in_use &&
      tether_elevation >=
          sim_params().min_levelwind_angle_for_tether_engagement) {
    is_engaged = true;
  }
  levelwind_engaged_.set_val(is_engaged);
}

void GroundStationV2Base::SetFromAvionicsPackets(
    const AvionicsPackets &avionics_packets) {
  const ControllerCommandMessage &cmd = avionics_packets.command_message;

  // The actual ground station reads from TetherDownMessage, so this isn't quite
  // proper. But the benefit of simulating the core switches assembling
  // TetherDownMessage is dubious.
  azi_cmd_target_.set_val(cmd.gs_azi_target);
  azi_cmd_dead_zone_.set_val(cmd.gs_azi_dead_zone);
  drum_linear_vel_cmd_from_wing_.set_val(cmd.winch_velocity);
  detwist_pos_cmd_from_wing_.set_val(cmd.detwist_position);
  mode_cmd_.set_val(static_cast<GroundStationMode>(cmd.gs_mode_request));
  unpause_transform_.set_val(cmd.gs_unpause_transform);
}

void GroundStationV2Base::UpdateDerivedStates() {
  Mat3 dcm_vessel_to_platform;
  // The GSv2 platform azimuth is defined as a heading with respect to the
  // x-axis of the vessel frame.
  // The parameter azi_ref_offset is defined to be exactly zero.
  DCHECK_EQ(GetSystemParams()->ground_station.azi_ref_offset, 0.0);
  AngleToDcm(platform_azi(), 0.0, 0.0, kRotationOrderZyx,
             &dcm_vessel_to_platform);
  Vec3 platform_omega;
  Vec3Scale(&kVec3Z, platform_azi_vel(), &platform_omega);
  platform_frame_.set_val(ReferenceFrame(parent_frame_, kVec3Zero, kVec3Zero,
                                         dcm_vessel_to_platform,
                                         platform_omega));
  dcm_v2p_.set_val(dcm_vessel_to_platform);

  // The winch drum frame's (wd) axes are aligned with the platform frame when
  // the drum angle is zero.
  wd0_frame_.set_val(ReferenceFrame(platform_frame_.val(),
                                    params_.drum_origin_p, kMat3Identity));

  Mat3 dcm_wd0_to_wd;
  AngleToDcm(0.0, 0.0, drum_angle(), kRotationOrderZyx, &dcm_wd0_to_wd);

  Vec3 drum_omega_vector;
  Vec3Scale(&kVec3X, drum_omega(), &drum_omega_vector);
  wd_frame_.set_val(ReferenceFrame(wd0_frame_.val(), kVec3Zero, kVec3Zero,
                                   dcm_wd0_to_wd, drum_omega_vector));

  Mat3 dcm_wd_to_gsg0;
  CalcDcmWdToGsg0(params_.detwist_elevation, detwist_angle(), &dcm_wd_to_gsg0);
  gsg0_frame_.set_val(
      ReferenceFrame(wd_frame_.val(), params_.gsg_pos_drum, dcm_wd_to_gsg0));

  panel_frame_.set_val(ReferenceFrame(platform_frame_.val(),
                                      sim_params().panel.origin_pos_p,
                                      sim_params().panel.dcm_p2panel));
}

void GroundStationV2::UpdateModeControllerCommands() {
  switch (mode()) {
    case kGroundStationModeReel:
      azi_vel_cmd_.set_val(reel_controller_.azi_vel_cmd());
      drum_vel_cmd_.set_val(reel_controller_.drum_vel_cmd());
      detwist_vel_.set_val(0.0);
      break;
    case kGroundStationModeTransform:
      azi_vel_cmd_.set_val(transform_controller_.azi_vel_cmd());
      drum_vel_cmd_.set_val(transform_controller_.drum_vel_cmd());
      detwist_vel_.set_val(
          Saturate(sim_params().detwist_angle_kp *
                       (Wrap(Saturate(-drum_angle(), 0.0,
                                      sim_params().mclaren.detwist_setpoint) -
                                 detwist_angle(),
                             -PI, PI)),
                   -transform_controller_.detwist_max_vel(),
                   transform_controller_.detwist_max_vel()));
      break;
    case kGroundStationModeHighTension:
      azi_vel_cmd_.set_val(high_tension_controller_.azi_vel_cmd());
      drum_vel_cmd_.set_val(high_tension_controller_.drum_vel_cmd());
      detwist_vel_.set_val(Saturate(
          sim_params().detwist_angle_kp *
              (Wrap(
                  high_tension_controller_.detwist_pos_cmd() - detwist_angle(),
                  -PI, PI)),
          -high_tension_controller_.detwist_max_vel(),
          high_tension_controller_.detwist_max_vel()));
      break;
    case kGroundStationModeManual:
    case kNumGroundStationModes:
    default:
      LOG(FATAL) << "Bad GS02 mode: " << static_cast<int32_t>(mode());
  }
}

void GroundStationV2::AddInternalConnections(ConnectionStore *connections) {
  connections->Add(1, [this](double /*t*/) {
    UpdateModeControllerCommands();
    UpdateDerivedStates();
  });
}

double GroundStationV2Base::CalcTetherFreeLength() const {
  return CalcTetherFreeLengthInternal(drum_angle());
}

double GroundStationV2Base::CalcTetherFreeLengthInternal(
    double drum_angle__) const {
  double length = g_sys.tether->length;
  const Gs02DrumAngles &angles =
      GetSystemParams()->ground_station.gs02.drum_angles;

  // Racetrack.
  if (drum_angle__ < angles.racetrack_high) {
    length -= racetrack_integrator_.WrappedLength(drum_angle__);
  }

  // Wide wrap section.
  const double wide_wrap_high = angles.racetrack_low;
  if (drum_angle__ < wide_wrap_high) {
    length -= (wide_wrap_high - fmax(angles.wide_wrap_low, drum_angle__)) *
              Sqrt(Square(sim_params().dx_dtheta_wide_wrap) +
                   Square(params_.drum_radius));
  }

  // Main wrap section.
  const double main_wrap_high = angles.wide_wrap_low;
  if (drum_angle__ < main_wrap_high) {
    length -= (main_wrap_high - drum_angle__) *
              Sqrt(Square(sim_params().dx_dtheta_main_wrap) +
                   Square(params_.drum_radius));
  }

  DCHECK(length >= 0.0);
  return length;
}

double GroundStationV2Base::MinDrumAngle() const {
  const Gs02DrumAngles &drum_angles =
      GetSystemParams()->ground_station.gs02.drum_angles;
  double main_wrap_length =
      CalcTetherFreeLengthInternal(drum_angles.wide_wrap_low);
  double main_wrap_dtheta =
      main_wrap_length / Sqrt(Square(sim_params().dx_dtheta_main_wrap) +
                              Square(params_.drum_radius));
  return drum_angles.wide_wrap_low - main_wrap_dtheta;
}

void GroundStationV2Base::CalcTetherAnchorPosVelNed(Vec3 *pos_ned,
                                                    Vec3 *vel_ned) const {
  const Gs02DrumAngles &angles =
      GetSystemParams()->ground_station.gs02.drum_angles;

  // If the drum angle is above racetrack_high, the anchor point is the GSG.
  if (drum_angle() > angles.racetrack_high) {
    wd_frame_.val().TransformTo(ned_frame_, ReferenceFrame::kPosition,
                                params_.gsg_pos_drum, pos_ned);
    ReferenceFrame at_gsg(wd_frame_.val(), params_.gsg_pos_drum);
    at_gsg.TransformTo(ned_frame_, ReferenceFrame::kVelocity, kVec3Zero,
                       vel_ned);
    return;
  }

  // We'll start at the GSG, and move along the wrapping.
  const double inner_radius =
      hypot(params_.gsg_pos_drum.y, params_.gsg_pos_drum.z);
  Vec3 pos_wd0 = {params_.gsg_pos_drum.x, 0.0, -inner_radius};
  Vec3 vel_wd0 = kVec3Zero;

  // Racetrack.
  if (drum_angle() < angles.racetrack_high) {
    const double dx_dtheta =
        (params_.gsg_pos_drum.x - sim_params().wrap_start_posx_drum) /
        (angles.racetrack_high - angles.racetrack_low);

    const double dz_dtheta = (-inner_radius + params_.drum_radius) /
                             (angles.racetrack_high - angles.racetrack_low);
    const double dtheta =
        fmax(drum_angle(), angles.racetrack_low) - angles.racetrack_high;

    pos_wd0.x += dx_dtheta * dtheta;
    pos_wd0.z += dz_dtheta * dtheta;

    vel_wd0.x = dx_dtheta * drum_omega();
    vel_wd0.z = dz_dtheta * drum_omega();
  }

  // Wide wrapping.
  const double wide_wrap_high = angles.racetrack_low;
  if (drum_angle() < wide_wrap_high) {
    const double dtheta =
        fmax(drum_angle(), angles.wide_wrap_low) - wide_wrap_high;
    pos_wd0.x += sim_params().dx_dtheta_wide_wrap * dtheta;

    vel_wd0.x = sim_params().dx_dtheta_wide_wrap * drum_omega();
    vel_wd0.z = 0.0;
  }

  // Main wrapping.
  const double main_wrap_high = angles.wide_wrap_low;
  if (drum_angle() < main_wrap_high) {
    pos_wd0.x +=
        sim_params().dx_dtheta_main_wrap * (drum_angle() - main_wrap_high);
    vel_wd0.x = sim_params().dx_dtheta_main_wrap;
    vel_wd0.z = 0.0;
  }

  wd0_frame_.val().TransformTo(ned_frame_, ReferenceFrame::kPosition, pos_wd0,
                               pos_ned);

  ReferenceFrame anchor_frame(wd0_frame_.val(), pos_wd0);
  anchor_frame.TransformTo(ned_frame_, ReferenceFrame::kVelocity, vel_wd0,
                           vel_ned);
}

void GroundStationV2Base::TetherDirToAngles(const Vec3 &tether_dir_v,
                                            double *tether_ele_v,
                                            double *tether_azi_v,
                                            double *gsg_yoke,
                                            double *gsg_termination) const {
  // TODO: Calibrate with real test data about where is the reference
  // angle (yoke=termination=0) pointing to.
  // Currently, the tether points away from the detwist plane along the -Z axis
  // of gsg0 frame when GSG yoke and termination are 0.0. +yoke rotates around
  // X axis and +termination rotates around Y axis.

  // The local frame (X'Y'Z') rotates the gsg0 frame (XYZ) by 90 deg
  // around Y-axis, so that the spherical azimuth (yoke) and elevation
  // (termination) are phi and -theta, where (phi, theta, 0) are euler angles
  // to rotate gsg0 frame to gsg2 frame in XYZ order.
  //       X'  Tether
  // X(Z') |   /
  //    \  |  /
  //     \ | /
  //      \|/_____ Y(Y')
  //       |
  //       |
  //       Z
  *tether_ele_v = VecVToElevation(&tether_dir_v);
  *tether_azi_v = VecVToAzimuth(&tether_dir_v);

  Mat3 dcm_gsg0_to_local;
  AngleToDcm(0.0, 0.5 * PI, 0.0, kRotationOrderZyx, &dcm_gsg0_to_local);
  ReferenceFrame local_frame(gsg0_frame(), kVec3Zero, dcm_gsg0_to_local);

  Vec3 tether_dir_local;
  parent_frame_.RotateTo(local_frame, tether_dir_v, &tether_dir_local);
  double r_unused;
  CartToSph(&tether_dir_local, gsg_yoke, gsg_termination, &r_unused);
  // The spherical elevation has the opposite sign of the rotational angle
  // around Y-axis.
  *gsg_termination = -*gsg_termination;
}

void GroundStationV2Base::Publish() const {
  sim_telem.gs02.azimuth = platform_azi();
  sim_telem.gs02.azimuth_vel = platform_azi_vel();
  sim_telem.gs02.dcm_v2p = dcm_v2p();
  sim_telem.gs02.drum_angle = drum_angle();
  sim_telem.gs02.drum_omega = drum_omega();
  sim_telem.gs02.levelwind_engaged = levelwind_engaged();
}

void GroundStationV2::Publish() const {
  GroundStationV2Base::Publish();
  sim_telem.gs02.mclaren_azi_vel_cmd = azi_vel_cmd_.val();
  sim_telem.gs02.mclaren_drum_vel_cmd = drum_vel_cmd_.val();
  sim_telem.gs02.mode = static_cast<int32_t>(mode());
  sim_telem.gs02.detwist_vel = detwist_vel_.val();
  sim_telem.gs02.detwist_angle =
      Wrap(detwist_angle_.val(), 0,
           PI * 2.0 * static_cast<double>(TETHER_DETWIST_REVS));
  sim_telem.gs02.n_state_machine = high_tension_controller_.n_state_machine();
  sim_telem.gs02.n_hpu_mode = high_tension_controller_.n_hpu_mode();
  sim_telem.gs02.brake_torque = high_tension_controller_.brake_torque();
  sim_telem.gs02.brake_torque_cmd = high_tension_controller_.brake_torque_cmd();
  sim_telem.gs02.tether_torque = high_tension_controller_.tether_torque();
  sim_telem.gs02.total_torque = high_tension_controller_.total_torque();
  sim_telem.gs02.azi_velocity_dir = high_tension_controller_.azi_velocity_dir();
  sim_telem.gs02.a_error = high_tension_controller_.a_error();
  sim_telem.gs02.wing_azi = high_tension_controller_.wing_azi_val();
  sim_telem.gs02.gs_azi = high_tension_controller_.azi_pos_val();

  for (const Model *m : sub_models_) {
    m->Publish();
  }
}

GroundStationV2::GroundStationV2(const ReferenceFrame &ned_frame,
                                 const ReferenceFrame &parent_frame,
                                 const Gs02Params &params,
                                 const Gs02SimParams &sim_params__)
    : GroundStationV2Base(ned_frame, parent_frame, params, sim_params__),
      reel_controller_(sim_params__.mclaren, params.drum_radius),
      transform_controller_(sim_params__.mclaren),
      high_tension_controller_(sim_params__.mclaren),
      platform_azi_(new_continuous_state(), "platform_azi", 0.0),
      platform_azi_vel_(new_continuous_state(), "platform_azi_vel", 0.0),
      drum_angle_(new_continuous_state(), "drum_angle", 0.0),
      detwist_angle_(new_continuous_state(), "detwist_angle",
                     sim_params__.mclaren.detwist_setpoint),
      drum_omega_(new_continuous_state(), "drum_omega", 0.0),
      ddrum_omega_dt_(new_continuous_state(), "ddrum_omega_dt", 0.0),
      azi_vel_cmd_(new_derived_value(), "azi_vel_cmd"),
      drum_vel_cmd_(new_derived_value(), "drum_vel_cmd"),
      detwist_vel_(new_derived_value(), "detwist_vel"),
      mode_z1_(new_discrete_state(), "mode_z1", 0.0, kGroundStationModeReel),
      ht2reel_(new_discrete_state(), "ht2reel", 0.0, false),
      prox_sensor_active_(new_discrete_state(), "prox_sensor_active", 0.0,
                          false) {
  set_sub_models(
      {&reel_controller_, &transform_controller_, &high_tension_controller_});
  UpdateDerivedStates();
  SetupDone();
}

void GroundStationV2::CalcDerivHelper(double /*t*/) {
  platform_azi_.set_deriv(platform_azi_vel());
  platform_azi_vel_.set_deriv(sim_params().azi_accel_kp *
                              (azi_vel_cmd_.val() - platform_azi_vel()));

  const double omega_n = sim_params().winch_drive_natural_freq;
  const double zeta = sim_params().winch_drive_damping_ratio;
  ddrum_omega_dt_.set_deriv(Square(omega_n) *
                                (drum_vel_cmd_.val() - drum_omega()) -
                            2.0 * zeta * omega_n * ddrum_omega_dt_.val());
  drum_omega_.set_deriv(ddrum_omega_dt_.val());
  drum_angle_.set_deriv(drum_omega());
  detwist_angle_.set_deriv(detwist_vel_.val());
}

void GroundStationV2::DiscreteStepHelper(double t) {
  // Update mode.
  switch (mode()) {
    case kGroundStationModeReel:
      if (mode_cmd() == kGroundStationModeHighTension) {
        set_mode(t, kGroundStationModeTransform);
        ht2reel_.DiscreteUpdate(t, false);
      }
      break;
    case kGroundStationModeHighTension:
      if (mode_cmd() == kGroundStationModeReel) {
        set_mode(t, kGroundStationModeTransform);
        ht2reel_.DiscreteUpdate(t, true);
      }
      break;
    case kGroundStationModeTransform:
      if (transform_controller_.transform_complete()) {
        set_mode(t, ht2reel_.val() ? kGroundStationModeReel
                                   : kGroundStationModeHighTension);
      }
      break;
    case kGroundStationModeManual:
    case kNumGroundStationModes:
    default:
      LOG(FATAL) << "Bad GS02 mode: " << static_cast<int32_t>(mode());
  }

  DCHECK(azi_cmd_target() >= 0.0 && azi_cmd_target() <= 2.0 * M_PI);
  reel_controller_.SetCommonInputs(t, platform_azi(), platform_azi_vel(),
                                   drum_angle(), drum_omega(),
                                   azi_cmd_target());
  transform_controller_.SetCommonInputs(t, platform_azi(), platform_azi_vel(),
                                        drum_angle(), drum_omega(),
                                        azi_cmd_target());
  high_tension_controller_.SetCommonInputs(t, platform_azi(),
                                           platform_azi_vel(), drum_angle(),
                                           drum_omega(), azi_cmd_target());

  // Apply mode- and controller-specific inputs.
  switch (mode()) {
    case kGroundStationModeReel:
      reel_controller_.Enable(t);
      transform_controller_.Disable(t);
      high_tension_controller_.Disable(t);
      reel_controller_.set_azi_cmd_dead_zone(t, azi_cmd_dead_zone());
      reel_controller_.set_drum_linear_vel_cmd_from_wing(
          t, drum_linear_vel_cmd_from_wing());
      reel_controller_.set_levelwind_engaged(t, levelwind_engaged());
      set_transform_stage(t, 0);
      break;
    case kGroundStationModeTransform:
      transform_controller_.Enable(t);
      reel_controller_.Disable(t);
      high_tension_controller_.Disable(t);
      set_transform_stage(t, transform_controller_.stage());
      transform_controller_.set_unpause_transform(t, unpause_transform());
      if (mode() != mode_z1_.val()) {
        transform_controller_.StartTransform(t, ht2reel_.val());
      }
      break;
    case kGroundStationModeHighTension:
      high_tension_controller_.set_tether_torque(t, gsg_pos_p(),
                                                 tether_force_p());
      high_tension_controller_.Enable(t);
      transform_controller_.Disable(t);
      reel_controller_.Disable(t);
      high_tension_controller_.set_detwist_pos_cmd_from_wing(
          t, Wrap(detwist_pos_cmd_from_wing(), -PI, PI));
      set_transform_stage(t, 0);
      break;
    case kGroundStationModeManual:
    case kNumGroundStationModes:
    default:
      LOG(FATAL) << "Bad GS02 mode: " << static_cast<int32_t>(mode());
  }

  mode_z1_.DiscreteUpdate(t, mode());

  // The proximity sensor activates when a sufficient portion of the tether has
  // been reeled in.
  prox_sensor_active_.DiscreteUpdate(
      t, CalcTetherFreeLength() < sim_params().prox_sensor_tether_free_length);
}

HitlGroundStationV2::HitlGroundStationV2(const ReferenceFrame &ned_frame,
                                         const ReferenceFrame &parent_frame,
                                         const Gs02Params &params,
                                         const Gs02SimParams &sim_params__)
    : GroundStationV2Base(ned_frame, parent_frame, params, sim_params__),
      platform_azi_(new_continuous_state(), "platform_azi", 0.0),
      platform_azi_vel_(new_continuous_state(), "platform_azi_vel", 0.0),
      dplatform_azi_(new_discrete_state(), "dplatform_azi", 0.0, 0.0),
      dplatform_azi_vel_(new_discrete_state(), "dplatform_azi_vel", 0.0, 0.0),
      int_err_platform_azi_(new_discrete_state(), "int_err_platform_azi", 0.0,
                            0.0),
      drum_angle_(new_continuous_state(), "drum_angle", 0.0),
      drum_angle_vel_(new_continuous_state(), "drum_angle_vel", 0.0),
      ddrum_angle_(new_discrete_state(), "ddrum_angle", 0.0, 0.0),
      ddrum_angle_vel_(new_discrete_state(), "ddrum_angle_vel", 0.0, 0.0),
      detwist_angle_vel_(new_continuous_state(), "detwist_angle_vel", 0.0),
      ddetwist_angle_(new_discrete_state(), "ddetwist_angle", 0.0, 0.0),
      ddetwist_angle_vel_(new_discrete_state(), "ddetwist_angle_vel", 0.0, 0.0),
      int_err_drum_angle_(new_discrete_state(), "int_err_drum_angle", 0.0, 0.0),
      detwist_angle_(new_continuous_state(), "detwist_angle", 0.0),
      prox_sensor_active_(new_discrete_state(), "prox_sensor_active", 0.0,
                          false),
      actual_mode_(new_derived_value(), "actual_mode", kGroundStationModeReel),
      actual_transform_stage_(new_derived_value(), "actual_transform_stage", 0),
      actual_platform_azi_(new_derived_value(), "actual_platform_azi", 0.0),
      actual_drum_angle_(new_derived_value(), "actual_drum_angle", 0.0),
      actual_detwist_angle_(new_derived_value(), "actual_detwist_angle", 0.0),
      actual_prox_sensor_active_(new_derived_value(),
                                 "actual_prox_sensor_active", 0.0) {
  UpdateDerivedStates();
  SetupDone();
}

void HitlGroundStationV2::Init(double azimuth, double drum_angle__) {
  platform_azi_.Clear();
  platform_azi_.set_val(azimuth);
  platform_azi_vel_.Clear();
  platform_azi_vel_.set_val(0.0);
  drum_angle_.Clear();
  drum_angle_.set_val(drum_angle__);
  drum_angle_vel_.Clear();
  drum_angle_vel_.set_val(0.0);
  detwist_angle_.Clear();
  detwist_angle_.set_val(0.0);
  detwist_angle_vel_.Clear();
  detwist_angle_vel_.set_val(0.0);
  ClearDerivedValues();
  UpdateDerivedStates();
}

void HitlGroundStationV2::SetFromAvionicsPackets(
    const AvionicsPackets &avionics_packets) {
  GroundStationV2Base::SetFromAvionicsPackets(avionics_packets);

  if (avionics_packets.ground_station_status_valid) {
    const auto &gs = avionics_packets.ground_station_status.status;
    actual_mode_.set_val(static_cast<GroundStationMode>(gs.mode));
    actual_transform_stage_.set_val(gs.transform_stage);
    actual_platform_azi_.set_val(gs.azimuth.position);
    actual_drum_angle_.set_val(gs.winch.position);
    actual_prox_sensor_active_.set_val(gs.bridle_proximity.proximity);
    actual_detwist_angle_.set_val(gs.detwist.position);
  } else {
    // The ground station status is not valid if we've never received it. In
    // that event, set the "actual" value to the current model values, so we
    // don't attempt to control a signal to a bogus data point.
    actual_mode_.set_val(mode());
    actual_transform_stage_.set_val(transform_stage());
    actual_platform_azi_.set_val(platform_azi_.val());
    actual_drum_angle_.set_val(drum_angle_.val());
    actual_prox_sensor_active_.set_val(prox_sensor_active());
    actual_detwist_angle_.set_val(detwist_angle());
  }
}

void HitlGroundStationV2::DiscreteStepHelper(double t) {
  set_mode(t, actual_mode_.val());
  set_transform_stage(t, actual_transform_stage_.val());

  // The model's platform azimuth and drum angle both track the actual signals
  // by way of a simple second-order transfer function. Doing so keeps
  // derivatives of these quantities reasonable as the simulator tracks reality.
  //
  // The transfer function is artifical, so we encode it here rather than
  // parameterizing it in a config file.
  double wn = 50.0;
  double zeta = 1.0;

  {  // Platform azimuth.
    double err = actual_platform_azi_.val() - platform_azi_.val();
    dplatform_azi_.DiscreteUpdate(t, platform_azi_vel_.val());
    dplatform_azi_vel_.DiscreteUpdate(
        t, err * wn * wn - 2.0 * zeta * wn * platform_azi_vel_.val());
  }

  {  // Drum angle.
    double err = actual_drum_angle_.val() - drum_angle_.val();
    ddrum_angle_.DiscreteUpdate(t, drum_angle_vel_.val());
    ddrum_angle_vel_.DiscreteUpdate(
        t, err * wn * wn - 2.0 * zeta * wn * drum_angle_vel_.val());
  }

  {  // Detwist angle.
    double err = actual_detwist_angle_.val() - detwist_angle_.val();
    ddetwist_angle_.DiscreteUpdate(t, detwist_angle_vel_.val());
    ddetwist_angle_vel_.DiscreteUpdate(
        t, err * wn * wn - 2.0 * zeta * wn * detwist_angle_vel_.val());
  }

  prox_sensor_active_.DiscreteUpdate(t, actual_prox_sensor_active_.val());
}

void HitlGroundStationV2::CalcDerivHelper(double /*t*/) {
  platform_azi_.set_deriv(dplatform_azi_.val());
  platform_azi_vel_.set_deriv(dplatform_azi_vel_.val());
  drum_angle_.set_deriv(ddrum_angle_.val());
  drum_angle_vel_.set_deriv(ddrum_angle_vel_.val());
  detwist_angle_.set_deriv(ddetwist_angle_.val());
  detwist_angle_vel_.set_deriv(ddetwist_angle_vel_.val());
}

void HitlGroundStationV2::AddInternalConnections(ConnectionStore *connections) {
  connections->Add(1, [this](double /*t*/) { UpdateDerivedStates(); });
}
