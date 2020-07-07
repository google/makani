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

#include "sim/models/sensors/gps.h"

#include <math.h>
#include <stdint.h>

#include <vector>

#include "avionics/common/novatel_types.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/system_types.h"
#include "sim/math/util.h"
#include "sim/models/rigid_bodies/wing.h"
#include "sim/physics/ground_frame.h"
#include "sim/sim_messages.h"
#include "sim/sim_params.h"
#include "sim/sim_telemetry.h"

namespace {

const char *WingGpsReceiverLabelToString(WingGpsReceiverLabel label) {
  switch (label) {
    case kWingGpsReceiverCrosswind:
      return "GpsCrosswind";
    case kWingGpsReceiverHover:
      return "GpsHover";
    case kWingGpsReceiverPort:
      return "GpsPort";
    case kWingGpsReceiverStar:
      return "GpsStar";
    case kWingGpsReceiverLabelForceSigned:
    case kNumWingGpsReceivers:
    default:
      LOG(FATAL) << "Invalid WingGpsRecieverLabel: "
                 << static_cast<int32_t>(label);
      return "<invalid>";
  }
}

}  // namespace

Gps::Gps(const GroundFrame &ground_frame, const Wing &wing,
         WingGpsReceiverLabel label, const GpsParams &gps_params,
         const GpsSimParams &gps_sim_params, FaultSchedule *fault_schedule)
    : Sensor(WingGpsReceiverLabelToString(label), gps_sim_params.ts),
      label_(label),
      rng_(full_name()),
      gps_params_(gps_params),
      gps_sim_params_(gps_sim_params),
      ranked_solution_states_(
          {SimGpsSolutionState(kSimGpsSolutionStatusError, kGpsSolutionTypeNone,
                               kSimGpsSolutionStatusError,
                               kGpsSolutionTypeNone),
           SimGpsSolutionState(
               kSimGpsSolutionStatusNoError, kGpsSolutionTypeDifferential,
               kSimGpsSolutionStatusNoError, kGpsSolutionTypeDifferential),
           SimGpsSolutionState(
               kSimGpsSolutionStatusNoError, kGpsSolutionTypeRtkFloat,
               kSimGpsSolutionStatusNoError, kGpsSolutionTypeRtkFloat),
           SimGpsSolutionState(
               kSimGpsSolutionStatusNoError, kGpsSolutionTypeRtkInt,
               kSimGpsSolutionStatusNoError, kGpsSolutionTypeRtkInt)}),
      wing_(wing),
      ground_frame_(ground_frame),
      time_of_week_ms_(new_discrete_state(), "time_of_week_ms", 0),
      solution_state_rank_(
          new_discrete_state(), "solution_state_rank",
          static_cast<int32_t>(ranked_solution_states_.size() - 1)),
      pos_(new_discrete_state(), "pos"),
      pos_sigma_(new_discrete_state(), "pos_sigma"),
      vel_(new_discrete_state(), "vel"),
      vel_sigma_(new_discrete_state(), "vel_sigma"),
      pos_rtcm_update_noise_(new_discrete_state(), "pos_rtcm_update_noise",
                             gps_sim_params.ts_rtcm_update_noise, kVec3Zero),
      vel_rtcm_update_noise_(new_discrete_state(), "vel_rtcm_update_noise",
                             gps_sim_params.ts_rtcm_update_noise, kVec3Zero),
      delayed_pos_data_(full_name(), "delayed_pos_data", gps_sim_params.ts,
                        (*g_sim.sim_opt & kSimOptImperfectSensors)
                            ? gps_sim_params.pos_delay
                            : 0.0,
                        SolutionData()),
      delayed_vel_data_(full_name(), "delayed_vel_data", gps_sim_params.ts,
                        (*g_sim.sim_opt & kSimOptImperfectSensors)
                            ? gps_sim_params.vel_delay
                            : 0.0,
                        SolutionData()),
      fault_func_(fault_schedule->ClaimFaultFunc(
          full_name(),
          {
              {kSimFaultGpsDropout, 0U}, {kSimFaultGpsSolutionStateChange, 4U},
          })) {
  set_sub_models({&delayed_pos_data_, &delayed_vel_data_});

  delayed_pos_data_.set_val_func([this](SolutionData *data) {
    data->solution = pos_.val();
    data->sigma = pos_sigma_.val();
    data->state_rank = solution_state_rank_.val();
    data->time_of_week_ms = time_of_week_ms_.val();
  });
  delayed_vel_data_.set_val_func([this](SolutionData *data) {
    data->solution = vel_.val();
    data->sigma = vel_sigma_.val();
    data->state_rank = solution_state_rank_.val();
    data->time_of_week_ms = time_of_week_ms_.val();
  });

  SetupDone();
}

void Gps::DiscreteStepHelper(double t) {
  int32_t state_rank = solution_state_rank_.val();
  Vec3 pos_sigma__ = {1.0, 1.0, 1.0};
  Vec3 vel_sigma__ = {1.0, 1.0, 1.0};

  // GPS receivers align their measurement epoch according to the modulus of
  // the time of week and sample time. Measurements align exactly to the one
  // second and k * sample period boundaries in milliseconds.
  int32_t ts_ms = static_cast<int32_t>(1000.0 * ts_);
  int32_t time_of_week_ms = ts_ms * static_cast<int32_t>(t * (1000 / ts_ms));
  time_of_week_ms_.DiscreteUpdate(t, time_of_week_ms);

  ReferenceFrame gps_frame(wing_.frame(), gps_params_.pos);
  Vec3 Xg_gps, pos__;
  gps_frame.TransformOriginTo(ground_frame_, ReferenceFrame::kPosition,
                              &Xg_gps);
  ground_frame_.CalcXEcefLocal(Xg_gps, &pos__);

  Vec3 Vg_gps, vel__;
  gps_frame.TransformOriginTo(ground_frame_, ReferenceFrame::kVelocity,
                              &Vg_gps);
  ground_frame_.CalcVEcefLocal(Vg_gps, &vel__);

  // TODO: This noise model is super ugly right now.
  // Please clean me up!
  if (*g_sim.sim_opt & kSimOptImperfectSensors) {
    double dropout_rate = CalcDropoutRate();
    Vec3Scale(&gps_sim_params_.sigma_per_dropout_rate, dropout_rate,
              &vel_sigma__);
    Vec3Scale(&vel_sigma__, gps_sim_params_.sigma_ratio, &pos_sigma__);

    state_rank = ApplyRandomStateTransitions(state_rank, dropout_rate);

    // Apply faults to the solution state.
    std::vector<double> fault_params;
    if (fault_func_(t, kSimFaultGpsDropout, &fault_params)) {
      pos__ = kVec3Zero;
      vel__ = kVec3Zero;
      state_rank = 0;
    } else if (fault_func_(t, kSimFaultGpsSolutionStateChange, &fault_params)) {
      ApplySolutionStateChangeFault(fault_params, &pos__, &state_rank);
    }

    Vec3 pos_noise = {rng_.GetNormal(), rng_.GetNormal(), rng_.GetNormal()};
    Vec3 vel_noise = {rng_.GetNormal(), rng_.GetNormal(), rng_.GetNormal()};
    Vec3Mult(&pos_noise, &pos_sigma__, &pos_noise);
    Vec3Mult(&vel_noise, &vel_sigma__, &vel_noise);

    Vec3 pos_rtcm_update_noise__ = {rng_.GetNormal(), rng_.GetNormal(),
                                    rng_.GetNormal()};
    Vec3 vel_rtcm_update_noise__ = {rng_.GetNormal(), rng_.GetNormal(),
                                    rng_.GetNormal()};
    Vec3Mult(&pos_rtcm_update_noise__, &pos_sigma__, &pos_rtcm_update_noise__);
    Vec3Mult(&vel_rtcm_update_noise__, &vel_sigma__, &vel_rtcm_update_noise__);
    Vec3Scale(&pos_rtcm_update_noise__,
              gps_sim_params_.pos_rtcm_update_noise_scale,
              &pos_rtcm_update_noise__);
    Vec3Scale(&vel_rtcm_update_noise__,
              gps_sim_params_.vel_rtcm_update_noise_scale,
              &vel_rtcm_update_noise__);

    pos_rtcm_update_noise_.DiscreteUpdate(t, pos_rtcm_update_noise__);
    vel_rtcm_update_noise_.DiscreteUpdate(t, vel_rtcm_update_noise__);

    SimGpsSolutionState sol_state = LookupSolutionState(state_rank);
    if (sol_state.pos_type == kGpsSolutionTypeDifferential) {
      Vec3Add3(&pos__, &pos_noise, &pos_rtcm_update_noise(), &pos__);
    }
    if (sol_state.vel_type == kGpsSolutionTypeDifferential) {
      Vec3Add3(&vel__, &vel_noise, &vel_rtcm_update_noise(), &vel__);
    }
  }

  SimGpsSolutionState sol_state = LookupSolutionState(state_rank);

  // Scale sigmas based on solution type.
  const double pos_sigma_scale =
      gps_sim_params_.pos_sigma_scales[sol_state.pos_type];
  DCHECK_GE(pos_sigma_scale, 0.0) << "Inalid pos_sigma_scale for solution type "
                                  << static_cast<int32_t>(sol_state.vel_type);
  Vec3Scale(&pos_sigma__, pos_sigma_scale, &pos_sigma__);
  const double vel_sigma_scale =
      gps_sim_params_.vel_sigma_scales[sol_state.vel_type];
  DCHECK_GE(vel_sigma_scale, 0.0) << "Inalid vel_sigma_scale for solution type "
                                  << static_cast<int32_t>(sol_state.vel_type);
  Vec3Scale(&vel_sigma__, vel_sigma_scale, &vel_sigma__);

  pos_.DiscreteUpdate(t, pos__);
  pos_sigma_.DiscreteUpdate(t, pos_sigma__);

  vel_.DiscreteUpdate(t, vel__);
  vel_sigma_.DiscreteUpdate(t, vel_sigma__);

  solution_state_rank_.DiscreteUpdate(t, state_rank);
}

double Gps::CalcDropoutRate() const {
  Vec3 antenna_dir_g;
  Mat3TransVec3Mult(&wing_.dcm_g2b(), &gps_params_.antenna_dir, &antenna_dir_g);

  ReferenceFrame wing_accelerating_frame;
  wing_.CalcAcceleratingFrame(&wing_accelerating_frame);
  ReferenceFrame gps_frame(wing_accelerating_frame, gps_params_.pos);
  Vec3 antenna_acc_g;
  gps_frame.TransformOriginTo(ground_frame_, ReferenceFrame::kAcceleration,
                              &antenna_acc_g);

  return gps_sim_params_.antenna_dir_dropout_rate *
             Square((antenna_dir_g.z + 1.0) / 2.0) +
         gps_sim_params_.acc_dropout_rate * Vec3Norm(&antenna_acc_g);
}

int32_t Gps::ApplyRandomStateTransitions(int32_t current_rank,
                                         double dropout_rate) {
  double dropin_rate = gps_sim_params_.dropin_rate_coeffs[0] -
                       gps_sim_params_.dropin_rate_coeffs[1] * dropout_rate;
  double ts = this->ts_;

  const int32_t max_rank =
      static_cast<int32_t>(ranked_solution_states_.size() - 1);
  int32_t new_rank = current_rank;

  if (current_rank == 0) {
    // State is worst possible. It can recover to any better state.
    if (rng_.GetUniformReal() < dropin_rate * ts) {
      new_rank = 1 + rng_.GetUniformInt32() % max_rank;
    }
  } else if (current_rank < max_rank) {
    // State is non-extreme. It can stay the same, degrade to the worst state,
    // or recover to the optimal state. For simplicity, we disallow transitions
    // between the mid-grade states.
    double r = rng_.GetUniformReal();
    if (r < dropout_rate * ts) {
      new_rank = 0;
    } else if (r < (dropout_rate + dropin_rate) * ts) {
      new_rank = max_rank;
    }
  } else {
    // State is optimal. It can degrade to any lesser state.
    DCHECK_EQ(current_rank, max_rank) << "Invalid current_rank: "
                                      << current_rank;
    if (rng_.GetUniformReal() < dropout_rate * ts) {
      new_rank = rng_.GetUniformInt32() % max_rank;
    }
  }

  return new_rank;
}

void Gps::ApplySolutionStateChangeFault(const std::vector<double> &fault_params,
                                        Vec3 *pos__,
                                        int32_t *state_rank) const {
  *state_rank = static_cast<int32_t>(fault_params[0]);
  DCHECK_GE(*state_rank, 0);
  DCHECK_LT(*state_rank, ranked_solution_states_.size());

  Vec3 bias = {fault_params[1], fault_params[2], fault_params[3]};
  Vec3Add(pos__, &bias, pos__);
}

void Gps::Publish() const {
  sim_telem.gps[label_].time_of_week_ms = time_of_week();
  sim_telem.gps[label_].pos = pos();
  sim_telem.gps[label_].pos_sigma = pos_sigma();
  sim_telem.gps[label_].vel = vel();
  sim_telem.gps[label_].vel_sigma = vel_sigma();
  sim_telem.gps[label_].pos_type = pos_solution_state().pos_type;
  sim_telem.gps[label_].vel_type = vel_solution_state().vel_type;
}

namespace {

NovAtelSolutionStatus SimGpsSolutionStatusToNovAtelSolutionStatus(
    SimGpsSolutionStatus status, bool is_velocity) {
  switch (status) {
    case kSimGpsSolutionStatusNoError:
      return kNovAtelSolutionStatusSolComputed;
    case kSimGpsSolutionStatusError:
      return is_velocity ? kNovAtelSolutionStatusCovTrace
                         : kNovAtelSolutionStatusInsufficientObs;
    default:
      LOG(FATAL) << "Invalid SimGpsSolutionStatus: "
                 << static_cast<int32_t>(status);
      return kNovAtelSolutionStatusInsufficientObs;
  }
}

NovAtelSolutionType GpsSolutionTypeToNovAtelSolutionType(GpsSolutionType type,
                                                         bool is_velocity) {
  switch (type) {
    case kGpsSolutionTypeNone:
      return kNovAtelSolutionTypeNone;
    case kGpsSolutionTypeFixedPosition:
      return kNovAtelSolutionTypeFixedPos;
    case kGpsSolutionTypeFixedHeight:
      return kNovAtelSolutionTypeFixedHeight;
    case kGpsSolutionTypeStandAlone:
      return is_velocity ? kNovAtelSolutionTypeDopplerVelocity
                         : kNovAtelSolutionTypeSingle;
    case kGpsSolutionTypeDifferential:
      return kNovAtelSolutionTypePsrdiff;
    case kGpsSolutionTypeRtkFloat:
      return kNovAtelSolutionTypeL1Float;
    case kGpsSolutionTypeRtkIonoFreeFloat:
      return kNovAtelSolutionTypeIonofreeFloat;
    case kGpsSolutionTypeRtkNarrowFloat:
      return kNovAtelSolutionTypeNarrowFloat;
    case kGpsSolutionTypeRtkInt:
      return kNovAtelSolutionTypeL1Int;
    case kGpsSolutionTypeRtkWideInt:
      return kNovAtelSolutionTypeWideInt;
    case kGpsSolutionTypeRtkNarrowInt:
      return kNovAtelSolutionTypeNarrowInt;
    default:
    case kGpsSolutionTypeForceSigned:
    case kGpsSolutionTypeUnsupported:
    case kNumGpsSolutionTypes:
      LOG(FATAL) << "Invalid GpsSolutionType: " << static_cast<int32_t>(type);
      return kNovAtelSolutionTypeSingle;
  }
}

}  // namespace

NovAtelGps::NovAtelGps(const GroundFrame &ground_frame, const Wing &wing,
                       WingGpsReceiverLabel label, const GpsParams &gps_params,
                       const GpsSimParams &gps_sim_params,
                       FaultSchedule *fault_schedule)
    : Gps(ground_frame, wing, label, gps_params, gps_sim_params,
          fault_schedule) {}

void NovAtelGps::UpdateSensorOutputs(SimSensorMessage *sensor_message,
                                     TetherUpMessage * /*tether_up*/) const {
  sensor_message->control_input_messages_updated.wing_gps_novatel[label_] =
      true;
  NovAtelSolutionMessage *gps =
      &sensor_message->control_input_messages.wing_gps_novatel[label_];

  gps->best_xyz_latency = 50000;
  gps->best_xyz.pos_sol_status =
      static_cast<uint32_t>(SimGpsSolutionStatusToNovAtelSolutionStatus(
          pos_solution_state().pos_status, false));
  gps->best_xyz.pos_type =
      static_cast<uint32_t>(GpsSolutionTypeToNovAtelSolutionType(
          pos_solution_state().pos_type, false));
  gps->best_xyz.pos_x = static_cast<double>(pos().x);
  gps->best_xyz.pos_y = static_cast<double>(pos().y);
  gps->best_xyz.pos_z = static_cast<double>(pos().z);
  gps->best_xyz.pos_x_sigma = static_cast<float>(pos_sigma().x);
  gps->best_xyz.pos_y_sigma = static_cast<float>(pos_sigma().y);
  gps->best_xyz.pos_z_sigma = static_cast<float>(pos_sigma().z);
  gps->best_xyz.vel_sol_status =
      static_cast<uint32_t>(SimGpsSolutionStatusToNovAtelSolutionStatus(
          vel_solution_state().vel_status, true));
  gps->best_xyz.vel_type =
      static_cast<uint32_t>(GpsSolutionTypeToNovAtelSolutionType(
          vel_solution_state().vel_type, true));
  gps->best_xyz.vel_x = static_cast<double>(vel().x);
  gps->best_xyz.vel_y = static_cast<double>(vel().y);
  gps->best_xyz.vel_z = static_cast<double>(vel().z);
  gps->best_xyz.vel_x_sigma = static_cast<float>(vel_sigma().x);
  gps->best_xyz.vel_y_sigma = static_cast<float>(vel_sigma().y);
  gps->best_xyz.vel_z_sigma = static_cast<float>(vel_sigma().z);
  gps->best_xyz.sol_age = static_cast<float>(0);
  gps->best_xyz.diff_age = static_cast<float>(0);
  gps->best_xyz.num_sol = 5;
  // TODO: Implement GPS time in simulation.
  gps->best_xyz.timestamp.week = static_cast<int32_t>(0);
  gps->best_xyz.timestamp.tow = time_of_week();

  // Typical values.
  gps->avg_cn0 = 41.0;
  gps->max_cn0 = 45.0;
  gps->idle_time = 50;
}

namespace {

SeptentrioPvtError SimGpsSolutionStatusToSeptentrioPvtError(
    SimGpsSolutionStatus status) {
  switch (status) {
    case kSimGpsSolutionStatusNoError:
      return kSeptentrioPvtErrorNone;
    case kSimGpsSolutionStatusError:
      return kSeptentrioPvtErrorNotEnoughMeasurements;
    default:
      LOG(FATAL) << "Invalid SimGpsSolutionStatus: "
                 << static_cast<int32_t>(status);
      return kSeptentrioPvtErrorNotEnoughMeasurements;
  }
}

SeptentrioPvtModeBits GpsSolutionTypeToSeptentrioPvtMode(GpsSolutionType type) {
  if (type == kGpsSolutionTypeFixedHeight) {
    return static_cast<SeptentrioPvtModeBits>(kSeptentrioPvtModeBit2dMode |
                                              kSeptentrioPvtModeStandAlone);
  }

  SeptentrioPvtMode mode;
  switch (type) {
    case kGpsSolutionTypeNone:
      mode = kSeptentrioPvtModeNoSolution;
      break;
    case kGpsSolutionTypeStandAlone:
      mode = kSeptentrioPvtModeStandAlone;
      break;
    case kGpsSolutionTypeDifferential:
      mode = kSeptentrioPvtModeDifferential;
      break;
    case kGpsSolutionTypeFixedPosition:
      mode = kSeptentrioPvtModeFixedLocation;
      break;
    case kGpsSolutionTypeRtkInt:
    case kGpsSolutionTypeRtkNarrowInt:
    case kGpsSolutionTypeRtkWideInt:
      mode = kSeptentrioPvtModeRtkFixed;
      break;
    case kGpsSolutionTypeRtkFloat:
    case kGpsSolutionTypeRtkIonoFreeFloat:
    case kGpsSolutionTypeRtkNarrowFloat:
      mode = kSeptentrioPvtModeRtkFloat;
      break;
    default:
    case kGpsSolutionTypeForceSigned:
    case kGpsSolutionTypeFixedHeight:
    case kGpsSolutionTypeUnsupported:
    case kNumGpsSolutionTypes:
      LOG(FATAL) << "Invalid GpsSolutionType: " << static_cast<int32_t>(type);
  }
  return static_cast<SeptentrioPvtModeBits>(mode);
}

}  // namespace

SeptentrioGps::SeptentrioGps(const GroundFrame &ground_frame, const Wing &wing,
                             WingGpsReceiverLabel label,
                             const GpsParams &gps_params,
                             const GpsSimParams &gps_sim_params,
                             FaultSchedule *fault_schedule)
    : Gps(ground_frame, wing, label, gps_params, gps_sim_params,
          fault_schedule) {}

void SeptentrioGps::UpdateSensorOutputs(SimSensorMessage *sensor_message,
                                        TetherUpMessage * /*tether_up*/) const {
  sensor_message->control_input_messages_updated.wing_gps_septentrio[label_] =
      true;
  SeptentrioSolutionMessage *gps =
      &sensor_message->control_input_messages.wing_gps_septentrio[label_];
  *gps = SeptentrioSolutionMessage();

  gps->pvt_cartesian.timestamp.tow = time_of_week();
  gps->pos_cov_cartesian.timestamp.tow = time_of_week();
  gps->vel_cov_cartesian.timestamp.tow = time_of_week();

  gps->pvt_cartesian.x = static_cast<double>(pos().x);
  gps->pvt_cartesian.y = static_cast<double>(pos().y);
  gps->pvt_cartesian.z = static_cast<double>(pos().z);

  gps->pos_cov_cartesian.cov_xx =
      static_cast<float>(pos_sigma().x * pos_sigma().x);
  gps->pos_cov_cartesian.cov_yy =
      static_cast<float>(pos_sigma().y * pos_sigma().y);
  gps->pos_cov_cartesian.cov_zz =
      static_cast<float>(pos_sigma().z * pos_sigma().z);

  gps->pvt_cartesian.v_x = static_cast<float>(vel().x);
  gps->pvt_cartesian.v_y = static_cast<float>(vel().y);
  gps->pvt_cartesian.v_z = static_cast<float>(vel().z);

  gps->vel_cov_cartesian.cov_xx =
      static_cast<float>(vel_sigma().x * vel_sigma().x);
  gps->vel_cov_cartesian.cov_yy =
      static_cast<float>(vel_sigma().y * vel_sigma().y);
  gps->vel_cov_cartesian.cov_zz =
      static_cast<float>(vel_sigma().z * vel_sigma().z);

  SeptentrioPvtModeBits mode =
      GpsSolutionTypeToSeptentrioPvtMode(pos_solution_state().pos_type);
  SeptentrioPvtError error =
      SimGpsSolutionStatusToSeptentrioPvtError(pos_solution_state().pos_status);

  // Set errors and modes. If upper bits of the mode fields are needed, they
  // should be set after the assignments here.
  //
  // TODO: Figure out how to assign modes and errors appropriately to
  // each of these fields.
  gps->pvt_cartesian.mode = mode;
  gps->pvt_cartesian.error = error;
  gps->pos_cov_cartesian.mode = mode;
  gps->pos_cov_cartesian.error = error;
  gps->vel_cov_cartesian.mode = mode;
  gps->vel_cov_cartesian.error = error;
}
