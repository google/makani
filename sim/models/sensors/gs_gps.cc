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

#include "sim/models/sensors/gs_gps.h"

#include <cmath>

#include "avionics/common/tether_convert.h"
#include "avionics/common/tether_message_types.h"
#include "common/c_math/geometry.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/ground_frame.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "sim/physics/ground_frame.h"
#include "sim/physics/reference_frame.h"
#include "sim/sim_messages.h"
#include "sim/sim_params.h"

GsGps::GsGps(const GroundFrame &ground_frame, const GsGpsParams &gs_gps_params,
             const bool has_compass, const GsGpsSimParams &gs_gps_sim_params)
    : Sensor("GS GPS", gs_gps_sim_params.ts),
      rng_(full_name()),
      ground_frame_(ground_frame),
      gs_gps_params_(gs_gps_params),
      has_compass_(has_compass),
      gs_gps_sim_params_(gs_gps_sim_params),
      parent_frame_(new_input_value(), "parent_frame"),
      time_of_week_ms_(new_discrete_state(), "time_of_week_ms", 0),
      pos_(new_discrete_state(), "position", kVec3Zero),
      vel_(new_discrete_state(), "velocity", kVec3Zero),
      heading_(new_discrete_state(), "heading", 0.0),
      pitch_(new_discrete_state(), "pitch", 0.0),
      length_(new_discrete_state(), "length", 0.0) {
  SetupDone();
}

void GsGps::CalcGpsPositionVelocityAngles(Vec3 *pos_ecef, Vec3 *vel_ecef,
                                          double *compass_heading,
                                          double *compass_pitch,
                                          double *compass_length) const {
  Vec3 gps_antenna_pos_g;
  ReferenceFrame gs_gps_frame(parent_frame_.val(),
                              gs_gps_params_.primary_antenna_p.pos);
  gs_gps_frame.TransformOriginTo(ground_frame_, ReferenceFrame::kPosition,
                                 &gps_antenna_pos_g);
  ground_frame_.CalcXEcefLocal(gps_antenna_pos_g, pos_ecef);

  Vec3 gps_antenna_vel_g;
  gs_gps_frame.TransformOriginTo(ground_frame_, ReferenceFrame::kVelocity,
                                 &gps_antenna_vel_g);
  ground_frame_.CalcVEcefLocal(gps_antenna_vel_g, vel_ecef);

  Vec3 d_gps_antennae_p, d_gps_antennae_g;
  Vec3Sub(&gs_gps_params_.secondary_antenna_p.pos,
          &gs_gps_params_.primary_antenna_p.pos, &d_gps_antennae_p);
  parent_frame_.val().RotateTo(ground_frame_, d_gps_antennae_p,
                               &d_gps_antennae_g);
  CartToSph(&d_gps_antennae_g, compass_heading, compass_pitch, compass_length);
  *compass_heading = Wrap(*compass_heading, 0.0, 2.0 * PI);
  // Negate compass_pitch because positive pitch corresponds to the
  // -z-direction.
  *compass_pitch = Wrap(-(*compass_pitch), -PI, PI);

  // TODO: Update the function HandleTetherUpToGsCompass to make
  // use of the relative position of the GS GPS antennae instead of the
  // heading_cal parameters. Then, deprecate the heading_cal param. Until
  // then, check that the heading_cal param and the positions of the GPS
  // antennae in platform frame are consistent.
  if (GetSystemParams()->gs_model == kGroundStationModelGSv2) {
    double heading_cal, pitch_cal, length_cal;
    CartToSph(&d_gps_antennae_p, &heading_cal, &pitch_cal, &length_cal);
    CHECK_LE(fabs(heading_cal + gs_gps_params_.heading_cal.bias), 1e-2f);
  }
}

void GsGps::DiscreteStepHelper(double t) {
  assert(ts_ > 0);
  int32_t ts_ms = static_cast<int32_t>(1000.0 * ts_);
  int32_t time_of_week_ms = ts_ms * static_cast<int32_t>(t * (1000 / ts_ms));
  time_of_week_ms_.DiscreteUpdate(t, time_of_week_ms);

  // Compute "true" values.
  Vec3 pos_ecef, vel_ecef;
  double heading, pitch, length;
  CalcGpsPositionVelocityAngles(&pos_ecef, &vel_ecef, &heading, &pitch,
                                &length);

  // Add noise.
  if (GetSimParams()->sim_opt & kSimOptImperfectSensors) {
    Vec3 pos_noise = {rng_.GetNormal(), rng_.GetNormal(), rng_.GetNormal()};
    Vec3 vel_noise = {rng_.GetNormal(), rng_.GetNormal(), rng_.GetNormal()};

    Vec3Mult(&pos_noise, &gs_gps_sim_params_.pos_sigma, &pos_noise);
    Vec3Mult(&vel_noise, &gs_gps_sim_params_.vel_sigma, &vel_noise);

    Vec3Add(&pos_ecef, &pos_noise, &pos_ecef);
    Vec3Add(&vel_ecef, &vel_noise, &vel_ecef);

    double heading_noise = rng_.GetNormal() * gs_gps_sim_params_.heading_sigma;
    double pitch_noise = rng_.GetNormal() * gs_gps_sim_params_.pitch_sigma;
    double length_noise = rng_.GetNormal() * gs_gps_sim_params_.length_sigma;

    heading += heading_noise;
    pitch += pitch_noise;
    length += length_noise;
  }

  // Re-wrap the heading and pitch after adding noise.
  heading = Wrap(heading, 0.0, 2.0 * PI);
  pitch = Wrap(pitch, -PI, PI);

  pos_.DiscreteUpdate(t, pos_ecef);
  vel_.DiscreteUpdate(t, vel_ecef);
  heading_.DiscreteUpdate(t, heading);
  pitch_.DiscreteUpdate(t, pitch);
  length_.DiscreteUpdate(t, length);
}

void GsGps::UpdateSensorOutputs(SimSensorMessage *sensor_message,
                                TetherUpMessage *tether_up) const {
  sensor_message->ground_input_messages_updated.ground_gps[0] = true;
  NovAtelSolutionMessage *sol =
      &sensor_message->ground_input_messages.ground_gps[0];

  sol->best_xyz_latency = 0;
  sol->best_xyz.num_sol = 10;
  sol->best_xyz.sol_age = 0;

  // TODO: Support both FixedPos and Single modes.
  sol->best_xyz.pos_sol_status = kNovAtelSolutionStatusSolComputed;
  sol->best_xyz.pos_type = kNovAtelSolutionTypeSingle;

  const Vec3 &X_ecef_gps = pos_.val();
  sol->best_xyz.pos_x = X_ecef_gps.x;
  sol->best_xyz.pos_y = X_ecef_gps.y;
  sol->best_xyz.pos_z = X_ecef_gps.z;

  const Vec3 &V_ecef_gps = vel_.val();
  sol->best_xyz.vel_x = V_ecef_gps.x;
  sol->best_xyz.vel_y = V_ecef_gps.y;
  sol->best_xyz.vel_z = V_ecef_gps.z;

  const Vec3 &pos_sigma = gs_gps_sim_params_.pos_sigma;
  sol->best_xyz.pos_x_sigma = static_cast<float>(pos_sigma.x);
  sol->best_xyz.pos_y_sigma = static_cast<float>(pos_sigma.y);
  sol->best_xyz.pos_z_sigma = static_cast<float>(pos_sigma.z);

  const Vec3 &vel_sigma = gs_gps_sim_params_.vel_sigma;
  sol->best_xyz.vel_x_sigma = static_cast<float>(vel_sigma.x);
  sol->best_xyz.vel_y_sigma = static_cast<float>(vel_sigma.y);
  sol->best_xyz.vel_z_sigma = static_cast<float>(vel_sigma.z);

  sol->best_xyz.timestamp.tow = time_of_week();

  sol->avg_cn0 = 44.0;
  sol->max_cn0 = 50.0;
  sol->idle_time = 50;

  if (has_compass_) {
    // Update NovAtelCompassMessage.
    sensor_message->ground_input_messages_updated.ground_compass[0] = true;
    NovAtelCompassMessage *compass =
        &sensor_message->ground_input_messages.ground_compass[0];

    // Copy some values from the BestXyz log.
    compass->heading_latency = sol->best_xyz_latency;
    compass->heading.pos_sol_status = sol->best_xyz.pos_sol_status;
    compass->heading.pos_type = sol->best_xyz.pos_type;
    compass->heading.num_tracked = sol->best_xyz.num_tracked;
    compass->heading.num_sol = sol->best_xyz.num_sol;

    compass->heading.heading = static_cast<float>(heading_.val());
    compass->heading.pitch = static_cast<float>(pitch_.val());

    // NovAtel OEM6 receiver with dual antennas reports -1 for this field as it
    // is in "ALIGN Heading" mode.
    compass->heading.length = -1.0f;

    compass->heading.heading_sigma =
        static_cast<float>(gs_gps_sim_params_.heading_sigma);
    compass->heading.pitch_sigma =
        static_cast<float>(gs_gps_sim_params_.pitch_sigma);

    // Populate the heading rate.
    compass->heading_rate_latency = 1000000;
    compass->heading_rate.pos_sol_status = sol->best_xyz.pos_sol_status;
    compass->heading_rate.pos_type = sol->best_xyz.pos_type;

    // TODO(b/135551868): Populate the HeadingRate log properly. In the
    // meantime, apply bogus values with large sigmas.
    compass->heading_rate.length_rate = 5.5f;
    compass->heading_rate.heading_rate = 0.22f;
    compass->heading_rate.pitch_rate = 9.0f;

    compass->heading_rate.length_rate_sigma = 1e6f;
    compass->heading_rate.heading_rate_sigma = 1e6f;
    compass->heading_rate.pitch_rate_sigma = 1e6f;

    // Produce the TetherUp version of NovAtelCompassMessage.
    uint16_t sequence = static_cast<uint16_t>(
        (tether_up->gps_compass.sequence + 1) % TETHER_SEQUENCE_ROLLOVER);
    NovAtelCompassMessageToTetherGsGpsCompass(compass, 0,
                                              &tether_up->gps_compass);
    tether_up->gps_compass.sequence = sequence;
    tether_up->gps_compass.no_update_count = 0;
  }

  // Produce the TetherUp version of NovAtelSolutionMessage.
  // TODO: How do AIO sequence numbers work in the simulator?
  uint16_t sequence = static_cast<uint16_t>(
      (tether_up->gps_position.sequence + 1) % TETHER_SEQUENCE_ROLLOVER);
  NovAtelSolutionMessageToTetherGpsPosition(sol, 0, &tether_up->gps_position);
  tether_up->gps_position.sequence = sequence;
  tether_up->gps_position.no_update_count = 0;

  // Populate TetherUpGpsStatus from NovAtelSolutionMessage.
  sequence = static_cast<uint16_t>((tether_up->gps_status.sequence + 1) %
                                   TETHER_SEQUENCE_ROLLOVER);
  NovAtelSolutionMessageToTetherGpsStatus(sol, 0, &tether_up->gps_status);
  tether_up->gps_status.sequence = sequence;
  tether_up->gps_status.no_update_count = 0;
}
