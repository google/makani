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

#include "sim/models/sensors/loadcell.h"

#include <float.h>
#include <glog/logging.h>
#include <math.h>
#include <stdint.h>

#include <limits>
#include <string>
#include <vector>

#include "avionics/common/safety_codes.h"
#include "avionics/network/aio_labels.h"
#include "common/c_math/mat2.h"
#include "common/c_math/util.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "control/avionics/avionics_interface_types.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/math/linalg.h"
#include "sim/models/signals/measurement.h"
#include "sim/sim_messages.h"
#include "sim/sim_telemetry.h"
#include "system/labels.h"

namespace {

// Determines the loadcell readings at a single pin loadcell from its bridle
// force.
Vec2 BridleForceToLoadcellMeasurements(const LoadcellParams &loadcell_params,
                                       const Vec3 &bridle_force_b) {
  // Determine bridle force in the loadcell's frame.
  Vec3 bridle_force_local;
  Mat3TransVec3Mult(&loadcell_params.dcm_loadcell2b, &bridle_force_b,
                    &bridle_force_local);

  const Vec2 bridle_force_local_xy = {bridle_force_local.x,
                                      bridle_force_local.y};
  Vec2 measurements;
  Mat2Vec2LeftDivide(&loadcell_params.channels_to_force_local_xy,
                     &bridle_force_local_xy, &measurements);
  return measurements;
}

}  // namespace

namespace sim {

void TetherForceToLoadcells(const WingParams &wing_params,
                            const LoadcellParams loadcell_params[],
                            const Vec3 &tether_force_b,
                            double loadcell_forces[]) {
  DCHECK_LT(fabs(wing_params.bridle_pos[kBridlePort].x -
                 wing_params.bridle_pos[kBridleStar].x),
            DBL_TOL);
  DCHECK_LT(fabs(wing_params.bridle_pos[kBridlePort].z -
                 wing_params.bridle_pos[kBridleStar].z),
            DBL_TOL);

  if (Vec3Norm(&tether_force_b) < DBL_TOL) {
    for (int32_t i = 0; i < kNumLoadcellSensors; ++i) {
      loadcell_forces[i] = 0.0;
    }
    return;
  }

  // Calculate knot_pos_b. For this calculation to be true, it is assumed each
  // bridle is rigid and under tension. The knot point and two bridle positions
  // form a plane.  The knot point is chosen so that the xz-component of the
  // tether_force is parallel to this plane.
  Vec3 knot_pos_b = {wing_params.bridle_pos[kBridlePort].x,
                     wing_params.bridle_y_offset,
                     wing_params.bridle_pos[kBridlePort].z};
  Vec3 tether_dir_xz = {tether_force_b.x, 0.0, tether_force_b.z};
  Vec3Normalize(&tether_dir_xz, &tether_dir_xz);
  Vec3Scale(&tether_dir_xz, wing_params.bridle_rad, &tether_dir_xz);
  Vec3Add(&knot_pos_b, &tether_dir_xz, &knot_pos_b);

  // Next we calcuate the direction of the force from each bridle position.
  Vec3 bridle_dir[kNumBridles];
  for (int32_t i = 0; i < kNumBridles; ++i) {
    Vec3Sub(&knot_pos_b, &wing_params.bridle_pos[i], &bridle_dir[i]);
    Vec3Normalize(&bridle_dir[i], &bridle_dir[i]);
  }

  // Solve for the magnitude of the tension on each bridle, assuming the force
  // on each bridle acts in the direction of the bridle knot.
  double tension_port, tension_star;
  {
    // The system of force balance equations is overdetermined. The x-equation
    // of the balance degenerates around a pitch angle of zero, so we omit
    // it. By contrast, the y-equation should not degenerate at all, and the
    // z-equation degnerates at pitch angle of +-PI/2, outside of operating
    // range.
    Mat2 A = {{{bridle_dir[kBridlePort].y, bridle_dir[kBridleStar].y},
               {bridle_dir[kBridlePort].z, bridle_dir[kBridleStar].z}}};
    Vec2 b = {tether_force_b.y, tether_force_b.z};
    Vec2 v;
    Mat2Vec2LeftDivide(&A, &b, &v);
    tension_port = v.x;
    tension_star = v.y;
    DCHECK_LT(fabs(tension_port * bridle_dir[kBridlePort].x +
                   tension_star * bridle_dir[kBridleStar].x - tether_force_b.x),
              1e-6);
  }

  // Compute bridle force vectors in the body frame.
  Vec3 port_bridle_force_b, star_bridle_force_b;
  if (tension_port < 0.0) {
    // Assume slack in port bridle; starboard bridle aligns with tether force.
    port_bridle_force_b = kVec3Zero;
    star_bridle_force_b = tether_force_b;
  } else if (tension_star < 0.0) {
    // Assume slack in starboard bridle; port bridle aligns with tether force.
    star_bridle_force_b = kVec3Zero;
    port_bridle_force_b = tether_force_b;
  } else {
    // Both bridles are in fact under tension; each bridle force is in the
    // the direction of the knot.
    Vec3Scale(&bridle_dir[kBridlePort], tension_port, &port_bridle_force_b);
    Vec3Scale(&bridle_dir[kBridleStar], tension_star, &star_bridle_force_b);
  }

  // Determine sensor measurements.
  Vec2 port_meas = BridleForceToLoadcellMeasurements(
      loadcell_params[kBridlePort], port_bridle_force_b);
  loadcell_forces[kLoadcellSensorPort0] = port_meas.x;
  loadcell_forces[kLoadcellSensorPort1] = port_meas.y;

  Vec2 star_meas = BridleForceToLoadcellMeasurements(
      loadcell_params[kBridleStar], star_bridle_force_b);
  loadcell_forces[kLoadcellSensorStarboard0] = star_meas.x;
  loadcell_forces[kLoadcellSensorStarboard1] = star_meas.y;
}

}  // namespace sim

Loadcell::Loadcell(const LoadcellParams *loadcell_params,
                   const LoadcellSimParams &loadcell_sim_params,
                   const WingParams &wing_params, FaultSchedule *faults)
    : Sensor("Loadcell"),
      loadcell_params_(loadcell_params),
      loadcell_sim_params_(loadcell_sim_params),
      wing_params_(wing_params),
      Fb_tether_(new_derived_value(), "Fb_tether"),
      tether_released_(new_derived_value(), "tether_released", false),
      actual_tensions_(),
      tensions_() {
  std::vector<Model *> sub_models__;

  actual_tensions_.reserve(kNumLoadcellSensors);
  tensions_.reserve(kNumLoadcellSensors);
  for (int32_t i = 0; i < kNumLoadcellSensors; ++i) {
    actual_tensions_.emplace_back(new_discrete_state(),
                                  "actual_tensions[" + std::to_string(i) + "]",
                                  0.0);
    std::vector<SensorModelParams> params({loadcell_sim_params_.sensors[i]});
    tensions_.emplace_back(full_name(), "tensions[" + std::to_string(i) + "]",
                           loadcell_sim_params.ts, params, actual_tensions_[i],
                           faults);
    sub_models__.push_back(&tensions_[i]);
  }
  set_sub_models(sub_models__);

  SetupDone();
}

void Loadcell::DiscreteStepHelper(double t) {
  double loadcell_forces[kNumLoadcellSensors];
  sim::TetherForceToLoadcells(wing_params_, loadcell_params_, Fb_tether(),
                              loadcell_forces);

  for (int32_t i = 0; i < kNumLoadcellSensors; ++i) {
    double tension__ = loadcell_forces[i];
    actual_tensions_[i].DiscreteUpdate(t, tension__);
  }
}

void Loadcell::UpdateSensorOutputs(SimSensorMessage *sensor_message,
                                   TetherUpMessage * /*tether_up*/) const {
  for (int32_t i = 0; i < kNumLoadcellNodes; ++i) {
    sensor_message->control_input_messages_updated.loadcell_messages[i] = true;
    LoadcellMessage &message =
        sensor_message->control_input_messages.loadcell_messages[i];

    // We don't simulate the interlock switch.
    message.tether_release_fully_armed = true;

    bool released = tether_released_.val();
    message.tether_released = released;
    message.tether_released_safety_code =
        released ? TETHER_RELEASE_SAFETY_CODE : 0U;
  }

  LoadcellMessage *messages =
      sensor_message->control_input_messages.loadcell_messages;
  for (int32_t bridle = 0; bridle < kNumBridles; ++bridle) {
    for (int32_t channel = 0; channel < NUM_LOADCELL_CHANNELS; ++channel) {
      LoadcellSensorLabel loadcell =
          BridleAndChannelToLoadcellSensorLabel((BridleLabel)bridle, channel);
      const LoadcellChannelParams &params =
          loadcell_params_[bridle].channels[channel];
      *GetMutableStrain(messages, &params.strain_location) =
          static_cast<float>(InvertCal(tensions(loadcell), &params.cal));
    }
  }
}

void Loadcell::Publish() const {
  for (int32_t i = 0; i < kNumLoadcellSensors; ++i) {
    sim_telem.loadcell.tensions[i] = tensions(i);
  }
}
