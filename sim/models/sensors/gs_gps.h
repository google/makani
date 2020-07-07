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

#ifndef SIM_MODELS_SENSORS_GS_GPS_H_
#define SIM_MODELS_SENSORS_GS_GPS_H_

#include "common/macros.h"
#include "control/system_types.h"
#include "sim/math/util.h"
#include "sim/models/sensors/sensor.h"
#include "sim/physics/ground_frame.h"
#include "sim/physics/reference_frame.h"
#include "sim/sim_messages.h"
#include "sim/state.h"

class GsGps : public Sensor {
  friend class GsGpsTest;

 public:
  GsGps(const GroundFrame &ground_frame, const GsGpsParams &gs_gps_params,
        bool has_compass, const GsGpsSimParams &gs_gps_sim_params);
  ~GsGps() {}

  void UpdateSensorOutputs(SimSensorMessage *sensor_message,
                           TetherUpMessage *tether_up) const;
  void CalcGpsPositionVelocityAngles(Vec3 *pos_ecef, Vec3 *vel_ecef,
                                     double *compass_heading,
                                     double *compass_pitch,
                                     double *compass_length) const;
  void set_parent_frame(const ReferenceFrame frame) {
    parent_frame_.set_val(frame);
  }
  void clear_parent_frame() { parent_frame_.Clear(); }

  int32_t time_of_week() const { return time_of_week_ms_.val(); }

 protected:
  NamedRandomNumberGenerator rng_;

 private:
  void DiscreteStepHelper(double t);

  // Connections to other models.
  const GroundFrame &ground_frame_;

  // Parameters.
  const GsGpsParams &gs_gps_params_;
  const bool has_compass_;
  const GsGpsSimParams &gs_gps_sim_params_;

  // Frame the GS GPS is rigidly attached to. This is typically the platform
  // frame.
  State<ReferenceFrame> parent_frame_;

  // Discrete state.
  DiscreteState<int32_t> time_of_week_ms_;
  DiscreteState<Vec3> pos_;        // Position measurement [m], ECEF.
  DiscreteState<Vec3> vel_;        // Velocity measurement [m], ECEF.
  DiscreteState<double> heading_;  // Heading measurement [rad], NED.
  DiscreteState<double> pitch_;    // Pitch measurement [rad], NED.
  DiscreteState<double> length_;   // Length measurement [m].

  DISALLOW_COPY_AND_ASSIGN(GsGps);
};

#endif  // SIM_MODELS_SENSORS_GS_GPS_H_
