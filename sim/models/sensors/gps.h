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

#ifndef SIM_MODELS_SENSORS_GPS_H_
#define SIM_MODELS_SENSORS_GPS_H_

#include <vector>

#include "avionics/common/novatel_types.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/math/util.h"
#include "sim/models/rigid_bodies/wing.h"
#include "sim/models/sensors/sensor.h"
#include "sim/models/signals/delayed_signal.h"
#include "sim/physics/ground_frame.h"
#include "sim/sim_messages.h"

enum SimGpsSolutionStatus {
  kSimGpsSolutionStatusNoError,
  kSimGpsSolutionStatusError
};

// Describes the full
//     (pos status, pos type, vel status, vel type)
// state of the GPS solution.
struct SimGpsSolutionState {
  SimGpsSolutionState(SimGpsSolutionStatus pos_status__,
                      GpsSolutionType pos_type__,
                      SimGpsSolutionStatus vel_status__,
                      GpsSolutionType vel_type__)
      : pos_status(pos_status__),
        pos_type(pos_type__),
        vel_status(vel_status__),
        vel_type(vel_type__) {}

  SimGpsSolutionStatus pos_status;
  GpsSolutionType pos_type;
  SimGpsSolutionStatus vel_status;
  GpsSolutionType vel_type;
};

class Gps : public Sensor {
 public:
  void Publish() const override;

 protected:
  Gps(const GroundFrame &ground_frame, const Wing &wing,
      WingGpsReceiverLabel label, const GpsParams &gps_params,
      const GpsSimParams &gps_sim_params, FaultSchedule *faults);
  virtual ~Gps() {}

  SimGpsSolutionState LookupSolutionState(int32_t rank) const {
    return ranked_solution_states_[rank];
  }

  int32_t time_of_week() const {
    return delayed_pos_data_.output().time_of_week_ms;
  }

  const Vec3 &pos() const { return delayed_pos_data_.output().solution; }
  const Vec3 &pos_sigma() const { return delayed_pos_data_.output().sigma; }
  SimGpsSolutionState pos_solution_state() const {
    return ranked_solution_states_[delayed_pos_data_.output().state_rank];
  }
  const Vec3 &pos_rtcm_update_noise() const {
    return pos_rtcm_update_noise_.val();
  }

  const Vec3 &vel() const { return delayed_vel_data_.output().solution; }
  const Vec3 &vel_sigma() const { return delayed_vel_data_.output().sigma; }
  SimGpsSolutionState vel_solution_state() const {
    return ranked_solution_states_[delayed_vel_data_.output().state_rank];
  }
  const Vec3 &vel_rtcm_update_noise() const {
    return vel_rtcm_update_noise_.val();
  }

  const WingGpsReceiverLabel label_;

  NamedRandomNumberGenerator rng_;

 private:
  void DiscreteStepHelper(double t) override;

  double CalcDropoutRate() const;

  // Potentially applies a random state transition based on the current dropout
  // rate (and on the dropin rate, which is derived from it).
  //
  // Args:
  //   current_rank: The current rank of the solution state.
  //   dropout_rate: The current dropout rate.
  // Returns:
  //   The rank of the new solution state.
  int32_t ApplyRandomStateTransitions(int32_t current_rank,
                                      double dropout_rate);

  void ApplySolutionStateChangeFault(const std::vector<double> &fault_params,
                                     Vec3 *pos__, int32_t *state_rank) const;

  // GPS parameters.
  const GpsParams &gps_params_;
  const GpsSimParams &gps_sim_params_;

  // Possible GPS solution states, in order of increasing desirability.
  const std::vector<SimGpsSolutionState> ranked_solution_states_;

  // Connections to other models.
  const Wing &wing_;
  const GroundFrame &ground_frame_;

  // Discrete state.
  DiscreteState<int32_t> time_of_week_ms_;
  DiscreteState<int32_t> solution_state_rank_;
  DiscreteState<Vec3> pos_, pos_sigma_, vel_, vel_sigma_;
  DiscreteState<Vec3> pos_rtcm_update_noise_, vel_rtcm_update_noise_;

  // All data corresponding to a position or velocity solution. These are
  // bundled together in order to apply delays consistently.
  struct SolutionData {
    SolutionData()
        : solution(kVec3Zero),
          sigma(kVec3Zero),
          state_rank(0),
          time_of_week_ms(0) {}

    Vec3 solution;
    Vec3 sigma;
    int32_t state_rank;
    int32_t time_of_week_ms;
  };

  // Sub-models.
  // These simulate processing and communication delays.
  DelayedSteppingSignal<SolutionData> delayed_pos_data_, delayed_vel_data_;

  const FaultSchedule::FaultFunc fault_func_;

  DISALLOW_COPY_AND_ASSIGN(Gps);
};

class NovAtelGps : public Gps {
 public:
  NovAtelGps(const GroundFrame &ground_frame, const Wing &wing,
             WingGpsReceiverLabel label, const GpsParams &gps_params,
             const GpsSimParams &gps_sim_params, FaultSchedule *faults);
  virtual ~NovAtelGps() {}

  void UpdateSensorOutputs(SimSensorMessage *sensor_message,
                           TetherUpMessage * /*tether_up*/) const override;

 private:
  DISALLOW_COPY_AND_ASSIGN(NovAtelGps);
};

class SeptentrioGps : public Gps {
 public:
  SeptentrioGps(const GroundFrame &ground_frame, const Wing &wing,
                WingGpsReceiverLabel label, const GpsParams &gps_params,
                const GpsSimParams &gps_sim_params,
                FaultSchedule *fault_schedule);
  virtual ~SeptentrioGps() {}

  void UpdateSensorOutputs(SimSensorMessage *sensor_message,
                           TetherUpMessage * /*tether_up*/) const override;

 private:
  DISALLOW_COPY_AND_ASSIGN(SeptentrioGps);
};

#endif  // SIM_MODELS_SENSORS_GPS_H_
