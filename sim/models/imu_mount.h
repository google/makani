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

// The IMU mount models the motor vibrations which affect all IMUs
// simultaneously.
#ifndef SIM_MODELS_IMU_MOUNT_H_
#define SIM_MODELS_IMU_MOUNT_H_

#include <array>

#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "sim/math/util.h"
#include "sim/models/model.h"
#include "sim/sim_types.h"

class ImuMount : public Model {
 public:
  explicit ImuMount(const ImuMountSimParams &params)
      : Model("ImuMount"),
        params_(params),
        rng_(full_name()),
        vibration_filter_state_(new_discrete_state(), "vibration_filter_state",
                                0.0, {{{{0.0}}}}),
        vibration_quadrature_filter_state_(new_discrete_state(),
                                           "vibration_quadrature_filter_state",
                                           0.0, {{0.0}}),
        vibration_(new_discrete_state(), "vibration", 0.0, {{0.0}}),
        vibration_quadrature_(new_discrete_state(), "vibration_quadrature", 0.0,
                              {{0.0}}) {
    SetupDone();
  }
  ~ImuMount() {}

  const Vec3 GetAccelVibration() const {
    Vec3 tmp = {vibration_.val()[0], vibration_.val()[1],
                vibration_quadrature_.val()[2]};
    Vec3Mult(&tmp, &params_.acc_scale, &tmp);
    return tmp;
  }
  const Vec3 GetGyroVibration() const {
    Vec3 tmp = {vibration_.val()[0], vibration_.val()[2],
                vibration_quadrature_.val()[1]};
    Vec3Mult(&tmp, &params_.gyro_scale, &tmp);
    return tmp;
  }

 private:
  void DiscreteStepHelper(double t) override;

  // Parameters.
  const ImuMountSimParams &params_;

  NamedRandomNumberGenerator rng_;

  // Discrete states.

  // States for the band-pass filter used to capture rotor vibrations.
  DiscreteState<std::array<std::array<double, 2>, 3>> vibration_filter_state_;

  // State for the all-pass filter used to generate a quadrature
  // signal (roughly 90 [deg] out of phase).
  DiscreteState<std::array<double, 3>> vibration_quadrature_filter_state_;

  // Vibration signals passed to the individual IMUs.
  DiscreteState<std::array<double, 3>> vibration_;
  DiscreteState<std::array<double, 3>> vibration_quadrature_;

  DISALLOW_COPY_AND_ASSIGN(ImuMount);
};

#endif  // SIM_MODELS_IMU_MOUNT_H_
