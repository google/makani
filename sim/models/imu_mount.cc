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

#include "sim/models/imu_mount.h"

#include <glog/logging.h>

#include <array>
#include <cmath>

#include "common/c_math/filter.h"
#include "control/system_params.h"
#include "sim/math/util.h"

namespace {

double AllPass(double input, double frequency, double *state) {
  double discrete_frequency = M_PI * frequency * (*g_sys.ts);
  DCHECK_LT(0.0, discrete_frequency);
  // The filter below approximates:
  //
  //     (2 * pi * frequency - s) / (2 * pi * frequency + s),
  //
  // which is an all-pass filter applying a 90 degree phase shift at
  // the given frequency.
  double coeff = (1.0 - discrete_frequency) / (1.0 + discrete_frequency);
  double prev_state = *state;
  *state = coeff * prev_state + (1.0 - coeff * coeff) * input;
  return prev_state - coeff * input;
}

}  // namespace

void ImuMount::DiscreteStepHelper(double t) {
  // We first band-pass filter white noise to simulate the IMU mount's
  // resonances.
  std::array<std::array<double, 2>, 3> vibration_filter_state =
      vibration_filter_state_.val();
  std::array<double, 3> vibration = {
      {rng_.GetNormal(), rng_.GetNormal(), rng_.GetNormal()}};
  // Next, an all-pass filter is applied to generate a signal
  // approximately 90 degrees out of phase (the "quadrature" component).
  std::array<double, 3> vibration_quadrature_filter_state =
      vibration_quadrature_filter_state_.val();
  std::array<double, 3> vibration_quadrature = vibration_quadrature_.val();

  for (uint32_t i = 0U; i < vibration.size(); ++i) {
    vibration[i] = BandPass2(vibration[i], params_.frequencies[i],
                             params_.damping_ratios[i], (*g_sys.ts),
                             vibration_filter_state[i].data());

    vibration_quadrature[i] = AllPass(vibration[i], params_.frequencies[i],
                                      &vibration_quadrature_filter_state[i]);
  }

  vibration_filter_state_.DiscreteUpdate(t, vibration_filter_state);
  vibration_quadrature_filter_state_.DiscreteUpdate(
      t, vibration_quadrature_filter_state);
  vibration_.DiscreteUpdate(t, vibration);
  vibration_quadrature_.DiscreteUpdate(t, vibration_quadrature);
}
