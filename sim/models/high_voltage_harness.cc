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

#include "sim/models/high_voltage_harness.h"

#include <glog/logging.h>
#include <math.h>
#include <stdint.h>

#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "sim/models/model.h"

HighVoltageHarness::HighVoltageHarness(
    const HighVoltageHarnessSimParams &params)
    : Model("HighVoltageHarness"), params_(params), motor_currents_() {
  motor_currents_.reserve(kNumMotors);
  for (int32_t i = 0; i < kNumMotors; ++i) {
    motor_currents_.emplace_back(new_derived_value(),
                                 "motor_current[" + std::to_string(i) + "]");
  }
  SetupDone();
}

// The high voltage harness is laid out as follows:
//
//            i_in ->          i_sbo->
//    1700 ------------+------------------------.
//             v i_pto |                        |
//                   [PTO]                    [SBO]
//                     |        i_1  ->         |
//                     +------------------------+
//             v i_pbo |                        | i_sto v
//                   [PBO]                    [STO]
//                     |        i_2  ->         |
//                     +------------------------+
//             v i_pti |                        | i_sbi v
//                   [PTI]                    [SBI]
//                     |        i_3  ->         |
//                     +------------------------+
//             v i_pbi |                        |
//                   [PBI]                    [STI]
//                     |                        |
//    -1700------------+------------------------.
//         <- i_out          <- i_sti
//
//
// The conductors crossing from port to starboard lie in a wiring
// channel with the following geometry:
//
//                            ^ x_b
//                            |
//                            .----> y_b
//
//
//    -------------------(-i_sti)/2------------------>
//    -------------------(-i_sti)/2------------------>
//    -----------------------i_3--------------------->
//    -----------------------i_2---------------------> pos.x
//    -----------------------i_1--------------------->
//    ---------------------i_sbo/2------------------->
//    ---------------------i_sbo/2------------------->
//
// The sum of these currents is nearly zero, as charge does not
// accumulate on the starboard pylon.  Experimentally, the magnetic
// field induced by i_1, i_2, i_3 is small compared to i_sbo, i_sti.
// Here we use the Biot-Savart law to compute the magnetic field at
// points near the body x-z plane and close to the wiring tray by
// assuming the conductors are infinitely long.
//
// Further analysis is in experimental/control/Magnetometer.
void HighVoltageHarness::CalcInducedMagneticField(const Vec3 &pos_b,
                                                  Vec3 *field_b) {
  Vec3 diff;
  Vec3Sub(&pos_b, &params_.pos, &diff);
  double r = Vec3Norm(&diff);

  // The model is only valid near the x-z plane of the kite, and
  // near the wiring channel.
  DCHECK_GT(0.5, fabs(pos_b.y));
  DCHECK_GT(1.5, hypot(diff.x, diff.z));

  // Permeability of free-space (close to that of air).
  constexpr double mu_0 = 4.0 * M_PI * 1e-7;
  constexpr double gauss_per_tesla = 1e4;

  // Induced field in body coordinates.
  field_b->x = 2.0 * diff.x * diff.z / (r * r);
  field_b->y = 0.0;
  field_b->z = 1.0 - 2.0 * diff.x * diff.x / (r * r);

  Vec3Scale(field_b, gauss_per_tesla * mu_0 / (2.0 * M_PI * r * r) *
                         (params_.conductor_spacing / 2.0) *
                         (motor_current(kMotorSbo) + motor_current(kMotorSti)),
            field_b);
}
