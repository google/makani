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

#include "sim/physics/ground_frame.h"

#include <math.h>

#include "common/c_math/coord_trans.h"
#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/system_types.h"
#include "sim/physics/contactor.h"
#include "sim/physics/reference_frame.h"
#include "sim/sim_types.h"

GroundFrame::GroundFrame(const ReferenceFrame &parent_frame,
                         const GroundFrameParams &ground_frame_params,
                         const GroundFrameSimParams &ground_frame_sim_params)
    : ReferenceFrame(parent_frame, kVec3Zero,
                     HeadingToDcmHelper(ground_frame_params.heading)),
      ground_frame_params_(ground_frame_params),
      ground_frame_sim_params_(ground_frame_sim_params),
      surface_(*this, nullptr) {
  surface_.set_collision_func([ground_frame_params](const Vec3 &X_contactor,
                                                    Vec3 *X_collision) -> bool {
    *X_collision = X_contactor;
    X_collision->z = ground_frame_params.ground_z;
    return X_contactor.z > ground_frame_params.ground_z;
  });
}

const Mat3 GroundFrame::CalcDcmGToNed() const {
  Mat3 dcm_g2ned = {{{cos(heading()), -sin(heading()), 0.0},
                     {sin(heading()), cos(heading()), 0.0},
                     {0.0, 0.0, 1.0}}};
  return dcm_g2ned;
}

const Mat3 GroundFrame::CalcDcmNedToG() const {
  Mat3 dcm_g2ned = CalcDcmGToNed();
  Mat3 dcm_ned2g;
  Mat3Trans(&dcm_g2ned, &dcm_ned2g);
  return dcm_ned2g;
}

const Vec3 *GroundFrame::CalcXNedLocal(const Vec3 &pos, Vec3 *X_ned) const {
  Mat3 dcm_g2ned = CalcDcmGToNed();
  Mat3Vec3Mult(&dcm_g2ned, &pos, X_ned);
  return X_ned;
}

const Vec3 *GroundFrame::CalcXEcefLocal(const Vec3 &pos, Vec3 *X_ecef) const {
  Vec3 X_ned;
  NedToEcef(CalcXNedLocal(pos, &X_ned), &ground_frame_sim_params_.pos_ecef,
            X_ecef);
  return X_ecef;
}

const Vec3 *GroundFrame::CalcVEcefLocal(const Vec3 &vel, Vec3 *V_ecef) const {
  Vec3 V_ned;
  Mat3 dcm_ned2ecef;
  CalcDcmNedToEcef(&ground_frame_sim_params_.pos_ecef, &dcm_ned2ecef);
  Mat3Vec3Mult(&dcm_ned2ecef, CalcXNedLocal(vel, &V_ned), V_ecef);
  return V_ecef;
}
