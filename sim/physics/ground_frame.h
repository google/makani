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

#ifndef SIM_PHYSICS_GROUND_FRAME_H_
#define SIM_PHYSICS_GROUND_FRAME_H_

#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/system_types.h"
#include "sim/physics/contactor.h"
#include "sim/physics/reference_frame.h"
#include "sim/sim_types.h"

class GroundFrame : public ReferenceFrame {
 public:
  explicit GroundFrame(const ReferenceFrame &parent_frame,
                       const GroundFrameParams &ground_frame_params,
                       const GroundFrameSimParams &ground_frame_sim_params);
  ~GroundFrame() {}

  const Vec3 *CalcXNedLocal(const Vec3 &pos, Vec3 *X_ned) const;
  const Vec3 *CalcXEcefLocal(const Vec3 &pos, Vec3 *X_ecef) const;
  const Vec3 *CalcVEcefLocal(const Vec3 &vel, Vec3 *V_ecef) const;
  const Mat3 CalcDcmNedToG() const;
  const Mat3 CalcDcmGToNed() const;

  const Vec3 &pos_ecef() const { return ground_frame_sim_params_.pos_ecef; }
  double heading() const { return ground_frame_params_.heading; }
  const ContactSurface &surface() const { return surface_; }

 private:
  const GroundFrameParams &ground_frame_params_;
  const GroundFrameSimParams &ground_frame_sim_params_;

  // Helper function needed to construct the ReferenceFrame object.
  Mat3 HeadingToDcmHelper(double heading) {
    Mat3 dcm;
    AngleToDcm(heading, 0.0, 0.0, kRotationOrderZyx, &dcm);
    return dcm;
  }

  // The contact surface for the ground frame is the ground (on-shore) or the
  // water (off-shore).
  ContactSurface surface_;

  DISALLOW_COPY_AND_ASSIGN(GroundFrame);
};

#endif  // SIM_PHYSICS_GROUND_FRAME_H_
