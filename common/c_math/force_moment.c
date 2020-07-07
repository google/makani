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

#include "common/c_math/force_moment.h"

#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"

const ForceMoment kForceMomentZero = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
const ForceMomentPos kForceMomentPosZero = {
    {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

// ForceMoment.

// Modifies a force-moment structure to be referenced to a different
// position.
const ForceMoment *ForceMomentRef(const ForceMoment *fm_in,
                                  const Vec3 *ref_disp, ForceMoment *fm_out) {
  Vec3 disp_moment;
  fm_out->force = fm_in->force;
  Vec3Add(&fm_in->moment, Vec3Cross(&fm_in->force, ref_disp, &disp_moment),
          &fm_out->moment);
  return fm_out;
}

const ForceMoment *ForceMomentAdd(const ForceMoment *fm1,
                                  const ForceMoment *fm2, ForceMoment *fm_out) {
  Vec3Add(&fm1->force, &fm2->force, &fm_out->force);
  Vec3Add(&fm1->moment, &fm2->moment, &fm_out->moment);
  return fm_out;
}

const ForceMoment *ForceMomentLinComb(double c0, const ForceMoment *fm0,
                                      double c1, const ForceMoment *fm1,
                                      ForceMoment *fm_out) {
  Vec3LinComb(c0, &fm0->force, c1, &fm1->force, &fm_out->force);
  Vec3LinComb(c0, &fm0->moment, c1, &fm1->moment, &fm_out->moment);
  return fm_out;
}

const ForceMoment *ForceMomentScale(const ForceMoment *fm_in, double scale,
                                    ForceMoment *fm_out) {
  Vec3Scale(&fm_in->force, scale, &fm_out->force);
  Vec3Scale(&fm_in->moment, scale, &fm_out->moment);
  return fm_out;
}

// ForceMomentPos.

// Modifies a force-moment structure to be referenced to a different
// position.  The position is set by the pos of fm_out.
//
// Warning: The pos field of fm_out must be set.
const ForceMomentPos *ForceMomentPosRef(const ForceMomentPos *fm_in,
                                        ForceMomentPos *fm_out) {
  Vec3 ref_disp, disp_moment;
  Vec3Sub(&fm_out->pos, &fm_in->pos, &ref_disp);
  fm_out->force = fm_in->force;
  Vec3Add(&fm_in->moment, Vec3Cross(&fm_in->force, &ref_disp, &disp_moment),
          &fm_out->moment);
  return fm_out;
}

// Sums two force-moment structures, each referenced to a point, to a
// third force-moment structure, at its reference point.
//
// Warning: The pos field of fm_out must be set.
const ForceMomentPos *ForceMomentPosAdd(const ForceMomentPos *fm1,
                                        const ForceMomentPos *fm2,
                                        ForceMomentPos *fm_out) {
  ForceMomentPos fm1_ref = {.pos = fm_out->pos};
  ForceMomentPos fm2_ref = {.pos = fm_out->pos};
  ForceMomentPosRef(fm1, &fm1_ref);
  ForceMomentPosRef(fm2, &fm2_ref);
  Vec3Add(&fm1_ref.force, &fm2_ref.force, &fm_out->force);
  Vec3Add(&fm1_ref.moment, &fm2_ref.moment, &fm_out->moment);
  return fm_out;
}

// Sums three force-moment structures, each referenced to a point, to
// a third force-moment structure, at its reference point.
//
// Warning: The pos field of fm_out must be set.
const ForceMomentPos *ForceMomentPosAdd3(const ForceMomentPos *fm1,
                                         const ForceMomentPos *fm2,
                                         const ForceMomentPos *fm3,
                                         ForceMomentPos *fm_out) {
  ForceMomentPos fm1_ref = {.pos = fm_out->pos};
  ForceMomentPos fm2_ref = {.pos = fm_out->pos};
  ForceMomentPos fm3_ref = {.pos = fm_out->pos};
  ForceMomentPosRef(fm1, &fm1_ref);
  ForceMomentPosRef(fm2, &fm2_ref);
  ForceMomentPosRef(fm3, &fm3_ref);
  Vec3Add3(&fm1_ref.force, &fm2_ref.force, &fm3_ref.force, &fm_out->force);
  Vec3Add3(&fm1_ref.moment, &fm2_ref.moment, &fm3_ref.moment, &fm_out->moment);
  return fm_out;
}

// Transforms a force-moment-position structure from coordinate system
// "a" to coordinate system "b".
const ForceMomentPos *ForceMomentPosPoseTransform(const Mat3 *dcm_a2b,
                                                  const Vec3 *R_a_b_origin,
                                                  const ForceMomentPos *fmx_a,
                                                  ForceMomentPos *fmx_b) {
  Mat3Vec3Mult(dcm_a2b, &fmx_a->force, &fmx_b->force);
  Mat3Vec3Mult(dcm_a2b, &fmx_a->moment, &fmx_b->moment);
  PoseTransform(dcm_a2b, R_a_b_origin, &fmx_a->pos, &fmx_b->pos);
  return fmx_b;
}

// Transforms a force-moment-position structure from coordinate system
// "b" to coordinate system "a".
const ForceMomentPos *ForceMomentPosInversePoseTransform(
    const Mat3 *dcm_a2b, const Vec3 *R_a_b_origin, const ForceMomentPos *fmx_b,
    ForceMomentPos *fmx_a) {
  Mat3TransVec3Mult(dcm_a2b, &fmx_b->force, &fmx_a->force);
  Mat3TransVec3Mult(dcm_a2b, &fmx_b->moment, &fmx_a->moment);
  InversePoseTransform(dcm_a2b, R_a_b_origin, &fmx_b->pos, &fmx_a->pos);
  return fmx_a;
}

// Converts a force-moment-position structure to a force-moment
// structure at position [0, 0, 0].
const ForceMoment *ForceMomentPosToForceMoment(const ForceMomentPos *fmx,
                                               ForceMoment *fm) {
  ForceMomentPos fmx_cg = {.pos = kVec3Zero};
  ForceMomentPosRef(fmx, &fmx_cg);
  fm->force = fmx_cg.force;
  fm->moment = fmx_cg.moment;
  return fm;
}
