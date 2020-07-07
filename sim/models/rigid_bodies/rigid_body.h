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

#ifndef SIM_MODELS_RIGID_BODIES_RIGID_BODY_H_
#define SIM_MODELS_RIGID_BODIES_RIGID_BODY_H_

#include <string>

#include "common/macros.h"
#include "sim/models/model.h"
#include "sim/physics/reference_frame.h"

class RigidBody : public Model {
 public:
  RigidBody(const std::string &name__, double ts__) : Model(name__, ts__) {}
  explicit RigidBody(const std::string &name__) : RigidBody(name__, 0.0) {}
  ~RigidBody() __attribute__((noinline)) {}

  virtual void CalcAcceleratingFrame(
      ReferenceFrame *accelerating_frame) const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(RigidBody);
};

#endif  // SIM_MODELS_RIGID_BODIES_RIGID_BODY_H_
