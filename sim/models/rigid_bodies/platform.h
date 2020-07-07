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

#ifndef SIM_MODELS_RIGID_BODIES_PLATFORM_H_
#define SIM_MODELS_RIGID_BODIES_PLATFORM_H_

#include "sim/models/actuators/ground_station_v2.h"
#include "sim/models/rigid_bodies/buoy.h"
#include "sim/models/rigid_bodies/rigid_body.h"

// This class implements a rigid body "platform".
// NOTE: The current purpose of this class is to implement the
// function CalcAcceleratingFrame which is needed by the Ground Station IMU.
// This class should be expanded to compute the dynamics of the platform.

class Platform : public RigidBody {
 public:
  Platform(const Buoy *buoy, const GroundStationV2Base *gs02);
  ~Platform() {}

  void CalcAcceleratingFrame(ReferenceFrame *accelerating_frame) const override;

 private:
  void UpdateDerivedStates();
  void DiscreteStepHelper(double t) override;
  void CalcDerivHelper(double t) override;
  void AddInternalConnections(ConnectionStore *connections) override;

  // Platform parameters.
  const Buoy *buoy_;
  const GroundStationV2Base *gs02_;

  DISALLOW_COPY_AND_ASSIGN(Platform);
};

#endif  // SIM_MODELS_RIGID_BODIES_PLATFORM_H_
