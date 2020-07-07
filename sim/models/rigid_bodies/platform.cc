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

#include "sim/models/rigid_bodies/platform.h"

Platform::Platform(const Buoy *buoy, const GroundStationV2Base *gs02)
    : RigidBody("Platform"), buoy_(buoy), gs02_(gs02) {
  SetupDone();
}

void Platform::DiscreteStepHelper(double /*t*/) {
  // Place-holder.
}

void Platform::CalcDerivHelper(double /*t*/) {
  // Place-holder.
}

void Platform::AddInternalConnections(ConnectionStore * /*connections*/) {
  // Place-holder
}

void Platform::CalcAcceleratingFrame(ReferenceFrame *accelerating_frame) const {
  ReferenceFrame buoy_accelerating_frame;
  buoy_->CalcAcceleratingFrame(&buoy_accelerating_frame);

  // The only degree-of-freedom between the buoy and the platform is the
  // azimuth slew.
  Vec3 platform_omega;
  Vec3Scale(&kVec3Z, gs02_->platform_azi_vel(), &platform_omega);
  Vec3 platform_domega;
  Vec3Scale(&kVec3Z, gs02_->dplatform_azi_vel(), &platform_domega);

  *accelerating_frame =
      ReferenceFrame(buoy_accelerating_frame, kVec3Zero, kVec3Zero, kVec3Zero,
                     gs02_->dcm_v2p(), platform_omega, platform_domega);
}
