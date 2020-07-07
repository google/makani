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

#include "sim/models/actuators/tether_release.h"

#include <glog/logging.h>
#include <stdint.h>

#include <vector>

#include "avionics/common/avionics_messages.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "sim/models/actuators/actuator.h"
#include "sim/sim_telemetry.h"

TetherReleaseBase::TetherReleaseBase()
    : Actuator("Tether Release"),
      release_triggered_(new_derived_value(), "release_triggered", false),
      released_(new_discrete_state(), "released", *g_sys.ts, false) {
  SetupDone();
}

void TetherReleaseBase::Publish() const {
  sim_telem.tether.released = released();
}

void TetherReleaseBase::DiscreteStepHelper(double t) {
  // Bad stuff happens if you go from released to unreleased, so don't
  // let this happen.
  released_.DiscreteUpdate(t, released() || release_triggered());
}

void TetherRelease::SetFromAvionicsPackets(
    const AvionicsPackets &avionics_packets) {
  set_release_triggered(avionics_packets.command_message.tether_release);
}

void HitlTetherRelease::SetFromAvionicsPackets(
    const AvionicsPackets &avionics_packets) {
  bool released__ = false;
  for (const LoadcellMessage &m : avionics_packets.loadcell_messages) {
    released__ |= m.tether_released;
  }

  set_release_triggered(released__);
}
