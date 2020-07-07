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

#ifndef SIM_MODELS_ACTUATORS_TETHER_RELEASE_H_
#define SIM_MODELS_ACTUATORS_TETHER_RELEASE_H_

#include <vector>

#include "sim/models/actuators/actuator.h"
#include "sim/sim_messages.h"
#include "sim/state.h"

class TetherReleaseBase : public Actuator {
 public:
  TetherReleaseBase();
  virtual ~TetherReleaseBase() {}

  void Publish() const override;

  bool released() const { return released_.val(); }

 protected:
  void set_release_triggered(bool val) { release_triggered_.set_val(val); }

 private:
  void DiscreteStepHelper(double t) override;

  bool release_triggered() const { return release_triggered_.val(); }

  // Actuator commands.
  State<bool> release_triggered_;

  // Discrete state.
  DiscreteState<bool> released_;
};

class TetherRelease : public TetherReleaseBase {
 public:
  TetherRelease() : TetherReleaseBase() {}
  virtual ~TetherRelease() {}

  void SetFromAvionicsPackets(const AvionicsPackets &avionics_packets) override;
};

class HitlTetherRelease : public TetherReleaseBase {
 public:
  HitlTetherRelease() : TetherReleaseBase() {}
  virtual ~HitlTetherRelease() {}

  void SetFromAvionicsPackets(const AvionicsPackets &avionics_packets) override;
};

#endif  // SIM_MODELS_ACTUATORS_TETHER_RELEASE_H_
