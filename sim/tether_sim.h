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

#ifndef SIM_TETHER_SIM_H_
#define SIM_TETHER_SIM_H_

#include "control/system_params.h"
#include "lib/pcap_to_hdf5/message_log.h"
#include "sim/models/tether_simulator_system.h"
#include "sim/sim_base.h"

namespace sim {

class TetherSim : public SimBase {
 public:
  TetherSim(FlightLogInterpolator *log_interpolator, double sim_time,
            const SystemParams &system_params, const SimParams &sim_params);
  ~TetherSim() {}

 protected:
  void PublishMessages() override;
  void ReceiveMessages() override{};

  OdeSolverStatus RunStep() override;

 private:
  ::lib::pcap_to_hdf5::MessageLog message_log_;
  DISALLOW_COPY_AND_ASSIGN(TetherSim);
};

}  // namespace sim

#endif  // SIM_TETHER_SIM_H_
