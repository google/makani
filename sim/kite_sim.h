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

#ifndef SIM_KITE_SIM_H_
#define SIM_KITE_SIM_H_

#include <stdint.h>
#include <string>

#include "avionics/common/avionics_messages.h"
#include "avionics/network/aio_labels.h"
#include "control/system_types.h"
#include "sim/math/ode_solver.h"
#include "sim/models/actuators/actuator.h"
#include "sim/sim_base.h"
#include "sim/sim_comms.h"
#include "sim/sim_messages.h"
#include "sim/sim_types.h"

namespace sim {

class KiteSim : public SimBase {
 public:
  KiteSim(double sim_time, const SystemParams &system_params,
          const SimParams &sim_params, SimCommsInterface *comms);
  virtual ~KiteSim() {}

 private:
  void PublishMessages() override;
  void ReceiveMessages() override;

  void UpdateSensorMessage();

  void UpdateAndSendSimTetherDownMessage(
      const SimTetherDownUpdateState &update_state);

  // Communications configuration.
  SimCommsInterface *comms_;

  SimTetherDownMessage sim_tether_down_;

  // Communication packets.
  AvionicsPackets avionics_packets_;
  SimSensorMessage sensor_message_;

  // The arbitrated lead controller.
  ControllerLabel lead_controller_;

  OdeSolverStatus RunStep() override;

  void InitRun() override;
  void FinishRun() override;
  bool UpdateTetherUpMessage(uint16_t sequence,
                             TetherUpMessage *tether_up) const;
  bool UpdateRadioTetherUpMessage(TetherUpMessage *in,
                                  TetherUpMessage *out) const;

  // Communication functions called each loop.
  void HandleSimCommand(SimCommandMessage *sim_command);

  // Additional utility functions.
  void SleepForRealtime();
  std::string GetSavedStateFilename() const;

  // Communication.
  SimCommandMessage sim_command_;

  uint32_t skip_sleep_counter_;

  // TetherUpMessage used in UpdateSensorOutputs.
  TetherUpMessage tether_up_;

  // State for tether messages.
  uint16_t tether_up_sequence_;
  uint16_t tether_down_sequence_;

  bool saved_to_file_;
  bool loaded_from_file_;

  // Timing information.
  static const int32_t kNumSkipSleeps = 5;

  DISALLOW_COPY_AND_ASSIGN(KiteSim);
};

}  // namespace sim

#endif  // SIM_KITE_SIM_H_
