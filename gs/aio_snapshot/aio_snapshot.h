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

#ifndef GS_AIO_SNAPSHOT_AIO_SNAPSHOT_H_
#define GS_AIO_SNAPSHOT_AIO_SNAPSHOT_H_

#include <stdbool.h>
#include <stdint.h>

#include <array>
#include <functional>

#include "avionics/linux/aio.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"
#include "control/control_telemetry.h"
#include "control/system_types.h"
#include "gs/aio_snapshot/aio_telemetry.h"
#include "lib/udpio/udpio.h"
#include "system/labels.h"

namespace ground_station {

// Snapshotting class for AIO messages.
//
// This class populates structures which summarize certain kinds of
// AIO traffic by regularly sampling the current value table, then
// transmits those summary structures over UDP.
class AioSnapshotMain {
 public:
  AioSnapshotMain(double snapshot_period, const CommsParams &params);
  ~AioSnapshotMain() {}

  static bool AioCallback(void *context) {
    AioSnapshotMain *main = static_cast<AioSnapshotMain *>(context);
    main->TakeSnapshots();
    main->SendSnapshots();
    return true;
  }

  void SetupComms();
  void CloseComms();
  void Run();

 private:
  template <class T, int32_t num_elts>
  void TakeArraySnapshot(
      const std::function<bool(AioNode, T *, uint16_t *, int64_t *)> &get_func,
      const std::array<AioNode, num_elts> &nodes, T elements[],
      bool elements_updated[]);

  void TakeSnapshots();
  void SendSnapshots();

  // Parameters.
  double snapshot_period_;
  const CommsParams &params_;

  // Arrays listing the AioNode objects associated with incoming messages.
  std::array<AioNode, kNumAioNodes> aio_nodes_;
  std::array<AioNode, kNumControllers> controller_nodes_;
  std::array<AioNode, kNumCoreSwitches> core_switch_nodes_;
  std::array<AioNode, kNumDrums> drum_nodes_;
  std::array<AioNode, kNumFlightComputers> flight_computer_nodes_;
  const AioNode gs_gps_node_ = kAioNodeGpsBaseStation;
  const AioNode gs_weather_node_ = kAioNodePlatformSensorsA;
  const AioNode joystick_node_ = kAioNodeJoystickA;
  std::array<AioNode, kNumLoadcellNodes> loadcell_nodes_;
  std::array<AioNode, kNumMotors> motor_nodes_;
  std::array<AioNode, kNumPlatforms> platform_nodes_;
  std::array<AioNode, kNumRecorderQ7s> recorder_q7_nodes_;
  std::array<AioNode, kNumRecorderTms570s> recorder_tms570_nodes_;
  std::array<AioNode, kNumServos> servo_nodes_;
  std::array<AioNode, 2> tether_down_nodes_;
  const AioNode winch_plc_node_ = kAioNodePlcTophat;
  const AioNode wing_gps_nodes_[kNumWingGpsReceivers];

  // Variables for forwarding messages in the AioTelemetry structures.
  udp_config aio_1_udpio_config_;
  AioTelemetry1 aio_telemetry_1_;
  udp_config aio_2_udpio_config_;
  AioTelemetry2 aio_telemetry_2_;
  udp_config aio_3_udpio_config_;
  AioTelemetry3 aio_telemetry_3_;

  DISALLOW_COPY_AND_ASSIGN(AioSnapshotMain);
};

}  // namespace ground_station

#endif  // GS_AIO_SNAPSHOT_AIO_SNAPSHOT_H_
