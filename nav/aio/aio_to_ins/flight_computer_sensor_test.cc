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

#include <gtest/gtest.h>

#include <stdint.h>
#include <string.h>

#include <vector>

#include "avionics/common/aio_header.h"
#include "avionics/common/avionics_messages.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_info.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"
#include "nav/aio/aio_to_ins/flight_computer_sensor.h"
#include "nav/aio/aio_to_ins/label_map.h"
#include "nav/ins/messages/labels.h"
#include "nav/ins/messages/messages.h"

namespace {

typedef std::vector<InsMessage> InsMessageVector;

void AppendInsMessageToInsMessageVector(const InsMessage *m, void *arg) {
  InsMessageVector *vec = reinterpret_cast<InsMessageVector *>(arg);
  vec->push_back(*m);
}

void AppendInsInertialMessageToInsMessageVector(const InsMessage *m,
                                                void *arg) {
  if (m->header.type == kInsMessageTypeInertial) {
    AppendInsMessageToInsMessageVector(m, arg);
  }
}

}  // namespace

TEST(ImuConingScullingData, Order) {
  // The FlightComputerSensorMessage contains an cs[] array of inertial
  // updates. Index cs[0] represents the latest information, while cs[n - 1]
  // represents to oldest information. This test verifies the insertion of
  // each cs[] element in the correct order.
  AioMessage aio;
  memset(&aio, 0, sizeof(aio));
  aio.header.type = kMessageTypeFlightComputerSensor;
  aio.header.source = kAioNodeFcA;
  aio.header.sequence = 1U;

  FlightComputerSensorMessage &data = aio.u.flight_computer_sensor;
  constexpr int32_t n = ARRAYSIZE(data.cs);
  const int64_t timestamp = 123981213;  // Receive timestamp.
  const int32_t period = 10000;  // Node transmit period.
  for (int32_t i = 0; i < n; ++i) {
    // Node timestamp and latency.
    data.cs[i].timestamp = static_cast<uint32_t>(314129231 - i * period);
    data.cs[i].latency = 120 + i;
  }

  InsMessageVector pending;
  EXPECT_TRUE(FlightComputerSensorToInsMessages(
      GetAioSourceToInsLabelMap(kControllerA), timestamp, &aio.header, &data,
      AppendInsInertialMessageToInsMessageVector, &pending));

  ASSERT_EQ(n, pending.size());
  for (int32_t i = 0; i < n; ++i) {
    const InsMessage &ins = pending[n - i - 1];
    EXPECT_EQ(kInsImuLabelPrimary, ins.header.label);
    EXPECT_EQ(static_cast<uint16_t>(aio.header.sequence - i),
              ins.header.seq_num);
    EXPECT_EQ(static_cast<int64_t>(timestamp - i * period - data.cs[i].latency),
              ins.header.timestamp);
  }
}

TEST(ImuConingScullingData, WrongImu) {
  AioMessage aio;
  memset(&aio, 0, sizeof(aio));
  aio.header.type = kMessageTypeFlightComputerSensor;

  // Attempt to insert a message from FcB into ControllerA. ControllerA
  // should ignore all inertial messages.
  aio.header.source = kAioNodeFcB;
  InsMessageVector pending;
  EXPECT_FALSE(FlightComputerSensorToInsMessages(
      GetAioSourceToInsLabelMap(kControllerA), 0, &aio.header,
      &aio.u.flight_computer_sensor,
      AppendInsInertialMessageToInsMessageVector, &pending));
  EXPECT_EQ(0, pending.size());

  // Attempt to insert a message from FcA into ControllerA. ControllerA
  // should accept all inertial messages.
  aio.header.source = kAioNodeFcA;
  EXPECT_TRUE(FlightComputerSensorToInsMessages(
      GetAioSourceToInsLabelMap(kControllerA), 0, &aio.header,
      &aio.u.flight_computer_sensor,
      AppendInsInertialMessageToInsMessageVector, &pending));
  EXPECT_EQ(ARRAYSIZE(aio.u.flight_computer_sensor.cs), pending.size());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
