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

#include <string.h>

#include <list>

#include "avionics/common/aio_header.h"
#include "avionics/common/avionics_messages.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_info.h"
#include "avionics/network/message_type.h"
#include "nav/aio/aio_to_ins/aio_to_ins.h"
#include "nav/aio/aio_to_ins/label_map.h"
#include "nav/ins/messages/messages.h"

namespace {

typedef std::list<InsMessage> InsMessageList;

void AppendInsMessageToInsMessageList(const InsMessage *m, void *arg) {
  InsMessageList *list = reinterpret_cast<InsMessageList *>(arg);
  list->push_back(*m);
}

}  // namespace

TEST(AioMessageToInsMessages, FlightComputerSensor) {
  AioMessage aio;
  memset(&aio, 0, sizeof(aio));
  aio.header.type = kMessageTypeFlightComputerSensor;
  aio.header.source = kAioNodeFcA;

  // Verify connection of the FlightComputerSensor message.
  InsMessageList pending;
  EXPECT_TRUE(AioMessageToInsMessages(
      GetAioSourceToInsLabelMap(kControllerA), 0, &aio.header, &aio.u,
      AppendInsMessageToInsMessageList, &pending));
  EXPECT_LT(0, pending.size());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
