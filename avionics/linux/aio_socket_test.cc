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

#include "avionics/common/network_config.h"
#include "avionics/linux/aio_socket.h"
#include "common/macros.h"

namespace {

bool FindMessageTypeInList(int32_t length, const MessageType *list,
                           MessageType type) {
  for (int32_t i = 0; i < length; ++i) {
    if (list[i] == type) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(AioSocketInitConfig, Normal) {
  const MessageType subscribed[] = {
    kMessageTypeMotorStatus,
    kMessageTypeStdio
  };
  AioSocketConfig config;
  memset(&config, 0x5A, sizeof(config));  // Corrupt with invalid data.
  AioSocketInitConfig(ARRAYSIZE(subscribed), subscribed, &config);

  // Expect sensible defaults.
  EXPECT_EQ(UDP_PORT_AIO, config.port);
  EXPECT_FALSE(config.reuse_socket);
  EXPECT_FALSE(config.join_all);
  EXPECT_FALSE(config.enable_loopback);

  // Verify expected subscription.
  for (int32_t i = 0; i < ARRAYSIZE(config.subscribed); ++i) {
    bool subscribe = FindMessageTypeInList(ARRAYSIZE(subscribed), subscribed,
                                           static_cast<MessageType>(i));
    EXPECT_EQ(subscribe, config.subscribed[i]);
  }

  // Allow not having any subscriptions.
  AioSocketInitConfig(0, NULL, &config);
  for (int32_t i = 0; i < ARRAYSIZE(config.subscribed); ++i) {
    EXPECT_FALSE(config.subscribed[i]);
  }
}

TEST(AioSocketSetPort, Normal) {
  AioSocketConfig config;
  AioSocketInitConfig(0, NULL, &config);
  AioSocketSetPort(1024, &config);
  EXPECT_EQ(1024, config.port);
  AioSocketSetPort(4096, &config);
  EXPECT_EQ(4096, config.port);
}

TEST(AioSocketSetReuseSocket, Normal) {
  AioSocketConfig config;
  AioSocketInitConfig(0, NULL, &config);
  AioSocketSetReuseSocket(false, &config);
  EXPECT_FALSE(config.reuse_socket);
  AioSocketSetReuseSocket(true, &config);
  EXPECT_TRUE(config.reuse_socket);
}

TEST(AioSocketSetJoinAll, Normal) {
  AioSocketConfig config;
  AioSocketInitConfig(0, NULL, &config);
  AioSocketSetJoinAll(false, &config);
  EXPECT_FALSE(config.join_all);
  AioSocketSetJoinAll(true, &config);
  EXPECT_TRUE(config.join_all);
}

TEST(AioSocketSetLoopback, Normal) {
  AioSocketConfig config;
  AioSocketInitConfig(0, NULL, &config);
  AioSocketSetLoopback(false, &config);
  EXPECT_FALSE(config.enable_loopback);
  AioSocketSetLoopback(true, &config);
  EXPECT_TRUE(config.enable_loopback);
}

TEST(AioSocketSetSubscribed, Normal) {
  AioSocketConfig config;
  AioSocketInitConfig(0, NULL, &config);

  const MessageType subscribed_1[] = {
    kMessageTypeMotorStatus,
    kMessageTypeStdio
  };
  AioSocketInitConfig(ARRAYSIZE(subscribed_1), subscribed_1, &config);

  // Verify expected subscription.
  AioSocketSetSubscribed(ARRAYSIZE(subscribed_1), subscribed_1, &config);
  for (int32_t i = 0; i < ARRAYSIZE(config.subscribed); ++i) {
    bool subscribe = FindMessageTypeInList(ARRAYSIZE(subscribed_1),
                                           subscribed_1,
                                           static_cast<MessageType>(i));
    EXPECT_EQ(subscribe, config.subscribed[i]);
  }

  // Change subscriptions.
  const MessageType subscribed_2[] = {
    kMessageTypeFlightComputerSensor,
    kMessageTypeServoStatus
  };
  AioSocketInitConfig(ARRAYSIZE(subscribed_2), subscribed_2, &config);

  // Verify expected subscription.
  AioSocketSetSubscribed(ARRAYSIZE(subscribed_2), subscribed_2, &config);
  for (int32_t i = 0; i < ARRAYSIZE(config.subscribed); ++i) {
    bool subscribe = FindMessageTypeInList(ARRAYSIZE(subscribed_2),
                                           subscribed_2,
                                           static_cast<MessageType>(i));
    EXPECT_EQ(subscribe, config.subscribed[i]);
  }

  // Allow not having any subscriptions.
  AioSocketSetSubscribed(0, NULL, &config);
  for (int32_t i = 0; i < ARRAYSIZE(config.subscribed); ++i) {
    EXPECT_FALSE(config.subscribed[i]);
  }
}

TEST(AioSocketOpen, JoinMulticastGroups) {
  const MessageType subscribed[] = {
    kMessageTypeMotorStatus,
    kMessageTypeStdio
  };
  AioSocketConfig config;
  AioSocketInitConfig(ARRAYSIZE(subscribed), subscribed, &config);

  // Default options.
  int32_t fd = AioSocketOpen(&config);
  EXPECT_LE(0, fd);
  EXPECT_LE(0, AioSocketClose(fd));
}

TEST(AioSocketOpen, ReuseSocket) {
  AioSocketConfig config;
  AioSocketInitConfig(0, NULL, &config);

  // Test reuse socket = false. We cannot bind multiple sockets.
  AioSocketSetReuseSocket(false, &config);
  int32_t fd_1 = AioSocketOpen(&config);
  ASSERT_LE(0, fd_1);
  int32_t fd_2 = AioSocketOpen(&config);
  EXPECT_GT(0, fd_2);
  EXPECT_LE(0, AioSocketClose(fd_1));
  EXPECT_LE(0, AioSocketClose(fd_2));

  // Test reuse socket = true. We can bind multiple sockets.
  AioSocketSetReuseSocket(true, &config);
  fd_1 = AioSocketOpen(&config);
  fd_2 = AioSocketOpen(&config);
  EXPECT_LE(0, fd_1);
  EXPECT_LE(0, fd_2);
  EXPECT_LE(0, AioSocketClose(fd_1));
  EXPECT_LE(0, AioSocketClose(fd_2));
}

TEST(AioSocketOpen, Loopback) {
  const MessageType subscribed[] = {
    kMessageTypeStdio
  };

  // Process 1 (loopback = false) should not be able to send messages to local
  // processes.
  AioSocketConfig config_1;
  AioSocketInitConfig(ARRAYSIZE(subscribed), subscribed, &config_1);
  AioSocketSetReuseSocket(true, &config_1);
  AioSocketSetLoopback(false, &config_1);
  int32_t fd_1 = AioSocketOpen(&config_1);
  ASSERT_LE(0, fd_1);

  // Process 2 (loopback = true) should be able to send messages to local
  // processes.
  AioSocketConfig config_2;
  AioSocketInitConfig(ARRAYSIZE(subscribed), subscribed, &config_2);
  AioSocketSetReuseSocket(true, &config_2);
  AioSocketSetLoopback(true, &config_2);
  int32_t fd_2 = AioSocketOpen(&config_2);
  ASSERT_LE(0, fd_2);

  // Message data.
  const uint8_t out[] = "Hello world";
  uint8_t in[ARRAYSIZE(out) + 1];

  // TODO: Test process 1 to process 2. We cannot test disabling
  // loopback because Jenkins slaves bind to the loopback interface.

  // Test process 2 to process 1.
  EXPECT_EQ(ARRAYSIZE(out), AioSocketSend(&config_2, kMessageTypeStdio,
                                          ARRAYSIZE(out), out, fd_2));
  EXPECT_EQ(ARRAYSIZE(out), AioSocketReceive(fd_1, ARRAYSIZE(in), in));
  EXPECT_EQ(ARRAYSIZE(out), AioSocketReceive(fd_2, ARRAYSIZE(in), in));

  // Done.
  EXPECT_LE(0, AioSocketClose(fd_1));
  EXPECT_LE(0, AioSocketClose(fd_2));
}

TEST(AioSocketOpen, JoinAll) {
  // Process 1 (join_all = false) should not receive messages that process 2
  // subscribes to.
  const MessageType subscribed_1[] = {
    kMessageTypeStdio,
  };
  AioSocketConfig config_1;
  AioSocketInitConfig(ARRAYSIZE(subscribed_1), subscribed_1, &config_1);
  AioSocketSetReuseSocket(true, &config_1);
  AioSocketSetLoopback(true, &config_1);
  AioSocketSetJoinAll(false, &config_1);
  int32_t fd_1 = AioSocketOpen(&config_1);
  ASSERT_LE(0, fd_1);

  // Process 2 (join_all = true) should receive messages that process 1
  // subscribes to.
  const MessageType subscribed_2[] = {
    kMessageTypeMotorStatus
  };
  AioSocketConfig config_2;
  AioSocketInitConfig(ARRAYSIZE(subscribed_2), subscribed_2, &config_2);
  AioSocketSetReuseSocket(true, &config_2);
  AioSocketSetLoopback(true, &config_2);
  AioSocketSetJoinAll(true, &config_2);
  int32_t fd_2 = AioSocketOpen(&config_2);
  ASSERT_LE(0, fd_2);

  // Message data.
  const uint8_t out[] = "Hello world";
  uint8_t in[ARRAYSIZE(out) + 1];

  // Test process 1 to process 2.
  EXPECT_EQ(ARRAYSIZE(out), AioSocketSend(&config_1, kMessageTypeStdio,
                                          ARRAYSIZE(out), out, fd_1));
  EXPECT_EQ(ARRAYSIZE(out), AioSocketReceive(fd_1, ARRAYSIZE(in), in));
  EXPECT_EQ(ARRAYSIZE(out), AioSocketReceive(fd_2, ARRAYSIZE(in), in));

  // Test process 2 to process 1.
  EXPECT_EQ(ARRAYSIZE(out), AioSocketSend(&config_2, kMessageTypeMotorStatus,
                                          ARRAYSIZE(out), out, fd_2));
  EXPECT_EQ(0, AioSocketReceive(fd_1, ARRAYSIZE(in), in));
  EXPECT_EQ(ARRAYSIZE(out), AioSocketReceive(fd_2, ARRAYSIZE(in), in));

  // Done.
  EXPECT_LE(0, AioSocketClose(fd_1));
  EXPECT_LE(0, AioSocketClose(fd_2));
}

TEST(AioSocketSelect, Normal) {
  const MessageType subscribed[] = {
    kMessageTypeStdio
  };
  AioSocketConfig config;
  AioSocketInitConfig(ARRAYSIZE(subscribed), subscribed, &config);
  AioSocketSetLoopback(true, &config);

  // Open socket.
  int32_t fd = AioSocketOpen(&config);
  ASSERT_LE(0, fd);

  // Wait for response.
  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(fd, &read_set);

  fd_set error_set;
  FD_ZERO(&error_set);
  FD_SET(fd, &error_set);

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 1000;
  int32_t rc = AioSocketSelect(fd + 1, &read_set, &error_set, &timeout);

  // Expect timeout.
  EXPECT_EQ(0, rc);
  EXPECT_FALSE(FD_ISSET(fd, &read_set));
  EXPECT_FALSE(FD_ISSET(fd, &error_set));

  // Send message.
  const uint8_t out[] = "Hello world";
  EXPECT_EQ(ARRAYSIZE(out), AioSocketSend(&config, kMessageTypeStdio,
                                          ARRAYSIZE(out), out, fd));

  // Wait for response.
  FD_ZERO(&read_set);
  FD_SET(fd, &read_set);

  FD_ZERO(&error_set);
  FD_SET(fd, &error_set);

  timeout.tv_sec = 0;
  timeout.tv_usec = 1000;
  rc = AioSocketSelect(fd + 1, &read_set, &error_set, &timeout);

  // Expect receive ready.
  EXPECT_EQ(1, rc);
  EXPECT_TRUE(FD_ISSET(fd, &read_set));
  EXPECT_FALSE(FD_ISSET(fd, &error_set));
  EXPECT_LE(0, AioSocketClose(fd));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
