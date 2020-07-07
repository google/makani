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

#ifndef AVIONICS_LINUX_AIO_INTERFACE_H_
#define AVIONICS_LINUX_AIO_INTERFACE_H_

#include <stdint.h>

#include "avionics/linux/aio_client.h"
#include "avionics/linux/aio_client_stats.h"
#include "avionics/linux/aio_socket.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_info.h"
#include "avionics/network/message_type.h"

class AioInterface {
 public:
  explicit AioInterface(int64_t periodic_interval_usec);
  virtual ~AioInterface(void);

  // Configuration interface. Set before Run().
  void SetPort(uint16_t port);
  void SetReuseSocket(bool reuse_socket);
  void SetJoinAll(bool join_all);
  void SetLoopback(bool enable_loopback);
  void SetSubscribed(int32_t num_subscribed, const MessageType *subscribed);

  // Run the AIO client main loop until OnPeriodic returns false.
  bool Run(AioNode source);

  // Stops periodic loop.
  void Stop(void);

  // Subscribe to receive messages of a particular type.
  bool Subscribe(MessageType type);

  // Unsubscribe from messages of a particular type.
  bool Unsubscribe(MessageType type);
  bool UnsubscribeAll(void);

  // Get AIO client statistics and reset counts. Repeated calls return zero.
  void GetStats(AioClientStats *stats);

  // Send an AIO message given an arbitrary header and packed data. This
  // function allows applications to spoof messages from another source
  // and increment sequence numbers arbitrarily.
  void SendSpoofed(const AioHeader *header, int32_t payload_length,
                   const uint8_t *payload);

  // Send an AIO message.
  void Send(MessageType type, int32_t payload_length, const uint8_t *payload);
  void Send(MessageType type, const AioMessageData *data);
  void SendPrintf(const char *format, ...)
      __attribute__((format(printf, 2, 3)));

 protected:
  // Override this virtual function to handle initialization upon calling Run.
  virtual void OnStart(void) {}

  // Override this virtual function to handle shutdown when Run terminates.
  virtual void OnStop(void) {}

  // Override these virtual functions to handle receive messages.
  virtual void OnNewMessagePacked(int64_t /* timestamp */,
                                  const AioHeader * /* header */,
                                  int32_t /* payload_length */,
                                  const uint8_t * /* payload */) {}
  virtual void OnNewStdioMessage(int64_t /* timestamp */,
                                 const AioHeader * /* header */,
                                 int32_t /* length */,
                                 const char * /* str */) {}
  virtual void OnNewMessage(int64_t /* timestamp */,
                            const AioHeader * /* header */,
                            const AioMessageData * /* data */) {}

  // Override this virtual function to handle periodic tasks.
  virtual void OnPeriodic(void) {}

 private:
  bool run_periodic_loop_;
  int64_t periodic_interval_usec_;
  AioSocketConfig config_;
  AioClient client_;

  static bool OnNewMessageWrapper(
      int64_t timestamp, const AioHeader *header, int32_t payload_length,
      const uint8_t *payload, void *arg);

  static bool OnPeriodicWrapper(void *arg) {
    auto *aio_interface = reinterpret_cast<AioInterface *>(arg);
    aio_interface->OnPeriodic();
    return aio_interface->run_periodic_loop_;
  }
};

#endif  // AVIONICS_LINUX_AIO_INTERFACE_H_
