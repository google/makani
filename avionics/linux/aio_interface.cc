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

#include "avionics/linux/aio_interface.h"

#include <stdarg.h>
#include <stdint.h>

#include "avionics/linux/aio_client.h"
#include "avionics/linux/aio_client_stats.h"
#include "avionics/linux/aio_socket.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_info.h"
#include "avionics/network/message_type.h"

AioInterface::AioInterface(int64_t periodic_interval_usec)
    : run_periodic_loop_(true), periodic_interval_usec_(periodic_interval_usec),
      config_(), client_() {
  AioSocketInitConfig(0, nullptr, &config_);
  AioClientInit(kAioNodeUnknown, &client_);
}

AioInterface::~AioInterface(void) {
  AioClientClose(&client_);
}

void AioInterface::SetPort(uint16_t port) {
  AioSocketSetPort(port, &config_);
}

void AioInterface::SetReuseSocket(bool reuse_socket) {
  AioSocketSetReuseSocket(reuse_socket, &config_);
}

void AioInterface::SetJoinAll(bool join_all) {
  AioSocketSetJoinAll(join_all, &config_);
}

void AioInterface::SetLoopback(bool enable_loopback) {
  AioSocketSetLoopback(enable_loopback, &config_);
}

void AioInterface::SetSubscribed(int32_t num_subscribed,
                                 const MessageType *subscribed) {
  AioSocketSetSubscribed(num_subscribed, subscribed, &config_);
}

bool AioInterface::Run(AioNode source) {
  AioClientInit(source, &client_);
  if (AioClientOpen(&config_, &client_) >= 0) {
    run_periodic_loop_ = true;
    OnStart();
    AioClientLoop(&config_, OnNewMessageWrapper, this, OnPeriodicWrapper, this,
                  periodic_interval_usec_, &client_);
    OnStop();
    return AioClientClose(&client_) >= 0;
  }
  return false;
}

void AioInterface::Stop(void) {
  run_periodic_loop_ = false;
}

bool AioInterface::Subscribe(MessageType type) {
  return AioSocketSubscribe(client_.aio_fd, type, &config_);
}

bool AioInterface::Unsubscribe(MessageType type) {
  return AioSocketUnsubscribe(client_.aio_fd, type, &config_);
}

bool AioInterface::UnsubscribeAll(void) {
  return AioSocketUnsubscribeAll(client_.aio_fd, &config_);
}

void AioInterface::GetStats(AioClientStats *stats) {
  AioClientGetStats(&client_, stats);
}

void AioInterface::SendSpoofed(const AioHeader *header, int32_t payload_length,
                               const uint8_t *payload) {
  AioClientSendSpoofed(&config_, header, payload_length, payload, &client_);
}

void AioInterface::Send(MessageType type, int32_t payload_length,
                        const uint8_t *payload) {
  AioClientSendPacked(&config_, type, payload_length, payload, &client_);
}

void AioInterface::Send(MessageType type, const AioMessageData *data) {
  AioClientSendAioMessageData(&config_, type, data, &client_);
}

void AioInterface::SendPrintf(const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  AioClientVprintf(&config_, &client_, format, ap);
  va_end(ap);
}

bool AioInterface::OnNewMessageWrapper(
    int64_t timestamp, const AioHeader *header, int32_t payload_length,
    const uint8_t *payload, void *arg) {
  auto *aio_interface = reinterpret_cast<AioInterface *>(arg);

  aio_interface->OnNewMessagePacked(timestamp, header, payload_length, payload);
  if (header->type == kMessageTypeStdio) {
    aio_interface->OnNewStdioMessage(timestamp, header, payload_length,
                                     reinterpret_cast<const char *>(payload));
  } else {
    AioMessageData data;
    if (UnpackAioMessageData(static_cast<MessageType>(header->type), payload,
                             &data) == static_cast<size_t>(payload_length)) {
      aio_interface->OnNewMessage(timestamp, header, &data);
    }
  }
  return aio_interface->run_periodic_loop_;
}
