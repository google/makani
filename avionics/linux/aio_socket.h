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

#ifndef AVIONICS_LINUX_AIO_SOCKET_H_
#define AVIONICS_LINUX_AIO_SOCKET_H_

#include <stdbool.h>
#include <stdint.h>
#include <sys/select.h>
#include <sys/time.h>

#include "avionics/network/message_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  // Port to bind.
  uint16_t port;
  // Allow multiple processes to bind to this address.
  bool reuse_socket;
  // Deliver messages from multicast groups joined by all processes.
  bool join_all;
  // Allow other processes to listen to outgoing messages.
  bool enable_loopback;
  // Each index corresponds to a MessageType. Set true to subscribe.
  bool subscribed[kNumMessageTypes];
} AioSocketConfig;

void AioSocketInitConfig(int32_t num_subscribed, const MessageType *subscribed,
                         AioSocketConfig *conf);
void AioSocketSetPort(uint16_t port, AioSocketConfig *conf);
void AioSocketSetReuseSocket(bool reuse_socket, AioSocketConfig *conf);
void AioSocketSetJoinAll(bool join_all, AioSocketConfig *conf);
void AioSocketSetLoopback(bool enable_loopback, AioSocketConfig *conf);
void AioSocketSetSubscribed(int32_t num_subscribed,
                            const MessageType *subscribed,
                            AioSocketConfig *conf);
bool AioSocketIsSubscribed(MessageType type, const AioSocketConfig *conf);

int32_t AioSocketOpen(const AioSocketConfig *conf);
int32_t AioSocketClose(int32_t fd);
bool AioSocketSubscribe(int32_t fd, MessageType type, AioSocketConfig *conf);
bool AioSocketUnsubscribe(int32_t fd, MessageType type, AioSocketConfig *conf);
bool AioSocketUnsubscribeAll(int32_t fd, AioSocketConfig *conf);
int32_t AioSocketReceive(int32_t fd, int32_t data_length, uint8_t *data);
int32_t AioSocketSend(const AioSocketConfig *conf, MessageType type,
                      int32_t data_length, const uint8_t *data, int32_t fd);

int32_t AioSocketSelect(int32_t nfds, fd_set *read_set, fd_set *error_set,
                        struct timeval *timeout);

#define AIO_SOCKET_IGNORE_ERROR(e) (                                    \
    (e) == EAGAIN || (EWOULDBLOCK != EAGAIN && (e) == EWOULDBLOCK) ||   \
    (e) == EINTR)

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_LINUX_AIO_SOCKET_H_
