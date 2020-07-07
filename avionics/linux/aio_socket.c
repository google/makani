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

#include "avionics/linux/aio_socket.h"

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define __USE_MISC  // For struct ip_mreq.
#include <netinet/in.h>
#include <sys/socket.h>

#include "avionics/common/network_config.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"

static void PrintError(int error, const char *func, int32_t line) {
  fprintf(stderr, __FILE__ ":%d %s: %s\n", line, func, strerror(error));
}

static int32_t OpenSocket(void) {
  // Open in non-blocking mode to prevent stalling (we're using UDP afterall).
  int32_t fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (fd < 0) {
    PrintError(errno, __func__, __LINE__);
  }
  return fd;
}

static bool ReuseSocket(bool reuse_socket, int32_t fd) {
  // Reuse socket to allow multiple processes to bind to this address/port.
  uint32_t opt = (uint32_t)reuse_socket;
  if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    PrintError(errno, __func__, __LINE__);
    return false;
  }
  return true;
}

static bool BindSocket(uint16_t port, int32_t fd) {
  struct sockaddr_in local_addr;
  memset(&local_addr, 0, sizeof(local_addr));
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(port);
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(fd, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
    PrintError(errno, __func__, __LINE__);
    return false;
  }
  return true;
}

static bool JoinMulticastGroup(int32_t fd, MessageType type) {
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr =
      htonl(IpAddressToUint32(AioMessageTypeToIpAddress(type)));
  mreq.imr_interface.s_addr = htonl(INADDR_ANY);
  if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
    PrintError(errno, __func__, __LINE__);
    return false;
  }
  return true;
}

static bool LeaveMulticastGroup(int32_t fd, MessageType type) {
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr =
      htonl(IpAddressToUint32(AioMessageTypeToIpAddress(type)));
  mreq.imr_interface.s_addr = htonl(INADDR_ANY);
  if (setsockopt(fd, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
    PrintError(errno, __func__, __LINE__);
    return false;
  }
  return true;
}

static bool JoinMulticastGroups(uint32_t num_subscribed, const bool *subscribed,
                                int32_t fd) {
  bool failure = false;
  for (uint32_t type = 0; type < num_subscribed; ++type) {
    if (subscribed[type]) {
      failure |= !JoinMulticastGroup(fd, type);
    }
  }
  return !failure;
}

static bool JoinMulticastAll(bool join_all, int32_t fd) {
  // Deliver messages from multicast groups joined by all or this socket.
  uint32_t opt = (uint32_t)join_all;
  if (setsockopt(fd, IPPROTO_IP, IP_MULTICAST_ALL, &opt, sizeof(opt)) < 0) {
    PrintError(errno, __func__, __LINE__);
    return false;
  }
  return true;
}

static bool EnableMulticastLoopback(bool enable_loopback, int32_t fd) {
  // Allow other processes to listen to outgoing messages.
  uint32_t opt = (uint32_t)enable_loopback;
  if (setsockopt(fd, IPPROTO_IP, IP_MULTICAST_LOOP, &opt, sizeof(opt)) < 0) {
    PrintError(errno, __func__, __LINE__);
    return false;
  }
  return true;
}

static bool ConfigureSocket(const AioSocketConfig *conf, int32_t fd) {
  return ReuseSocket(conf->reuse_socket, fd) && BindSocket(conf->port, fd) &&
      JoinMulticastGroups((uint32_t)ARRAYSIZE(conf->subscribed),
                          conf->subscribed, fd) &&
      JoinMulticastAll(conf->join_all, fd) &&
      EnableMulticastLoopback(conf->enable_loopback, fd);
}

// Application interface.

void AioSocketInitConfig(int32_t num_subscribed, const MessageType *subscribed,
                         AioSocketConfig *conf) {
  assert(conf != NULL);

  memset(conf, 0, sizeof(*conf));
  conf->port = UDP_PORT_AIO;
  AioSocketSetSubscribed(num_subscribed, subscribed, conf);
}

void AioSocketSetPort(uint16_t port, AioSocketConfig *conf) {
  assert(conf != NULL);
  conf->port = port;
}

void AioSocketSetReuseSocket(bool reuse_socket, AioSocketConfig *conf) {
  assert(conf != NULL);
  conf->reuse_socket = reuse_socket;
}

void AioSocketSetJoinAll(bool join_all, AioSocketConfig *conf) {
  assert(conf != NULL);
  conf->join_all = join_all;
}

void AioSocketSetLoopback(bool enable_loopback, AioSocketConfig *conf) {
  assert(conf != NULL);
  conf->enable_loopback = enable_loopback;
}

void AioSocketSetSubscribed(int32_t num_subscribed,
                            const MessageType *subscribed,
                            AioSocketConfig *conf) {
  assert(num_subscribed == 0 || subscribed != NULL);
  assert(conf != NULL);

  // Assign subscribed message type table. In the Linux multicast
  // implementation, when multiple processes share a single address/port
  // via SO_REUSEADDR they also share a common list of multicast groups.
  // This table prevents the application from processing unwanted messages.
  memset(conf->subscribed, 0, sizeof(conf->subscribed));
  for (int32_t i = 0; i < num_subscribed; ++i) {
    MessageType type = subscribed[i];
    if (type < ARRAYSIZE(conf->subscribed)) {
      conf->subscribed[type] = true;
    } else {
      assert(false);
    }
  }
}

bool AioSocketIsSubscribed(MessageType type, const AioSocketConfig *conf) {
  return type < ARRAYSIZE(conf->subscribed) && conf->subscribed[type];
}

int32_t AioSocketOpen(const AioSocketConfig *conf) {
  assert(conf != NULL);

  int32_t fd = OpenSocket();
  if (fd >= 0 && !ConfigureSocket(conf, fd)) {
    AioSocketClose(fd);
    fd = -1;
  }
  return fd;
}

int32_t AioSocketClose(int32_t fd) {
  int32_t rc = 0;
  if (fd >= 0) {
    rc = close(fd);
    if (rc < 0) {
      PrintError(errno, __func__, __LINE__);
    }
  }
  return rc;
}

bool AioSocketSubscribe(int32_t fd, MessageType type, AioSocketConfig *conf) {
  assert(conf != NULL);
  assert(IsValidMessageType(type));
  conf->subscribed[type] = true;
  return fd >= 0 ? JoinMulticastGroup(fd, type) : true;
}

bool AioSocketUnsubscribe(int32_t fd, MessageType type, AioSocketConfig *conf) {
  assert(conf != NULL);
  assert(IsValidMessageType(type));
  conf->subscribed[type] = false;
  return fd >= 0 ? LeaveMulticastGroup(fd, type) : true;
}

bool AioSocketUnsubscribeAll(int32_t fd, AioSocketConfig *conf) {
  assert(conf != NULL);

  bool error = false;
  for (uint32_t type = 0; type < (uint32_t)ARRAYSIZE(conf->subscribed);
       ++type) {
    if (conf->subscribed[type]) {
      error |= !AioSocketUnsubscribe(fd, type, conf);
    }
  }
  return !error;
}

int32_t AioSocketReceive(int32_t fd, int32_t data_length, uint8_t *data) {
  assert(data_length >= 0 && data != NULL);

  ssize_t rc = recv(fd, data, (size_t)data_length, MSG_DONTWAIT);
  if (rc < 0) {
    if (AIO_SOCKET_IGNORE_ERROR(errno)) {
      rc = 0;  // Ignore these errors.
    } else {
      PrintError(errno, __func__, __LINE__);
    }
  }
  return (int32_t)rc;
}

int32_t AioSocketSend(const AioSocketConfig *conf, MessageType type,
                      int32_t data_length, const uint8_t *data, int32_t fd) {
  assert(conf != NULL);
  assert(IsValidMessageType(type));
  assert(data_length >= 0 && data != NULL);

  struct sockaddr_in remote_addr = {
    .sin_family = AF_INET,
    .sin_port = htons(conf->port),
    .sin_addr = {
      .s_addr = htonl(IpAddressToUint32(AioMessageTypeToIpAddress(type))),
    },
  };
  ssize_t rc = sendto(fd, data, (size_t)data_length, MSG_DONTWAIT,
                      (struct sockaddr *)&remote_addr, sizeof(remote_addr));
  if (rc < 0) {
    if (AIO_SOCKET_IGNORE_ERROR(errno)) {
      // TODO(b/131333372): Remove this message.
      // Print errors on message send for now.
      PrintError(errno, __func__, __LINE__);
      rc = 0;  // Ignore these errors.
    } else {
      PrintError(errno, __func__, __LINE__);
    }
  }
  return (int32_t)rc;
}

// The primary purpose of this function is to allow test programs to stub
// select().
int32_t AioSocketSelect(int32_t nfds, fd_set *read_set, fd_set *error_set,
                        struct timeval *timeout) {
  int32_t rc = select(nfds, read_set, NULL, error_set, timeout);
  if (rc < 0) {
    if (errno == EINTR) {
      rc = 0;  // Ignore these errors.
    } else {
      PrintError(errno, __func__, __LINE__);
    }
  }
  return rc;
}
