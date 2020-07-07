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

// This module provides an event based AIO client that does not depend on
// the CVT or another higher level interface. The application should call
// AioClientInit() to configure the interface, then AioClientOpen() to
// open the socket, and finally AioClientWait/Poll*() to receive new messages.

#include "avionics/linux/aio_client.h"

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/if.h>

#include "avionics/common/aio_header.h"
#include "avionics/common/aio_version_code.h"
#include "avionics/common/pack_aio_header.h"
#include "avionics/linux/clock.h"
#include "avionics/linux/aio_socket.h"
#include "avionics/linux/netlink_socket.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/aio_node_to_ip_address.h"
#include "avionics/network/message_info.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"

#define UDP_MAX_LENGTH 65000  // Account for IP/UDP headers.
#define MAX_PAYLOAD_LENGTH (UDP_MAX_LENGTH - PACK_AIOHEADER_SIZE)
#define NETLINK_QUERY_PERIOD_US (1 * 1000 * 1000)

// Handle AIO receive.

static bool IsValidSource(AioNode source) {
  return 0 <= source && source < kNumAioNodes;
}

static bool IsValidPackedSize(MessageType type, int32_t length) {
  assert(0 <= type && type < ARRAYSIZE(kAioMessageInfo));

  return kAioMessageInfo[type].pack_size <= 0  // For variable sized messages.
      || (int32_t)kAioMessageInfo[type].pack_size == length;
}

static bool ReadAioHeader(int32_t length, const uint8_t *packed, AioHeader *h) {
  return length >= PACK_AIOHEADER_SIZE
      && UnpackAioHeader(packed, 1, h) == PACK_AIOHEADER_SIZE
      && h->version == AIO_VERSION
      && IsValidSource(h->source)
      && IsValidMessageType(h->type)
      && IsValidPackedSize(h->type, length - PACK_AIOHEADER_SIZE);
}

static bool IsDuplicateAioMessage(int64_t timestamp, const AioHeader *h,
                                  AioClientInput *input) {
  assert(0 <= h->source && h->source < ARRAYSIZE(input->dedup));
  assert(0 <= h->type && h->type < ARRAYSIZE(input->dedup[0]));

  AioDeduplicate *dedup = &input->dedup[h->source][h->type];
  if (!AioHeaderIsDuplicate(timestamp, h, dedup)) {
    dedup->timestamp = timestamp;
    dedup->sequence = h->sequence;
    return false;
  }
  return true;
}

static int32_t ReceiveAioMessage(const AioSocketConfig *config, int32_t fd,
                                 int64_t timestamp, int32_t length,
                                 uint8_t *data, AioHeader *header,
                                 int32_t *payload_length, uint8_t **payload,
                                 AioClientInput *input, AioClientStats *stats) {
  int32_t recv_length = AioSocketReceive(fd, length, data);
  if (recv_length > 0) {
    if (ReadAioHeader(recv_length, data, header)
        && AioSocketIsSubscribed(header->type, config)) {
      if (IsDuplicateAioMessage(timestamp, header, input)) {
        recv_length = 0;
      } else {
        *payload_length = recv_length - PACK_AIOHEADER_SIZE;
        *payload = &data[PACK_AIOHEADER_SIZE];
      }
      ++stats->received_valid_packets;
    } else {
      ++stats->received_invalid_packets;
      recv_length = -1;
    }
  }
  return recv_length;
}

// Handle AIO send.

static void SetAioHeader(AioNode source, MessageType type,
                         AioClientOutput *output, AioHeader *header) {
  // Always increment sequence number regardless of the transmission result.
  assert(0 <= type && type < ARRAYSIZE(output->seq_num));
  output->seq_num[type] = (uint16_t)(output->seq_num[type] + 1U);

  *header = (AioHeader){
    .version = AIO_VERSION,
    .source = source,
    .type = type,
    .sequence = output->seq_num[type],
    .timestamp = (uint32_t)ClockGetUs()
  };
}

__attribute__((format(printf, 1, 0))) static int32_t WriteVprintf(
    const char *format, va_list ap, int32_t buf_length, uint8_t *buf) {
  int32_t length = vsnprintf((char *)buf, (size_t)buf_length, format, ap);
  if (length < 0) {
    length = 0;
  } else if (length >= buf_length) {
    length = buf_length;
    buf[buf_length - 1] = '\0';
  }
  return length;
}

static void SendAioMessage(const AioSocketConfig *config, MessageType type,
                           int32_t length, const uint8_t *packet,
                           int32_t fd, AioClientStats *stats) {
  if (AioSocketSend(config, type, length, packet, fd) > 0) {
    ++stats->sent_packets;
  } else {
    ++stats->unsent_packets;
  }
  // No return. Applications should not require knowledge of the transmission
  // result.
}

static void SendPacked(const AioSocketConfig *config, const AioHeader *header,
                       int32_t payload_length, const uint8_t *payload,
                       int32_t fd, AioClientStats *stats) {
  if (0 <= payload_length && payload_length <= MAX_PAYLOAD_LENGTH) {
    uint8_t packet[UDP_MAX_LENGTH];
    int32_t o = 0;

    o += (int32_t)PackAioHeader(header, 1, &packet[o]);
    memcpy(&packet[o], payload, (size_t)payload_length);
    o += payload_length;
    SendAioMessage(config, header->type, o, packet, fd, stats);
  }
}

static void SendAioMessageData(const AioSocketConfig *config,
                               const AioHeader *header,
                               const AioMessageData *data, int32_t fd,
                               AioClientStats *stats) {
  uint8_t packet[UDP_MAX_LENGTH];
  int32_t o = 0;

  o += (int32_t)PackAioHeader(header, 1, &packet[o]);
  o += (int32_t)PackAioMessageData(header->type, data, &packet[o]);
  SendAioMessage(config, header->type, o, packet, fd, stats);
}

__attribute__((format(printf, 3, 0))) static void SendVprintf(
    const AioSocketConfig *config, const AioHeader *header, const char *format,
    va_list ap, int32_t fd, AioClientStats *stats) {
  uint8_t packet[UDP_MAX_LENGTH];
  int32_t o = 0;

  o += (int32_t)PackAioHeader(header, 1, &packet[o]);
  o += WriteVprintf(format, ap, UDP_MAX_LENGTH - o, &packet[o]);
  SendAioMessage(config, header->type, o, packet, fd, stats);
}

// Handle AIO errors.

static bool ReopenAioSocket(const AioSocketConfig *config, int32_t *fd,
                            AioClientStats *stats) {
  int32_t old_fd = *fd;
  *fd = -1;
  AioSocketClose(old_fd);
  *fd = AioSocketOpen(config);

  if (*fd >= 0) {
    ++stats->aio_socket_reopened;
    return true;
  }
  return false;
}

// Handle netlink receive.

static void ReceiveNetlinkMessage(const AioSocketConfig *config,
                                  AioClient *client) {
  uint8_t nl_msg[UDP_MAX_LENGTH];
  int32_t nl_len = (int32_t)recv(client->netlink_fd, nl_msg, sizeof(nl_msg),
                                 MSG_DONTWAIT);
  if (nl_len > 0) {
    // The kernel may send multiple address messages per request. We therefore
    // OR the validity flag here and clear it when the link goes down.
    client->netlink_if_index_valid |= NetlinkGetInterfaceIndexByIp(
        nl_len, nl_msg, &client->ip, &client->netlink_if_index);

    if (client->netlink_if_index_valid) {
      // Check for the latest interface statistics.
      if (NetlinkGetInterfaceStats(nl_len, nl_msg, client->netlink_if_index,
                                   &client->stats.netlink)) {
        ++client->stats.netlink_stats_updates;
      }

      // Check interface flags to determine the link state. IFF_UP does not
      // toggle when the access switch changes state. Use IFF_LOWER_UP instead.
      uint32_t new_if_flags;
      if (NetlinkGetInterfaceFlags(nl_len, nl_msg, client->netlink_if_index,
                                   &new_if_flags)) {
        // Clear valid when link goes down to force reacquisition of if_index
        // since the kernel may increment if_index when the interface bounces.
        client->netlink_if_index_valid = (new_if_flags & IFF_LOWER_UP) != 0x0;

        // Reopen the socket when the interface changes from down to up.
        if (client->netlink_if_flags_valid
            && (client->netlink_if_flags & IFF_LOWER_UP) == 0x0
            && (new_if_flags & IFF_LOWER_UP) != 0x0) {
          ReopenAioSocket(config, &client->aio_fd, &client->stats);
        }
        client->netlink_if_flags = new_if_flags;
        client->netlink_if_flags_valid = client->netlink_if_index_valid;
        ++client->stats.netlink_interface_updates;
      }
    }

    // Reopen the socket for route changes. This strategy handles the situation
    // where AioClient does not have a valid IP address for the network
    // interface (e.g., for host computers and monitors).
    IpAddress dst = AioMessageTypeToIpAddress(0);
    if (NetlinkCheckRouteUpdateByDst(nl_len, nl_msg, &dst)) {
      ReopenAioSocket(config, &client->aio_fd, &client->stats);
      ++client->stats.netlink_route_updates;
    }
  }
}

// Handle netlink error.

static void HandleNetlinkError(AioClient *client) {
  NetlinkSocketClose(client->netlink_fd);
  client->netlink_fd = -1;
  client->netlink_if_index_valid = false;
  client->netlink_if_flags_valid = false;

  ++client->stats.netlink_socket_errors;
}

// Handle I/O.

static void SelectFdSets(int32_t fd, fd_set *read_set, fd_set *error_set,
                         int32_t *nfds) {
  if (fd >= 0) {
    if (fd >= *nfds) {
      *nfds = fd + 1;
    }
    FD_SET(fd, read_set);
    FD_SET(fd, error_set);
  }
}

static int32_t WaitOnSelect(int32_t nfds, fd_set *read_set, fd_set *error_set,
                            struct timeval *timeout, int64_t *wake_time,
                            AioClient *client) {
  // Measure idle time.
  int64_t sleep_time = ClockGetUs();

  // Wait until update or timeout.
  int32_t rc = select(nfds, read_set, NULL, error_set, timeout);
  if (rc < 0 && errno == EINTR) {
    rc = 0;  // Ignore these errors.
  }

  // Call after checking errno since clock_gettime() may update errno.
  *wake_time = ClockGetUs();
  client->stats.idle_time += (int32_t)(*wake_time - sleep_time);

  if (rc < 0) {
    ++client->stats.select_errors;
  }
  return rc;
}

static int32_t HandleAioSelectUpdate(const AioSocketConfig *config,
                                     const fd_set *read_set,
                                     const fd_set *error_set,
                                     AioClient *client) {
  int32_t rc = 0;
  if (client->aio_fd >= 0) {
    if (FD_ISSET(client->aio_fd, read_set)) {
      rc = 1;
    }
    if (FD_ISSET(client->aio_fd, error_set)) {
      // Reopening the socket clears any receive messages indicated in read_set.
      ReopenAioSocket(config, &client->aio_fd, &client->stats);
      ++client->stats.aio_socket_errors;
      rc = -1;
    }
  }
  return rc;
}

static void HandleNetlinkSelectUpdate(const AioSocketConfig *config,
                                      const fd_set *read_set,
                                      const fd_set *error_set,
                                      AioClient *client) {
  if (client->netlink_fd >= 0) {
    if (FD_ISSET(client->netlink_fd, read_set)) {
      ReceiveNetlinkMessage(config, client);
    }
    if (FD_ISSET(client->netlink_fd, error_set)) {
      HandleNetlinkError(client);
    }
  }
}

static int32_t WaitForMessage(const AioSocketConfig *config,
                              struct timeval *timeout, int64_t *timestamp,
                              AioClient *client) {
  fd_set read_set, error_set;
  FD_ZERO(&read_set);
  FD_ZERO(&error_set);

  int32_t nfds = 0;
  SelectFdSets(client->aio_fd, &read_set, &error_set, &nfds);
  SelectFdSets(client->netlink_fd, &read_set, &error_set, &nfds);

  int32_t rc = WaitOnSelect(nfds, &read_set, &error_set, timeout, timestamp,
                            client);
  if (rc > 0) {
    HandleNetlinkSelectUpdate(config, &read_set, &error_set, client);
    rc = HandleAioSelectUpdate(config, &read_set, &error_set, client);
  }
  return rc;
}

static void QueryNetlink(AioClient *client) {
  if (client->netlink_fd < 0) {
    client->netlink_if_index_valid = false;
    client->netlink_if_flags_valid = false;
    client->netlink_fd = NetlinkSocketOpen();
  }
  if (client->netlink_fd >= 0) {
    if (!client->netlink_if_index_valid) {
      NetlinkSendGetAddr(client->netlink_fd, 0U);
    } else {
      NetlinkSendGetLink(client->netlink_fd, 0U);
    }
  }
}

static int32_t WaitUntilTimeoutUs(const AioSocketConfig *config,
                                  int64_t timeout_usec, int64_t *timestamp,
                                  AioClient *client) {
  // Limit the rate at which we query the netlink interface to prevent the
  // kernel from reporting netlink error messages.
  int64_t now = ClockGetUs();
  if (now - client->netlink_query_time > NETLINK_QUERY_PERIOD_US) {
    QueryNetlink(client);
    client->netlink_query_time = now;
  }

  // Set timeout to query netlink statistics. The kernel notifies us
  // immediately upon changes to the interface.
  int64_t netlink_next = client->netlink_query_time + NETLINK_QUERY_PERIOD_US;
  int64_t netlink_timeout = netlink_next - now;
  assert(0 <= netlink_timeout && netlink_timeout <= NETLINK_QUERY_PERIOD_US);

  // Determine end time.
  int64_t min_timeout = (timeout_usec >= 0 && timeout_usec < netlink_timeout) ?
      timeout_usec : netlink_timeout;

  struct timeval timeout_tv = {
    .tv_sec = (int32_t)(min_timeout / 1000000),
    .tv_usec = (int32_t)(min_timeout % 1000000)
  };
  return WaitForMessage(config, &timeout_tv, timestamp, client);
}

// Returns: false=when on_new_message returns false, otherwise true. Failures
// from WaitUntilTimeoutUs() and ReceiveAioMessage() return true.
static bool PollMessage(const AioSocketConfig *config, int64_t timeout,
                        AioClientOnNewMessageFunction on_new_message,
                        void *arg, AioClient *client) {
  int64_t timestamp;
  if (WaitUntilTimeoutUs(config, timeout, &timestamp, client) > 0) {
    AioHeader header;
    uint8_t data[UDP_MAX_LENGTH], *payload;
    int32_t payload_length;

    if (ReceiveAioMessage(config, client->aio_fd, timestamp,
                          (int32_t)sizeof(data), data, &header,
                          &payload_length, &payload, &client->input,
                          &client->stats) > 0) {
      return on_new_message == NULL
          || on_new_message(timestamp, &header, payload_length, payload, arg);
    }
  }
  return true;  // Continue.
}

// Returns: false=when on_new_message returns false, otherwise true. Failures
// internal to PollMessage(), such as from WaitUntilTimeoutUs() and
// ReceiveAioMessage(), return true.
static bool PollUntilEndTime(const AioSocketConfig *config, int64_t end_time,
                             AioClientOnNewMessageFunction on_new_message,
                             void *arg, AioClient *client) {
  bool cont = true;
  int64_t remaining_time;
  while (cont && (remaining_time = end_time - ClockGetUs()) > 0) {
    cont = PollMessage(config, remaining_time, on_new_message, arg, client);
  }
  return cont;  // Continue.
}

static void PollWithPeriodicFunction(
    const AioSocketConfig *config, AioClientOnNewMessageFunction on_new_message,
    void *on_new_message_arg, AioClientPeriodicFunction periodic,
    void *periodic_arg, int64_t periodic_interval_usec, AioClient *client) {
  assert(periodic != NULL);
  assert(periodic_interval_usec > 0);

  // We do not want to terminate this loop on error. PollUntilEndTime
  // should re-establish a connection if an error occurs.
  int64_t exec_time = ClockGetUs();
  while (PollUntilEndTime(config, exec_time, on_new_message, on_new_message_arg,
                          client) && periodic(periodic_arg)) {
    exec_time += periodic_interval_usec;
  }
}

static void PollWithoutPeriodicFunction(
    const AioSocketConfig *config, AioClientOnNewMessageFunction on_new_message,
    void *on_new_message_arg, AioClient *client) {

  // We do not want to terminate this loop on error. PollMessage should
  // re-establish a connection if an error occurs.
  while (PollMessage(config, -1, on_new_message, on_new_message_arg, client)) {
  }
}

// Handle statistics.

static void GetStats(AioClient *client, AioClientStats *stats) {
  *stats = client->stats;
  memset(&client->stats, 0, sizeof(client->stats));

  int64_t now = ClockGetUs();
  stats->total_time = (int32_t)(now - client->stats_time);
  client->stats_time = now;
}

// Application interface.

int32_t AioClientMain(const AioSocketConfig *config, AioNode source,
                      AioClientOnNewMessageFunction on_new_message,
                      void *on_new_message_arg,
                      AioClientPeriodicFunction periodic, void *periodic_arg,
                      int64_t periodic_interval_usec, AioClient *client) {
  AioClientInit(source, client);
  if (AioClientOpen(config, client) >= 0) {
    AioClientLoop(config, on_new_message, on_new_message_arg, periodic,
                  periodic_arg, periodic_interval_usec, client);
    return AioClientClose(client);
  }
  return -1;
}

void AioClientInit(AioNode source, AioClient *client) {
  assert(IsValidSource(source));
  assert(client != NULL);

  memset(client, 0, sizeof(*client));
  client->source = source;
  client->ip = AioNodeToIpAddress(source);
  client->aio_fd = -1;
  client->netlink_fd = -1;
  client->stats_time = ClockGetUs();
}

int32_t AioClientOpen(const AioSocketConfig *config, AioClient *client) {
  assert(config != NULL);
  assert(client != NULL);

  return ReopenAioSocket(config, &client->aio_fd, &client->stats) ? 0 : -1;
}

int32_t AioClientClose(AioClient *client) {
  assert(client != NULL);

  int32_t aio_fd = client->aio_fd;
  int32_t netlink_fd = client->netlink_fd;

  client->aio_fd = -1;
  client->netlink_fd = -1;

  bool error = false;
  error |= AioSocketClose(aio_fd) < 0;
  error |= NetlinkSocketClose(netlink_fd) < 0;

  return error ? -1 : 0;
}

int32_t AioClientWaitForMessage(const AioSocketConfig *config, int64_t timeout,
                                int64_t *timestamp, AioClient *client) {
  assert(config != NULL);
  assert(timestamp != NULL);
  assert(client != NULL);

  return WaitUntilTimeoutUs(config, timeout, timestamp, client);
}

int32_t AioClientReceive(const AioSocketConfig *config, int64_t timestamp,
                         int32_t length, uint8_t *data, AioHeader *header,
                         int32_t *payload_length, uint8_t **payload,
                         AioClient *client) {
  assert(config != NULL);
  assert(length > 0);
  assert(data != NULL);
  assert(header != NULL);
  assert(payload_length != NULL);
  assert(payload != NULL);
  assert(client != NULL);

  return ReceiveAioMessage(config, client->aio_fd, timestamp, length, data,
                           header, payload_length, payload, &client->input,
                           &client->stats);
}

void AioClientLoop(const AioSocketConfig *config,
                   AioClientOnNewMessageFunction on_new_message,
                   void *on_new_message_arg, AioClientPeriodicFunction periodic,
                   void *periodic_arg, int64_t periodic_interval_usec,
                   AioClient *client) {
  assert(config != NULL);
  assert((periodic == NULL) ^ (periodic_interval_usec > 0));
  assert(client != NULL);

  if (periodic_interval_usec > 0) {
    PollWithPeriodicFunction(config, on_new_message, on_new_message_arg,
                             periodic, periodic_arg, periodic_interval_usec,
                             client);
  } else {
    PollWithoutPeriodicFunction(config, on_new_message, on_new_message_arg,
                                client);
  }
}

void AioClientGetStats(AioClient *client, AioClientStats *stats) {
  assert(client != NULL);
  assert(stats != NULL);

  GetStats(client, stats);
}

void AioClientSendSpoofed(const AioSocketConfig *config,
                          const AioHeader *header, int32_t payload_length,
                          const uint8_t *payload, AioClient *client) {
  assert(config != NULL);
  assert(header != NULL);
  assert(payload_length >= 0 && payload != NULL);
  assert(client != NULL);

  SendPacked(config, header, payload_length, payload, client->aio_fd,
             &client->stats);
}

void AioClientSendPacked(const AioSocketConfig *config, MessageType type,
                         int32_t payload_length, const uint8_t *payload,
                         AioClient *client) {
  assert(config != NULL);
  assert(payload_length >= 0 && payload != NULL);
  assert(client != NULL);

  AioHeader header;
  SetAioHeader(client->source, type, &client->output, &header);
  SendPacked(config, &header, payload_length, payload, client->aio_fd,
             &client->stats);
}

void AioClientSendAioMessageData(const AioSocketConfig *config,
                                 MessageType type, const AioMessageData *data,
                                 AioClient *client) {
  assert(config != NULL);
  assert(data != NULL);
  assert(client != NULL);

  AioHeader header;
  SetAioHeader(client->source, type, &client->output, &header);
  SendAioMessageData(config, &header, data, client->aio_fd, &client->stats);
}

void AioClientPrintf(const AioSocketConfig *config, AioClient *client,
                     const char *format, ...) {
  assert(config != NULL);
  assert(client != NULL);
  assert(format != NULL);

  va_list ap;
  va_start(ap, format);
  AioClientVprintf(config, client, format, ap);
  va_end(ap);
}

void AioClientVprintf(const AioSocketConfig *config, AioClient *client,
                      const char *format, va_list ap) {
  assert(config != NULL);
  assert(client != NULL);
  assert(format != NULL);

  AioHeader header;
  SetAioHeader(client->source, kMessageTypeStdio, &client->output, &header);
  SendVprintf(config, &header, format, ap, client->aio_fd, &client->stats);
}
