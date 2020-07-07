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

#ifndef AVIONICS_LINUX_AIO_CLIENT_H_
#define AVIONICS_LINUX_AIO_CLIENT_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/aio_header.h"
#include "avionics/common/network_addresses.h"
#include "avionics/linux/aio_client_stats.h"
#include "avionics/linux/aio_socket.h"
#include "avionics/linux/netlink_socket.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_info.h"
#include "avionics/network/message_type.h"

#ifdef __cplusplus
extern "C" {
#endif

// Callback function to execute upon reception of a new message. When the
// callback function returns false, AioClientLoop terminates.
typedef bool (* const AioClientOnNewMessageFunction)(
    int64_t timestamp, const AioHeader *header, int32_t payload_length,
    const uint8_t *payload, void *arg);

// Callback function to execute on a periodic interval. When the callback
// function returns false, AioClientLoop terminates.
typedef bool (* const AioClientPeriodicFunction)(void *arg);

// Store input message state.
typedef struct {
  AioDeduplicate dedup[kNumAioNodes][kNumMessageTypes];
} AioClientInput;

// Store output message state.
typedef struct {
  uint16_t seq_num[kNumMessageTypes];
} AioClientOutput;

// Client state.
typedef struct {
  // AIO interface.
  AioClientInput input;
  AioClientOutput output;
  AioClientStats stats;
  AioNode source;
  IpAddress ip;
  int32_t aio_fd;      // AIO socket file descriptor.
  int64_t stats_time;  // Used to compute total_time in AioClientStats.

  // Netlink interface.
  int32_t netlink_fd;           // Netlink socket file descriptor.
  int64_t netlink_query_time;   // Time of last netlink interface query [us].
  bool netlink_if_index_valid;  // Indicates netlink_if_index is valid.
  int32_t netlink_if_index;     // AIO interface index.
  bool netlink_if_flags_valid;  // Indicates netlink_if_flags is valid.
  uint32_t netlink_if_flags;    // AIO interface flags.
} AioClient;

// Initialize and execute AioClient. This function calls AioClientInit, opens
// the socket, and then calls AioClientLoop. It closes the socket and terminates
// after AioClientLoop terminates.
// Returns: -1=error, 0=success.
int32_t AioClientMain(const AioSocketConfig *config, AioNode source,
                      AioClientOnNewMessageFunction on_new_message,
                      void *on_new_message_arg,
                      AioClientPeriodicFunction periodic, void *periodic_arg,
                      int64_t periodic_interval_usec, AioClient *client);

// Initialize AioClient. Call this function first.
void AioClientInit(AioNode source, AioClient *client);

// Open the AIO network interface. Call this function before calling the
// functions below.
// Returns: -1=error, 0=success.
int32_t AioClientOpen(const AioSocketConfig *config, AioClient *client);
int32_t AioClientClose(AioClient *client);

// Wait until next message arrives. Call AioClientReceive() to obtain the new
// message from the kernel.
// Returns: -1=error, 0=no update, >0=update.
int32_t AioClientWaitForMessage(const AioSocketConfig *config, int64_t timeout,
                                int64_t *timestamp, AioClient *client);

// Obtain the next pending message from the kernel without blocking.
// Returns: -1=error, 0=no update, >0=update.
int32_t AioClientReceive(const AioSocketConfig *config, int64_t timestamp,
                         int32_t length, uint8_t *data, AioHeader *header,
                         int32_t *payload_length, uint8_t **payload,
                         AioClient *client);

// Poll repeatedly. This function calls on_new_message(..., on_new_message_arg)
// for each new message and periodic(periodic_arg) on periodic_interval_usec.
// This function terminates when on_new_message or periodic returns false.
void AioClientLoop(const AioSocketConfig *config,
                   AioClientOnNewMessageFunction on_new_message,
                   void *on_new_message_arg, AioClientPeriodicFunction periodic,
                   void *periodic_arg, int64_t periodic_interval_usec,
                   AioClient *client);

// Get AioClient statistics and reset counts. Repeated calls return zero.
void AioClientGetStats(AioClient *client, AioClientStats *stats);

// Send an AIO message given an arbitrary header and packed data. This function
// allows applications to spoof messages from another source and increment
// sequence numbers arbitrarily.
void AioClientSendSpoofed(const AioSocketConfig *config,
                          const AioHeader *header, int32_t payload_length,
                          const uint8_t *payload, AioClient *client);

// Send an AIO message given packed data.
void AioClientSendPacked(const AioSocketConfig *config, MessageType type,
                         int32_t payload_length, const uint8_t *payload,
                         AioClient *client);

// Send an AIO message given AioMessageData.
void AioClientSendAioMessageData(const AioSocketConfig *config,
                                 MessageType type, const AioMessageData *data,
                                 AioClient *client);

// Send kMessageTypeStdio.
void AioClientPrintf(const AioSocketConfig *config, AioClient *client,
                     const char *format, ...)
    __attribute__((format(printf, 3, 4)));
void AioClientVprintf(const AioSocketConfig *config, AioClient *client,
                      const char *format, va_list ap)
    __attribute__((format(printf, 3, 0)));

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_LINUX_AIO_CLIENT_H_
