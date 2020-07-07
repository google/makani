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

#include "avionics/bootloader/firmware/update_server.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/bootloader/firmware/flash.h"
#include "avionics/common/crc.h"
#include "avionics/common/endian.h"
#include "avionics/common/network_config.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/identity/identity_types.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/startup/ldscript.h"

#define PROTO_MAJOR 0
#define PROTO_MINOR 1
#define STARTUP_TIMEOUT_US  (1500 * 1000)       // 1500ms
#define RESPONSE_TIMEOUT_US (200 * 1000)        // 200ms
#define CLIENT_TIMEOUT_US   (60 * 1000 * 1000)  // 1 minute

static int32_t g_bytes_received = 0;

typedef struct {
  uint8_t message_id;      // kPacketIdStart.
  uint8_t protocol_major;  // PROTO_MAJOR or higher.
  uint8_t protocol_minor;  // PROTO_MINOR or higher.
  uint8_t update_type;     // UpdateType.
  int32_t binary_size;     // The size of the image to be transferred.
} __attribute__((__packed__)) StartMessage;

typedef struct {
  uint8_t message_id;      // kPacketIdReady.
  uint8_t protocol_major;  // PROTO_MAJOR.
  uint8_t protocol_minor;  // PROTO_MINOR.
  int8_t unused;           // Previously used as AppType.
  uint8_t unused_2;        // Previously used as board index.
  int8_t hardware_type;    // HardwareType from identity.h.
} __attribute__((__packed__)) ReadyMessage;

typedef struct {
  uint8_t message_id;  // kPacketIdData.
  int32_t offset;      // Bytes previously transferred.
  uint8_t final;       // Is this the last packet ? 1 : 0.
  uint8_t data[];
} __attribute__((__packed__)) DataMessage;

typedef struct {
  uint8_t message_id;       // kPacketIdDataReceived.
  int32_t bytes_received;   // Total bytes transferred so far.
  uint8_t final;            // Was that the last packet.
} __attribute__((__packed__)) DataReceivedMessage;

typedef union {
  unsigned char raw[MAX_UDP_PAYLOAD_SIZE];
  DataMessage data_message;
  StartMessage start_message;
} RxBuffer;

typedef enum {
  kHostAndPortRestrict,
  kHostAndPortQuery
} HostAndPortOption;

// Listen for a message on my_port, timing out if it takes more than timeout_us.
// Returns the length of the packet received on success, 0 on error.
// Callers should either set option to kHostAndPortRestrict, which requires that
// the received packet also match a supplied source host_address and host_port,
// or set it to kHostAndPortQuery and receive the host_address and host_port as
// output parameters.
static int32_t ReceiveFromPort(uint16_t my_port, int64_t timeout_us,
                               HostAndPortOption option, int32_t buffer_len,
                               void *buffer, IpAddress *host_address,
                               EthernetAddress *host_mac,
                               uint16_t *host_port) {
  int64_t stop_time = ClockGetUs() + timeout_us;
  assert(host_address);
  assert(host_port);
  do {
    ExtWatchdogPoll();
    IpAddress source_address;
    EthernetAddress source_mac;
    uint16_t source_port, destination_port;
    int32_t packet_len = NetPollUdp(
        buffer_len, buffer, &source_address, &source_mac, &source_port,
        &destination_port);
    if (packet_len > 0) {
      if (packet_len > buffer_len) {
        printf("Overflowed our buffer!\n");
        return 0;
      }
      if (destination_port == my_port) {
        if (option == kHostAndPortRestrict) {
          if (memcmp(host_address, &source_address, sizeof(IpAddress)) ||
              *host_port != source_port) {
            continue;
          }
        } else {
          assert(option == kHostAndPortQuery);
          *host_address = source_address;
          *host_mac = source_mac;
          *host_port = source_port;
        }
        return packet_len;
      }
    }
  } while (ClockGetUs() < stop_time);
  return 0;
}

// Returns true if the supplied packet is the next data packet we're expecting,
// based on g_bytes_received.
static bool IsNextDataPacket(const RxBuffer *rx_buffer, int32_t packet_len) {
  if (rx_buffer->data_message.message_id != kPacketIdData) {
    printf("Bad code: %d vs. %d.\n", rx_buffer->data_message.message_id,
           kPacketIdData);
    return false;
  }
  if (packet_len <= (int32_t)sizeof(DataMessage)) {
    printf("Packet_len too short: %ld.\n", packet_len);
    return false;
  }
  int32_t offset = 0;
  ReadInt32Be(&rx_buffer->data_message.offset, &offset);
  if (offset != g_bytes_received) {
    return false;
  }
  return true;
}

// Sends packet and listens for a response, resending it periodically until the
// right response comes back.  Returns the length of the packet received, or -1
// on timeout.
static int32_t SendPacket(
    void *packet, int32_t packet_len, int64_t client_timeout_us,
    RxBuffer *rx_buffer, IpAddress host_address, EthernetAddress host_mac,
    uint16_t host_port) {
  int64_t stop_time = ClockGetUs() + client_timeout_us;
  do {
    ExtWatchdogPoll();
    if (!NetSendUdp(host_address, host_mac, UDP_PORT_BOOTLOADER, host_port,
                    packet, packet_len, NULL)) {
      continue;
    }

    int i;
    // Try up to 5 times to get a response before resending the packet.  This
    // helps cut down on the stale packets in the system.
    // Wait up to RESPONSE_TIMEOUT_US for the first incoming packet.
    int64_t timeout_us = RESPONSE_TIMEOUT_US;
    for (i = 0; i < 5; ++i) {
      int32_t count = ReceiveFromPort(UDP_PORT_BOOTLOADER, timeout_us,
                                      kHostAndPortRestrict, sizeof(RxBuffer),
                                      rx_buffer->raw, &host_address, &host_mac,
                                      &host_port);
      if (count <= 0) {
        break;
      }
      if (IsNextDataPacket(rx_buffer, count)) {
        return count;
      }
      // After the first packet we get back, just handle whatever's already
      // queued up, don't wait.
      timeout_us = 0;
    }
  } while (ClockGetUs() < stop_time);
  return -1;  // Timeout
}

// Prepares the message that acknowledges receipt of the client's start message.
static void InitReadyMessage(
    ReadyMessage *message, HardwareType hardware_type,
    uint8_t proto_high, uint8_t proto_low) {
  message->message_id = kPacketIdReady;
  message->protocol_major = proto_high;
  message->protocol_minor = proto_low;
  message->hardware_type = hardware_type;
  message->unused = -1;  // Set to deprecated kAppTypeUnknown.
  message->unused_2 = -1;
}

// Prepares the message that acknowledges receipt of the client's data message.
static void InitDataReceivedMessage(
    DataReceivedMessage *message, uint8_t done) {
  message->message_id = kPacketIdDataReceived;
  WriteInt32Be(g_bytes_received, &message->bytes_received);
  message->final = done;
}

static FlashUpdateType FlashUpdateTypeFromUpdateType(UpdateType update_type) {
  switch (update_type) {
    case kUpdateTypeBootloader:
      return kFlashUpdateTypeBootloader;
    case kUpdateTypeApplication:
      return kFlashUpdateTypeApplication;
    case kUpdateTypeConfigParams:
      return kFlashUpdateTypeConfigParams;
    case kUpdateTypeCalibParams:
      return kFlashUpdateTypeCalibParams;
    case kUpdateTypeSerialParams:
      return kFlashUpdateTypeSerialParams;
    case kUpdateTypeCarrierSerialParams:
      return kFlashUpdateTypeCarrierSerialParams;
    default:
      return kFlashUpdateTypeInvalid;
  }
}

// Listens on port for a start message from a bootloader client, validating the
// protocol it speaks and reporting the size of the update to come if one is
// found.
static bool GetUpdateClient(uint16_t port, RxBuffer *rx_buffer,
                            int32_t *update_size, UpdateType *update_type,
                            IpAddress *host_address, EthernetAddress *host_mac,
                            uint16_t *host_port) {
  int32_t count = ReceiveFromPort(port, STARTUP_TIMEOUT_US, kHostAndPortQuery,
                                  sizeof(RxBuffer), rx_buffer->raw,
                                  host_address, host_mac, host_port);
  if (count <= 0) {
    return false;
  }

  if (count < (int32_t)sizeof(StartMessage)) {
    printf("Failed to get an update client; count was %ld.\n", count);
    return false;
  }
  StartMessage *message = &rx_buffer->start_message;
  if (message->message_id != kPacketIdStart) {
    printf("Failed to get an update client; message id was %d.\n",
           message->message_id);
    return false;
  }
  if (message->protocol_major != PROTO_MAJOR ||
      (message->protocol_minor < PROTO_MINOR && message->protocol_minor != 0)) {
    // We assume that any bootloader client newer than us will know more about
    // whether it can talk to us than we do.  We don't support talking to older
    // clients, but the current client will lie and say it's (0,0) so that older
    // servers won't freak out; we'll send them our true version and see if they
    // accept it.  We define that a change to PROTO_MAJOR means we can't talk to
    // each other at all.
    printf("Unrecognized protocol version %d %d.\n",
           message->protocol_major, message->protocol_minor);
    return false;
  }
  if (FlashUpdateTypeFromUpdateType(message->update_type)
      == kFlashUpdateTypeInvalid) {
    printf("Unrecognized update type: %d.\n", message->update_type);
    return false;
  }
  if (IsRunningFromBootSegment() &&
      message->update_type == kUpdateTypeBootloader) {
    printf("Can't burn the bootloader while running from it.\n");
    return false;
  }
  if (!IsRunningFromBootSegment() &&
      message->update_type == kUpdateTypeApplication) {
    printf("Can't burn the application while running from it.\n");
    return false;
  }
  ReadInt32Be(&message->binary_size, update_size);
  *update_type = (UpdateType)message->update_type;
  printf("Got an update client.\n");
  return true;
}

static bool ValidateExecutable(uint32_t start, uint32_t size) {
  uint32_t crc = Crc32(0U, size - 4, (const void *)start);
  uint32_t stored_crc = *(const uint32_t *)(start + size - 4);
  return crc == stored_crc;
}

// Pad short updates to flash address length to prevent ECC errors.
static bool PadToAddressLength(UpdateType update_type, int32_t offset,
                               int32_t data_len, uint8_t *data) {
  int32_t addr_len = GetFlashAddressLength(update_type);
  for (int32_t pos = offset; pos < addr_len; pos += FLASH_BANK_WIDTH) {
    uint8_t buffer[FLASH_BANK_WIDTH];
    int32_t data_remaining = offset + data_len - pos;
    int32_t data_offset = data_len - data_remaining;
    if (data_remaining <= 0) {
      memset(buffer, 0x0, sizeof(buffer));
    } else if (data_remaining < FLASH_BANK_WIDTH) {
      memcpy(buffer, &data[data_offset], data_remaining);
      memset(&buffer[data_remaining], 0x0, FLASH_BANK_WIDTH - data_remaining);
    } else {
      memcpy(buffer, &data[data_offset], FLASH_BANK_WIDTH);
    }
    if (!FlashWrite(update_type, pos, FLASH_BANK_WIDTH, buffer)) {
      return false;
    }
  }
  return true;
}

// The flow of UpdateServer() is:
//
// Listen for an update client for RESPONSE_TIMEOUT_US.
// If you don't get one, return.
// If you do, the client has just told you how big an update to expect.
// Find where in flash it will live [the segments allocated to the bootloader or
// to the application].
// Erase enough flash to hold it.
// While there are more packets to receive:
//   Receive a data packet.
//   Burn it to flash.
//   Acknowledge receipt of the packet, prompting the client to send the next.

bool UpdateServer(HardwareType hardware_type) {
  int32_t update_size;
  UpdateType update_type;
  static RxBuffer rx_buffer;
  IpAddress host_address;
  EthernetAddress host_mac;
  uint16_t host_port;
  if (!GetUpdateClient(UDP_PORT_BOOTLOADER, &rx_buffer, &update_size,
                       &update_type, &host_address, &host_mac, &host_port)) {
    printf("[%s] Got no update client.\n",
           IsRunningFromBootSegment() ? "bootloader" : "bootloader app");
    return false;
  }

  ReadyMessage ready_message;
  InitReadyMessage(&ready_message, hardware_type, PROTO_MAJOR, PROTO_MINOR);
  int32_t count = SendPacket(&ready_message, sizeof(ReadyMessage),
                             CLIENT_TIMEOUT_US, &rx_buffer, host_address,
                             host_mac, host_port);
  if (count < 0) {
    printf("Timed out while waiting for the first data packet.\n");
    return false;
  }
  FlashUpdateType flash_update_type =
      FlashUpdateTypeFromUpdateType(update_type);
  FlashInit(flash_update_type);
  printf("Erasing flash.\n");
  if (!FlashEraseSectors(flash_update_type)) {
    return false;
  }

  int16_t data_len = count - sizeof(DataMessage);
  DataMessage *message = &rx_buffer.data_message;

  printf("Writing flash.\n");
  if (message->final) {
    if (!PadToAddressLength(flash_update_type, g_bytes_received, data_len,
                            message->data)) {
      return false;
    }
  } else if (!FlashWrite(flash_update_type, g_bytes_received, data_len,
                         message->data)) {
    return false;
  }

  g_bytes_received += data_len;
  uint8_t done = (g_bytes_received == update_size);
  assert(done == message->final);

  DataReceivedMessage data_received_message;
  while (g_bytes_received < update_size) {
    InitDataReceivedMessage(&data_received_message, done);

    count = SendPacket(&data_received_message, sizeof(DataReceivedMessage),
                       CLIENT_TIMEOUT_US, &rx_buffer, host_address, host_mac,
                       host_port);
    if (count < 0) {
      printf("Timed out while waiting for the next data packet.\n");
      return false;
    }
    data_len = count - sizeof(DataMessage);

    if (message->final) {
      if (!PadToAddressLength(flash_update_type, g_bytes_received, data_len,
                              message->data)) {
        return false;
      }
    } else if (!FlashWrite(flash_update_type, g_bytes_received, data_len,
                           message->data)) {
      return false;
    }

    g_bytes_received += data_len;
    done = message->final;
    assert((g_bytes_received == update_size) == !!done);
  }
  assert(g_bytes_received == update_size);
  assert(done);

  InitDataReceivedMessage(&data_received_message, done);
  // Send the last packet a few times, just in case, since there's no handshake
  // to confirm that it got there.
  NetSendUdp(host_address, host_mac, UDP_PORT_BOOTLOADER, host_port,
             &data_received_message, sizeof(DataReceivedMessage), NULL);
  NetSendUdp(host_address, host_mac, UDP_PORT_BOOTLOADER, host_port,
             &data_received_message, sizeof(DataReceivedMessage), NULL);
  NetSendUdp(host_address, host_mac, UDP_PORT_BOOTLOADER, host_port,
             &data_received_message, sizeof(DataReceivedMessage), NULL);

  if (update_type == kUpdateTypeApplication ||
      update_type == kUpdateTypeBootloader) {
    uint32_t address = GetFlashWriteAddress(update_type, 0);
    if (!ValidateExecutable(address, update_size)) {
      printf("Checksum failed!\n");
      fflush(stdout);
      return false;
    }
  }

  printf("Flash burning complete!\n");
  fflush(stdout);
  return true;
}
