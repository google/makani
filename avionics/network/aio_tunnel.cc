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

// Tunnel AIO traffic.

#include <arpa/inet.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <snappy.h>

#include <cstring>
#include <list>

#include "avionics/common/aio_version_code.h"
#include "avionics/common/network_config.h"
#include "avionics/common/pack_aio_header.h"
#include "avionics/linux/aio_socket.h"
#include "avionics/linux/clock.h"
#include "common/macros.h"

const uint16_t kTunnelVersion = 0x1113;
const int32_t kUdpMaxLength = 65000;  // Account for IP/UDP headers.

const int32_t kHeartbeatPeriodUs = 2 * 1000 * 1000;
const int32_t kHeartbeatTimeoutUs = 11 * 1000 * 1000;

// TODO: This code needs to be rewritten using C++ classes.
// TODO: Support hostname resolution.

DEFINE_bool(client, false, "Send remote AIO traffic out over multicast.");
DEFINE_bool(repeater, false, "Forward AIO traffic from a server or repeater.");
DEFINE_bool(server, false, "Originate AIO traffic from multicast.");
DEFINE_int32(port, UDP_PORT_AIO_TUNNEL, "Local port.");
DEFINE_int32(sport, UDP_PORT_AIO_TUNNEL, "Remote (server) port.");
DEFINE_string(source, "", "Address of remote server or repeater.");
DEFINE_string(filter, "", "Filter message types (comma separated).");
DEFINE_bool(compress, true, "Compress AIO traffic in transit.");
DEFINE_bool(forge_aio_version, false, "Rewrite AIO version to match compiled "
            "version. (this may cause messages to be invalid)");

namespace {

enum TunnelMessageType {
  kTunnelMessageTypeHeartbeat,
  kTunnelMessageTypeHeartbeatAck,
  kTunnelMessageTypeAio,
  kTunnelMessageTypeAioCompressed,
};

struct TunnelMessage {
  int16_t version;
  int8_t type;
  int8_t reserved_padding;
} __attribute__((packed));

static_assert(kNumMessageTypes <= 256, "kNumMessageTypes is greater than 256.");
struct SelectTypes {
  uint8_t mask[32];
} __attribute__((packed));

struct TunnelHeartbeatMessage {
  TunnelMessage header;
  SelectTypes select_types;
} __attribute__((packed));

struct ClientConnection {
  struct sockaddr_in socket;
  int64_t update_time;
  SelectTypes select_types;
};

int32_t g_aio_sock_fd;
int32_t g_tunnel_sock_fd;
AioSocketConfig g_aio_sock_config;
SelectTypes g_select_types;
std::list<ClientConnection> g_clients;
struct sockaddr_in g_server;
int64_t g_server_update_time;
bool g_server_connected;

uint8_t *GetTunnelMessagePayload(TunnelMessage *message) {
  uint8_t *data = reinterpret_cast<uint8_t *>(message);
  return &data[sizeof(*message)];
}

void SelectMessage(SelectTypes *select, MessageType message_type) {
  select->mask[message_type / 8] |=
      static_cast<uint8_t>(1 << (message_type % 8));
}

bool IsMessageSelected(const SelectTypes *select, MessageType message_type) {
  return (select->mask[message_type / 8] & (1 << (message_type % 8))) != 0;
}

bool CompressAioMessage(const uint8_t *aio_message, int32_t aio_message_size,
                        uint8_t *compressed_data,
                        int32_t max_compressed_size,
                        int32_t *compressed_size) {
  memcpy(compressed_data, aio_message, sizeof(AioHeader));
  if (max_compressed_size - sizeof(AioHeader) <
      snappy::MaxCompressedLength(aio_message_size - sizeof(AioHeader))) {
    LOG(ERROR) << "Failed to compress AIO message due to insufficient buffer "
               << "size.";
    return false;
  }
  size_t compressed_size_t;
  snappy::RawCompress(
      reinterpret_cast<const char*>(aio_message + sizeof(AioHeader)),
      aio_message_size - sizeof(AioHeader),
      reinterpret_cast<char*>(compressed_data + sizeof(AioHeader)),
      &compressed_size_t);
  *compressed_size = static_cast<int32_t>(compressed_size_t
                                          + sizeof(AioHeader));
  return true;
}

bool UncompressAioMessage(const uint8_t *compressed_data,
                          int32_t compressed_data_size,
                          uint8_t *aio_message, int32_t max_uncompressed_size,
                          int32_t *uncompressed_size) {
  memcpy(aio_message, compressed_data, sizeof(AioHeader));
  size_t uncompressed_size_t;
  if (!snappy::GetUncompressedLength(
          reinterpret_cast<const char*>(compressed_data + sizeof(AioHeader)),
          compressed_data_size - sizeof(AioHeader), &uncompressed_size_t)) {
    return false;
  }
  if (uncompressed_size_t > max_uncompressed_size - sizeof(AioHeader)) {
    return false;
  }
  if (!snappy::RawUncompress(
          reinterpret_cast<const char*>(compressed_data + sizeof(AioHeader)),
          compressed_data_size - sizeof(AioHeader),
          reinterpret_cast<char*>(aio_message + sizeof(AioHeader)))) {
    return false;
  }
  *uncompressed_size = static_cast<int32_t>(uncompressed_size_t
                                            + sizeof(AioHeader));
  return true;
}

int32_t OpenAioSocket(void) {
  MessageType types[kNumMessageTypes];
  int32_t num_message_types = kNumMessageTypes;
  if (FLAGS_server) {
    for (int32_t i = 0; i < kNumMessageTypes; i++) {
      types[i] = static_cast<MessageType>(i);
    }
  } else {
    num_message_types = 0;
  }

  AioSocketInitConfig(num_message_types, types, &g_aio_sock_config);
  AioSocketSetJoinAll(true, &g_aio_sock_config);
  AioSocketSetPort(UDP_PORT_AIO, &g_aio_sock_config);
  AioSocketSetLoopback(true, &g_aio_sock_config);
  AioSocketSetReuseSocket(true, &g_aio_sock_config);

  g_aio_sock_fd = AioSocketOpen(&g_aio_sock_config);
  return g_aio_sock_fd;
}

int32_t OpenTunnelSocket(void) {
  g_tunnel_sock_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (g_tunnel_sock_fd < 0) return g_tunnel_sock_fd;

  struct sockaddr_in local_addr;
  memset(&local_addr, 0, sizeof(local_addr));
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(static_cast<uint16_t>(FLAGS_port));
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(g_tunnel_sock_fd, reinterpret_cast<struct sockaddr *>(&local_addr),
           sizeof(local_addr)) < 0) {
    LOG(ERROR) << "Got socket error: " << std::strerror(g_tunnel_sock_fd);
    return EXIT_FAILURE;
  }

  return g_tunnel_sock_fd;
}

int32_t TunnelReceive(int32_t data_length, uint8_t *data,
                      struct sockaddr_in *source) {
  assert(data_length >= 0);
  assert(data != NULL);
  assert(source != NULL);

  socklen_t addrlen = sizeof(*source);
  ssize_t rc = recvfrom(g_tunnel_sock_fd, data, (size_t)data_length,
                        MSG_DONTWAIT, (struct sockaddr *)source, &addrlen);
  if (rc < 0) {
    if (AIO_SOCKET_IGNORE_ERROR(errno)) {
      rc = 0;  // Ignore these errors.
    } else {
      LOG(ERROR) << "Got socket error: " << std::strerror(static_cast<int>(rc));
      return EXIT_FAILURE;
    }
  }
  if (rc > 0) {
    if (addrlen != sizeof(*source)) {
      LOG(ERROR) << "Receive socket address was wrong size.";
      return EXIT_FAILURE;
    } else if (source->sin_family != AF_INET) {
      LOG(ERROR) << "Receive socket address family incorrect.";
      return EXIT_FAILURE;
    }
  }
  return static_cast<int32_t>(rc);
}

void SendAioMessageMulticast(int32_t data_length, uint8_t *data) {
  assert(data_length >= static_cast<int32_t>(sizeof(AioHeader)));
  assert(data != NULL);

  AioHeader header;
  UnpackAioHeader(data, 1, &header);
  if (FLAGS_forge_aio_version) {
    header.version = AIO_VERSION;
    PackAioHeader(&header, 1, data);
  }
  AioSocketSend(&g_aio_sock_config, (MessageType)header.type, data_length, data,
                g_aio_sock_fd);
}

void TunnelSend(int data_length, uint8_t *data,
                const struct sockaddr_in *socket) {
  assert(data_length >= static_cast<int32_t>(sizeof(TunnelMessage)));
  assert(data != NULL);
  assert(socket != NULL);

  ssize_t rc = sendto(g_tunnel_sock_fd, data, static_cast<size_t>(data_length),
                      MSG_DONTWAIT,
                      reinterpret_cast<const struct sockaddr *>(socket),
                      sizeof(*socket));
  if (rc < 0) {
    if (AIO_SOCKET_IGNORE_ERROR(errno)) {
      rc = 0;  // Ignore these errors.
    } else {
      LOG(ERROR) << "Got socket error: " << std::strerror(static_cast<int>(rc));
    }
  }
}

void SendClientMessage(int32_t data_length, uint8_t *data,
                       const std::list<ClientConnection> &clients) {
  assert(data_length >= static_cast<int32_t>(sizeof(TunnelMessage)));
  assert(data != NULL);

  TunnelMessage *message = reinterpret_cast<TunnelMessage *>(data);
  AioHeader aio_header;
  if (message->type == kTunnelMessageTypeAio ||
      message->type == kTunnelMessageTypeAioCompressed) {
    UnpackAioHeader(GetTunnelMessagePayload(message), 1, &aio_header);
  }
  for (auto &i : clients) {
    if ((message->type == kTunnelMessageTypeAio ||
         message->type == kTunnelMessageTypeAioCompressed)
        && !IsMessageSelected(&i.select_types,
                              static_cast<MessageType>(aio_header.type))) {
      continue;
    }
    TunnelSend(data_length, data, &i.socket);
  }
}

void SendHeartbeat(const struct sockaddr_in *socket) {
  assert(socket != NULL);
  TunnelHeartbeatMessage message;
  message.header.version = kTunnelVersion;
  message.header.type = kTunnelMessageTypeHeartbeat;
  message.select_types = g_select_types;
  TunnelSend(sizeof(message), reinterpret_cast<uint8_t *>(&message), socket);
}

void SendHeartbeatAck(const struct sockaddr_in *socket) {
  assert(socket != NULL);
  TunnelMessage message;
  message.version = kTunnelVersion;
  message.type = kTunnelMessageTypeHeartbeatAck;
  TunnelSend(sizeof(message), reinterpret_cast<uint8_t *>(&message), socket);
}

std::string SocketToString(const struct sockaddr_in *socket) {
  assert(socket != NULL);
  char s[16];
  std::ostringstream ss;
  ss << inet_ntop(AF_INET, &socket->sin_addr, s, sizeof(s))
     << ":" << ntohs(socket->sin_port);
  return ss.str();
}

void HandleHeartbeat(const TunnelHeartbeatMessage *heartbeat_msg,
                     struct sockaddr_in *remote_socket) {
  bool found = false;
  for (auto &i : g_clients) {
    if (memcmp(&i.socket, remote_socket, sizeof(i.socket)) == 0) {
      i.update_time = ClockGetUs();
      found = true;
      i.select_types = heartbeat_msg->select_types;
      break;
    }
  }
  if (!found) {
    LOG(INFO) << "New client at " << SocketToString(remote_socket);
    ClientConnection conn;
    conn.socket = *remote_socket;
    conn.update_time = ClockGetUs();
    conn.select_types = heartbeat_msg->select_types;
    g_clients.push_back(conn);
  }
  SendHeartbeatAck(remote_socket);
}

void HandleTunnelTraffic(void) {
  static uint8_t tunnel_buf[kUdpMaxLength];
  static uint8_t aio_buf[kUdpMaxLength];
  struct sockaddr_in remote_socket;
  int32_t rx_size = TunnelReceive(sizeof(tunnel_buf),
                                  tunnel_buf, &remote_socket);
  TunnelMessage *tunnel_msg = reinterpret_cast<TunnelMessage *>(tunnel_buf);
  if (rx_size < static_cast<int32_t>(sizeof(*tunnel_msg))
      || tunnel_msg->version != kTunnelVersion) {
    return;
  }
  switch (tunnel_msg->type) {
    case kTunnelMessageTypeHeartbeat:
      if (FLAGS_server || FLAGS_repeater) {
        TunnelHeartbeatMessage *heartbeat_msg =
            reinterpret_cast<TunnelHeartbeatMessage *>(tunnel_msg);
        if (rx_size < static_cast<int32_t>(sizeof(*heartbeat_msg))) {
          break;
        }
        HandleHeartbeat(heartbeat_msg, &remote_socket);
      }
      break;
    case kTunnelMessageTypeHeartbeatAck:
      if (FLAGS_repeater || FLAGS_client) {
        if (memcmp(&remote_socket, &g_server, sizeof(g_server)) == 0) {
          if (g_server_update_time + kHeartbeatTimeoutUs < ClockGetUs()) {
            LOG(INFO) << "Connected to server at "
                      << SocketToString(&remote_socket);
          }
          g_server_update_time = ClockGetUs();
          g_server_connected = true;
        }
      }
      break;
    case kTunnelMessageTypeAio:
      if (FLAGS_repeater) {
        SendClientMessage(rx_size, tunnel_buf, g_clients);
      }
      if (FLAGS_client) {
        SendAioMessageMulticast(
            rx_size - static_cast<int32_t>(sizeof(*tunnel_msg)),
            GetTunnelMessagePayload(tunnel_msg));
      }
      break;
    case kTunnelMessageTypeAioCompressed:
      if (FLAGS_repeater) {
        SendClientMessage(rx_size, tunnel_buf, g_clients);
      }
      if (FLAGS_client) {
        int32_t aio_data_size;
        if (UncompressAioMessage(
                GetTunnelMessagePayload(tunnel_msg),
                rx_size - static_cast<int32_t>(sizeof(*tunnel_msg)),
                aio_buf, sizeof(aio_buf), &aio_data_size)) {
          SendAioMessageMulticast(aio_data_size, aio_buf);
        } else {
          LOG(WARNING) << "Got invalid compressed AIO payload.";
        }
      }
      break;
    default:
      LOG(WARNING) << "Got invalid type.";
      break;
  }
}

void HandleAioTraffic(void) {
  // Receive local AIO messages and transmit.
  static uint8_t aio_buf[kUdpMaxLength];
  static uint8_t tunnel_buf[kUdpMaxLength];
  TunnelMessage *tunnel_msg = reinterpret_cast<TunnelMessage *>(tunnel_buf);
  int32_t rc = AioSocketReceive(
      g_aio_sock_fd, static_cast<int32_t>(sizeof(aio_buf)), aio_buf);
  if (rc > static_cast<int32_t>(sizeof(AioHeader))) {
    tunnel_msg->version = kTunnelVersion;
    if (FLAGS_compress) {
      int32_t compressed_size;
      if (!CompressAioMessage(
              aio_buf, rc, GetTunnelMessagePayload(tunnel_msg),
              static_cast<int32_t>(sizeof(tunnel_buf) - sizeof(TunnelMessage)),
              &compressed_size)) {
        return;
      }
      if (compressed_size < rc) {
        // Send compressed if it's smaller, otherwise send original.
        tunnel_msg->type = kTunnelMessageTypeAioCompressed;
        SendClientMessage(
            compressed_size + static_cast<int32_t>(sizeof(*tunnel_msg)),
            tunnel_buf, g_clients);
        return;
      }
    }
    tunnel_msg->type = kTunnelMessageTypeAio;
    memcpy(GetTunnelMessagePayload(tunnel_msg), aio_buf, rc);
    SendClientMessage(rc + static_cast<int32_t>(sizeof(*tunnel_msg)),
                      tunnel_buf, g_clients);
  }
}

void PruneOldConnections(void) {
  auto i = g_clients.begin();
  while (i != g_clients.end()) {
    if (i->update_time + kHeartbeatTimeoutUs < ClockGetUs()) {
      LOG(INFO) << "Lost client " << SocketToString(&i->socket);
      i = g_clients.erase(i);
    } else {
      i++;
    }
  }
}

int32_t Run(void) {
  if (FLAGS_server || FLAGS_client) {
    if (OpenAioSocket() < 0) {
      LOG(ERROR) << "Failed to open AIO socket.";
      return EXIT_FAILURE;
    }
  }
  if (OpenTunnelSocket() < 0) {
    LOG(ERROR) << "Failed to open tunnel socket.";
    return EXIT_FAILURE;
  }

  // Try to drop privileges if run as root.
  if (geteuid() == 0) {
    LOG(INFO) << "Started as root -- dropping privileges.";
    int32_t rc;
    if ((rc = setgid(65534)) != 0 || (rc = setuid(65534)) != 0) {
      LOG(ERROR) << "Unable to drop privileges: " << std::strerror(rc);
      return EXIT_FAILURE;
    }
  }

  int32_t num_poll_fds = 1;
  struct pollfd poll_fds[2];
  poll_fds[0].fd = g_tunnel_sock_fd;
  poll_fds[0].events = POLLIN | POLLPRI;
  if (FLAGS_server) {
    poll_fds[num_poll_fds].fd = g_aio_sock_fd;
    poll_fds[num_poll_fds].events = POLLIN | POLLPRI;
    num_poll_fds++;
  }

  int64_t last_heartbeat_time = 0;

  while (true) {
    int32_t poll_rc = poll(poll_fds, num_poll_fds, 100);
    if (poll_rc < 0) {
      LOG(ERROR) << "Error polling sockets.";
      return EXIT_FAILURE;
    } else if (poll_rc > 0) {
      HandleTunnelTraffic();
      if (FLAGS_server) {
        HandleAioTraffic();
      }
    }
    if (FLAGS_server || FLAGS_repeater) {
      PruneOldConnections();
    }
    if (FLAGS_client || FLAGS_repeater) {
      // Periodically send heartbeat.
      if (last_heartbeat_time + kHeartbeatPeriodUs < ClockGetUs()) {
        last_heartbeat_time = ClockGetUs();
        SendHeartbeat(&g_server);
      }
      if (g_server_connected &&
          g_server_update_time + kHeartbeatTimeoutUs < ClockGetUs()) {
        g_server_connected = false;
        LOG(INFO) << "Lost connection to " << SocketToString(&g_server);
      }
    }
  }
}

}  // namespace.

int main(int argc, char *argv[]) {
  FLAGS_stderrthreshold = 0;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetUsageMessage("Tunnel AIO traffic across broadcast domains.  "
                          "NEVER run a connected client and server on the same "
                          "broadcast domain!  This will form a loop.");
  if (!(FLAGS_server || FLAGS_client || FLAGS_repeater)) {
    LOG(ERROR) << "Must specify an operating mode "
               << "(server, client, or repeater).";
    google::ShowUsageWithFlagsRestrict(argv[0], "aio_tunnel");
    return EXIT_FAILURE;
  }
  if (FLAGS_server && (FLAGS_client || FLAGS_repeater)) {
    LOG(ERROR) << "Cannot specify --server with --repeater or --client.";
    google::ShowUsageWithFlagsRestrict(argv[0], "aio_tunnel");
    return EXIT_FAILURE;
  }
  if (FLAGS_source.empty() && (FLAGS_client || FLAGS_repeater)) {
    LOG(ERROR) << "Must specify --source with --repeater or --client.";
    google::ShowUsageWithFlagsRestrict(argv[0], "aio_tunnel");
    return EXIT_FAILURE;
  }
  if (FLAGS_forge_aio_version && !FLAGS_client) {
    LOG(ERROR) << "--forge_aio_version only works with --client.";
    google::ShowUsageWithFlagsRestrict(argv[0], "aio_tunnel");
    return EXIT_FAILURE;
  }
  if (!FLAGS_filter.empty()) {
    if (FLAGS_server) {
      LOG(ERROR) << "Cannot use --filter in --server mode.";
      google::ShowUsageWithFlagsRestrict(argv[0], "aio_tunnel");
      return EXIT_FAILURE;
    }
    memset(g_select_types.mask, 0, sizeof(g_select_types.mask));
    std::stringstream ss(FLAGS_filter);
    std::string message;
    while (getline(ss, message, ',')) {
      MessageType message_type = StringToMessageType(message.c_str());
      if (message_type == kNumMessageTypes) {
        LOG(ERROR) << "Invalid message type in --filter: " << message;
        google::ShowUsageWithFlagsRestrict(argv[0], "aio_tunnel");
        return EXIT_FAILURE;
      } else {
        SelectMessage(&g_select_types, message_type);
      }
    }
  } else {
    memset(g_select_types.mask, 0xFF, sizeof(g_select_types.mask));
  }
  if (FLAGS_client || FLAGS_repeater) {
    g_server.sin_family = AF_INET;
    g_server.sin_port = htons(static_cast<uint16_t>(FLAGS_sport));
    if (inet_pton(AF_INET, FLAGS_source.c_str(), &g_server.sin_addr) <= 0) {
      LOG(ERROR) << "Source specification was not a valid IP address.";
      return EXIT_FAILURE;
    }
  }
  LOG(INFO) << "Starting tunnel on port " << FLAGS_port;
  if (FLAGS_client || FLAGS_repeater) {
    LOG(INFO) << "Remote server at " << SocketToString(&g_server);
  }
  return Run();
}
