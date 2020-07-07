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

#include "lib/udpio/udpio.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>


// Do a bunch of common (shared) configuration for UDP sockets.
static int32_t sock_init(in_addr_t addr, uint16_t port, int32_t non_blocking,
                         struct sockaddr_in *dest) {
  int32_t sock, flags;
  if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    fprintf(stderr, "Failed to create socket!\n");
    return UDP_ERR_SOCKET;
  }
  if (non_blocking) {
    // Set socket to non-blocking
    if ((flags = fcntl(sock, F_GETFL, 0)) < 0) {
      fprintf(stderr, "Failed to get socket flags!\n");
      return UDP_ERR_SOCKET;
    }
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) {
      fprintf(stderr, "Failed to set socket flags!\n");
      return UDP_ERR_SOCKET;
    }
  }
  // Construct the sockaddr_in structure
  memset(dest, 0, sizeof(*dest));
  dest->sin_family = AF_INET;
  dest->sin_addr.s_addr = htonl(addr);
  dest->sin_port = htons(port);
  return sock;
}


// Create a new listener in `udp_config' bound to the given port
// `port'. If multiple listeners on the given port is required,
// set the multiple_listeners flag. The socket to can be set to
// blocking or non-blocking using the non_blocking flag.
int32_t udp_setup_listener(udp_config *cfg, uint16_t port, int32_t non_blocking,
                           int32_t multiple_listeners) {
  int32_t so_reuseaddr = 1;
  cfg->sock = sock_init(INADDR_ANY, port, non_blocking, &cfg->io);
  if (multiple_listeners) {
    if (setsockopt(cfg->sock, SOL_SOCKET, SO_REUSEADDR, &so_reuseaddr,
                   sizeof(so_reuseaddr)) == -1) {
      fprintf(stderr, "Could not set socket options.\n");
    }
  }
  if (bind(cfg->sock, (struct sockaddr *)&cfg->io, sizeof(cfg->io)) < 0) {
    fprintf(stderr,
            "Warning: Couldn't bind server on port %d.  "
            "Maybe something's already running?  "
            "Maybe you're running two copies of this program?  "
            "Either way, probably this program won't work.\n", port);
  }
  return cfg->sock;
}

void udp_set_timeout(udp_config *cfg, double timeout) {
  struct timeval tv = {(int32_t)floor(timeout),
                       (int32_t)((timeout - floor(timeout))*1.0e6)};
  if (setsockopt(cfg->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)))
    perror("Could not set timeout");
}


// Just receive on the socket in `udp_config', copying the received
// packet directly into the provided buffer.
int32_t udp_recv(const udp_config *cfg, uint8_t *msg) {
  struct sockaddr from_addr;
  uint32_t addr_len = sizeof(from_addr);
  int32_t num_recv = (int32_t)recvfrom(cfg->sock, msg, UDP_BUFLEN, 0,
                                       &from_addr, &addr_len);
  if (num_recv < 0) {
    return UDP_ERR_MSG_SIZE;
  } else if (num_recv == UDP_BUFLEN) {
    fprintf(stderr,
            "Warning: recvfrom() returned %d bytes, "
            "which is the maximum allocated in " __FILE__
            "\nSome data may have been lost; consider increasing this value.\n",
            UDP_BUFLEN);
  }
  return num_recv;
}


// Create a new sending socket in `udp_config' to the given address
// `addr' and port `port'.  If the brodcast flag `broadcast' is
// nonzero, then the sender will broadcast its data on the local
// subnet. Socket is set to blocking.
int32_t udp_setup_sender(udp_config *cfg, const char *addr, uint16_t port,
                         int32_t broadcast) {
  cfg->sock = sock_init(inet_network(addr), port, 0, &cfg->io);
  if (broadcast) {
    if (setsockopt(cfg->sock, SOL_SOCKET, SO_BROADCAST,
                   &broadcast, sizeof(broadcast)) == -1) {
      fprintf(stderr,
              "Couldn't set socket to broadcast mode: "
              "setsockopt (SO_BROADCAST)\n");
    }
  }
  return cfg->sock;
}


// Send a message buffer `size' bytes long via the socket in `udp_config'.
int32_t udp_send(const udp_config *cfg, const uint8_t *msg, int32_t msg_size) {
  int32_t num_sent = (int32_t)sendto(cfg->sock, msg, (size_t)msg_size, 0,
                                     (const struct sockaddr *)&cfg->io,
                                     sizeof(cfg->io));
  if (num_sent != msg_size) {
    perror("sendto: ");
    return UDP_ERR_MSG_SIZE;
  }
  return num_sent;
}


// Close the udp socket
int32_t udp_close(const udp_config *cfg) {
  int32_t ret = close(cfg->sock);
  if (ret < 0)
    perror("Error closing socket");
  return ret;
}
