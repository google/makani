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

#ifndef LIB_UDPIO_UDPIO_H_
#define LIB_UDPIO_UDPIO_H_

#include <arpa/inet.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/socket.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UDP_ERR_MSG_SIZE -1
#define UDP_ERR_MSG_TYPE -2
#define UDP_ERR_SOCKET -3

// Max size of a received UDP received packet in bytes.
#define UDP_BUFLEN 65000

typedef struct {
  int32_t sock;
  struct sockaddr_in io;
} udp_config;

// These macros allow you to pass pack/unpack functions to a
// structure that converts it to/from a uint8_t array before sending
// it or after receiving it.
#define UDP_SEND_PACKED(udp_cfg_ptr, pack_func, data_ptr)               \
  do {                                                                  \
    uint8_t _buf[UDP_BUFLEN];                                           \
    int32_t _num_packed = (int32_t)pack_func(data_ptr, 1, _buf);        \
    int32_t _num_sent = udp_send(udp_cfg_ptr, _buf, _num_packed);       \
    assert(_num_packed == _num_sent);                                   \
    (void)_num_sent;                                                    \
  } while (0)

#define UDP_RECV_PACKED(udp_cfg_ptr, unpack_func, data_ptr, updated_ptr) \
  do {                                                                  \
    uint8_t _buf[UDP_BUFLEN];                                           \
    int32_t _num_recv = udp_recv(udp_cfg_ptr, _buf);                    \
    *(updated_ptr) = false;                                             \
    if (_num_recv > 0) {                                                \
      int32_t _num_unpacked = (int32_t)unpack_func(_buf, 1, data_ptr);  \
      assert(_num_unpacked == _num_recv);                               \
      if (_num_unpacked == _num_recv) *(updated_ptr) = true;            \
    }                                                                   \
  } while (0)

int32_t udp_setup_listener(udp_config *cfg, uint16_t port,
                           int32_t non_blocking, int32_t multiple_listeners);
void udp_set_timeout(udp_config *cfg, double timeout);
int32_t udp_recv(const udp_config *cfg, uint8_t *msg);
int32_t udp_setup_sender(udp_config *cfg, const char *addr, uint16_t port,
                         int32_t broadcast);
int32_t udp_send(const udp_config *cfg, const uint8_t *msg, int32_t size);
int32_t udp_close(const udp_config *cfg);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIB_UDPIO_UDPIO_H_
