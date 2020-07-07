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

#include <stdio.h>
#include <string.h>

#include "lib/udpio/udpio.h"

#define BUFSIZE 4096

udp_config config;
unsigned char buffer[BUFSIZE];

int main(int argc __attribute__((unused)),
         char **argv __attribute__((unused))) {
  int sent;
  unsigned int i;
  udp_setup_sender(&config, "127.0.0.1", 22222, 0);
  printf("Set up sender to 127.0.0.1 port 22222.\n");

  memset(buffer, 0xdf, BUFSIZE);

  for (i = 1; i < BUFSIZE; i += i) {
    if ((sent = udp_send(&config, buffer + i, i)) > 0) {
      printf("Sent %d bytes.\n", sent);
    }
  }
  udp_close(&config);
  return 0;
}
