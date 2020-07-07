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

#include "lib/udpio/udpio.h"

udp_config config;
unsigned char buffer[4096];

int main(int argc __attribute__((unused)),
         char **argv __attribute__((unused))) {
  int out;
  unsigned int i;
  udp_setup_listener(&config, 22222, 1, 0);  // non-blocking, single listener
  printf("Set up listener on port 22222.\n");

  while (1) {
    if ((out = udp_recv(&config, buffer)) > 0) {
      printf("Received %d bytes: ", out);
      for (i = 0; (int)i < out; i++) {
        printf("%02x ", buffer[i]);
      }
      printf("\n\n");
    }
  }
  udp_close(&config);
  return 0;
}
