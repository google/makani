#!/usr/bin/python
# Copyright 2020 Makani Technologies LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""Fake bootloader client used for testing the parallel bootloader.

   Usage:

     ./fake_bootloader_client --target sometarget somebinary.elf

   Or, to try it out in conjunction with the bootload script and the
   parallel bootloader:

     lib/scripts/m600/bootload --bootloader_client=./fake_bootloader_client

   For testing on Linux, the --nobootload_flight_controller flag to
   the bootload script is also useful (to avoid the "Binary is not
   built for ARM" error).
"""

import logging
import os
import random
import sys
import time

import bootloader_client

logging.basicConfig(level=logging.INFO,
                    format='%(levelname)s: %(message)s')


def RandomSleep(mean, sigma):
  dt = max(0.0, random.gauss(mean, sigma))
  time.sleep(dt)


def Main():
  """Pretend to update a TMS570 board with the supplied binary."""

  # Reopen standard output without buffering so that output gets to
  # the parallel bootloader expeditiously.  The "-u" shebang option to
  # Python is not available due to the bash stubs instituted by Bazel.
  sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)

  parsed_args = bootloader_client.ParseArguments()
  binary_path = parsed_args['file']
  binary_size = os.stat(binary_path).st_size
  address_string = parsed_args['new_ip']

  RandomSleep(0.5, 0.5)
  logging.info('Binary size: %d bytes; target IP: %s.',
               binary_size, address_string)
  RandomSleep(1.0, 0.1)
  logging.info('Target hardware type: kHardwareTypeSomething.')

  RandomSleep(2.0, 0.5)
  logging.info('Got an acknowledgement from target; starting upload.')

  for bytes_sent_successfully in range(0, binary_size, 102400) + [binary_size]:
    RandomSleep(1.0, 0.1)
    logging.info('Sent %d bytes...', bytes_sent_successfully)
    sys.stdout.flush()

  RandomSleep(0.5, 0.5)
  logging.info('Successfully transferred %d bytes; cleaning up.', binary_size)

if __name__ == '__main__':
  Main()
