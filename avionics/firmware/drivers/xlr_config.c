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

#include "avionics/firmware/drivers/xlr_config.h"

#include "common/macros.h"

// See XLR PRO documentation.
// http://www.digi.com/resources/documentation/digidocs/PDFs/90002202.pdf

// Compose initialization strings as AT plus the two letter ASCII command and
// parameter. Do not include the carriage return.
static const char *kInitStrings[] = {
  // MAC/PHY level commands.
  "ATBR3",  // Set RF data rate (3=140.7 kpbs).
  "ATPL4",  // Set power level (0=1 mW, 1=10 mW, 2=100 mW, 3=500 mW, 4=1 W).
  "ATRR0",  // Unicast retries.
  "ATMT0",  // Broadcast multi-transmit.

  // RF network commands.
  "ATCE2",  // Node messaging options (2=non-router mode).
  "ATBH0",  // Boardcast hops.
  "ATNH1",  // Network hops.
  "ATTO80",  // Transmit options (80=point-to-point/multipoint, ACK disabled).
  "ATCI11",  // Cluster ID (11=transparent data).
  "ATDEE8",  // Destination end-point (E8=Digi data endpoint).
  "ATSEE8",  // Source end-point (E8=Digi data endpoint).

  // Serial interface parameters.
  "ATBD9",  // Set serial interface baud rate (9=460800 bps).
  "ATNB0",  // Set parity (0=no parity).
  "ATSB0",  // Set stop bits (0=one stop bit).
  "ATRO0",  // Packetization timeout.
  "ATAP1",  // Driver requires API communications (AP1).
  "ATD60",  // RTS flow control (0=disabled).
  "ATD70",  // CTS flow control (0=disabled).
  "AT4E0",  // Serial protocol (0=RS-232).

  // Ethernet and IP socket mode.
  "ATES0",  // Disable IP socket mode (necessary for RS-232 without DTR).

  // Ethernet bridging commands.
  "ATBE0",  // Disable Ethernet bridging.
};

const XlrConfig kXlrConfigPort1 = {
  .device = &kSci1Interrupt,
  .at_string = kInitStrings,
  .num_at_strings = ARRAYSIZE(kInitStrings),
};

const XlrConfig kXlrConfigPort2 = {
  .device = &kSci2Interrupt,
  .at_string = kInitStrings,
  .num_at_strings = ARRAYSIZE(kInitStrings),
};
