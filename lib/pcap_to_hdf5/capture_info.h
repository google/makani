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

#ifndef LIB_PCAP_TO_HDF5_CAPTURE_INFO_H_
#define LIB_PCAP_TO_HDF5_CAPTURE_INFO_H_

#include <stdint.h>

#include "avionics/common/aio_header.h"

enum BadPacketReason {
  kBadPacketReasonForceSigned = -1,
  kBadPacketReasonNone,
  kBadPacketReasonAioAddress,
  kBadPacketReasonAioHeader,
  kBadPacketReasonAioLength,
  kBadPacketReasonAioPort,
  kBadPacketReasonAioSource,
  kBadPacketReasonAioType,
  kBadPacketReasonAioVersion,
  kBadPacketReasonEthernetHeader,
  kBadPacketReasonEthernetProtocol,
  kBadPacketReasonIpHeader,
  kBadPacketReasonIpProtocol,
  kBadPacketReasonLinkProtocol,
  kBadPacketReasonPcapHeader,
  kBadPacketReasonSllHeader,
  kBadPacketReasonUdpHeader,
  kNumBadPacketReasons
};

typedef struct {
  int64_t tv_sec;
  int64_t tv_usec;
  uint32_t source_address;
  uint32_t destination_address;
} CaptureHeader;

typedef struct {
  uint32_t reason;  // See BadPacketReason.
  // Addresses in the capture_header are only populated if
  // reason < kBadpacketReasonIp.
  CaptureHeader capture_header;
  // These fields are only populated if reason < kBadPacketReasonAio.
  AioHeader aio_header;
  int32_t aio_payload_length;
} BadPacketInfo;

typedef struct {
  int64_t min_tv_sec;
  int64_t min_tv_usec;
  double gs_gps_time_coeff;
  double gs_gps_time_bias;
  double wing_gps_time_coeff;
  double wing_gps_time_bias;
} LogInfo;

#endif  // LIB_PCAP_TO_HDF5_CAPTURE_INFO_H_
