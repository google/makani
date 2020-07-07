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

#include "avionics/firmware/output/slow_status.h"

#include <stdbool.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/build_info.h"
#include "avionics/common/cvt.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/identity/board_hardware.h"
#include "avionics/firmware/identity/carrier_hardware.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/firmware/output/aio_node_status.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/firmware/serial/carrier_serial.h"
#include "avionics/firmware/startup/ldscript.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"

static union {
  SlowStatusMessage access;
  CoreSwitchSlowStatusMessage core;
  BootloaderSlowStatusMessage bootloader;
} g_slow_status;

static const AioNode kGpsTimeSources[] = {
  kAioNodeFcA,
  kAioNodeFcB,
  kAioNodeGpsBaseStation
};

static uint32_t g_gps_source_index = 0U;

static void UpdateGpsTime(int64_t now, GpsTimeData *out) {
  memset(out, 0, sizeof(*out));
  out->latency = INT32_MAX;  // Stale value, don't use.

  // Rotate through possible sources sequentially for each message.
  for (int32_t i = 0; i < ARRAYSIZE(kGpsTimeSources); ++i) {
    g_gps_source_index = (g_gps_source_index + 1U) % ARRAYSIZE(kGpsTimeSources);
    AioNode source = kGpsTimeSources[g_gps_source_index];
    GpsTimeMessage in;
    int64_t timestamp;
    if (CvtGetGpsTimeMessage(source, &in, NULL, &timestamp)) {
      out->latency = SaturateLatency((int64_t)in.latency + (now - timestamp));
      out->time_of_week = in.time_of_week;
      out->source = source;
      break;
    }
  }

  // Clear out CVT to eliminate unread warnings.
  for (int32_t i = 0; i < ARRAYSIZE(kGpsTimeSources); ++i) {
    CvtClearMessage(kGpsTimeSources[i], kMessageTypeGpsTime);
  }
}

static void HandleSerialParams(const SerialParams *in_params,
                               SerialParams *out_params) {
  if (in_params != NULL) {
    *out_params = *in_params;
  } else {
    memset(out_params, 0x0, sizeof(*out_params));
  }
}

void OutputInitSlowStatusMessage(void) {
  memset(&g_slow_status, 0, sizeof(g_slow_status));
  AioNodeInitStatus(&g_slow_status.access.node_status);
  GetBuildInfo(&g_slow_status.access.build_info);
  g_gps_source_index = 0U;
  HandleSerialParams(GetBoardSerialParams(),
                     &g_slow_status.access.serial_params);
  HandleSerialParams(GetCarrierSerialParams(),
                     &g_slow_status.access.carrier_serial_params);
}

void OutputInitCoreSwitchSlowStatusMessage(void) {
  memset(&g_slow_status, 0, sizeof(g_slow_status));
  AioNodeInitStatus(&g_slow_status.core.node_status);
  GetBuildInfo(&g_slow_status.core.build_info);
  g_gps_source_index = 0U;
  HandleSerialParams(GetBoardSerialParams(),
                     &g_slow_status.core.serial_params);
}

void OutputInitBootloaderSlowStatusMessage(void) {
  memset(&g_slow_status, 0, sizeof(g_slow_status));
  AioNodeInitStatus(&g_slow_status.bootloader.node_status);
  GetBuildInfo(&g_slow_status.bootloader.build_info);
  g_slow_status.bootloader.bootloader_segment = IsRunningFromBootSegment();
  HandleSerialParams(GetBoardSerialParams(),
                     &g_slow_status.bootloader.serial_params);
  g_slow_status.bootloader.hardware_type = GetBoardHardwareType();
  g_slow_status.bootloader.carrier_hardware_type = GetCarrierHardwareType();
}

bool OutputSendSlowStatusMessage(int64_t now) {
  AioNodeGetStatus(&g_slow_status.access.node_status);
  NetMonGetAccessSwitchStats(&g_slow_status.access.switch_stats);
  AioGetStats(&g_slow_status.access.network_status.aio_stats);
  CvtGetStats(&g_slow_status.access.network_status.cvt_stats);
  UpdateGpsTime(now, &g_slow_status.access.gps_time);
  return NetSendAioSlowStatusMessage(&g_slow_status.access);
}

bool OutputSendCoreSwitchSlowStatusMessage(int64_t now) {
  AioNodeGetStatus(&g_slow_status.core.node_status);
  NetMonGetCoreSwitchStats(&g_slow_status.core.switch_stats);
  AioGetStats(&g_slow_status.core.network_status.aio_stats);
  CvtGetStats(&g_slow_status.core.network_status.cvt_stats);
  UpdateGpsTime(now, &g_slow_status.core.gps_time);
  return NetSendAioCoreSwitchSlowStatusMessage(&g_slow_status.core);
}

bool OutputSendBootloaderSlowStatusMessage(void) {
  AioNodeGetStatus(&g_slow_status.bootloader.node_status);
  return NetSendAioBootloaderSlowStatusMessage(&g_slow_status.bootloader);
}

const SlowStatusMessage *OutputGetSlowStatusMessage(void) {
  return &g_slow_status.access;
}

const CoreSwitchSlowStatusMessage *OutputGetCoreSwitchSlowStatusMessage(void) {
  return &g_slow_status.core;
}
