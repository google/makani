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

#include "avionics/firmware/gps/gps_interface.h"

#include <assert.h>
#include <limits.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/novatel_types.h"
#include "avionics/common/septentrio_types.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/drivers/novatel.h"
#include "avionics/firmware/drivers/novatel_recv.h"
#include "avionics/firmware/drivers/septentrio.h"
#include "avionics/firmware/gps/novatel_config.h"
#include "avionics/firmware/gps/septentrio_config.h"
#include "avionics/firmware/network/net_send.h"
#include "common/c_math/util.h"
#include "common/macros.h"

#define GPS_SOLUTION_BUFFERS 2
#define GPS_PPS_BUFFERS 2

const GpsConfig kGpsConfigBase = {
  .novatel = &kNovAtelBase,
  .septentrio = &kSeptentrioBase
};

const GpsConfig kGpsConfigRover = {
  .novatel = &kNovAtelRover,
  .septentrio = &kSeptentrioRover
};

// Solution message.
static uint32_t g_sol_index = 0U;
static int64_t g_sol_timestamp[GPS_SOLUTION_BUFFERS] = {INT32_MIN, INT32_MIN};
static union {
  NovAtelSolutionMessage nov;
  SeptentrioSolutionMessage sept;
} g_sol[GPS_SOLUTION_BUFFERS];

// Observations message.
static int64_t g_obs_timestamp = INT32_MIN;
static union {
  NovAtelObservationsMessage nov;
  SeptentrioObservationsMessage sept;
} g_obs;

// Compass message.
static NovAtelCompassMessage g_comp;

// Gps satellite message.
static GpsSatellitesMessage g_sat;
int64_t g_sat_timestamp = INT32_MIN;

// Gps time message.
static GpsTimeMessage g_time;

// NovAtel timestamps.
static int64_t g_heading_timestamp = INT32_MIN;  // NovAtel OEM617D only.
static int64_t g_heading_rate_timestamp = INT32_MIN;  // NovAtel OEM617D only.

// 1-PPS timestamp.
static uint32_t g_pps_index = 0U;
static int64_t g_pps_timestamp[GPS_PPS_BUFFERS] = {INT32_MIN, INT32_MIN};

// Serial debug.
static SerialDebugMessage g_serial;

static void AppendSerialDataToSerialDebug(const SciData *in,
                                          SerialDebugMessage *out) {
  int32_t length = out->length + in->length;
  if (length > ARRAYSIZE(out->data)) {
    length = ARRAYSIZE(out->data);
  }
  memcpy(&out->data[out->length], in->data, length - out->length);
  out->length = length;
}

static void UpdateGpsEphemeris(const GpsEphemeris *new_eph) {
  GpsEphemeris *eph = NULL;
  int32_t oldest = 0;
  int32_t old_prn = g_sat.eph[0].prn;
  uint16_t old_wnc = g_sat.eph[0].wnc;
  uint32_t old_tow = g_sat.eph[0].tow;

  for (int32_t i = 0; i < ARRAYSIZE(g_sat.eph) && !eph; ++i) {
    if (g_sat.eph[i].prn == new_eph->prn) {
      eph = &g_sat.eph[i];
    } else if ((old_prn != 0 && g_sat.eph[i].prn == 0)
               || g_sat.eph[i].wnc < old_wnc
               || (g_sat.eph[i].wnc == old_wnc && g_sat.eph[i].tow < old_tow)) {
      old_prn = g_sat.eph[i].prn;
      old_wnc = g_sat.eph[i].wnc;
      old_tow = g_sat.eph[i].tow;
      oldest = i;
    }
  }
  if (!eph) {
    eph = &g_sat.eph[oldest];
  }
  memcpy(eph, new_eph, sizeof(*eph));
}

static void UpdateGpsTime(uint32_t meas_tow) {
  // Compute time-of-week of 1-PPS (occurs on 1 second boundaries).
  uint32_t pps_tow = meas_tow - (meas_tow % 1000);

  // Check for 1-PPS update (occurs before message reception).
  uint32_t next_index = (g_pps_index + 1) % GPS_PPS_BUFFERS;
  if (g_pps_timestamp[next_index] > g_pps_timestamp[g_pps_index]) {
    g_pps_index = next_index;
  }

  // Write output message.
  g_time.time_of_week = (int32_t)pps_tow;
}

static void ComputeNovAtelCn0Statistics(const NovAtelLogRange *range,
                                        float *avg, float *max) {
  *avg = 0.0f;
  *max = 0.0f;

  int32_t n = 0;
  for (int32_t i = 0; i < range->num_obs; ++i) {
    // See NovAtel OEM6 documentation for RANGE log. Table 126: Channel
    // Tracking Status on page 604 (rev 8) documents the meaning of the
    // status bits.
    if ((range->status_bits[i] & 0x03E70000) == 0x0) {  // GPS L1 C/A.
      ++n;
      *avg += range->cn0[i];
      if (range->cn0[i] > *max) {
        *max = range->cn0[i];
      }
    }
  }
  if (n > 0) {
    *avg /= n;
  }
}

static void UpdateNovAtelGpsTime(const NovAtelTimestamp *timestamp) {
  // See NovAtel OEM6 documentation for "GPS Reference Time Status" on page 35.
  if (timestamp->time_status >= 100) {
    UpdateGpsTime(timestamp->tow);
  }
}

static void UpdateNovAtelMessages(const NovAtelHeader *hdr,
                                  const NovAtelLog *log, int64_t now) {
  switch (hdr->message_id) {
    case kNovAtelMessageIdBestXyz:
      g_sol[0].nov.best_xyz = log->best_xyz;
      g_sol[0].nov.idle_time = hdr->idle_time;
      g_sol_timestamp[0] = now;
      UpdateNovAtelGpsTime(&log->best_xyz.timestamp);
      break;
    case kNovAtelMessageIdHeading:
      g_comp.heading = log->heading;
      g_heading_timestamp = now;
      UpdateNovAtelGpsTime(&log->heading.timestamp);
      break;
    case kNovAtelMessageIdHeadingRate:
      g_comp.heading_rate = log->heading_rate;
      g_heading_rate_timestamp = now;
      UpdateNovAtelGpsTime(&log->heading_rate.timestamp);
      break;
    case kNovAtelMessageIdIonUtc:
      g_sat.iono = log->ion_utc.iono;
      g_sat.utc = log->ion_utc.utc;
      g_sat_timestamp = now;
      break;
    case kNovAtelMessageIdRange:
      g_obs.nov.range = log->range;
      g_obs_timestamp = now;
      ComputeNovAtelCn0Statistics(&log->range, &g_sol[0].nov.avg_cn0,
                                  &g_sol[0].nov.max_cn0);
      UpdateNovAtelGpsTime(&log->range.timestamp);
      break;
    case kNovAtelMessageIdRawEphem:
      UpdateGpsEphemeris(&log->raw_ephem.eph);
      g_sat_timestamp = now;
      break;
    case kNovAtelMessageIdRxStatus:
      g_sol[0].nov.rx_status = log->rx_status;
      break;
    default:
      break;
  }
}

static void PollNovAtelReceiver(const NovAtelDevice *dev, int64_t now) {
  SciData in;
  SciAcquireReceiveData(dev->this_port, &in);
  in.length =
      SaturateInt32(in.length, 0, ARRAYSIZE(g_serial.data) - g_serial.length);
  NovAtelInsertDataFromThisPort(in.length, in.data);
  AppendSerialDataToSerialDebug(&in, &g_serial);
  SciReleaseReceiveData(dev->this_port, &in);

  SciAcquireReceiveData(dev->other_port, &in);
  NovAtelInsertDataFromOtherPort(in.length, in.data);
  SciReleaseReceiveData(dev->other_port, &in);

  NovAtelProto proto;
  if (NovAtelPoll(dev, &proto)) {
    // Handle binary messages.
    static NovAtelLog log;
    const NovAtelHeader *hdr;
    if (proto == kNovAtelProtoBinary && NovAtelGetBinary(&hdr, &log)) {
      UpdateNovAtelMessages(hdr, &log, now);
    }

    // Handle RTCM messages.
    uint16_t message_number;
    int32_t length;
    const uint8_t *data;
    if (proto == kNovAtelProtoRtcm3
        && NovAtelGetRtcm3(&message_number, &length, &data)) {
      GpsSendRtcmMessage(message_number, length, data);
    }
  }
}

static void SendNovAtelSolutionMessage(int64_t now) {
  g_sol[0].nov.best_xyz_latency = SaturateLatency(now - g_sol_timestamp[0]);
  NetSendAioNovAtelSolutionMessage(&g_sol[0].nov);
}

static void SendNovAtelObservationsMessage(int64_t now) {
  g_obs.nov.pps_latency_usec =
      SaturateLatency(now - g_pps_timestamp[g_pps_index]);
  g_obs.nov.latency_usec = SaturateLatency(now - g_obs_timestamp);
  NetSendAioNovAtelObservationsMessage(&g_obs.nov);
}

static void SendNovAtelCompassMessage(int64_t now) {
  g_comp.heading_latency = SaturateLatency(now - g_heading_timestamp);
  g_comp.heading_rate_latency = SaturateLatency(now - g_heading_rate_timestamp);
  NetSendAioNovAtelCompassMessage(&g_comp);
}

static void ComputeSeptentrioCn0Statistics(
    const SeptentrioBlockMeasEpoch *meas_epoch, float *avg, float *max) {
  *avg = 0.0f;
  *max = 0.0f;

  int32_t n = 0;
  for (int32_t i = 0; i < meas_epoch->num_obs; ++i) {
    // See the MeasEpochChannelType1 sub-block definition on pg. 18 of the SBF
    // Reference Guide.
    uint8_t signal_number = meas_epoch->type[i] & 0x1F;
    if (signal_number == 0U) {  // GPS L1 C/A.
      float cn0 = 10.0f + 0.25f * meas_epoch->cn0[i];
      ++n;
      *avg += cn0;
      if (cn0 > *max) {
        *max = cn0;
      }
    }
  }
  if (n > 0) {
    *avg /= n;
  }
}

static void UpdateSeptentrioGpsTime(const SeptentrioTimestamp *timestamp) {
  // See do not use values in SBF Reference Guide under "SBF Block Time Stamp".
  if (timestamp->tow != 4294967295) {
    UpdateGpsTime(timestamp->tow);
  }
}

static void UpdateSeptentrioMessages(const SeptentrioHeader *hdr,
                                     const SeptentrioBlock *blk, int64_t now) {
  g_sol_index %= GPS_SOLUTION_BUFFERS;
  SeptentrioSolutionMessage *sol = &g_sol[g_sol_index].sept;
  switch (hdr->block_id) {
    case kSeptentrioIdMeasEpoch:
      memcpy(&g_obs.sept.meas_epoch, &blk->meas_epoch,
             sizeof(g_obs.sept.meas_epoch));
      ComputeSeptentrioCn0Statistics(&blk->meas_epoch,
                                     &g_sol[g_sol_index].sept.avg_cn0,
                                     &g_sol[g_sol_index].sept.max_cn0);
      UpdateSeptentrioGpsTime(&blk->meas_epoch.timestamp);
      g_obs_timestamp = now;
      break;
    case kSeptentrioIdGpsNav:
      UpdateGpsEphemeris(&blk->gps_nav.eph);
      g_sat_timestamp = now;
      break;
    case kSeptentrioIdGpsIon:
      memcpy(&g_sat.iono, &blk->gps_ion.iono, sizeof(g_sat.iono));
      g_sat_timestamp = now;
      break;
    case kSeptentrioIdGpsUtc:
      memcpy(&g_sat.utc, &blk->gps_utc.utc, sizeof(g_sat.utc));
      g_sat_timestamp = now;
      break;
    case kSeptentrioIdPvtCartesian:
      memcpy(&sol->pvt_cartesian, &blk->pvt_cartesian,
             sizeof(sol->pvt_cartesian));
      g_sol_timestamp[g_sol_index] = now;
      UpdateSeptentrioGpsTime(&blk->pvt_cartesian.timestamp);
      break;
    case kSeptentrioIdPosCovCartesian:
      memcpy(&sol->pos_cov_cartesian, &blk->pos_cov_cartesian,
             sizeof(sol->pos_cov_cartesian));
      g_sol_timestamp[g_sol_index] = now;
      UpdateSeptentrioGpsTime(&blk->pos_cov_cartesian.timestamp);
      break;
    case kSeptentrioIdVelCovCartesian:
      memcpy(&sol->vel_cov_cartesian, &blk->vel_cov_cartesian,
             sizeof(sol->vel_cov_cartesian));
      g_sol_timestamp[g_sol_index] = now;
      UpdateSeptentrioGpsTime(&blk->vel_cov_cartesian.timestamp);
      break;
    case kSeptentrioIdBaseVectorCart:
      memcpy(&sol->base_vector_cart, &blk->base_vector_cart,
             sizeof(sol->base_vector_cart));
      g_sol_timestamp[g_sol_index] = now;
      UpdateSeptentrioGpsTime(&blk->base_vector_cart.timestamp);
      break;
    case kSeptentrioIdEndOfPvt:
      break;
    default:
      break;
  }

  // Increment solution index when all required messages arrive.
  uint32_t tow = sol->pvt_cartesian.timestamp.tow;
  if (sol->pos_cov_cartesian.timestamp.tow == tow
      && sol->vel_cov_cartesian.timestamp.tow == tow
      && sol->base_vector_cart.timestamp.tow == tow
      && g_sol_timestamp[g_sol_index] > 0) {
    g_sol_index = (g_sol_index + 1) % GPS_SOLUTION_BUFFERS;
    g_sol_timestamp[g_sol_index] = INT32_MIN;
  }
}

static void PollSeptentrioReceiver(const SeptentrioDevice *sept, int64_t now) {
  SciData in;
  SciAcquireReceiveData(sept->this_port, &in);
  in.length =
      SaturateInt32(in.length, 0, ARRAYSIZE(g_serial.data) - g_serial.length);
  SeptentrioInsertDataFromThisPort(in.length, in.data);
  AppendSerialDataToSerialDebug(&in, &g_serial);
  SciReleaseReceiveData(sept->this_port, &in);

  SciAcquireReceiveData(sept->other_port, &in);
  SeptentrioInsertDataFromOtherPort(in.length, in.data);
  SciReleaseReceiveData(sept->other_port, &in);

  SeptentrioProto proto;
  if (SeptentrioPoll(sept, now, &proto)) {
    static SeptentrioBlock blk;
    const SeptentrioHeader *hdr;
    uint16_t message_number;
    int32_t length;
    const uint8_t *data;
    switch (proto) {
      case kSeptentrioProtoSbf:
        if (SeptentrioGetSbf(&hdr, &blk)) {
          UpdateSeptentrioMessages(hdr, &blk, now);
        }
        break;
      case kSeptentrioProtoSnmp:
        break;
      case kSeptentrioProtoRtcm3:
        if (SeptentrioGetRtcm3(&message_number, &length, &data)) {
          GpsSendRtcmMessage(message_number, length, data);
        }
        break;
      default:
        break;
    }
  }
}

static void SendSeptentrioSolutionMessage(int64_t now) {
  // Output previous index.
  int32_t idx = (g_sol_index + GPS_SOLUTION_BUFFERS - 1) % GPS_SOLUTION_BUFFERS;
  g_sol[idx].sept.latency_usec = SaturateLatency(now - g_sol_timestamp[idx]);
  NetSendAioSeptentrioSolutionMessage(&g_sol[idx].sept);
}

static void SendSeptentrioObservationsMessage(int64_t now) {
  g_obs.sept.pps_latency_usec =
      SaturateLatency(now - g_pps_timestamp[g_pps_index]);
  g_obs.sept.latency_usec = SaturateLatency(now - g_obs_timestamp);
  NetSendAioSeptentrioObservationsMessage(&g_obs.sept);
}

void GpsInit(void) {
  // Solution message.
  memset(g_sol, 0, sizeof(g_sol));
  g_sol_index = 0U;
  for (int32_t i = 0; i < GPS_SOLUTION_BUFFERS; ++i) {
    g_sol_timestamp[i] = INT32_MIN;
  }

  // Observations message.
  memset(&g_obs, 0, sizeof(g_obs));
  g_obs_timestamp = INT32_MIN;

  // Compass message.
  memset(&g_comp, 0, sizeof(g_comp));

  // Satellites message.
  memset(&g_sat, 0, sizeof(g_sat));
  g_sat_timestamp = INT32_MIN;

  // Gps time message.
  memset(&g_time, 0, sizeof(g_time));

  // NovAtel timestamps.
  g_heading_timestamp = INT32_MIN;  // NovAtel OEM617D only.
  g_heading_rate_timestamp = INT32_MIN;  // NovAtel OEM617D only.

  // 1-PPS timestamp.
  g_pps_index = 0U;
  for (int32_t i = 0; i < GPS_PPS_BUFFERS; ++i) {
    g_pps_timestamp[i] = INT32_MIN;
  }

  // Serial debug.
  memset(&g_serial, 0, sizeof(g_serial));

  // Initialize drivers.
  NovAtelInit();
  SeptentrioInit();
}

void GpsPoll(const GpsConfig *gps, GpsReceiverType receiver, int64_t now) {
  switch (receiver) {
    case kGpsReceiverTypeNone:
      break;
    case kGpsReceiverTypeNovAtel:
      PollNovAtelReceiver(gps->novatel, now);
      break;
    case kGpsReceiverTypeSeptentrio:
      PollSeptentrioReceiver(gps->septentrio, now);
      break;
    default:
      break;
  }
}

void GpsUpdatePpsTimestamp(int64_t now) {
  // Incremented on message reception.
  uint32_t next_index = (g_pps_index + 1) % GPS_PPS_BUFFERS;
  g_pps_timestamp[next_index] = now;
}

void GpsSendSerialDebugMessage(void) {
  NetSendAioSerialDebugMessage(&g_serial);
  memset(g_serial.data, 0, sizeof(g_serial.data));
  g_serial.length = 0;
}

void GpsSendSolutionMessage(GpsReceiverType receiver, int64_t now) {
  switch (receiver) {
    case kGpsReceiverTypeNone:
      break;
    case kGpsReceiverTypeNovAtel:
      SendNovAtelSolutionMessage(now);
      break;
    case kGpsReceiverTypeSeptentrio:
      SendSeptentrioSolutionMessage(now);
      break;
    default:
      break;
  }
}

void GpsSendObservationsMessage(GpsReceiverType receiver, int64_t now) {
  switch (receiver) {
    case kGpsReceiverTypeNone:
      break;
    case kGpsReceiverTypeNovAtel:
      SendNovAtelObservationsMessage(now);
      break;
    case kGpsReceiverTypeSeptentrio:
      SendSeptentrioObservationsMessage(now);
      break;
    default:
      break;
  }
}

void GpsSendCompassMessage(GpsReceiverType receiver, int64_t now) {
  switch (receiver) {
    case kGpsReceiverTypeNone:
      break;
    case kGpsReceiverTypeNovAtel:
      SendNovAtelCompassMessage(now);
      break;
    case kGpsReceiverTypeSeptentrio:
      break;
    default:
      break;
  }
}

void GpsSendSatellitesMessage(int64_t now) {
  g_sat.latency_usec = SaturateLatency(now - g_sat_timestamp);
  NetSendAioGpsSatellitesMessage(&g_sat);
}

void GpsSendTimeMessage(int64_t now) {
  g_time.latency = SaturateLatency(now - g_pps_timestamp[g_pps_index]);
  NetSendAioGpsTimeMessage(&g_time);
}

void GpsGetTimeOfWeek(int64_t now, int32_t *latency, int32_t *time_of_week) {
  *latency = SaturateLatency(now - g_pps_timestamp[g_pps_index]);
  *time_of_week = g_time.time_of_week;
}

void GpsSendRtcmMessage(uint16_t message_number, int32_t length,
                        const uint8_t *data) {
  assert(data != NULL);

  // Declare static to prevent stack overflow.
  static union {
    GpsRtcm1006Message rtcm1006;
    GpsRtcm1033Message rtcm1033;
    GpsRtcm1072Message rtcm1072;
    GpsRtcm1074Message rtcm1074;
    GpsRtcm1082Message rtcm1082;
    GpsRtcm1084Message rtcm1084;
    GpsRtcm1230Message rtcm1230;
    GpsRtcmMessage rtcm;
  } u;

  memset(&u, 0, sizeof(u));
  if (message_number == 1006) {
    GpsRtcm1006Message *out = &u.rtcm1006;
    if (0 < length && length <= ARRAYSIZE(out->data)) {
      memcpy(out->data, data, length);
      out->length = length;
      NetSendAioGpsRtcm1006Message(out);
    }
  } else if (message_number == 1033) {
    GpsRtcm1033Message *out = &u.rtcm1033;
    if (0 < length && length <= ARRAYSIZE(out->data)) {
      memcpy(out->data, data, length);
      out->length = length;
      NetSendAioGpsRtcm1033Message(out);
    }
  } else if (message_number == 1072) {
    GpsRtcm1072Message *out = &u.rtcm1072;
    if (0 < length && length <= ARRAYSIZE(out->data)) {
      memcpy(out->data, data, length);
      out->length = length;
      NetSendAioGpsRtcm1072Message(out);
    }
  } else if (message_number == 1074) {
    GpsRtcm1074Message *out = &u.rtcm1074;
    if (0 < length && length <= ARRAYSIZE(out->data)) {
      memcpy(out->data, data, length);
      out->length = length;
      NetSendAioGpsRtcm1074Message(out);
    }
  } else if (message_number == 1082) {
    GpsRtcm1082Message *out = &u.rtcm1082;
    if (0 < length && length <= ARRAYSIZE(out->data)) {
      memcpy(out->data, data, length);
      out->length = length;
      NetSendAioGpsRtcm1082Message(out);
    }
  } else if (message_number == 1084) {
    GpsRtcm1084Message *out = &u.rtcm1084;
    if (0 < length && length <= ARRAYSIZE(out->data)) {
      memcpy(out->data, data, length);
      out->length = length;
      NetSendAioGpsRtcm1084Message(out);
    }
  } else if (message_number == 1230) {
    GpsRtcm1230Message *out = &u.rtcm1230;
    if (0 < length && length <= ARRAYSIZE(out->data)) {
      memcpy(out->data, data, length);
      out->length = length;
      NetSendAioGpsRtcm1230Message(out);
    }
  } else {
    GpsRtcmMessage *out = &u.rtcm;
    if (0 < length && length <= ARRAYSIZE(out->data)) {
      out->message_number = message_number;
      memcpy(out->data, data, length);
      out->length = length;
      NetSendAioGpsRtcmMessage(out);
    }
  }
}

void GpsInsertRtcm(const GpsConfig *gps, GpsReceiverType receiver,
                   int32_t length, const uint8_t *data) {
  switch (receiver) {
    case kGpsReceiverTypeNone:
      break;
    case kGpsReceiverTypeNovAtel:
      NovAtelInsertRtcm(gps->novatel, length, data);
      break;
    case kGpsReceiverTypeSeptentrio:
      SeptentrioInsertRtcm(gps->septentrio, length, data);
      break;
    default:
      break;
  }
}

bool GpsIsReady(GpsReceiverType receiver) {
  switch (receiver) {
    case kGpsReceiverTypeNovAtel:
      return NovAtelIsReady();
    case kGpsReceiverTypeSeptentrio:
      return SeptentrioIsReady();
    case kGpsReceiverTypeNone:
    default:
      return false;
  }
}
