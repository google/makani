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

#include "avionics/common/novatel_binary.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/common/crc.h"
#include "avionics/common/endian.h"
#include "avionics/common/gps_parse.h"
#include "avionics/common/novatel_types.h"
#include "common/c_math/util.h"
#include "common/macros.h"

typedef bool (*DecodeFunc)(const NovAtelHeader *hdr, const uint8_t *data,
                           NovAtelLog *out);

static bool DecodeBinaryBestXyzImpl(const NovAtelHeader *hdr,
                                    const uint8_t *data,
                                    NovAtelLogBestXyz *log) {
  assert(hdr != NULL);
  assert(data != NULL);
  assert(log != NULL);

  // Validate message length.
  if (hdr->message_length != 112) {
    return false;
  }

  log->timestamp = hdr->timestamp;
  int32_t o = 0;

  o += ReadInt32Le(&data[o], &log->pos_sol_status);
  o += ReadInt32Le(&data[o], &log->pos_type);
  o += ReadDoubleLe(&data[o], &log->pos_x);
  o += ReadDoubleLe(&data[o], &log->pos_y);
  o += ReadDoubleLe(&data[o], &log->pos_z);
  o += ReadFloatLe(&data[o], &log->pos_x_sigma);
  o += ReadFloatLe(&data[o], &log->pos_y_sigma);
  o += ReadFloatLe(&data[o], &log->pos_z_sigma);
  o += ReadInt32Le(&data[o], &log->vel_sol_status);
  o += ReadInt32Le(&data[o], &log->vel_type);
  o += ReadDoubleLe(&data[o], &log->vel_x);
  o += ReadDoubleLe(&data[o], &log->vel_y);
  o += ReadDoubleLe(&data[o], &log->vel_z);
  o += ReadFloatLe(&data[o], &log->vel_x_sigma);
  o += ReadFloatLe(&data[o], &log->vel_y_sigma);
  o += ReadFloatLe(&data[o], &log->vel_z_sigma);
  log->station_id[0] = data[o++];
  log->station_id[1] = data[o++];
  log->station_id[2] = data[o++];
  log->station_id[3] = data[o++];
  o += ReadFloatLe(&data[o], &log->vel_latency);
  o += ReadFloatLe(&data[o], &log->diff_age);
  o += ReadFloatLe(&data[o], &log->sol_age);
  log->num_tracked = data[o++];
  log->num_sol = data[o++];
  log->num_gg_l1 = data[o++];
  log->num_gg_l1_l2 = data[o++];
  ++o;  // Reserved.
  log->ext_sol_status = data[o++];
  ++o;  // Reserved.
  log->sig_mask = data[o++];
  assert(o == hdr->message_length);
  return true;
}

static bool DecodeBinaryBestXyz(const NovAtelHeader *hdr, const uint8_t *data,
                                NovAtelLog *out) {
  assert(out != NULL);
  return DecodeBinaryBestXyzImpl(hdr, data, &out->best_xyz);
}

static bool DecodeBinaryHeading(const NovAtelHeader *hdr, const uint8_t *data,
                                NovAtelLog *out) {
  assert(hdr != NULL);
  assert(data != NULL);
  assert(out != NULL);

  // Validate message length.
  if (hdr->message_length != 44) {
    return false;
  }

  NovAtelLogHeading *log = &out->heading;
  log->timestamp = hdr->timestamp;
  int32_t o = 0;
  o += ReadInt32Le(&data[o], &log->pos_sol_status);
  o += ReadInt32Le(&data[o], &log->pos_type);
  o += ReadFloatLe(&data[o], &log->length);
  o += ReadFloatLe(&data[o], &log->heading);
  log->heading *= (float)PI / 180.0f;
  o += ReadFloatLe(&data[o], &log->pitch);
  log->pitch *= (float)PI / 180.0f;
  o += 4;  // Reserved.
  o += ReadFloatLe(&data[o], &log->heading_sigma);
  log->heading_sigma *= (float)PI / 180.0f;
  o += ReadFloatLe(&data[o], &log->pitch_sigma);
  log->pitch_sigma *= (float)PI / 180.0f;
  for (int32_t i = 0; i < ARRAYSIZE(log->station_id); ++i) {
    log->station_id[i] = data[o++];
  }
  o += ReadUint8Le(&data[o], &log->num_tracked);
  o += ReadUint8Le(&data[o], &log->num_sol);
  o += ReadUint8Le(&data[o], &log->num_obs);
  o += ReadUint8Le(&data[o], &log->num_multi);
  o += ReadUint8Le(&data[o], &log->sol_source);
  o += ReadUint8Le(&data[o], &log->ext_sol_status);
  o += ReadUint8Le(&data[o], &log->galileo_beidou_mask);
  o += ReadUint8Le(&data[o], &log->gps_glonass_mask);
  assert(o == hdr->message_length);
  return true;
}

static bool DecodeBinaryHeadingRate(const NovAtelHeader *hdr,
                                    const uint8_t *data, NovAtelLog *out) {
  assert(hdr != NULL);
  assert(data != NULL);
  assert(out != NULL);

  // Validate message length.
  if (hdr->message_length != 52) {
    return false;
  }

  NovAtelLogHeadingRate *log = &out->heading_rate;
  log->timestamp = hdr->timestamp;
  int32_t o = 0;
  o += ReadInt32Le(&data[o], &log->pos_sol_status);
  o += ReadInt32Le(&data[o], &log->pos_type);
  o += ReadFloatLe(&data[o], &log->latency);
  o += ReadFloatLe(&data[o], &log->length_rate);
  o += ReadFloatLe(&data[o], &log->heading_rate);
  log->heading_rate *= (float)PI / 180.0f;
  o += ReadFloatLe(&data[o], &log->pitch_rate);
  log->pitch_rate *= (float)PI / 180.0f;
  o += ReadFloatLe(&data[o], &log->length_rate_sigma);
  o += ReadFloatLe(&data[o], &log->heading_rate_sigma);
  log->heading_rate_sigma *= (float)PI / 180.0f;
  o += ReadFloatLe(&data[o], &log->pitch_rate_sigma);
  log->pitch_rate_sigma *= (float)PI / 180.0f;
  o += 4;  // Reserved.
  for (int32_t i = 0; i < ARRAYSIZE(log->rover_id); ++i) {
    log->rover_id[i] = data[o++];
  }
  for (int32_t i = 0; i < ARRAYSIZE(log->master_id); ++i) {
    log->master_id[i] = data[o++];
  }
  o += ReadUint8Le(&data[o], &log->sol_source);
  o += 3;  // Reserved.
  assert(o == hdr->message_length);
  return true;
}

static bool DecodeBinaryHwMonitor(const NovAtelHeader *hdr, const uint8_t *data,
                                  NovAtelLog *out) {
  assert(hdr != NULL);
  assert(data != NULL);
  assert(out != NULL);

  // Validate minimum message length.
  if (hdr->message_length < 4) {
    return false;
  }

  NovAtelLogHwMonitor *log = &out->hw_monitor;
  memset(log, 0, sizeof(*log));
  log->timestamp = hdr->timestamp;
  int32_t o = 0;

  // Validate number of measurements.
  o += ReadInt32Le(&data[o], &log->num_measurements);
  if (hdr->message_length != 4 + 8 * log->num_measurements) {
    return false;
  }
  if (log->num_measurements > ARRAYSIZE(log->reading)) {
    log->num_measurements = ARRAYSIZE(log->reading);
  }

  // Parse each measurement.
  for (int32_t m = 0; m < log->num_measurements; ++m) {
    o += ReadFloatLe(&data[o], &log->reading[m]);
    o += ReadUint32Le(&data[o], &log->status[m]);
  }
  assert(o == 4 + 8 * log->num_measurements);
  return true;
}

static bool DecodeBinaryIonUtc(const NovAtelHeader *hdr, const uint8_t *data,
                               NovAtelLog *out) {
  assert(hdr != NULL);
  assert(data != NULL);
  assert(out != NULL);

  // Validate message length.
  if (hdr->message_length != 108) {
    return false;
  }

  NovAtelLogIonUtc *log = &out->ion_utc;
  log->timestamp = hdr->timestamp;
  int32_t i32, o = 0;
  uint32_t u32;

  o += ReadDoubleLe(&data[o], &log->iono.alpha0);
  o += ReadDoubleLe(&data[o], &log->iono.alpha1);
  o += ReadDoubleLe(&data[o], &log->iono.alpha2);
  o += ReadDoubleLe(&data[o], &log->iono.alpha3);
  o += ReadDoubleLe(&data[o], &log->iono.beta0);
  o += ReadDoubleLe(&data[o], &log->iono.beta1);
  o += ReadDoubleLe(&data[o], &log->iono.beta2);
  o += ReadDoubleLe(&data[o], &log->iono.beta3);
  o += ReadUint32Le(&data[o], &u32);
  log->utc.wnt = (uint16_t)u32;
  o += ReadUint32Le(&data[o], &log->utc.tot);
  o += ReadDoubleLe(&data[o], &log->utc.a0);
  o += ReadDoubleLe(&data[o], &log->utc.a1);
  o += ReadUint32Le(&data[o], &u32);
  log->utc.wn_lsf = (uint16_t)u32;
  o += ReadUint32Le(&data[o], &u32);
  log->utc.dn = (uint16_t)u32;
  o += ReadInt32Le(&data[o], &i32);
  log->utc.dt_ls = (int16_t)i32;
  o += ReadInt32Le(&data[o], &i32);
  log->utc.dt_lsf = (int16_t)i32;
  o += 4;  // Ignored dt_utc.
  assert(o == hdr->message_length);
  return true;
}

static bool DecodeBinaryPsrXyz(const NovAtelHeader *hdr, const uint8_t *data,
                               NovAtelLog *out) {
  assert(out != NULL);
  return DecodeBinaryBestXyzImpl(hdr, data, &out->psr_xyz);
}

static bool DecodeBinaryRange(const NovAtelHeader *hdr, const uint8_t *data,
                              NovAtelLog *out) {
  assert(hdr != NULL);
  assert(data != NULL);
  assert(out != NULL);

  // Validate minimum message length.
  if (hdr->message_length < 4) {
    return false;
  }

  NovAtelLogRange *log = &out->range;
  memset(log, 0, sizeof(*log));
  log->timestamp = hdr->timestamp;
  int32_t o = 0;

  // Validate number of observations.
  o += ReadInt32Le(&data[o], &log->num_obs);
  if (hdr->message_length != 4 + 44 * log->num_obs) {
    return false;
  }
  if (log->num_obs > ARRAYSIZE(log->prn)) {
    log->num_obs = ARRAYSIZE(log->prn);
  }

  // Parse each observation.
  for (int32_t obs = 0; obs < log->num_obs; ++obs) {
    o += ReadUint16Le(&data[o], &log->prn[obs]);
    o += ReadUint16Le(&data[o], &log->glofreq[obs]);
    o += ReadDoubleLe(&data[o], &log->psr[obs]);
    o += ReadFloatLe(&data[o], &log->psr_std[obs]);
    o += ReadDoubleLe(&data[o], &log->adr[obs]);
    o += ReadFloatLe(&data[o], &log->adr_std[obs]);
    o += ReadFloatLe(&data[o], &log->dopp[obs]);
    o += ReadFloatLe(&data[o], &log->cn0[obs]);
    o += ReadFloatLe(&data[o], &log->locktime[obs]);
    o += ReadUint32Le(&data[o], &log->status_bits[obs]);
  }
  assert(o == 4 + 44 * log->num_obs);
  return true;
}

static bool DecodeBinaryRawEphem(const NovAtelHeader *hdr, const uint8_t *data,
                                 NovAtelLog *out) {
  assert(hdr != NULL);
  assert(data != NULL);
  assert(out != NULL);

  // Validate message length.
  if (hdr->message_length != 102) {
    return false;
  }

  NovAtelLogRawEphem *log = &out->raw_ephem;
  log->timestamp = hdr->timestamp;
  int32_t o = 0;
  uint32_t u32;

  o += ReadUint32Le(&data[o], &u32);
  log->eph.prn = (uint8_t)u32;
  o += ReadUint32Le(&data[o], &u32);  // Field ref_week not used.
  o += ReadUint32Le(&data[o], &u32);  // Field ref_secs not used.
  log->eph.tow = hdr->timestamp.tow;
  log->eph.wnc = hdr->timestamp.week;

  uint32_t subframe[10];
  void (* const parse[])(const uint32_t *sf1, GpsEphemeris *eph) = {
    GpsParseSubframe1,
    GpsParseSubframe2,
    GpsParseSubframe3
  };
  for (int32_t sf = 0; sf < 3; ++sf) {
    for (int32_t i = 0; i < 10; ++i, o += 3) {
      subframe[i] = ((uint32_t)data[o + 0] << 22
                     | (uint32_t)data[o + 1] << 14
                     | (uint32_t)data[o + 2] << 6);
    }
    parse[sf](subframe, &log->eph);
  }
  assert(o == hdr->message_length);

  return true;
}

static bool DecodeBinaryRtkXyz(const NovAtelHeader *hdr, const uint8_t *data,
                               NovAtelLog *out) {
  assert(out != NULL);
  return DecodeBinaryBestXyzImpl(hdr, data, &out->rtk_xyz);
}

static bool DecodeBinaryRxConfig(const NovAtelHeader *hdr, const uint8_t *data,
                                 NovAtelLog *out) {
  assert(hdr != NULL);
  assert(data != NULL);
  assert(out != NULL);

  NovAtelLogRxConfig *log = &out->rx_config;
  if (!NovAtelBinaryDecodeHeader(hdr->message_length, data, &log->header)) {
    return false;
  }
  log->data = data + log->header.header_length;
  return true;
}

static bool DecodeBinaryRxStatus(const NovAtelHeader *hdr, const uint8_t *data,
                                 NovAtelLog *out) {
  assert(hdr != NULL);
  assert(data != NULL);
  assert(out != NULL);

  // Validate minimum message length.
  if (hdr->message_length < 8) {
    return false;
  }

  NovAtelLogRxStatus *log = &out->rx_status;
  memset(log, 0, sizeof(*log));
  log->timestamp = hdr->timestamp;
  int32_t o = 0;

  o += ReadUint32Le(&data[o], &log->error);

  // Validate number of statuses.
  o += ReadInt32Le(&data[o], &log->num_stats);
  if (hdr->message_length != 8 + 16 * log->num_stats) {
    return false;
  }
  if (log->num_stats > ARRAYSIZE(log->status)) {
    log->num_stats = ARRAYSIZE(log->status);
  }

  // Parse each status.
  for (int32_t s = 0; s < log->num_stats; ++s) {
    o += ReadUint32Le(&data[o], &log->status[s]);
    o += ReadUint32Le(&data[o], &log->priority[s]);
    o += ReadUint32Le(&data[o], &log->event_set[s]);
    o += ReadUint32Le(&data[o], &log->event_clear[s]);
  }
  assert(o == 8 + 16 * log->num_stats);
  return true;
}

static bool DecodeMessageFormat(const NovAtelHeader *hdr, const uint8_t *data,
                                DecodeFunc binary_fn, NovAtelLog *out) {
  assert(hdr != NULL);
  assert(data != NULL);
  assert(binary_fn != NULL);
  assert(out != NULL);

  switch (hdr->format) {
    case kNovAtelFormatBinary:
      return binary_fn(hdr, data, out);
    case kNovAtelFormatAscii:
      return false;  // Not supported.
    case kNovAtelFormatAbbAsciiNmea:
      return false;  // Not supported.
    default:
      break;
  }
  return false;
}

bool NovAtelBinaryDecodeHeader(int32_t length, const uint8_t *data,
                               NovAtelHeader *hdr) {
  assert(data != NULL);
  assert(hdr != NULL);

  uint16_t u16;
  int32_t i32;

  // See Table 4: Binary Message Header Structure.
  int32_t o = NOVATEL_SYNC_LENGTH;

  // Header length.
  hdr->header_length = data[o];
  ++o;
  if (hdr->header_length < NOVATEL_HEADER_LENGTH) {
    return false;
  }

  // Message id enumeration.
  o += ReadUint16Le(&data[o], &u16);
  hdr->message_id = (NovAtelMessageId)u16;

  // Message type: response bit.
  if ((data[o] & 0x80) != 0) {
    ReadInt32Le(&data[hdr->header_length], &i32);
    hdr->response = (NovAtelResponse)i32;
  } else {
    hdr->response = kNovAtelResponseNone;
  }

  // Message type: format.
  hdr->format = (NovAtelFormat)((data[o] >> 5) & 0x03);
  ++o;

  // Port enumeration.
  hdr->port = (NovAtelPort)data[o];
  ++o;

  // Message length.
  o += ReadUint16Le(&data[o], &u16);
  hdr->message_length = (int32_t)u16;
  if (hdr->header_length + hdr->message_length + NOVATEL_CRC_LENGTH
      != length) {
    return false;
  }

  // Other fields.
  o += ReadUint16Le(&data[o], &hdr->sequence);
  o += ReadUint8Le(&data[o], &hdr->idle_time);
  o += ReadUint8Le(&data[o], &hdr->timestamp.time_status);
  o += ReadUint16Le(&data[o], &hdr->timestamp.week);
  o += ReadUint32Le(&data[o], &hdr->timestamp.tow);
  o += ReadUint32Le(&data[o], &hdr->receiver_status);
  o += (int32_t)sizeof(uint16_t);  // Reserved.
  o += ReadUint16Le(&data[o], &hdr->receiver_sw_version);
  assert(o == NOVATEL_HEADER_LENGTH);
  return true;
}

bool NovAtelBinaryDecode(const NovAtelHeader *hdr, const uint8_t *data,
                         NovAtelLog *out) {
  assert(hdr != NULL);
  assert(data != NULL);
  assert(out != NULL);

  switch (hdr->message_id) {
    // Invalid.
    case kNovAtelMessageIdNone:
      break;

      // NovAtel command messages.
    case kNovAtelMessageIdCom:
    case kNovAtelMessageIdInterfaceMode:
    case kNovAtelMessageIdLog:
      break;

      // NovAtel log messages.
    case kNovAtelMessageIdBestXyz:
      return DecodeMessageFormat(hdr, &data[hdr->header_length],
                                 DecodeBinaryBestXyz, out);
    case kNovAtelMessageIdHeading:
      return DecodeMessageFormat(hdr, &data[hdr->header_length],
                                 DecodeBinaryHeading, out);
    case kNovAtelMessageIdHeadingRate:
      return DecodeMessageFormat(hdr, &data[hdr->header_length],
                                 DecodeBinaryHeadingRate, out);
    case kNovAtelMessageIdHwMonitor:
      return DecodeMessageFormat(hdr, &data[hdr->header_length],
                                 DecodeBinaryHwMonitor, out);
    case kNovAtelMessageIdIonUtc:
      return DecodeMessageFormat(hdr, &data[hdr->header_length],
                                 DecodeBinaryIonUtc, out);
    case kNovAtelMessageIdPsrXyz:
      return DecodeMessageFormat(hdr, &data[hdr->header_length],
                                 DecodeBinaryPsrXyz, out);
    case kNovAtelMessageIdRange:
      return DecodeMessageFormat(hdr, &data[hdr->header_length],
                                 DecodeBinaryRange, out);
    case kNovAtelMessageIdRawEphem:
      return DecodeMessageFormat(hdr, &data[hdr->header_length],
                                 DecodeBinaryRawEphem, out);
    case kNovAtelMessageIdRtkXyz:
      return DecodeMessageFormat(hdr, &data[hdr->header_length],
                                 DecodeBinaryRtkXyz, out);
    case kNovAtelMessageIdRxConfig:
      return DecodeMessageFormat(hdr, &data[hdr->header_length],
                                 DecodeBinaryRxConfig, out);
    case kNovAtelMessageIdRxStatus:
      return DecodeMessageFormat(hdr, &data[hdr->header_length],
                                 DecodeBinaryRxStatus, out);
    default:
      break;
  }
  return false;
}

// Write header and CRC common to all messages.
static int32_t WriteCommon(NovAtelMessageId message_id,
                           NovAtelFormat message_format,
                           int32_t header_plus_message_length,
                           uint8_t *data) {
  assert((message_id & 0xFFFF) == message_id);
  assert(header_plus_message_length >= NOVATEL_HEADER_LENGTH);
  assert(header_plus_message_length % 4 == 0);
  assert(header_plus_message_length <=
         (NOVATEL_WRITE_LENGTH - NOVATEL_CRC_LENGTH));

  // Message header. See Table 4: Binary Message Header Structure.
  int32_t o = 0;
  data[o++] = NOVATEL_SYNC0;
  data[o++] = NOVATEL_SYNC1;
  data[o++] = NOVATEL_SYNC2;
  data[o++] = NOVATEL_HEADER_LENGTH;
  o += WriteUint16Le(message_id, &data[o]);
  data[o++] = (uint8_t)(message_format << 5);
  data[o++] = (uint8_t)kNovAtelPortThisPort;
  int32_t message_length = header_plus_message_length - NOVATEL_HEADER_LENGTH;
  o += WriteUint16Le((uint16_t)message_length, &data[o]);
  while (o < NOVATEL_HEADER_LENGTH) {
    data[o++] = 0;
  }

  // Message data.
  o += message_length;

  // Message CRC.
  uint32_t crc32 = ~Crc32(NOVATEL_CRC_INIT, o, data);
  o += WriteUint32Le(crc32, &data[o]);

  return o;  // Total length written.
}

// See Section 2.5.14: COM COM port configuration control.
// Warning! "Use the COM command before using the INTERFACEMODE command on each
// port. Turn break detection off using the COM command to stop the port from
// resetting because it is interpreting incoming bits as a break command."
int32_t NovAtelBinaryWriteCom(NovAtelPort port, uint32_t baud,
                              bool enable_break, uint8_t *data) {
  int32_t o = NOVATEL_HEADER_LENGTH;
  o += WriteInt32Le(port, &data[o]);
  o += WriteUint32Le(baud, &data[o]);
  o += WriteInt32Le(0, &data[o]);  // Parity.
  o += WriteInt32Le(8, &data[o]);  // Data bits.
  o += WriteInt32Le(1, &data[o]);  // Stop bits.
  o += WriteInt32Le(0, &data[o]);  // Handshaking.
  o += WriteInt32Le(0, &data[o]);  // Echo.
  o += WriteInt32Le(enable_break, &data[o]);
  return WriteCommon(kNovAtelMessageIdCom, kNovAtelFormatBinary, o,
                     data);
}

// Set interface mode (i.e., types of messages accepted) for specified port.
int32_t NovAtelBinaryWriteInterfaceMode(NovAtelPort port, NovAtelPortMode rx,
                                        NovAtelPortMode tx, uint8_t *data) {
  int32_t o = NOVATEL_HEADER_LENGTH;
  o += WriteInt32Le(port, &data[o]);
  o += WriteInt32Le(rx, &data[o]);  // Received interface mode.
  o += WriteInt32Le(tx, &data[o]);  // Transmit interface mode.
  o += WriteInt32Le(1, &data[o]);   // Response generation.
  return WriteCommon(kNovAtelMessageIdInterfaceMode, kNovAtelFormatBinary, o,
                     data);
}

// Request data logs.
int32_t NovAtelBinaryWriteLog(NovAtelPort output_port,
                              NovAtelMessageId message_id, NovAtelFormat format,
                              NovAtelTrigger trigger, double period,
                              double offset, bool hold, uint8_t *data) {
  int32_t o = NOVATEL_HEADER_LENGTH;
  o += WriteInt32Le(output_port, &data[o]);
  o += WriteInt16Le(message_id, &data[o]);
  o += WriteUint8Le((uint8_t)(format << 5), &data[o]);
  o += (int32_t)sizeof(uint8_t);  // Reserved
  o += WriteInt32Le(trigger, &data[o]);
  o += WriteDoubleLe(period, &data[o]);
  o += WriteDoubleLe(offset, &data[o]);
  o += WriteInt32Le(hold, &data[o]);
  return WriteCommon(kNovAtelMessageIdLog, kNovAtelFormatBinary, o, data);
}
