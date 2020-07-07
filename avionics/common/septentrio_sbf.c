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

#include "avionics/common/septentrio_sbf.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/common/septentrio_types.h"

static int32_t ReadTimestamp(const uint8_t *data, SeptentrioTimestamp *ts) {
  int32_t o = 0;
  o += ReadUint32Le(&data[o], &ts->tow);
  o += ReadUint16Le(&data[o], &ts->wnc);
  return o;
}

#define MEAS_EPOCH_TYPE2_LENGTH 12

static bool DecodeMeasEpochType2(int32_t obs_sb1, int32_t obs,
                                 int32_t sb2_length, int32_t data_length,
                                 const uint8_t *data,
                                 SeptentrioBlockMeasEpoch *blk) {
  assert(0 <= obs_sb1 && obs_sb1 < SEPTENTRIO_OBSERVATIONS);
  assert(0 <= obs && obs < SEPTENTRIO_OBSERVATIONS);

  // Validate data length.
  if (sb2_length < MEAS_EPOCH_TYPE2_LENGTH || data_length < sb2_length) {
    return false;
  }

  // Parse.
  blk->rx_channel[obs] = blk->rx_channel[obs_sb1];
  blk->svid[obs] = blk->svid[obs_sb1];
  blk->type[obs] = data[0];
  blk->locktime[obs] = data[1];
  blk->cn0[obs] = data[2];
  blk->info[obs] = data[5];

  // Code.
  uint32_t u32 = (uint32_t)((data[3] & 0x07) << 16 | data[7] << 8 | data[6]);
  blk->code[obs] = blk->code[obs_sb1] + SignExtend(u32, 19);

  // Doppler.
  u32 = (uint32_t)((data[3] & 0xF8) << 16 | data[11] << 8 | data[10]);
  blk->doppler[obs] = blk->doppler[obs_sb1] + SignExtend(u32, 21);

  // Carrier.
  u32 = (uint32_t)(data[4] << 16 | data[9] << 8 | data[8]);
  blk->carrier[obs] = blk->carrier[obs_sb1] + SignExtend(u32, 24);

  return true;
}

#define MEAS_EPOCH_TYPE1_LENGTH 20

static int32_t DecodeMeasEpochType1(int32_t sb1_length, int32_t sb2_length,
                                    int32_t data_length, const uint8_t *data,
                                    SeptentrioBlockMeasEpoch *blk) {
  // Validate data length.
  if (sb1_length < MEAS_EPOCH_TYPE1_LENGTH || data_length < sb1_length) {
    return -1;
  }

  // Number of observations.
  int32_t obs = blk->num_obs;
  ++blk->num_obs;

  // Observation channel, type, prn, misc.
  int32_t o = 0;
  uint8_t misc;
  o += ReadUint8Le(&data[o], &blk->rx_channel[obs]);
  o += ReadUint8Le(&data[o], &blk->type[obs]);
  o += ReadUint8Le(&data[o], &blk->svid[obs]);
  o += ReadUint8Le(&data[o], &misc);

  // Code (36-bit, unsigned).
  uint32_t code_lsb;
  o += ReadUint32Le(&data[o], &code_lsb);
  blk->code[obs] = (int64_t)(misc & 0x0F) << 32 | (int64_t)code_lsb;

  // Read Doppler (32-bit, signed).
  o += ReadInt32Le(&data[o], &blk->doppler[obs]);

  // Read carrier (24-bit, signed).
  uint16_t carrier_lsb;
  uint8_t carrier_msb;
  o += ReadUint16Le(&data[o], &carrier_lsb);
  o += ReadUint8Le(&data[o], &carrier_msb);
  uint32_t u32 = (uint32_t)(carrier_msb << 16 | carrier_lsb);
  blk->carrier[obs] = SignExtend(u32, 24);

  // Read C/N0.
  o += ReadUint8Le(&data[o], &blk->cn0[obs]);

  // Read Lock time.
  uint16_t locktime;
  o += ReadUint16Le(&data[o], &locktime);

  // Limit lock time to 254 seconds to be consistent with type 2 messages.
  if (locktime == 0xFFFF) {
    blk->locktime[obs] = 0xFF;
  } else if (locktime > 254) {
    blk->locktime[obs] = 254;
  } else {
    blk->locktime[obs] = (uint8_t)locktime;
  }

  // Read observation info.
  o += ReadUint8Le(&data[o], &blk->info[obs]);

  // Read number of type 2 sub-blocks.
  int32_t num_sb2 = (int32_t)data[o];
  ++o;

  // Skip padding.
  assert(MEAS_EPOCH_TYPE1_LENGTH == o);
  o = sb1_length;

  // Read type 2 sub-blocks.
  for (int32_t sb2 = 0;
       sb2 < num_sb2 && blk->num_obs < SEPTENTRIO_OBSERVATIONS;
       ++sb2, ++blk->num_obs) {
    if (DecodeMeasEpochType2(obs, blk->num_obs, sb2_length, data_length - o,
                             &data[o], blk)) {
      o += sb2_length;
    } else {
      return -1;
    }
  }
  return o;
}

#define MEAS_EPOCH_LENGTH 12

static bool DecodeMeasEpoch(const SeptentrioHeader *hdr,
                            const uint8_t *data,
                            SeptentrioBlockMeasEpoch *blk) {
  // Validate data length.
  if (hdr->data_length < MEAS_EPOCH_LENGTH) {
    return false;
  }
  memset(blk, 0, sizeof(*blk));

  // Timestamp.
  int32_t o = 0;
  o += ReadTimestamp(&data[o], &blk->timestamp);

  // Read number of type 1 sub-blocks and lengths.
  int32_t num_sb1 = (int32_t)data[o];
  ++o;
  int32_t sb1_length = (int32_t)data[o];
  ++o;
  int32_t sb2_length = (int32_t)data[o];
  ++o;

  // Common.
  blk->common_flags = data[o];
  ++o;
  blk->cum_clk_jumps = (hdr->block_rev >= 1) ? data[o] : 0;  // Added in rev 1.
  ++o;
  ++o;  // Reserved.
  assert(MEAS_EPOCH_LENGTH == o);

  // Process sub-blocks.
  blk->num_obs = 0;
  for (int32_t sb1 = 0;
       sb1 < num_sb1 && blk->num_obs < SEPTENTRIO_OBSERVATIONS; ++sb1) {
    int32_t parsed = DecodeMeasEpochType1(sb1_length, sb2_length,
                                          hdr->data_length - o, &data[o], blk);
    if (parsed > 0) {
      o += parsed;
    } else {
      return false;
    }
  }
  return true;
}

#define PVT_CARTESIAN_LENGTH 77

static bool DecodePvtCartesian(const SeptentrioHeader *hdr,
                               const uint8_t *data,
                               SeptentrioBlockPvtCartesian *blk) {
  // Validate data length.
  if (hdr->data_length < PVT_CARTESIAN_LENGTH) {
    return false;
  }
  memset(blk, 0, sizeof(*blk));

  // Parse.
  int32_t o = 0;
  o += ReadTimestamp(&data[o], &blk->timestamp);
  o += ReadUint8Le(&data[o], &blk->mode);
  o += ReadUint8Le(&data[o], &blk->error);
  o += ReadDoubleLe(&data[o], &blk->x);
  o += ReadDoubleLe(&data[o], &blk->y);
  o += ReadDoubleLe(&data[o], &blk->z);
  o += ReadFloatLe(&data[o], &blk->undulation);
  o += ReadFloatLe(&data[o], &blk->v_x);
  o += ReadFloatLe(&data[o], &blk->v_y);
  o += ReadFloatLe(&data[o], &blk->v_z);
  o += ReadFloatLe(&data[o], &blk->cog);
  o += ReadDoubleLe(&data[o], &blk->rx_clk_bias);
  o += ReadFloatLe(&data[o], &blk->rx_clk_drift);
  o += ReadUint8Le(&data[o], &blk->time_system);
  o += ReadUint8Le(&data[o], &blk->datum);
  o += ReadUint8Le(&data[o], &blk->nr_sv);
  o += ReadUint8Le(&data[o], &blk->wa_corr_info);
  o += ReadUint16Le(&data[o], &blk->reference_id);
  o += ReadUint16Le(&data[o], &blk->mean_corr_age);
  o += ReadUint32Le(&data[o], &blk->signal_info);
  o += ReadUint8Le(&data[o], &blk->alert_flag);
  assert(PVT_CARTESIAN_LENGTH == o);

  // Added in revision 1.
  blk->nr_biases = 0;
  blk->ppp_info = 0;

  // Added in revision 2.
  blk->latency = 0;
  blk->h_accuracy = 0;
  blk->v_accuracy = 0;
  blk->misc = 0;

  if ((hdr->data_length - o) >= 3 && hdr->block_rev >= 1) {
    o += ReadUint8Le(&data[o], &blk->nr_biases);
    o += ReadUint16Le(&data[o], &blk->ppp_info);
    assert(hdr->data_length >= o);

    if ((hdr->data_length - o) >= 7 && hdr->block_rev >= 2) {
      o += ReadUint16Le(&data[o], &blk->latency);
      o += ReadUint16Le(&data[o], &blk->h_accuracy);
      o += ReadUint16Le(&data[o], &blk->v_accuracy);
      o += ReadUint8Le(&data[o], &blk->misc);
      assert(hdr->data_length >= o);
    }
  }
  return true;
}

#define POS_COV_CARTESIAN_LENGTH 48

static bool DecodePosCovCartesian(const SeptentrioHeader *hdr,
                                  const uint8_t *data,
                                  SeptentrioBlockPosCovCartesian *blk) {
  // Validate data length.
  if (hdr->data_length < POS_COV_CARTESIAN_LENGTH) {
    return false;
  }
  memset(blk, 0, sizeof(*blk));

  // Parse.
  int32_t o = 0;
  o += ReadTimestamp(&data[o], &blk->timestamp);
  o += ReadUint8Le(&data[o], &blk->mode);
  o += ReadUint8Le(&data[o], &blk->error);
  o += ReadFloatLe(&data[o], &blk->cov_xx);
  o += ReadFloatLe(&data[o], &blk->cov_yy);
  o += ReadFloatLe(&data[o], &blk->cov_zz);
  o += ReadFloatLe(&data[o], &blk->cov_bb);
  o += ReadFloatLe(&data[o], &blk->cov_xy);
  o += ReadFloatLe(&data[o], &blk->cov_xz);
  o += ReadFloatLe(&data[o], &blk->cov_xb);
  o += ReadFloatLe(&data[o], &blk->cov_yz);
  o += ReadFloatLe(&data[o], &blk->cov_yb);
  o += ReadFloatLe(&data[o], &blk->cov_zb);
  assert(POS_COV_CARTESIAN_LENGTH == o);
  return true;
}

#define VEL_COV_CARTESIAN_LENGTH 48

static bool DecodeVelCovCartesian(const SeptentrioHeader *hdr,
                                  const uint8_t *data,
                                  SeptentrioBlockVelCovCartesian *blk) {
  // Validate data length.
  if (hdr->data_length < VEL_COV_CARTESIAN_LENGTH) {
    return false;
  }
  memset(blk, 0, sizeof(*blk));

  // Parse.
  int32_t o = 0;
  o += ReadTimestamp(&data[o], &blk->timestamp);
  o += ReadUint8Le(&data[o], &blk->mode);
  o += ReadUint8Le(&data[o], &blk->error);
  o += ReadFloatLe(&data[o], &blk->cov_xx);
  o += ReadFloatLe(&data[o], &blk->cov_yy);
  o += ReadFloatLe(&data[o], &blk->cov_zz);
  o += ReadFloatLe(&data[o], &blk->cov_tt);
  o += ReadFloatLe(&data[o], &blk->cov_xy);
  o += ReadFloatLe(&data[o], &blk->cov_xz);
  o += ReadFloatLe(&data[o], &blk->cov_xt);
  o += ReadFloatLe(&data[o], &blk->cov_yz);
  o += ReadFloatLe(&data[o], &blk->cov_yt);
  o += ReadFloatLe(&data[o], &blk->cov_zt);
  assert(VEL_COV_CARTESIAN_LENGTH == o);
  return true;
}

#define BASE_VECTOR_INFO_CART_LENGTH 52

static bool DecodeBaseVectorInfoCart(int32_t idx, int32_t sb1_length,
                                     int32_t data_length, const uint8_t *data,
                                     SeptentrioBlockBaseVectorCart *blk) {
  assert(0 <= idx && idx <= SEPTENTRIO_BASE_STATIONS);

  // Validate minimum data length.
  if (sb1_length < BASE_VECTOR_INFO_CART_LENGTH || data_length < sb1_length) {
    return false;
  }

  // Parse.
  int32_t o = 0;
  o += ReadUint8Le(&data[o], &blk->nr_sv[idx]);
  o += ReadUint8Le(&data[o], &blk->error[idx]);
  o += ReadUint8Le(&data[o], &blk->mode[idx]);
  o += ReadUint8Le(&data[o], &blk->misc[idx]);
  o += ReadDoubleLe(&data[o], &blk->delta_posx[idx]);
  o += ReadDoubleLe(&data[o], &blk->delta_posy[idx]);
  o += ReadDoubleLe(&data[o], &blk->delta_posz[idx]);
  o += ReadFloatLe(&data[o], &blk->delta_velx[idx]);
  o += ReadFloatLe(&data[o], &blk->delta_vely[idx]);
  o += ReadFloatLe(&data[o], &blk->delta_velz[idx]);
  o += ReadUint16Le(&data[o], &blk->azimuth[idx]);
  o += ReadInt16Le(&data[o], &blk->elevation[idx]);
  o += ReadUint16Le(&data[o], &blk->ref_id[idx]);
  o += ReadUint16Le(&data[o], &blk->corr_age[idx]);
  o += ReadUint32Le(&data[o], &blk->signal_info[idx]);
  assert(BASE_VECTOR_INFO_CART_LENGTH == o);
  return true;
}

#define BASE_VECTOR_CART_LENGTH 8

static bool DecodeBaseVectorCart(const SeptentrioHeader *hdr,
                                 const uint8_t *data,
                                 SeptentrioBlockBaseVectorCart *blk) {
  // Validate minimum data length.
  if (hdr->data_length < BASE_VECTOR_CART_LENGTH) {
    return false;
  }
  memset(blk, 0, sizeof(*blk));

  // Common.
  int32_t o = 0;
  o += ReadTimestamp(&data[o], &blk->timestamp);
  o += ReadUint8Le(&data[o], &blk->num_base_stations);

  // Read sub-block 1 length.
  int32_t sb1_length = data[o];
  ++o;
  assert(BASE_VECTOR_CART_LENGTH == o);

  // Sub-block 1.
  if (blk->num_base_stations > SEPTENTRIO_BASE_STATIONS) {
    blk->num_base_stations = SEPTENTRIO_BASE_STATIONS;
  }
  for (uint8_t b = 0; b < blk->num_base_stations; ++b) {
    if (DecodeBaseVectorInfoCart(b, sb1_length, hdr->data_length - o, &data[o],
                                 blk)) {
      o += sb1_length;
    } else {
      return false;
    }
  }
  return true;
}

#define END_OF_PVT_LENGTH 6

static bool DecodeEndOfPvt(const SeptentrioHeader *hdr, const uint8_t *data,
                           SeptentrioBlockEndOfPvt *blk) {
  // Validate data length.
  if (hdr->data_length < END_OF_PVT_LENGTH) {
    return false;
  }
  memset(blk, 0, sizeof(*blk));

  // Parse.
  int32_t o = 0;
  o += ReadTimestamp(&data[o], &blk->timestamp);
  assert(END_OF_PVT_LENGTH == o);
  return true;
}

#define GPS_NAV_LENGTH 132

static bool DecodeGpsNav(const SeptentrioHeader *hdr, const uint8_t *data,
                         SeptentrioBlockGpsNav *blk) {
  // Validate data length.
  if (hdr->data_length < GPS_NAV_LENGTH) {
    return false;
  }
  memset(blk, 0, sizeof(*blk));

  // Parse.
  int32_t o = 0;
  o += ReadTimestamp(&data[o], &blk->timestamp);
  blk->eph.tow = blk->timestamp.tow;
  blk->eph.wnc = blk->timestamp.wnc;

  o += ReadUint8Le(&data[o], &blk->eph.prn);
  ++o;  // Reserved.
  o += ReadUint16Le(&data[o], &blk->eph.wn);  // [week].
  o += ReadUint8Le(&data[o], &blk->eph.l2_ca_or_p);
  o += ReadUint8Le(&data[o], &blk->eph.ura);
  o += ReadUint8Le(&data[o], &blk->eph.health);
  o += ReadUint8Le(&data[o], &blk->eph.l2pdata);
  o += ReadUint16Le(&data[o], &blk->eph.iodc);
  o += ReadUint8Le(&data[o], &blk->eph.iode2);
  o += ReadUint8Le(&data[o], &blk->eph.iode3);
  o += ReadUint8Le(&data[o], &blk->eph.fit_interval_flag);
  ++o;  // Reserved.
  o += ReadFloatLe(&data[o], &blk->eph.t_gd);       // [s].
  o += ReadUint32Le(&data[o], &blk->eph.t_oc);      // [s].
  o += ReadFloatLe(&data[o], &blk->eph.a_f2);       // [s/s/s].
  o += ReadFloatLe(&data[o], &blk->eph.a_f1);       // [s/s].
  o += ReadFloatLe(&data[o], &blk->eph.a_f0);       // [s].
  o += ReadFloatLe(&data[o], &blk->eph.c_rs);       // [m].
  o += ReadFloatLe(&data[o], &blk->eph.delta_n);    // [semi-circle/s].
  o += ReadDoubleLe(&data[o], &blk->eph.m_0);       // [semi-circle].
  o += ReadFloatLe(&data[o], &blk->eph.c_uc);       // [rad].
  o += ReadDoubleLe(&data[o], &blk->eph.ecc);       // [#].
  o += ReadFloatLe(&data[o], &blk->eph.c_us);       // [rad].
  o += ReadDoubleLe(&data[o], &blk->eph.sqrt_a);    // [m^(1/2)].
  o += ReadUint32Le(&data[o], &blk->eph.t_oe);      // [s].
  o += ReadFloatLe(&data[o], &blk->eph.c_ic);       // [rad].
  o += ReadDoubleLe(&data[o], &blk->eph.omega_0);   // [semi-circle].
  o += ReadFloatLe(&data[o], &blk->eph.c_is);       // [rad].
  o += ReadDoubleLe(&data[o], &blk->eph.i_0);       // [semi-circle].
  o += ReadFloatLe(&data[o], &blk->eph.c_rc);       // [m].
  o += ReadDoubleLe(&data[o], &blk->eph.omega);     // [semi-circle].
  o += ReadFloatLe(&data[o], &blk->eph.omega_dot);  // [semi-circle/s].
  o += ReadFloatLe(&data[o], &blk->eph.i_dot);      // [semi-circle/s].
  o += 2;  // Ignore wnt_oc [week].
  o += 2;  // Ignore wnt_oe [week].
  assert(GPS_NAV_LENGTH == o);
  return true;
}

#define GPS_ION_LENGTH 40

static bool DecodeGpsIon(const SeptentrioHeader *hdr, const uint8_t *data,
                         SeptentrioBlockGpsIon *blk) {
  // Validate data length.
  if (hdr->data_length < GPS_ION_LENGTH) {
    return false;
  }
  memset(blk, 0, sizeof(*blk));

  // Parse.
  int32_t o = 0;
  float f;
  o += ReadTimestamp(&data[o], &blk->timestamp);
  o += ReadUint8Le(&data[o], &blk->prn);
  ++o;  // Reserved.
  o += ReadFloatLe(&data[o], &f);
  blk->iono.alpha0 = (double)f;  // [s].
  o += ReadFloatLe(&data[o], &f);
  blk->iono.alpha1 = (double)f;  // [s/semi-circle].
  o += ReadFloatLe(&data[o], &f);
  blk->iono.alpha2 = (double)f;  // [s/semi-circle^2].
  o += ReadFloatLe(&data[o], &f);
  blk->iono.alpha3 = (double)f;  // [s/semi-circle^3].
  o += ReadFloatLe(&data[o], &f);
  blk->iono.beta0 = (double)f;   // [s].
  o += ReadFloatLe(&data[o], &f);
  blk->iono.beta1 = (double)f;   // [s/semi-circle].
  o += ReadFloatLe(&data[o], &f);
  blk->iono.beta2 = (double)f;   // [s/semi-circle^2].
  o += ReadFloatLe(&data[o], &f);
  blk->iono.beta3 = (double)f;   // [s/semi-circle^3].
  assert(GPS_ION_LENGTH == o);
  return true;
}

#define GPS_UTC_LENGTH 29

static bool DecodeGpsUtc(const SeptentrioHeader *hdr, const uint8_t *data,
                         SeptentrioBlockGpsUtc *blk) {
  // Validate data length.
  if (hdr->data_length < GPS_UTC_LENGTH) {
    return false;
  }
  memset(blk, 0, sizeof(*blk));

  // Parse.
  int32_t o = 0;
  float f;
  o += ReadTimestamp(&data[o], &blk->timestamp);
  o += ReadUint8Le(&data[o], &blk->prn);
  ++o;  // Reserved.
  o += ReadFloatLe(&data[o], &f);
  blk->utc.a1 = (double)f;  // [s/s].
  o += ReadDoubleLe(&data[o], &blk->utc.a0);   // [s].
  o += ReadUint32Le(&data[o], &blk->utc.tot);  // [s].
  // GPS-ICD-200: WNt transmitted as week mod 256.
  blk->utc.wnt = (uint16_t)((blk->timestamp.wnc & ~0xFF) | data[o]);  // [week].
  ++o;
  blk->utc.dt_ls = (int8_t)data[o];  // [s].
  ++o;
  // GPS-ICD-200: WN_lsf transmitted as week mod 256.
  blk->utc.wn_lsf = (uint16_t)((blk->timestamp.wnc & ~0xFF)
                               | data[o]);  // [week].
  ++o;
  blk->utc.dn = data[o];  // [day].
  ++o;
  blk->utc.dt_lsf = data[o];  // [s].
  ++o;
  assert(GPS_UTC_LENGTH == o);
  return true;
}

// Decode Septentrio binary format (SBF).
//
// Args:
//   hdr: Parsed SBF message header.
//   data: SBF message data pointer immediately following the header.
//   blk: Output union. Use the header->block_id to determine the type.
//
// Return:
//   True indicates the output contains valid data.
bool SeptentrioSbfDecode(const SeptentrioHeader *hdr, const uint8_t *data,
                         SeptentrioBlock *blk) {
  switch (hdr->block_id) {
    // Measurement blocks.
    case kSeptentrioIdMeasEpoch:
      return DecodeMeasEpoch(hdr, &data[hdr->header_length], &blk->meas_epoch);
    case kSeptentrioIdMeasExtra:
    case kSeptentrioIdIqCorr:
    case kSeptentrioIdEndOfMeas:
      // Fall through intentional.
      return false;

      // Navigation page blocks.
    case kSeptentrioIdGpsRawCa:
    case kSeptentrioIdGpsRawL2c:
    case kSeptentrioIdGloRawCa:
    case kSeptentrioIdGalINav:
    case kSeptentrioIdGeoRawL1:
    case kSeptentrioIdQzsRawL1Ca:
    case kSeptentrioIdQzsRawL2c:
      // Fall through intentional.
      return false;

      // GPS decoded message blocks.
    case kSeptentrioIdGpsNav:
      return DecodeGpsNav(hdr, &data[hdr->header_length], &blk->gps_nav);
    case kSeptentrioIdGpsAlm:
      return false;
    case kSeptentrioIdGpsIon:
      return DecodeGpsIon(hdr, &data[hdr->header_length], &blk->gps_ion);
    case kSeptentrioIdGpsUtc:
      return DecodeGpsUtc(hdr, &data[hdr->header_length], &blk->gps_utc);

      // GLONASS decoded message blocks.
    case kSeptentrioIdGloNav:
    case kSeptentrioIdGloAlm:
    case kSeptentrioIdGloTime:
      // Fall through intentional.
      return false;

      // Galileo decoded message blocks.
    case kSeptentrioIdGalNav:
    case kSeptentrioIdGalAlm:
    case kSeptentrioIdGalIon:
    case kSeptentrioIdGalUtc:
    case kSeptentrioIdGstGps:
    case kSeptentrioIdGalSarRlm:
      // Fall through intentional.
      return false;

      // SBAS decoded message blocks.
    case kSeptentrioIdGeoMt00:
    case kSeptentrioIdGeoPrnMask:
    case kSeptentrioIdGeoFastCorr:
    case kSeptentrioIdGeoIntegrity:
    case kSeptentrioIdGeoFastCorrDegr:
    case kSeptentrioIdGeoNav:
    case kSeptentrioIdGeoDegrFactors:
    case kSeptentrioIdGeoNetworkTime:
    case kSeptentrioIdGeoAlm:
    case kSeptentrioIdGeoIgpMask:
    case kSeptentrioIdGeoLongTermCorr:
    case kSeptentrioIdGeoIonoDelay:
    case kSeptentrioIdGeoServiceLevel:
    case kSeptentrioIdGeoClockEphCovMatrix:
      // Fall through intentional.
      return false;

      // Position, velocity, time blocks.
    case kSeptentrioIdPvtCartesian:
      return DecodePvtCartesian(hdr, &data[hdr->header_length],
                                &blk->pvt_cartesian);
    case kSeptentrioIdPvtGeodetic:
      return false;
    case kSeptentrioIdPosCovCartesian:
      return DecodePosCovCartesian(hdr, &data[hdr->header_length],
                                   &blk->pos_cov_cartesian);
    case kSeptentrioIdPosCovGeodetic:
      return false;
    case kSeptentrioIdVelCovCartesian:
      return DecodeVelCovCartesian(hdr, &data[hdr->header_length],
                                   &blk->vel_cov_cartesian);
    case kSeptentrioIdVelCovGeodetic:
    case kSeptentrioIdDop:
    case kSeptentrioIdPosCart:
    case kSeptentrioIdPosLocal:
    case kSeptentrioIdPvtSatCartesian:
    case kSeptentrioIdPvtResiduals:
    case kSeptentrioIdRaimStatistics:
    case kSeptentrioIdGeoCorrections:
      // Fall through intentional.
      return false;
    case kSeptentrioIdBaseVectorCart:
      return DecodeBaseVectorCart(hdr, &data[hdr->header_length],
                                  &blk->base_vector_cart);
    case kSeptentrioIdBaseVectorGeod:
    case kSeptentrioIdPvtSupport:
      // Fall through intentional.
      return false;
    case kSeptentrioIdEndOfPvt:
      return DecodeEndOfPvt(hdr, &data[hdr->header_length],
                            &blk->end_of_pvt);

      // GNSS attitude blocks.
    case kSeptentrioIdAttEuler:
    case kSeptentrioIdAttCovEuler:
    case kSeptentrioIdEndOfAtt:
      // Fall through intentional.
      return false;

      // Receiver time blocks.
    case kSeptentrioIdReceiverTime:
    case kSeptentrioIdXPpsOffset:
      // Fall through intentional.
      return false;

      // External event blocks.
    case kSeptentrioIdExtEvent:
    case kSeptentrioIdExtEventPvtCartesian:
    case kSeptentrioIdExtEventPvtGeodetic:
      // Fall through intentional.
      return false;

      // Differential correction blocks.
    case kSeptentrioIdDiffCorrIn:
    case kSeptentrioIdBaseStation:
    case kSeptentrioIdRtcmDatum:
      // Fall through intentional.
      return false;

      // Status blocks.
    case kSeptentrioIdChannelStatus:
    case kSeptentrioIdReceiverStatus:
    case kSeptentrioIdSatVisibility:
    case kSeptentrioIdInputLink:
    case kSeptentrioIdOutputLink:
    case kSeptentrioIdQualityInd:
      // Fall through intentional.
      return false;

      // Miscellaneous blocks.
    case kSeptentrioIdReceiverSetup:
    case kSeptentrioIdCommands:
    case kSeptentrioIdComment:
    case kSeptentrioIdAsciiIn:
      // Fall through intentional.
      return false;

      // Default.
    default:
      return false;
  }
}
