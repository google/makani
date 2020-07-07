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

#ifndef AVIONICS_COMMON_NOVATEL_TYPES_H_
#define AVIONICS_COMMON_NOVATEL_TYPES_H_

#include <stdint.h>

#include "avionics/common/gps_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NOVATEL_HWMONITOR_MEASUREMENTS 32
#define NOVATEL_OBSERVATIONS 32

// See Table 4: Binary Message Header Structure.
typedef enum {
  kNovAtelFormatBinary       = 0,
  kNovAtelFormatAscii        = 1,
  kNovAtelFormatAbbAsciiNmea = 2
} NovAtelFormat;

// See Table 5: Detailed Serial Port Identifiers.
typedef enum {
  kNovAtelPortNoPorts     = 0,
  kNovAtelPortCom1All     = 1,
  kNovAtelPortCom2All     = 2,
  kNovAtelPortThisPortAll = 6,
  kNovAtelPortAllPorts    = 8,
  kNovAtelPortCom1        = 32,
  kNovAtelPortCom2        = 64,
  kNovAtelPortThisPort    = 192
} NovAtelPort;

// See Table 8: GPS Time Status.
typedef enum {
  kNovAtelTimeUnknown         = 20,
  kNovAtelTimeApproximate     = 60,
  kNovAtelTimeCoarseAdjusting = 80,
  kNovAtelTimeCoarse          = 100,
  kNovAtelTimeCoarseSteering  = 120,
  kNovAtelTimeFreeWheeling    = 130,
  kNovAtelTimeFineAdjusting   = 140,
  kNovAtelTimeFine            = 160,
  kNovAtelTimeFineSteering    = 180,
  kNovAtelTimeSatTime         = 200
} NovAtelTime;

// See Table 10 and Table 43.
typedef enum {
  // Invalid.
  kNovAtelMessageIdNone = -1,

  // NovAtel command messages.
  kNovAtelMessageIdCom           = 4,
  kNovAtelMessageIdInterfaceMode = 3,
  kNovAtelMessageIdLog           = 1,

  // NovAtel log messages.
  kNovAtelMessageIdBestXyz     = 241,
  kNovAtelMessageIdHeading     = 971,
  kNovAtelMessageIdHeadingRate = 1698,
  kNovAtelMessageIdHwMonitor   = 963,
  kNovAtelMessageIdIonUtc      = 8,
  kNovAtelMessageIdPsrXyz      = 243,
  kNovAtelMessageIdRange       = 43,
  kNovAtelMessageIdRawEphem    = 41,
  kNovAtelMessageIdRtkXyz      = 244,
  kNovAtelMessageIdRxConfig    = 128,
  kNovAtelMessageIdRxStatus    = 93,
} NovAtelMessageId;

// See Table 22: Datum Transformation Parameters.
typedef enum {
  kNovAtelDatumWgs84 = 61
} NovAtelDatum;

// See Table 31: Serial Port Interface Modes.
typedef enum {
  kNovAtelPortModeNone          = 0,
  kNovAtelPortModeNovAtel       = 1,
  kNovAtelPortModeRtcm          = 2,
  kNovAtelPortModeRtca          = 3,
  kNovAtelPortModeCmr           = 4,
  kNovAtelPortModeOmniStar      = 5,
  kNovAtelPortModeImu           = 6,
  kNovAtelPortModeRtcmNoCr      = 8,
  kNovAtelPortModeCdgps         = 9,
  kNovAtelPortModeTCom1         = 10,
  kNovAtelPortModeTCom2         = 11,
  kNovAtelPortModeTCom3         = 12,
  kNovAtelPortModeTAux          = 13,
  kNovAtelPortModeRtcmV3        = 14,
  kNovAtelPortModeNovAtelBinary = 15,
  kNovAtelPortModeGeneric       = 18,
  kNovAtelPortModeMrtca         = 20
} NovAtelPortMode;

// See unnamed table on page 145.
typedef enum {
  kNovAtelTriggerOnNew     = 0,
  kNovAtelTriggerOnChanged = 1,
  kNovAtelTriggerOnTime    = 2,
  kNovAtelTriggerOnNext    = 3,
  kNovAtelTriggerOnce      = 4,
  kNovAtelTriggerOnMark    = 5
} NovAtelTrigger;

// See Table 50: Position or velocity type.
typedef enum {
  kNovAtelSolutionTypeNone            = 0,
  kNovAtelSolutionTypeFixedPos        = 1,
  kNovAtelSolutionTypeFixedHeight     = 2,
  // 3-7 reserved.
  kNovAtelSolutionTypeDopplerVelocity = 8,
  // 9-15 reserved.
  kNovAtelSolutionTypeSingle          = 16,
  kNovAtelSolutionTypePsrdiff         = 17,
  kNovAtelSolutionTypeWaas            = 18,
  kNovAtelSolutionTypePropagated      = 19,
  kNovAtelSolutionTypeOmnistar        = 20,
  // 21-31 reserved.
  kNovAtelSolutionTypeL1Float         = 32,
  kNovAtelSolutionTypeIonofreeFloat   = 33,
  kNovAtelSolutionTypeNarrowFloat     = 34,
  kNovAtelSolutionTypeL1Int           = 48,
  kNovAtelSolutionTypeWideInt         = 49,
  kNovAtelSolutionTypeNarrowInt       = 50,
  kNovAtelSolutionTypeRtkDirectIns    = 51,
  // 52-56 INS calculated position types.
  kNovAtelSolutionTypeOmnistarHp      = 64,
  kNovAtelSolutionTypeOmnistarXp      = 65,
  kNovAtelSolutionTypeCdgps           = 66
} NovAtelSolutionType;

// See Table 51: Solution status.
typedef enum {
  kNovAtelSolutionStatusSolComputed      = 0,
  kNovAtelSolutionStatusInsufficientObs  = 1,
  kNovAtelSolutionStatusNoConvergence    = 2,
  kNovAtelSolutionStatusSingularity      = 3,
  kNovAtelSolutionStatusCovTrace         = 4,
  kNovAtelSolutionStatusTestDist         = 5,
  kNovAtelSolutionStatusColdStart        = 6,
  kNovAtelSolutionStatusVHLimit          = 7,
  kNovAtelSolutionStatusVariance         = 8,
  kNovAtelSolutionStatusResiduals        = 9,
  kNovAtelSolutionStatusDeltaPos         = 10,
  kNovAtelSolutionStatusNegativeVar      = 11,
  // 12 reserved.
  kNovAtelSolutionStatusIntegrityWarning = 13,
  // 14-17 INS solution status values.
  kNovAtelSolutionStatusPending          = 18,
  kNovAtelSolutionStatusInvalidFix       = 19,
  kNovAtelSolutionStatusUnauthorized     = 20
} NovAtelSolutionStatus;

// See Table 109: Response Messages.
typedef enum {
  kNovAtelResponseNone = -1,
  kNovAtelResponseOk = 1
} NovAtelResponse;

typedef struct {
  uint8_t time_status;
  uint16_t week;
  uint32_t tow;
} NovAtelTimestamp;

typedef struct {
  int32_t header_length;
  int32_t message_length;
  NovAtelMessageId message_id;
  NovAtelFormat format;
  NovAtelPort port;
  NovAtelResponse response;
  NovAtelTimestamp timestamp;
  uint16_t sequence;
  uint8_t idle_time;
  uint32_t receiver_status;
  uint16_t receiver_sw_version;
} NovAtelHeader;

typedef struct {
  NovAtelTimestamp timestamp;
  int32_t pos_sol_status;  // See NovAtelSolutionStatus.
  int32_t pos_type;        // See NovAtelSolutionType.
  double pos_x;
  double pos_y;
  double pos_z;
  float pos_x_sigma;
  float pos_y_sigma;
  float pos_z_sigma;
  int32_t vel_sol_status;  // See NovAtelSolutionStatus.
  int32_t vel_type;        // See NovAtelSolutionType.
  double vel_x;
  double vel_y;
  double vel_z;
  float vel_x_sigma;
  float vel_y_sigma;
  float vel_z_sigma;
  uint8_t station_id[4];
  float vel_latency;
  float diff_age;
  float sol_age;
  uint8_t num_tracked;
  uint8_t num_sol;
  uint8_t num_gg_l1;
  uint8_t num_gg_l1_l2;
  uint8_t ext_sol_status;
  uint8_t sig_mask;
} NovAtelLogBestXyz;

typedef struct {
  NovAtelTimestamp timestamp;
  int32_t pos_sol_status;  // See NovAtelSolutionStatus.
  int32_t pos_type;        // See NovAtelSolutionType.
  float length;            // [m]
  float heading;           // [rad]
  float pitch;             // [rad]
  float heading_sigma;     // [rad]
  float pitch_sigma;       // [rad]
  uint8_t station_id[4];
  uint8_t num_tracked;
  uint8_t num_sol;
  uint8_t num_obs;
  uint8_t num_multi;
  uint8_t sol_source;
  uint8_t ext_sol_status;
  uint8_t galileo_beidou_mask;
  uint8_t gps_glonass_mask;
} NovAtelLogHeading;

typedef struct {
  NovAtelTimestamp timestamp;
  int32_t pos_sol_status;    // See NovAtelSolutionStatus.
  int32_t pos_type;          // See NovAtelSolutionType.
  float latency;             // [s]
  float length_rate;         // [m/s]
  float heading_rate;        // [rad/s]
  float pitch_rate;          // [rad/s]
  float length_rate_sigma;   // [m/s]
  float heading_rate_sigma;  // [rad/s]
  float pitch_rate_sigma;    // [rad/s]
  uint8_t rover_id[4];
  uint8_t master_id[4];
  uint8_t sol_source;
} NovAtelLogHeadingRate;

typedef struct {
  NovAtelTimestamp timestamp;
  int32_t num_measurements;
  float reading[NOVATEL_HWMONITOR_MEASUREMENTS];
  uint32_t status[NOVATEL_HWMONITOR_MEASUREMENTS];
} NovAtelLogHwMonitor;

typedef struct {
  NovAtelTimestamp timestamp;
  GpsIonosphere iono;
  GpsUtc utc;
} NovAtelLogIonUtc;

// Data fields num_gg_l1 and num_gg_l1_l2 are not valid in this log.
typedef NovAtelLogBestXyz NovAtelLogPsrXyz;

typedef struct {
  NovAtelTimestamp timestamp;
  int32_t num_obs;
  uint16_t prn[NOVATEL_OBSERVATIONS];
  uint16_t glofreq[NOVATEL_OBSERVATIONS];
  double psr[NOVATEL_OBSERVATIONS];
  float psr_std[NOVATEL_OBSERVATIONS];
  double adr[NOVATEL_OBSERVATIONS];
  float adr_std[NOVATEL_OBSERVATIONS];
  float dopp[NOVATEL_OBSERVATIONS];
  float cn0[NOVATEL_OBSERVATIONS];
  float locktime[NOVATEL_OBSERVATIONS];
  uint32_t status_bits[NOVATEL_OBSERVATIONS];
} NovAtelLogRange;

typedef struct {
  NovAtelTimestamp timestamp;
  GpsEphemeris eph;
} NovAtelLogRawEphem;

typedef NovAtelLogBestXyz NovAtelLogRtkXyz;

typedef struct {
  NovAtelHeader header;
  const uint8_t *data;
} NovAtelLogRxConfig;

typedef enum {
  kNovAtelRxStatusForceUnsigned = -1,
  kNovAtelRxStatusReceiver,
  kNovAtelRxStatusAux1,
  kNovAtelRxStatusAux2,
  kNovAtelRxStatusAux3,
  kNumNovAtelRxStatuses,
} NovAtelRxStatus;

typedef struct {
  NovAtelTimestamp timestamp;
  uint32_t error;
  int32_t num_stats;
  uint32_t status[kNumNovAtelRxStatuses];
  uint32_t priority[kNumNovAtelRxStatuses];
  uint32_t event_set[kNumNovAtelRxStatuses];
  uint32_t event_clear[kNumNovAtelRxStatuses];
} NovAtelLogRxStatus;

typedef union {
  NovAtelLogBestXyz best_xyz;
  NovAtelLogHeading heading;
  NovAtelLogHeadingRate heading_rate;
  NovAtelLogHwMonitor hw_monitor;
  NovAtelLogIonUtc ion_utc;
  NovAtelLogPsrXyz psr_xyz;
  NovAtelLogRange range;
  NovAtelLogRawEphem raw_ephem;
  NovAtelLogRtkXyz rtk_xyz;
  NovAtelLogRxConfig rx_config;
  NovAtelLogRxStatus rx_status;
} NovAtelLog;

const char *NovAtelSolutionTypeToString(NovAtelSolutionType solution_type);
const char *NovAtelSolutionStatusToString(NovAtelSolutionStatus status);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_NOVATEL_TYPES_H_
