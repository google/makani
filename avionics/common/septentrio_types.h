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

#ifndef AVIONICS_COMMON_SEPTENTRIO_TYPES_H_
#define AVIONICS_COMMON_SEPTENTRIO_TYPES_H_

#include <stdint.h>

#include "avionics/common/gps_types.h"

#define SEPTENTRIO_OBSERVATIONS  32
#define SEPTENTRIO_BASE_STATIONS 1

// Notice! All data fields have a do-not-use value. See AsteRx-m Firmware
// v3.3.0 SBF Reference Guide for details.

typedef enum {
  // Measurement blocks.
  kSeptentrioIdMeasEpoch = 4027,
  kSeptentrioIdMeasExtra = 4000,
  kSeptentrioIdIqCorr    = 4046,
  kSeptentrioIdEndOfMeas = 5922,

  // Navigation page blocks.
  kSeptentrioIdGpsRawCa   = 4017,
  kSeptentrioIdGpsRawL2c  = 4018,
  kSeptentrioIdGloRawCa   = 4026,
  kSeptentrioIdGalINav    = 4023,
  kSeptentrioIdGeoRawL1   = 4020,
  kSeptentrioIdQzsRawL1Ca = 4066,
  kSeptentrioIdQzsRawL2c  = 4067,

  // GPS decoded message blocks.
  kSeptentrioIdGpsNav = 5891,
  kSeptentrioIdGpsAlm = 5892,
  kSeptentrioIdGpsIon = 5893,
  kSeptentrioIdGpsUtc = 5894,

  // GLONASS decoded message blocks.
  kSeptentrioIdGloNav  = 4004,
  kSeptentrioIdGloAlm  = 4005,
  kSeptentrioIdGloTime = 4036,

  // Galileo decoded message blocks.
  kSeptentrioIdGalNav    = 4002,
  kSeptentrioIdGalAlm    = 4003,
  kSeptentrioIdGalIon    = 4030,
  kSeptentrioIdGalUtc    = 4031,
  kSeptentrioIdGstGps    = 4032,
  kSeptentrioIdGalSarRlm = 4034,

  // SBAS decoded message blocks.
  kSeptentrioIdGeoMt00              = 5925,
  kSeptentrioIdGeoPrnMask           = 5926,
  kSeptentrioIdGeoFastCorr          = 5927,
  kSeptentrioIdGeoIntegrity         = 5928,
  kSeptentrioIdGeoFastCorrDegr      = 5929,
  kSeptentrioIdGeoNav               = 5896,
  kSeptentrioIdGeoDegrFactors       = 5930,
  kSeptentrioIdGeoNetworkTime       = 5918,
  kSeptentrioIdGeoAlm               = 5897,
  kSeptentrioIdGeoIgpMask           = 5931,
  kSeptentrioIdGeoLongTermCorr      = 5932,
  kSeptentrioIdGeoIonoDelay         = 5933,
  kSeptentrioIdGeoServiceLevel      = 5917,
  kSeptentrioIdGeoClockEphCovMatrix = 5934,

  // Position, velocity, time blocks.
  kSeptentrioIdPvtCartesian    = 4006,
  kSeptentrioIdPvtGeodetic     = 4007,
  kSeptentrioIdPosCovCartesian = 5905,
  kSeptentrioIdPosCovGeodetic  = 5906,
  kSeptentrioIdVelCovCartesian = 5907,
  kSeptentrioIdVelCovGeodetic  = 5908,
  kSeptentrioIdDop             = 4001,
  kSeptentrioIdPosCart         = 4044,
  kSeptentrioIdPosLocal        = 4052,
  kSeptentrioIdPvtSatCartesian = 4008,
  kSeptentrioIdPvtResiduals    = 4009,
  kSeptentrioIdRaimStatistics  = 4011,
  kSeptentrioIdGeoCorrections  = 5935,
  kSeptentrioIdBaseVectorCart  = 4043,
  kSeptentrioIdBaseVectorGeod  = 4028,
  kSeptentrioIdPvtSupport      = 4076,
  kSeptentrioIdEndOfPvt        = 5921,

  // GNSS attitude blocks.
  kSeptentrioIdAttEuler    = 5938,
  kSeptentrioIdAttCovEuler = 5939,
  kSeptentrioIdEndOfAtt    = 5943,

  // Receiver time blocks.
  kSeptentrioIdReceiverTime = 5914,
  kSeptentrioIdXPpsOffset   = 5911,

  // External event blocks.
  kSeptentrioIdExtEvent             = 5924,
  kSeptentrioIdExtEventPvtCartesian = 4037,
  kSeptentrioIdExtEventPvtGeodetic  = 4038,

  // Differential correction blocks.
  kSeptentrioIdDiffCorrIn  = 5919,
  kSeptentrioIdBaseStation = 5949,
  kSeptentrioIdRtcmDatum   = 4049,

  // Status blocks.
  kSeptentrioIdChannelStatus  = 4013,
  kSeptentrioIdReceiverStatus = 4014,
  kSeptentrioIdSatVisibility  = 4012,
  kSeptentrioIdInputLink      = 4090,
  kSeptentrioIdOutputLink     = 4091,
  kSeptentrioIdQualityInd     = 4082,

  // Miscellaneous blocks.
  kSeptentrioIdReceiverSetup = 5902,
  kSeptentrioIdCommands      = 4015,
  kSeptentrioIdComment       = 5936,
  kSeptentrioIdAsciiIn       = 4075
} SeptentrioId;

typedef enum {
  kSeptentrioProtoAscii,
  kSeptentrioProtoSbf,
  kSeptentrioProtoSnmp,
  kSeptentrioProtoRtcm3
} SeptentrioProto;

typedef struct {
  uint32_t tow;  // [ms].
  uint16_t wnc;  // [week].
} SeptentrioTimestamp;

typedef struct {
  int32_t header_length;
  int32_t data_length;
  SeptentrioId block_id;
  int32_t block_rev;
} SeptentrioHeader;

typedef enum {
  kSeptentrioMeasCommonMultipathMitigation = 1 << 0,
  kSeptentrioMeasCommonSmoothed            = 1 << 1,
  kSeptentrioMeasCommonCarrierPhaseAligned = 1 << 2,
  kSeptentrioMeasCommonClockSteering       = 1 << 3
} SeptentrioMeasCommonFlags;

typedef enum {
  kSeptentrioMeasInfoCodeSmoothed       = 1 << 0,
  kSeptentrioMeasInfoSmoothingInterval  = 1 << 1,
  kSeptentrioMeasInfoHalfCycleAmbiguity = 1 << 2
  // GLONASS frequency number follows.
} SeptentrioMeasInfoFlags;

typedef struct {
  SeptentrioTimestamp timestamp;
  uint8_t common_flags;   // See SeptentrioMeasCommonFlags.
  uint8_t cum_clk_jumps;  // Added in rev 1, set to zero otherwise [#].
  uint8_t num_obs;        // Number of valid observations in data arrays.
  uint8_t rx_channel[SEPTENTRIO_OBSERVATIONS];  // [#].
  uint8_t type[SEPTENTRIO_OBSERVATIONS];        // Signal number, antenna ID.
  uint8_t svid[SEPTENTRIO_OBSERVATIONS];        // [#].
  int64_t code[SEPTENTRIO_OBSERVATIONS];        // [0.001 m].
  int32_t doppler[SEPTENTRIO_OBSERVATIONS];     // [0.0001 Hz].
  int32_t carrier[SEPTENTRIO_OBSERVATIONS];     // [0.001 cycles].
  uint8_t cn0[SEPTENTRIO_OBSERVATIONS];         // [0.25 dB-Hz].
  uint8_t locktime[SEPTENTRIO_OBSERVATIONS];    // [s].
  uint8_t info[SEPTENTRIO_OBSERVATIONS];        // See SeptentrioMeasInfoFlags.
} SeptentrioBlockMeasEpoch;

// See page 64 of AsteRx-m Firmware v3.3.0 SBF Reference Guide.
typedef enum {
  kSeptentrioPvtModeNoSolution              = 0,
  kSeptentrioPvtModeStandAlone              = 1,
  kSeptentrioPvtModeDifferential            = 2,
  kSeptentrioPvtModeFixedLocation           = 3,
  kSeptentrioPvtModeRtkFixed                = 4,
  kSeptentrioPvtModeRtkFloat                = 5,
  kSeptentrioPvtModeSbasAided               = 6,
  kSeptentrioPvtModeMovingBaseRtkFixed      = 7,
  kSeptentrioPvtModeMovingBaseRtkFloat      = 8,
  // Index 9 not defined.
  kSeptentrioPvtModePrecisePointPositioning = 10
} SeptentrioPvtMode;

// See page 64 of AsteRx-m Firmware v3.3.0 SBF Reference Guide.
typedef enum {
  kSeptentrioPvtModeBitSolutionMask = 0x0F,  // See SeptentrioPvtMode.
  kSeptentrioPvtModeBitFixPending   = 1 << 6,
  kSeptentrioPvtModeBit2dMode       = 1 << 7
} SeptentrioPvtModeBits;

// See page 64 of AsteRx-m Firmware v3.3.0 SBF Reference Guide.
typedef enum {
  kSeptentrioPvtErrorNone                                = 0,
  kSeptentrioPvtErrorNotEnoughMeasurements               = 1,
  kSeptentrioPvtErrorNotEnoughEphemerides                = 2,
  kSeptentrioPvtErrorDopTooLarge                         = 3,
  kSeptentrioPvtErrorResidualsTooLarge                   = 4,
  kSeptentrioPvtErrorNoCovergence                        = 5,
  kSeptentrioPvtErrorNotEnoughMeasurementsAfterRejection = 6,
  kSeptentrioPvtErrorPositionProhibited                  = 7,
  kSeptentrioPvtErrorNotEnoughDifferentialCorrections    = 8,
  kSeptentrioPvtErrorBaseStationCoordinatesUnavailable   = 9,
  kSeptentrioPvtErrorAmbiguitiesNotFixed                 = 10
} SeptentrioPvtError;

// See page 65 of AsteRx-m Firmware v3.3.0 SBF Reference Guide.
typedef enum {
  kSeptentrioPvtRaimNotActive           = 0,
  kSeptentrioPvtRaimIntegritySuccessful = 1,
  kSeptentrioPvtRaimIntegrityFailed     = 2
} SeptentrioPvtRaim;

// See page 65 of AsteRx-m Firmware v3.3.0 SBF Reference Guide.
typedef enum {
  kSeptentrioPvtAlertBitRaimMask         = 0x03,  // See SeptentrioPvtRaim.
  kSeptentrioPvtAlertBitGalileoIntegrity = 1 << 2,
  kSeptentrioPvtAlertBitAccuracyLimit    = 1 << 4
} SeptentrioPvtAlertBits;

typedef struct {
  SeptentrioTimestamp timestamp;
  uint8_t mode;        // See SeptentrioPvtModeBits.
  uint8_t error;       // See SeptentrioPvtError, do not use data if non-zero.
  double x;            // [m].
  double y;            // [m].
  double z;            // [m].
  float undulation;    // [m].
  float v_x;           // [m/s].
  float v_y;           // [m/s].
  float v_z;           // [m/s].
  float cog;           // [deg].
  double rx_clk_bias;  // [ms].
  float rx_clk_drift;  // [ppm].
  uint8_t time_system;
  uint8_t datum;
  uint8_t nr_sv;  // [#].
  uint8_t wa_corr_info;
  uint16_t reference_id;
  uint16_t mean_corr_age;  // [0.01 s].
  uint32_t signal_info;
  uint8_t alert_flag;  // See SeptentrioPvtAlertBits.

  // Added in revision 1.
  uint8_t nr_biases;
  uint16_t ppp_info;  // [s].

  // Added in revision 2.
  uint16_t latency;     // [0.0001 s].
  uint16_t h_accuracy;  // [0.01 m].
  uint16_t v_accuracy;  // [0.01 m].
  uint8_t misc;
} SeptentrioBlockPvtCartesian;

typedef struct {
  SeptentrioTimestamp timestamp;
  uint8_t mode;   // See SeptentrioPvtModeBits.
  uint8_t error;  // See SeptentrioPvtError, do not use data if non-zero.
  float cov_xx;   // [m^2].
  float cov_yy;   // [m^2].
  float cov_zz;   // [m^2].
  float cov_bb;   // [m^2].
  float cov_xy;   // [m^2].
  float cov_xz;   // [m^2].
  float cov_xb;   // [m^2].
  float cov_yz;   // [m^2].
  float cov_yb;   // [m^2].
  float cov_zb;   // [m^2].
} SeptentrioBlockPosCovCartesian;

typedef struct {
  SeptentrioTimestamp timestamp;
  uint8_t mode;   // See SeptentrioPvtModeBits.
  uint8_t error;  // See SeptentrioPvtError, do not use data if non-zero.
  float cov_xx;   // [m^2/s^2].
  float cov_yy;   // [m^2/s^2].
  float cov_zz;   // [m^2/s^2].
  float cov_tt;   // [m^2/s^2].
  float cov_xy;   // [m^2/s^2].
  float cov_xz;   // [m^2/s^2].
  float cov_xt;   // [m^2/s^2].
  float cov_yz;   // [m^2/s^2].
  float cov_yt;   // [m^2/s^2].
  float cov_zt;   // [m^2/s^2].
} SeptentrioBlockVelCovCartesian;

typedef struct {
  SeptentrioTimestamp timestamp;
  uint8_t num_base_stations;
  uint8_t nr_sv[SEPTENTRIO_BASE_STATIONS];  // Number of satellites at base.
  uint8_t error[SEPTENTRIO_BASE_STATIONS];  // See SeptentrioPvtError.
  uint8_t mode[SEPTENTRIO_BASE_STATIONS];  // See SeptentrioPvtModeBits.
  uint8_t misc[SEPTENTRIO_BASE_STATIONS];
  double delta_posx[SEPTENTRIO_BASE_STATIONS];  // [m].
  double delta_posy[SEPTENTRIO_BASE_STATIONS];  // [m].
  double delta_posz[SEPTENTRIO_BASE_STATIONS];  // [m].
  float delta_velx[SEPTENTRIO_BASE_STATIONS];   // [m/s].
  float delta_vely[SEPTENTRIO_BASE_STATIONS];   // [m/s].
  float delta_velz[SEPTENTRIO_BASE_STATIONS];   // [m/s].
  uint16_t azimuth[SEPTENTRIO_BASE_STATIONS];   // [0.01 deg].
  int16_t elevation[SEPTENTRIO_BASE_STATIONS];  // [0.01 deg].
  uint16_t ref_id[SEPTENTRIO_BASE_STATIONS];
  uint16_t corr_age[SEPTENTRIO_BASE_STATIONS];  // [0.01 s].
  uint32_t signal_info[SEPTENTRIO_BASE_STATIONS];
} SeptentrioBlockBaseVectorCart;

typedef struct {
  SeptentrioTimestamp timestamp;
} SeptentrioBlockEndOfPvt;

typedef struct {
  SeptentrioTimestamp timestamp;
  GpsEphemeris eph;
} SeptentrioBlockGpsNav;

typedef struct {
  SeptentrioTimestamp timestamp;
  uint8_t prn;
  GpsIonosphere iono;
} SeptentrioBlockGpsIon;

typedef struct {
  SeptentrioTimestamp timestamp;
  uint8_t prn;
  GpsUtc utc;
} SeptentrioBlockGpsUtc;

typedef union {
  SeptentrioBlockMeasEpoch meas_epoch;
  SeptentrioBlockPvtCartesian pvt_cartesian;
  SeptentrioBlockPosCovCartesian pos_cov_cartesian;
  SeptentrioBlockVelCovCartesian vel_cov_cartesian;
  SeptentrioBlockBaseVectorCart base_vector_cart;
  SeptentrioBlockEndOfPvt end_of_pvt;
  SeptentrioBlockGpsNav gps_nav;
  SeptentrioBlockGpsIon gps_ion;
  SeptentrioBlockGpsUtc gps_utc;
} SeptentrioBlock;

typedef struct {
  SeptentrioHeader hdr;
  SeptentrioBlock block;
} SeptentrioContext;

#ifdef __cplusplus
extern "C" {
#endif

const char *SeptentrioPvtModeToString(SeptentrioPvtMode pvt_mode);
const char *SeptentrioPvtErrorToString(SeptentrioPvtError pvt_error);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_SEPTENTRIO_TYPES_H_
