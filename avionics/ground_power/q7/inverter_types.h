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

#ifndef AVIONICS_GROUND_POWER_Q7_INVERTER_TYPES_H_
#define AVIONICS_GROUND_POWER_Q7_INVERTER_TYPES_H_

#include "avionics/common/avionics_messages.h"

typedef enum {
  kInverterModbusParamsCharTimeout     = 5000,
  kInverterModbusParamsMsgTimeout      = 250000,
  kInverterModbusParamsTcpPort         = 502,
  kInverterModbusParamsGetParamDataLen = 1,
  kInverterModbusParamsMaxQueryRetries = 3,
} InverterModbusParams;

typedef enum {
  kInverterConstsFaultInductorEnable  = 1,
  kInverterConstsFaultInductorDisable = 0,
  kInverterConstsFaultInductorReset   = 1,
} InverterConsts;

typedef enum {
  kInverterIdsInverter1 = 0,
  kInverterIdsInverter2 = 1,
  kInverterIdsInverter3 = 2,
  kInverterIdsInverter4 = 3,
  kInverterIdsInverter5 = 4,
  kInverterIdsInverter6 = 5,
  kNumInverters         = 6,
} InverterIds;

typedef enum {
  kInverterMessageTypeCommand  = 0,
  kInverterMessageTypeGetParam = 1,
  kInverterMessageTypeSetParam = 2,
} InverterMessageTypes;

typedef struct {
  InverterMessageTypes message_type;
  union {
    GroundPowerCommandMessage command_message;
    GroundPowerSetParamMessage set_param_message;
    GroundPowerGetParamMessage get_param_message;
  } message_union;
} GroundPowerCommandQueueMessage;

// https://makani-private.googlesource.com/satcon/+/master/DSP/code/albedoX_EQX_PG_Rectifer/source/App_Modbus.c
typedef enum {
  // Inverter Modbus Registers.
  kInverterRegisterCommand               = 437,
  kInverterRegisterCtrlBoardAirTemp      = 2050,
  kInverterRegisterMeanPower             = 43,
  kInverterRegisterFaultData             = 10,
  kInverterRegisterSetCrowbarEnable      = 2200,
  kInverterRegisterSetFaultInductor      = 418,
  kInverterRegisterSetFaultInductorReset = 420,
  kInverterRegisterSetAccess             = 1016,
  kInverterRegisterSetDCVolts            = 1025,
  kInverterRegisterSetTetherCompScale    = 425,
  kInverterRegisterSetTetherFilterVal    = 424,
  kInverterRegisterSetTetherResistance   = 422,
  kInverterRegisterTetherCompCalc        = 423,
  kInverterRegisterVCommonMode           = 1131,
  kInverterRegisterVdc                   = 1110,
  kInverterRegisterL1                    = 24,
} InverterRegister;

typedef enum {
  // Inverter Status Register array offsets and lengths.
  kInverterParamsFaultData             = 0,
  kInverterParamsFaultDataLen          = 8,
  kInverterParamsVIData                = 8,
  kInverterParamsVIDataLen             = 12,
  kInverterParamsVCommonModeData       = 20,
  kInverterParamsVCommonModeDataLen    = 2,
  kInverterParamsTemperatureData       = 22,
  kInverterParamsTemperatureDataLen    = 5,
  kInverterParamsFaultInductorData     = 27,
  kInverterParamsFaultInductorDataLen  = 1,
  kInverterParamsTetherCompCalcData    = 28,
  kInverterParamsTetherCompCalcDataLen = 1,
  kInverterParamsMeanPowerData         = 29,
  kInverterParamsMeanPowerDataLen      = 3,
  kInverterParamsL1Data                = 32,
  kInverterParamsL1DataLen             = 3,
  kInverterParamsDataVectorSize        = 35,
} InverterParams;

typedef enum {
  kInverterStatusInit     = 0,
  kInverterStatusRunning  = 1,
  kInverterStatusStopped  = 2,
  kInverterStatusEStopped = 3,
  kInverterStatusFaulted  = 4,
} InverterStatusFlag;

typedef enum {
  kInverterFaultStateRunning  = 0,
  kInverterFaultStateStopped  = 4,
  kInverterFaultStateEStopped = 276,
} InverterFaultState;

typedef enum {
  kInverterCommandReset = 3,
  kInverterCommandStart = 1,
  kInverterCommandStop  = 0,
} InverterCommand;

typedef enum {
  kInverterFaultDCInputNotReady       = (1 << 0),
  kInverterFaultVoltageMismatch       = (1 << 1),
  kInverterFaultCommandStop           = (1 << 2),
  kInverterFaultPreChargeError        = (1 << 3),
  kInverterFaultEStop                 = (1 << 4),
  kInverterFaultLowPower              = (1 << 5),
  kInverterFaultLowInutCurrent        = (1 << 6),
  kInverterFaultLowVoltageRideThrough = (1 << 7),
  kInverterFaultDoorOpen              = (1 << 8),
  kInverterFaultDCBreakerNotClosed    = (1 << 9),
  kInverterFaultACBreakerNotClosed    = (1 << 10),
  kInverterFaultACFuse1               = (1 << 11),
  kInverterFaultManualStop            = (1 << 12),
  kInverterFaultACFuse2               = (1 << 13),
  kInverterFaultReservedD01           = (1 << 14),
  kInverterFaultChangeInOperatingMode = (1 << 15),
} InverterFaultD0;

typedef enum {
  kInverterFaultDCInputOverVoltage   = (1 << 0),
  kInverterFaultDCInputUnderVoltage  = (1 << 1),
  kInverterFaultDCBusOverVoltage     = (1 << 2),
  kInverterFaultDCBusUnderVoltage    = (1 << 3),
  kInverterFaultDCGroundFault        = (1 << 4),
  kInverterFaultGridOverVoltS        = (1 << 5),
  kInverterFaultGridOverVoltF        = (1 << 6),
  kInverterFaultGridUnderVoltF       = (1 << 7),
  kInverterFaultGridUnderVoltS       = (1 << 8),
  kInverterFaultVoltUnbalancedF      = (1 << 9),
  kInverterFaultGridOverFreq         = (1 << 10),
  kInverterFaultGridUnderFreq        = (1 << 11),
  kInverterFaultNeutralOverCurrentS  = (1 << 12),
  kInverterFaultNeutralOverCurrentF  = (1 << 13),
  kInverterFaultLineTransientOverVlt = (1 << 14),
  kInverterFaultDriveSignalsNotSyncd = (1 << 15),
} InverterFaultD1;

typedef enum {
  kInverterFaultProgramCheckError    = (1 << 0),
  kInverterFaultFPGAVersionError     = (1 << 1),
  kInverterFaultPowerHistoryError    = (1 << 2),
  kInverterFaultHistoryChecksumError = (1 << 3),
  kInverterFaultParameterChksumError = (1 << 4),
  kInverterFaultPLLUnlockError       = (1 << 5),
  kInverterFaultUACReadyError        = (1 << 6),
  kInverterFaultPVReverseError       = (1 << 7),
  kInverterFaultUFBScaling           = (1 << 8),
  kInverterFaultIFBScaling           = (1 << 9),
  kInverterFaultIDiff                = (1 << 10),
  kInverterFaultParameterChange      = (1 << 11),
  kInverterFaultOverModulated        = (1 << 12),
  kInverterFaultADC                  = (1 << 13),
  kInverterFaultNVRAM                = (1 << 14),
  kInverterFaultFPGA                 = (1 << 15),
} InverterFaultD2;

typedef enum {
  kInverterFaultBoardPower   = (1 << 0),
  kInverterFaultReservedD31  = (1 << 1),
  kInverterFaultGFCI         = (1 << 2),
  kInverterFaultGridOFFast   = (1 << 3),
  kInverterFaultFPGAWatchdog = (1 << 4),
  kInverterFaultACSurge      = (1 << 5),
  kInverterFaultInvFuse1     = (1 << 6),
  kInverterFaultInvFuse2     = (1 << 7),
  kInverterFaultInv1OverTemp = (1 << 8),
  kInverterFaultInvOverTemp  = (1 << 9),
  kInverterFaultTSFMOverTemp = (1 << 10),
  kInverterFaultL1OverTemp   = (1 << 11),
  kInverterFaultL2OverTemp   = (1 << 12),
  kInverterFaultZGFDI        = (1 << 13),
  kInverterFaultIGFDI        = (1 << 14),
  kInverterFaultUGFDI        = (1 << 15),
} InverterFaultD3;

typedef enum {
  kInverterFaultFBA1                   = (1 << 0),
  kInverterFaultFBB1                   = (1 << 1),
  kInverterFaultFBC1                   = (1 << 2),
  kInverterFaultFBA2                   = (1 << 3),
  kInverterFaultFBB2                   = (1 << 4),
  kInverterFaultFBC2                   = (1 << 5),
  kInverterFaultDCInputOverCurrent     = (1 << 6),
  kInverterFaultDCInputOverCurrentInst = (1 << 7),
  kInverterFaultDCUnderVoltInst        = (1 << 8),
  kInverterFaultDCOverVoltInst         = (1 << 9),
  kInverterFaultInvSWOC                = (1 << 10),
  kInverterFaultInvHWOC1               = (1 << 11),
  kInverterFaultInvHWOC2               = (1 << 12),
  kInverterFaultGridOverCurrent        = (1 << 13),
  kInverterFaultGridCurrentUnbalanced  = (1 << 14),
  kInverterFaultInverterUnderVolt      = (1 << 15),
} InverterFaultD4;

typedef enum {
  kInverterFaultOverTemperature   = (1 << 0),
  kInverterFaultAmbientOverTemp   = (1 << 1),
  kInverterFaultHeatSink1OverTemp = (1 << 2),
  kInverterFaultHeatSink2OverTemp = (1 << 3),
  kInverterFaultHeatSink3OverTemp = (1 << 4),
  kInverterFaultHeatSink4OverTemp = (1 << 5),
  kInverterFaultHeatSink5OverTemp = (1 << 6),
  kInverterFaultHeatSink6OverTemp = (1 << 7),
  kInverterFaultGridSeq           = (1 << 8),
  kInverterFaultMD5Lock           = (1 << 9),
  kInverterFaultInvHWOC3          = (1 << 10),
  kInverterFaultDSPOCInst         = (1 << 11),
  kInverterFaultHWOCInst          = (1 << 12),
  kInverterFaultReservedD51       = (1 << 13),
  kInverterFaultReservedD52       = (1 << 14),
  kInverterFaultReservedD53       = (1 << 15),
} InverterFaultD5;

typedef enum {
  kInverterFaultFan1              = (1 << 0),
  kInverterFaultFan2              = (1 << 1),
  kInverterFaultDCContactorOpen   = (1 << 2),
  kInverterFaultDCContactorClosed = (1 << 3),
  kInverterFaultACContactorOpen   = (1 << 4),
  kInverterFaultACContactorClosed = (1 << 5),
  kInverterFaultDCSurge           = (1 << 6),
  kInverterFaultGridUnderVoltInst = (1 << 7),
  kInverterFaultSystemHalted      = (1 << 8),
  kInverterFaultCalDataChkSum     = (1 << 9),
  kInverterFaultGateFBA3          = (1 << 10),
  kInverterFaultGateFBB3          = (1 << 11),
  kInverterFaultGateFBC3          = (1 << 12),
  kInverterFaultGateFBA4          = (1 << 13),
  kInverterFaultGateFBB4          = (1 << 14),
  kInverterFaultGateFBC4          = (1 << 15),
} InverterFaultD6;

typedef enum {
  kInverterFaultFan1Warning  = (1 << 0),
  kInverterFaultFan2Warning  = (1 << 1),
  kInverterFaultFan3Warning  = (1 << 2),
  kInverterFaultLowPower2    = (1 << 3),
  kInverterFaultHTDownRating = (1 << 4),
  kInverterFaultRegWarning   = (1 << 5),
  kInverterFaultReservedD71  = (1 << 6),
  kInverterFaultReservedD72  = (1 << 7),
  kInverterFaultReservedD73  = (1 << 8),
  kInverterFaultReservedD74  = (1 << 9),
  kInverterFaultReservedD75  = (1 << 10),
  kInverterFaultReservedD76  = (1 << 11),
  kInverterFaultReservedD77  = (1 << 12),
  kInverterFaultReservedD78  = (1 << 13),
  kInverterFaultReservedD79  = (1 << 14),
  kInverterFaultReservedD710 = (1 << 15),
} InverterFaultD7;

#endif  // AVIONICS_GROUND_POWER_Q7_INVERTER_TYPES_H_
