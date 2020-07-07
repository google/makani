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

#ifndef AVIONICS_MOTOR_FIRMWARE_FLAGS_H_
#define AVIONICS_MOTOR_FIRMWARE_FLAGS_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  kMotorCommandNone           = 0,
  kMotorCommandClearError     = (1 << 0),
  kMotorCommandRun            = (1 << 1),
  kMotorCommandDisarm         = (1 << 2),
  kMotorCommandSendAdcLog     = (1 << 3),
  kMotorCommandSendControlLog = (1 << 4)
} MotorCommandFlag;

// TODO: Use error and warning typedefs throughout code instead of
// uint32_t.

typedef enum {
  // Single motor faults.
  kMotorErrorNone               = 0,
  kMotorErrorTimeout            = (1 << 0),
  kMotorErrorBadMode            = (1 << 1),
  kMotorErrorBadCommand         = (1 << 2),
  kMotorErrorOverCurrentIaP     = (1 << 3),
  kMotorErrorOverCurrentIaN     = (1 << 4),
  kMotorErrorOverCurrentIbP     = (1 << 5),
  kMotorErrorOverCurrentIbN     = (1 << 6),
  kMotorErrorOverCurrentIcP     = (1 << 7),
  kMotorErrorOverCurrentIcN     = (1 << 8),
  kMotorErrorOverCurrentIBusP   = (1 << 9),
  kMotorErrorOverCurrentIBusN   = (1 << 10),
  kMotorErrorOverSpeed          = (1 << 11),
  kMotorErrorOverVoltage        = (1 << 12),
  kMotorErrorUnderVoltage       = (1 << 13),
  kMotorErrorFaultCurrentIaP    = (1 << 14),
  kMotorErrorFaultCurrentIaN    = (1 << 15),
  kMotorErrorFaultCurrentIbP    = (1 << 16),
  kMotorErrorFaultCurrentIbN    = (1 << 17),
  kMotorErrorFaultCurrentIcP    = (1 << 18),
  kMotorErrorFaultCurrentIcN    = (1 << 19),

  // Stacked motor faults.
  kMotorErrorVoltageRel         = (1 << 20),
  kMotorErrorTimeoutRemoteMotor = (1 << 21),
  kMotorErrorTimeoutPairMotor   = (1 << 22),
  kMotorErrorRemoteFaultBlock0  = (1 << 23),
  kMotorErrorRemoteFaultBlock1  = (1 << 24),  // Need more if >4 stack levels --
  kMotorErrorRemoteFaultBlock2  = (1 << 25),  // assert checks for kNumMotors==8
  kMotorErrorRemoteFaultBlock3  = (1 << 26),  // in stacking.c.
  kMotorErrorPromoteWarning     = (1 << 27),
  kMotorErrorBlockShorting      = (1 << 28),  // Short stack force-tripping.

  kMotorErrorAll                = (1u << 29) - 1u,
} MotorErrorFlag;

#define MOTOR_FLAGS_REMOTE_FAULT_MASK (kMotorErrorRemoteFaultBlock0 |   \
                                       kMotorErrorRemoteFaultBlock1 |   \
                                       kMotorErrorRemoteFaultBlock2 |   \
                                       kMotorErrorRemoteFaultBlock3)

typedef enum {
  kMotorWarningNone                  = 0,
  kMotorWarningPowerGood1Hs          = (1 << 0),
  kMotorWarningPowerGood2Hs          = (1 << 1),
  kMotorWarningPowerGood3Hs          = (1 << 2),
  kMotorWarningPowerGood1Ls          = (1 << 3),
  kMotorWarningPowerGood2Ls          = (1 << 4),
  kMotorWarningPowerGood3Ls          = (1 << 5),
  kMotorWarningPowerGoodPhantom      = (1 << 6),
  kMotorWarningPowerGoodHetPinEna    = (1 << 7),
  kMotorWarningDesat                 = (1 << 8),
  kMotorWarningNoTx                  = (1 << 9),

  // Motor thermal warnings. The ordering here should be consistent with the
  // ordering in motor_thermal_types.h.
  kMotorWarningOverTempBoard         = (1 << 10),
  kMotorWarningOverTempControllerAir = (1 << 11),
  kMotorWarningOverTempStatorCore    = (1 << 12),
  kMotorWarningOverTempStatorCoil    = (1 << 13),
  kMotorWarningOverTempNacelleAir    = (1 << 14),
  kMotorWarningOverTempRotor         = (1 << 15),
  // Skip unused thermal channel.
  kMotorWarningOverTempHeatPlate1    = (1 << 17),
  kMotorWarningOverTempHeatPlate2    = (1 << 18),
  kMotorWarningOverTempCapacitor     = (1 << 19),
  kMotorWarningOverTempHt3000A       = (1 << 20),
  kMotorWarningOverTempHt3000B       = (1 << 21),
  kMotorWarningOverTempHt3000C       = (1 << 22),

  // Read errors.
  kMotorWarningTempReadErrors        = (1 << 23),
  kMotorWarningAngleSensorReadError  = (1 << 24),

  kMotorWarningPhaseCurrentSum       = (1 << 25),

  // Configuration warnings.
  kMotorWarningAdcFaultDisabledSome  = (1 << 26),
  kMotorWarningAdcFaultDisabledAll   = (1 << 27),

  kMotorWarningAll                   = (1u << 28) - 1u,
} MotorWarningFlag;
#define MOTOR_WARNING_TEMP_OFFSET                       \
  (__builtin_ffs(kMotorWarningOverTempBoard) - 1)

typedef enum {
  kMotorStatusInit     = 0,
  kMotorStatusArmed    = (1 << 0),
  kMotorStatusRunning  = (1 << 1),
  kMotorStatusWindDown = (1 << 2),
  kMotorStatusError    = (1 << 3),
} MotorStatusFlag;

#endif  // AVIONICS_MOTOR_FIRMWARE_FLAGS_H_
