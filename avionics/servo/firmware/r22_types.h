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

#ifndef AVIONICS_SERVO_FIRMWARE_R22_TYPES_H_
#define AVIONICS_SERVO_FIRMWARE_R22_TYPES_H_

// See Copley Control's Binary Serial Interface v1.2 document (ref AN112).
// http://www.copleycontrols.com/Motion/pdf/Binary-Serial.pdf

typedef enum {
  kR22StateDisabled                               = 0,
  kR22StateCurrentLoopByProgrammedCurrent         = 1,
  kR22StateCurrentLoopByAnalog                    = 2,
  kR22StateCurrentLoopByPwm                       = 3,
  kR22StateCurrentLoopByFunctionGenerator         = 4,
  kR22StateCurrentLoopByUv                        = 5,
  kR22StateVelocityLoopByProgrammedVelocity       = 11,
  kR22StateVelocityLoopByAnalog                   = 12,
  kR22StateVelocityLoopByPwm                      = 13,
  kR22StateVelocityLoopByFunctionGenerator        = 14,
  kR22StateServoPositionByTrajectoryGenerator     = 21,
  kR22StateServoPositionByAnalog                  = 22,
  kR22StateServoPositionByDigital                 = 23,
  kR22StateServoPositionByFunctionGenerator       = 24,
  kR22StateServoPositionByCamming                 = 25,
  kR22StateServoPositionByCanOpen                 = 30,
  kR22StateMicrostepPositionByTrajectoryGenerator = 31,
  kR22StateMicrostepPositionByDigital             = 33,
  kR22StateMicrostepPositionByFunctionGenerator   = 34,
  kR22StateMicrostepPositionByCamming             = 35,
  kR22StateMicrostepPositionByCanOpen             = 40,
  kR22StateMicrostepDiagnosticMode                = 42
} R22State;

typedef enum {
  kR22OpCodeNoOperation                 = 0,
  kR22OpCodeRetrieveOperatingMode       = 7,
  kR22OpCodeGetFlashCrcValue            = 10,
  kR22OpCodeSwapOperatingModes          = 11,
  kR22OpCodeGetVariableValue            = 12,
  kR22OpCodeSetVariableValue            = 13,
  kR22OpCodeCopyVariableValue           = 14,
  kR22OpCodeTraceCommand                = 15,
  kR22OpCodeReset                       = 16,
  kR22OpCodeTrajectoryCommand           = 17,
  kR22OpCodeErrorLogCommand             = 18,
  kR22OpCodeCopleyVirtualMachineCommand = 20,
  kR22OpCodeEncoderCommand              = 27,
  kR22OpCodeGetCanObjectCommand         = 28,
  kR22OpCodeSetCanObjectCommand         = 29,
  kR22OpCodeDynamicFileCommandInterface = 33
} R22OpCode;

typedef enum {
  kR22MemoryRam   = 0x01,
  kR22MemoryFlash = 0x02
} R22Memory;

typedef enum {
  kR22TrajectoryAbort    = 0,
  kR22TrajectoryInitiate = 1,
  kR22TrajectoryHome     = 2
} R22TrajectorySubcommand;

typedef enum {
  kR22ErrorLogGetCountSinceManufacture = 0,
  kR22ErrorLogGetCountSinceClear       = 1,
  kR22ErrorLogGetResetSinceManufacture = 2,
  kR22ErrorLogClear                    = 3,
  kR22ErrorLogGetData                  = 4
} R22ErrorLogSubcommand;

typedef enum {
  kR22ErrorNone                          = 0,
  kR22ErrorTooMuchDataSent               = 1,
  kR22ErrorChecksum                      = 2,
  kR22ErrorIllegalOpCode                 = 3,
  kR22ErrorNotEnoughData                 = 4,
  kR22ErrorUnexpectedData                = 5,
  kR22ErrorErasingFlashMemory            = 6,
  kR22ErrorWritingFlashMemory            = 7,
  kR22ErrorIllegalMemoryPageSpecified    = 8,
  kR22ErrorUnknownVariableId             = 9,
  kR22ErrorParameterValueOutOfRange      = 10,
  kR22ErrorReadOnlyParameter             = 11,
  kR22ErrorInvalidTraceChannel           = 12,
  kR22ErrorInvalidTraceVariableNumber    = 13,
  kR22ErrorInvalidModeOfOperation        = 14,
  kR22ErrorVariableDoesNotExist          = 15,
  kR22ErrorIllegalSerialForwarding       = 16,
  kR22ErrorDataFlashCrcError             = 17,
  kR22ErrorIllegalMoveAttempt            = 18,
  kR22ErrorIllegalVelocityLimit          = 19,
  kR22ErrorIllegalAccelerationLimit      = 20,
  kR22ErrorIllegalDecelerationLimit      = 21,
  kR22ErrorIllegalJerkLimit              = 22,
  kR22ErrorTrajectoryBufferUnderflow     = 23,
  kR22ErrorTrajectoryBufferOverflow      = 24,
  kR22ErrorInvalidTrajectoryMode         = 25,
  kR22ErrorCvmLocationNotAvailable       = 26,
  kR22ErrorIllegalOperationCvmRunning    = 27,
  kR22ErrorCvmTooBigToUpload             = 28,
  kR22ErrorInternalFileSystem            = 29,
  kR22ErrorProgramDoesNotExist           = 30,
  kR22ErrorInvalidNodeIdSerialForwarding = 31,
  kR22ErrorCanNetworkFailure             = 32,
  kR22ErrorAsciiParsingFailure           = 33,
  kR22ErrorInternalError                 = 34,
  kR22ErrorFileSystemReadOnlyCammingMode = 35,
  kR22ErrorIllegalAxisSpecified          = 36,
  kR22ErrorInvalidFpgaDataStored         = 37,
  kR22ErrorUnableToInitializeFpga        = 38,
  kR22ErrorFpgaFailedToConfigure         = 39,
  kR22ErrorFileAlreadyExists             = 40,
  kR22ErrorNoFreeFileEntriesInDirectory  = 41,
  kR22ErrorFileDoesNotExist              = 42,
  kR22ErrorNoFreeSpaceInFileSystem       = 43,
  kR22ErrorInvalidFileFormat             = 44,
  kR22ErrorEndOfFileWhileReading         = 45,
  kR22ErrorSendingCommandToEncoder       = 46,
  kR22ErrorIllegalOperationInSerialMode  = 47,
  kR22ErrorUnableToCalculateFilter       = 48,
  kR22ErrorIndexerRegister31NotSet       = 49,
  kR22ErrorTimeout                       = 50
} R22Error;

typedef enum {
  kR22StatusBitShortCircuitDetected           = (1L << 0),
  kR22StatusBitDriveOverTemperature           = (1L << 1),
  kR22StatusBitOverVoltage                    = (1L << 2),
  kR22StatusBitUnderVoltage                   = (1L << 3),
  kR22StatusBitMotorTemperatureSensorActive   = (1L << 4),
  kR22StatusBitFeedbackError                  = (1L << 5),
  kR22StatusBitMotorPhasingError              = (1L << 6),
  kR22StatusBitCurrentOutputLimited           = (1L << 7),
  kR22StatusBitVoltageOutputLimited           = (1L << 8),
  kR22StatusBitPositiveLimitSwitchActive      = (1L << 9),
  kR22StatusBitNegativeLimitSwitchActive      = (1L << 10),
  kR22StatusBitEnableInputNotActive           = (1L << 11),
  kR22StatusBitDriveDisabledBySoftware        = (1L << 12),
  kR22StatusBitTryingToStopMotor              = (1L << 13),
  kR22StatusBitMotorBrakeActive               = (1L << 14),
  kR22StatusBitPwmOutputDisabled              = (1L << 15),
  kR22StatusBitPositiveSoftwareLimitCondition = (1L << 16),
  kR22StatusBitNegativeSoftwareLimitCondition = (1L << 17),
  kR22StatusBitTrackingError                  = (1L << 18),
  kR22StatusBitTrackingWarning                = (1L << 19),
  kR22StatusBitDriveHasBeenReset              = (1L << 20),
  kR22StatusBitPositionWrapper                = (1L << 21),
  kR22StatusBitDriveFault                     = (1L << 22),
  kR22StatusBitVelocityLimitReached           = (1L << 23),
  kR22StatusBitAccelerationLimitReached       = (1L << 24),
  kR22StatusBitPositionOutsideTrackingWindow  = (1L << 25),
  kR22StatusBitHomeSwitchActive               = (1L << 26),
  kR22StatusBitTrajectoryRunning              = (1L << 27),
  kR22StatusBitVelocityWindowExceeded         = (1L << 28),
  kR22StatusBitPhaseNotInitialized            = (1L << 29),
  kR22StatusBitCommandFault                   = (1L << 30),
  kR22StatusBitNotDefined                     = (1L << 31)
} R22StatusBit;

// TODO: Evaluate critical status bits.
#define R22_STATUS_BIT_ERROR_MASK (             \
    kR22StatusBitShortCircuitDetected           \
    | kR22StatusBitOverVoltage                  \
    | kR22StatusBitFeedbackError                \
    | kR22StatusBitMotorPhasingError            \
    | kR22StatusBitTrackingError                \
    | kR22StatusBitDriveFault                   \
    | kR22StatusBitCommandFault)

#define R22_STATUS_BIT_WARNING_MASK (                   \
    kR22StatusBitDriveOverTemperature                   \
    | kR22StatusBitUnderVoltage                         \
    | kR22StatusBitPositiveLimitSwitchActive            \
    | kR22StatusBitNegativeLimitSwitchActive            \
    | kR22StatusBitPositiveSoftwareLimitCondition       \
    | kR22StatusBitNegativeSoftwareLimitCondition       \
    | kR22StatusBitTrackingWarning                      \
    | kR22StatusBitNotDefined)

const char *R22ErrorName(R22Error code);
const char *R22StatusBitName(R22StatusBit code);
void R22LogStatusBits(R22StatusBit mask);

#endif  // AVIONICS_SERVO_FIRMWARE_R22_TYPES_H_
