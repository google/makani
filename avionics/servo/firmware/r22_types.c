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

#include "avionics/servo/firmware/r22_types.h"

#include "avionics/firmware/drivers/log.h"

const char *R22ErrorName(R22Error code) {
  switch (code) {
    case kR22ErrorNone:
      return "None";
    case kR22ErrorTooMuchDataSent:
      return "TooMuchDataSent";
    case kR22ErrorChecksum:
      return "Checksum";
    case kR22ErrorIllegalOpCode:
      return "IllegalOpCode";
    case kR22ErrorNotEnoughData:
      return "NotEnoughData";
    case kR22ErrorUnexpectedData:
      return "UnexpectedData";
    case kR22ErrorErasingFlashMemory:
      return "ErasingFlashMemory";
    case kR22ErrorWritingFlashMemory:
      return "WritingFlashMemory";
    case kR22ErrorIllegalMemoryPageSpecified:
      return "IllegalMemoryPageSpecified";
    case kR22ErrorUnknownVariableId:
      return "UnknownVariableId";
    case kR22ErrorParameterValueOutOfRange:
      return "ParameterValueOutOfRange";
    case kR22ErrorReadOnlyParameter:
      return "ReadOnlyParameter";
    case kR22ErrorInvalidTraceChannel:
      return "InvalidTraceChannel";
    case kR22ErrorInvalidTraceVariableNumber:
      return "InvalidTraceVariableNumber";
    case kR22ErrorInvalidModeOfOperation:
      return "InvalidModeOfOperation";
    case kR22ErrorVariableDoesNotExist:
      return "VariableDoesNotExist";
    case kR22ErrorIllegalSerialForwarding:
      return "IllegalSerialForwarding";
    case kR22ErrorDataFlashCrcError:
      return "DataFlashCrcError";
    case kR22ErrorIllegalMoveAttempt:
      return "IllegalMoveAttempt";
    case kR22ErrorIllegalVelocityLimit:
      return "IllegalVelocityLimit";
    case kR22ErrorIllegalAccelerationLimit:
      return "IllegalAccelerationLimit";
    case kR22ErrorIllegalDecelerationLimit:
      return "IllegalDecelerationLimit";
    case kR22ErrorIllegalJerkLimit:
      return "IllegalJerkLimit";
    case kR22ErrorTrajectoryBufferUnderflow:
      return "TrajectoryBufferUnderflow";
    case kR22ErrorTrajectoryBufferOverflow:
      return "TrajectoryBufferOverflow";
    case kR22ErrorInvalidTrajectoryMode:
      return "InvalidTrajectoryMode";
    case kR22ErrorCvmLocationNotAvailable:
      return "CvmLocationNotAvailable";
    case kR22ErrorIllegalOperationCvmRunning:
      return "IllegalOperationCvmRunning";
    case kR22ErrorCvmTooBigToUpload:
      return "CvmTooBigToUpload";
    case kR22ErrorInternalFileSystem:
      return "InternalFileSystem";
    case kR22ErrorProgramDoesNotExist:
      return "ProgramDoesNotExist";
    case kR22ErrorInvalidNodeIdSerialForwarding:
      return "InvalidNodeIdSerialForwarding";
    case kR22ErrorCanNetworkFailure:
      return "CanNetworkFailure";
    case kR22ErrorAsciiParsingFailure:
      return "AsciiParsingFailure";
    case kR22ErrorInternalError:
      return "InternalError";
    case kR22ErrorFileSystemReadOnlyCammingMode:
      return "FileSystemReadOnlyCammingMode";
    case kR22ErrorIllegalAxisSpecified:
      return "IllegalAxisSpecified";
    case kR22ErrorInvalidFpgaDataStored:
      return "InvalidFpgaDataStored";
    case kR22ErrorUnableToInitializeFpga:
      return "UnableToInitializeFpga";
    case kR22ErrorFpgaFailedToConfigure:
      return "FpgaFailedToConfigure";
    case kR22ErrorFileAlreadyExists:
      return "FileAlreadyExists";
    case kR22ErrorNoFreeFileEntriesInDirectory:
      return "NoFreeFileEntriesInDirectory";
    case kR22ErrorFileDoesNotExist:
      return "FileDoesNotExist";
    case kR22ErrorNoFreeSpaceInFileSystem:
      return "NoFreeSpaceInFileSystem";
    case kR22ErrorInvalidFileFormat:
      return "InvalidFileFormat";
    case kR22ErrorEndOfFileWhileReading:
      return "EndOfFileWhileReading";
    case kR22ErrorSendingCommandToEncoder:
      return "SendingCommandToEncoder";
    case kR22ErrorIllegalOperationInSerialMode:
      return "IllegalOperationInSerialMode";
    case kR22ErrorUnableToCalculateFilter:
      return "UnableToCalculateFilter";
    case kR22ErrorIndexerRegister31NotSet:
      return "IndexerRegister31NotSet";
    case kR22ErrorTimeout:
      return "Timeout";
    default:
      return "<Unknown>";
  }
}

const char *R22StatusBitName(R22StatusBit code) {
  switch (code) {
    case kR22StatusBitShortCircuitDetected:
      return "ShortCircuitDetected";
    case kR22StatusBitDriveOverTemperature:
      return "DriveOverTemperature";
    case kR22StatusBitOverVoltage:
      return "OverVoltage";
    case kR22StatusBitUnderVoltage:
      return "UnderVoltage";
    case kR22StatusBitMotorTemperatureSensorActive:
      return "MotorTemperatureSensorActive";
    case kR22StatusBitFeedbackError:
      return "MotorFeedbackError";
    case kR22StatusBitMotorPhasingError:
      return "MotorPhasingError";
    case kR22StatusBitCurrentOutputLimited:
      return "CurrentOutputLimited";
    case kR22StatusBitVoltageOutputLimited:
      return "VoltageOutputLimited";
    case kR22StatusBitPositiveLimitSwitchActive:
      return "PositiveLimitSwitchActive";
    case kR22StatusBitNegativeLimitSwitchActive:
      return "NegativeLimitSwitchActive";
    case kR22StatusBitEnableInputNotActive:
      return "EnableInputNotActive";
    case kR22StatusBitDriveDisabledBySoftware:
      return "DriveDisabledBySoftware";
    case kR22StatusBitTryingToStopMotor:
      return "TryingToStopMotor";
    case kR22StatusBitMotorBrakeActive:
      return "MotorBrakeActive";
    case kR22StatusBitPwmOutputDisabled:
      return "PwmOutputDisabled";
    case kR22StatusBitPositiveSoftwareLimitCondition:
      return "PositiveSoftwareLimitCondition";
    case kR22StatusBitNegativeSoftwareLimitCondition:
      return "NegativeSoftwareLimitCondition";
    case kR22StatusBitTrackingError:
      return "TrackingError";
    case kR22StatusBitTrackingWarning:
      return "TrackingWarning";
    case kR22StatusBitDriveHasBeenReset:
      return "DriveHasBeenReset";
    case kR22StatusBitPositionWrapper:
      return "PositionWrapper";
    case kR22StatusBitDriveFault:
      return "DriveFault";
    case kR22StatusBitVelocityLimitReached:
      return "VelocityLimitReached";
    case kR22StatusBitAccelerationLimitReached:
      return "AccelerationLimitReached";
    case kR22StatusBitPositionOutsideTrackingWindow:
      return "PositionOutsideTrackingWindow";
    case kR22StatusBitHomeSwitchActive:
      return "HomeSwitchActive";
    case kR22StatusBitTrajectoryRunning:
      return "TrajectoryRunning";
    case kR22StatusBitVelocityWindowExceeded:
      return "VelocityWindowExceeded";
    case kR22StatusBitPhaseNotInitialized:
      return "PhaseNotInitialized";
    case kR22StatusBitCommandFault:
      return "CommandFault";
    case kR22StatusBitNotDefined:
      return "NotDefined";
    default:
      return "<Unknown>";
  }
}

void R22LogStatusBits(R22StatusBit mask) {
  while (mask) {
    R22StatusBit mask_1 = mask - 1;
    LOG_PRINTF("R22 status: %s\n", R22StatusBitName((mask ^ mask_1) & ~mask_1));
    mask &= mask_1;
  }
}
