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

#include "avionics/common/septentrio_types.h"

#include <assert.h>
#include <stdbool.h>

const char *SeptentrioPvtModeToString(SeptentrioPvtMode pvt_mode) {
  switch (pvt_mode) {
    case kSeptentrioPvtModeNoSolution:
      return "NoSolution";
    case kSeptentrioPvtModeStandAlone:
      return "StandAlone";
    case kSeptentrioPvtModeDifferential:
      return "Differential";
    case kSeptentrioPvtModeFixedLocation:
      return "FixedLocation";
    case kSeptentrioPvtModeRtkFixed:
      return "RtkFixed";
    case kSeptentrioPvtModeRtkFloat:
      return "RtkFloat";
    case kSeptentrioPvtModeSbasAided:
      return "SbasAided";
    case kSeptentrioPvtModeMovingBaseRtkFixed:
      return "MovingBaseRtkFixed";
    case kSeptentrioPvtModeMovingBaseRtkFloat:
      return "MovingBaseRtkFloat";
    case kSeptentrioPvtModePrecisePointPositioning:
      return "PrecisePointPositioning";
    default:
      assert(false);
      return "<Unknown>";
  }
}

const char *SeptentrioPvtErrorToString(SeptentrioPvtError pvt_error) {
  switch (pvt_error) {
    case kSeptentrioPvtErrorNone:
      return "None";
    case kSeptentrioPvtErrorNotEnoughMeasurements:
      return "NotEnoughMeasurements";
    case kSeptentrioPvtErrorNotEnoughEphemerides:
      return "NotEnoughEphemerides";
    case kSeptentrioPvtErrorDopTooLarge:
      return "DopTooLarge";
    case kSeptentrioPvtErrorResidualsTooLarge:
      return "ResidualsTooLarge";
    case kSeptentrioPvtErrorNoCovergence:
      return "NoCovergence";
    case kSeptentrioPvtErrorNotEnoughMeasurementsAfterRejection:
      return "NotEnoughMeasurementsAfterRejection";
    case kSeptentrioPvtErrorPositionProhibited:
      return "PositionProhibited";
    case kSeptentrioPvtErrorNotEnoughDifferentialCorrections:
      return "NotEnoughDifferentialCorrections";
    case kSeptentrioPvtErrorBaseStationCoordinatesUnavailable:
      return "BaseStationCoordinatesUnavailable";
    case kSeptentrioPvtErrorAmbiguitiesNotFixed:
      return "AmbiguitiesNotFixed";
    default:
      assert(false);
      return "<Unknown>";
  }
}
