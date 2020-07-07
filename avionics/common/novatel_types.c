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

#include "avionics/common/novatel_types.h"

#include <assert.h>
#include <stdbool.h>

const char *NovAtelSolutionTypeToString(NovAtelSolutionType solution_type) {
  switch (solution_type) {
    case kNovAtelSolutionTypeNone:
      return "None";
    case kNovAtelSolutionTypeFixedPos:
      return "FixedPos";
    case kNovAtelSolutionTypeFixedHeight:
      return "FixedHeight";
    case kNovAtelSolutionTypeDopplerVelocity:
      return "DopplerVelocity";
    case kNovAtelSolutionTypeSingle:
      return "Single";
    case kNovAtelSolutionTypePsrdiff:
      return "Psrdiff";
    case kNovAtelSolutionTypeWaas:
      return "Waas";
    case kNovAtelSolutionTypePropagated:
      return "Propagated";
    case kNovAtelSolutionTypeOmnistar:
      return "Omnistar";
    case kNovAtelSolutionTypeL1Float:
      return "L1Float";
    case kNovAtelSolutionTypeIonofreeFloat:
      return "IonofreeFloat";
    case kNovAtelSolutionTypeNarrowFloat:
      return "NarrowFloat";
    case kNovAtelSolutionTypeL1Int:
      return "L1Int";
    case kNovAtelSolutionTypeWideInt:
      return "WideInt";
    case kNovAtelSolutionTypeNarrowInt:
      return "NarrowInt";
    case kNovAtelSolutionTypeRtkDirectIns:
      return "RtkDirectIns";
    case kNovAtelSolutionTypeOmnistarHp:
      return "OmnistarHp";
    case kNovAtelSolutionTypeOmnistarXp:
      return "OmnistarXp";
    case kNovAtelSolutionTypeCdgps:
      return "Cdgps";
    default:
      assert(false);
      return "<Unknown>";
  }
}

const char *NovAtelSolutionStatusToString(NovAtelSolutionStatus status) {
  switch (status) {
    case kNovAtelSolutionStatusSolComputed:
      return "SolComputed";
    case kNovAtelSolutionStatusInsufficientObs:
      return "InsufficientObs";
    case kNovAtelSolutionStatusNoConvergence:
      return "NoConvergence";
    case kNovAtelSolutionStatusSingularity:
      return "Singularity";
    case kNovAtelSolutionStatusCovTrace:
      return "CovTrace";
    case kNovAtelSolutionStatusTestDist:
      return "TestDist";
    case kNovAtelSolutionStatusColdStart:
      return "ColdStart";
    case kNovAtelSolutionStatusVHLimit:
      return "VHLimit";
    case kNovAtelSolutionStatusVariance:
      return "Variance";
    case kNovAtelSolutionStatusResiduals:
      return "Residuals";
    case kNovAtelSolutionStatusDeltaPos:
      return "DeltaPos";
    case kNovAtelSolutionStatusNegativeVar:
      return "NegativeVar";
    case kNovAtelSolutionStatusIntegrityWarning:
      return "IntegrityWarning";
    case kNovAtelSolutionStatusPending:
      return "Pending";
    case kNovAtelSolutionStatusInvalidFix:
      return "InvalidFix";
    case kNovAtelSolutionStatusUnauthorized:
      return "Unauthorized";
    default:
      assert(false);
      return "<Unknown>";
  }
}
