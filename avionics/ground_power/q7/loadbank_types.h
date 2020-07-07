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

#ifndef AVIONICS_GROUND_POWER_Q7_LOADBANK_TYPES_H_
#define AVIONICS_GROUND_POWER_Q7_LOADBANK_TYPES_H_

typedef enum {
  kLoadbankParamsCharTimeout   = 5000,
  kLoadbankParamsMsgTimeout    = 250000,
  kLoadbankParamsRelaysPerBank = 8,
  kLoadbankParamsNLoadMax      = 15,
  kLoadbankParamsNLoadSteps    = 15,
  kLoadbankParamsStepSize      = 100,
  kLoadbankParamsHysteresis    = 100,
  kLoadbankParamsRelayAddress  = 16,
  kLoadbankParamsTcpPort       = 502,
} LoadbankParams;

typedef enum {
  kLoadbankId0 = 0,
  kLoadbankId1 = 1,
  kNumLoadbanks,
} LoadbankIds;

typedef enum {
  kSharkParamsCharTimeout = 5000,
  kSharkParamsMsgTimeout  = 250000,
  kSharkParamsNumDataRegs = 6,
  kSharkParamsTcpPort     = 502,
  kSharkParamsDataRegAddr = 899,
} SharkParams;

typedef union {
  uint16_t raw_regs[2];
  float float_value;
} SharkFloat;

typedef struct {
  SharkFloat power;
  SharkFloat VARs;
  SharkFloat VAs;
} SharkMessage;

#endif  // AVIONICS_GROUND_POWER_Q7_LOADBANK_TYPES_H_
