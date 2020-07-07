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

#include "avionics/firmware/output/aio_node_status.h"

#include "avionics/common/bits.h"
#include "avionics/firmware/cpu/registers.h"

void AioNodeInitStatus(AioNodeStatus *status) {
  SetBitByValue16(kAioNodeStatusPowerUpReset, SYS.ESR.PORST, &status->status);
  SetBitByValue16(kAioNodeStatusWatchdogReset, SYS.ESR.WDRST, &status->status);
  SetBitByValue16(kAioNodeStatusOscillatorReset, SYS.ESR.OSCRST,
                  &status->status);
  SetBitByValue16(kAioNodeStatusCpuReset, SYS.ESR.CPURST, &status->status);
  SetBitByValue16(kAioNodeStatusSoftwareReset, SYS.ESR.SWRST, &status->status);
}

void AioNodeGetStatus(AioNodeStatus *status) {
  SetBitByValue16(kAioNodeStatusEsmError, ESM.SR3.ESF, &status->status);
}
