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

#include "avionics/firmware/cpu/swi.h"

#include <stdint.h>

#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/cpu/registers_def.h"

void SwiGetVector(uint8_t *vect, uint8_t *data) {
  union SYS_SSIVEC reg = SYS.SSIVEC;
  *vect = reg.SSIVECT;
  *data = reg.SSIDATA;
}

void SwiTriggerInterrupt1(uint8_t data) {
  SYS.SSIR1.raw = 0x7500 | data;
}

void SwiTriggerInterrupt2(uint8_t data) {
  SYS.SSIR2.raw = 0x8400 | data;
}

void SwiTriggerInterrupt3(uint8_t data) {
  SYS.SSIR3.raw = 0x9300 | data;
}

void SwiTriggerInterrupt4(uint8_t data) {
  SYS.SSIR4.raw = 0xA200 | data;
}

void SwiClearFlag1(void) {
  SYS.SSIF.raw = SYS_SSIF_SSI_FLAG1;
}

void SwiClearFlag2(void) {
  SYS.SSIF.raw = SYS_SSIF_SSI_FLAG2;
}

void SwiClearFlag3(void) {
  SYS.SSIF.raw = SYS_SSIF_SSI_FLAG3;
}

void SwiClearFlag4(void) {
  SYS.SSIF.raw = SYS_SSIF_SSI_FLAG4;
}

void SwiClearFlag(uint8_t index) {
  SYS.SSIF.raw = 1U << index;
}
