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

// The NHET object code below was generated from the thermal_nhet.het
// assembly.

#include "avionics/motor/firmware/nhet.h"

const Instruction kHetInitThermal[10] = {
  // THERM1.
  {
    0x00003440,
    0x000032B0,
    0x00000000,
    0x00000000
  },

  // THERM2.
  {
    0x00005440,
    0x000044B2,
    0x00000000,
    0x00000000
  },

  // THERM3.
  {
    0x00007440,
    0x00006634,
    0x00000000,
    0x00000000
  },

  // COUNT.
  {
    0x00008CE0,
    0x001312CF,
    0x00000000,
    0x00000000
  },

  // LATCH1.
  {
    0x0000A804,
    0x00400090,
    0x00000000,
    0x00000000
  },

  // CLEAN1.
  {
    0x0000C800,
    0x00400088,
    0x00000000,
    0x00000000
  },

  // LATCH2.
  {
    0x0000E806,
    0x00400092,
    0x00000000,
    0x00000000
  },

  // CLEAN2.
  {
    0x00010801,
    0x0040008A,
    0x00000000,
    0x00000000
  },

  // LATCH3.
  {
    0x00012808,
    0x00400014,
    0x00000000,
    0x00000000
  },

  // CLEAN3.
  {
    0x00000802,
    0x0040000C,
    0x00000000,
    0x00000000
  }
};
