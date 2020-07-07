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

// The NHET object code below was generated from the svpwm_nhet.het
// assembly.

#include "avionics/motor/firmware/nhet.h"

const Instruction kHetInitSvpwm[28] = {
  // START.
  {
    0x02002C60,
    0x080000A9,
    0x00000000,
    0x00000000
  },

  // REAL_CNT.
  {
    0x00004C20,
    0x00000054,
    0x00000000,
    0x00000000
  },

  // LAST_CHK.
  {
    0x00006803,
    0x00400004,
    0x00000000,
    0x00000000
  },

  // BRANCH.
  {
    0x00008100,
    0x00020004,
    0x00000080,
    0x00000000
  },

  // YA1 (pin 22).
  {
    0x0000A000,
    0x0040B618,
    0x00000A80,
    0x00000000
  },

  // YA2 (pin 23).
  {
    0x0000C000,
    0x0040D718,
    0x00001F80,
    0x00000000
  },

  // YAN1 (pin 4).
  {
    0x0000E000,
    0x0040E418,
    0x00000A80,
    0x00000000
  },

  // YAN2 (pin 5).
  {
    0x00010000,
    0x00410508,
    0x00001F80,
    0x00000000
  },

  // YB1 (pin 14).
  {
    0x00012000,
    0x00412E18,
    0x00000A80,
    0x00000000
  },

  // YB2 (pin 15).
  {
    0x00014000,
    0x00414F18,
    0x00001F80,
    0x00000000
  },

  // YBN1 (pin 16).
  {
    0x00016000,
    0x00417018,
    0x00000A80,
    0x00000000
  },

  // YBN2 (pin 17).
  {
    0x00018000,
    0x00419108,
    0x00001F80,
    0x00000000
  },

  // YC1 (pin 18).
  {
    0x0001A000,
    0x0041B218,
    0x00000A80,
    0x00000000
  },

  // YC2 (pin 19).
  {
    0x0001C000,
    0x0041D318,
    0x00001F80,
    0x00000000
  },

  // YCN1 (pin 20).
  {
    0x0001E000,
    0x0041F418,
    0x00000A80,
    0x00000000
  },

  // YCN2 (pin 21).
  {
    0x00000000,
    0x00401508,
    0x00001F80,
    0x00000000
  },

  // ZA1 (pin 22).
  {
    0x00022000,
    0x00423618,
    0x00000A80,
    0x00000000
  },

  // ZA2 (pin 23).
  {
    0x00024000,
    0x00425718,
    0x00001F80,
    0x00000000
  },

  // ZAN1 (pin 4).
  {
    0x00026000,
    0x00426418,
    0x00000A80,
    0x00000000
  },

  // ZAN2 (pin 5).
  {
    0x00028000,
    0x00428508,
    0x00001F80,
    0x00000000
  },

  // ZB1 (pin 14).
  {
    0x0002A000,
    0x0042AE18,
    0x00000A80,
    0x00000000
  },

  // ZB2 (pin 15).
  {
    0x0002C000,
    0x0042CF18,
    0x00001F80,
    0x00000000
  },

  // ZBN1 (pin 16).
  {
    0x0002E000,
    0x0042F018,
    0x00000A80,
    0x00000000
  },

  // ZBN2 (pin 17).
  {
    0x00030000,
    0x00431108,
    0x00001F80,
    0x00000000
  },

  // ZC1 (pin 18).
  {
    0x00032000,
    0x00433218,
    0x00000A80,
    0x00000000
  },

  // ZC2 (pin 19).
  {
    0x00034000,
    0x00435318,
    0x00001F80,
    0x00000000
  },

  // ZCN1 (pin 20).
  {
    0x00036000,
    0x00437418,
    0x00000A80,
    0x00000000
  },

  // ZCN2 (pin 21).
  {
    0x00000000,
    0x00401508,
    0x00001F80,
    0x00000000
  }
};
