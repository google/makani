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

#include "avionics/common/gps_parse.h"

#include <assert.h>
#include <stdint.h>
#include <stddef.h>

#include "avionics/common/endian.h"
#include "avionics/common/gps_types.h"

// Word format: [2-bit padding] [24-bit data] [6-bit parity].

void GpsParseSubframe1(const uint32_t *sf1, GpsEphemeris *eph) {
  assert(sf1 != NULL);
  assert(eph != NULL);
  uint32_t u32;

  eph->wn = (sf1[2] >> 20) & 0x03FF;  // 10 bits.
  eph->l2_ca_or_p = (sf1[2] >> 18) & 0x03;  // 2 bits.
  eph->ura = (sf1[2] >> 14) & 0x0F;  // 4 bits.
  eph->health = (sf1[2] >> 8) & 0x3F;  // 6 bits.

  u32 = ((sf1[2] << 2) & 0x0300) | ((sf1[7] >> 22) & 0xFF);  // 10 bits.
  eph->iodc = (uint16_t)u32;

  eph->l2pdata = (sf1[3] >> 29) & 0x01;  // 1 bit.

  // 8-bit signed, lsb 2^-31 s.
  eph->t_gd = SignedToFloat((sf1[6] >> 6) & 0xFF, 8, -31);

  // 16-bit unsigned, lsb 2^4 s.
  eph->t_oc = (sf1[7] >> 6) & 0xFFFF;  // 16 bits.
  eph->t_oc *= (1U << 4);

  // 8-bit signed, lsb 2^-55 s/s^2.
  eph->a_f2 = SignedToFloat((sf1[8] >> 22) & 0xFF, 8, -55);

  // 16-bit signed, lsb 2^-43 s/s.
  eph->a_f1 = SignedToFloat((sf1[8] >> 6) & 0xFFFF, 16, -43);

  // 22-bit signed, lsb 2^31 s.
  eph->a_f0 = SignedToFloat((sf1[9] >> 8) & 0x03FFFFF, 22, -31);
}

void GpsParseSubframe2(const uint32_t *sf2, GpsEphemeris *eph) {
  assert(sf2 != NULL);
  assert(eph != NULL);
  uint32_t u32;

  eph->iode2 = (sf2[2] >> 22) & 0xFF;  // 8 bits.

  // 16-bit signed, lsb 2^-5 m.
  eph->c_rs = SignedToFloat((sf2[2] >> 6) & 0xFFFF, 16, -5);

  // 16-bit signed, lsb 2^-43 semi-circle/s.
  eph->delta_n = SignedToFloat((sf2[3] >> 14) & 0xFFFF, 16, -43);

  // 32-bit signed, lsb 2^-31 semi-circle.
  u32 = ((sf2[3] << 18) & 0xFF000000) | ((sf2[4] >> 6) & 0x00FFFFFF);  // 32.
  eph->m_0 = SignedToDouble(u32, 32, -31);

  // 16-bit signed, lsb 2^-29 rad.
  eph->c_uc = SignedToFloat((sf2[5] >> 14) & 0xFFFF, 16, -29);

  // 32-bit unsigned, lsb 2^-33.
  u32 = ((sf2[5] << 18) & 0xFF000000) | ((sf2[6] >> 6) & 0x00FFFFFF);  // 32.
  eph->ecc = UnsignedToDouble(u32, 32, -33);

  // 16-bit signed, lsb 2^-29 rad.
  eph->c_us = SignedToFloat((sf2[7] >> 14) & 0xFFFF, 16, -29);

  // 32-bit unsigned, lsb 2^-19 sqrt(m).
  u32 = ((sf2[7] << 18) & 0xFF000000) | ((sf2[8] >> 6) & 0x00FFFFFF);  // 32.
  eph->sqrt_a = UnsignedToDouble(u32, 32, -19);

  // 16-bit unsigned, lsb 2^4 s.
  eph->t_oe = (sf2[9] >> 14) & 0xFFFF;  // 16 bits.
  eph->t_oe *= (1U << 4);

  eph->fit_interval_flag = (sf2[9] >> 13) & 0x01;  // 1 bit.
}

void GpsParseSubframe3(const uint32_t *sf3, GpsEphemeris *eph) {
  assert(sf3 != NULL);
  assert(eph != NULL);
  uint32_t u32;

  // 16-bit signed, lsb 2^-29 rad.
  eph->c_ic = SignedToFloat((sf3[2] >> 14) & 0xFFFF, 16, -29);

  // 32-bit signed, lsb 2^-31 semi-circle.
  u32 = ((sf3[2] << 18) & 0xFF000000) | ((sf3[3] >> 6) & 0x00FFFFFF);  // 32.
  eph->omega_0 = SignedToDouble(u32, 32, -31);

  // 16-bit signed, lsb 2^-29 rad.
  eph->c_is = SignedToFloat((sf3[4] >> 14) & 0xFFFF, 16, -29);

  // 32-bit signed, lsb 2^-31 semi-circle.
  u32 = ((sf3[4] << 18) & 0xFF000000) | ((sf3[5] >> 6) & 0x00FFFFFF);  // 32.
  eph->i_0 = SignedToDouble(u32, 32, -31);

  // 16-bit signed, lsb 2^-5 m.
  eph->c_rc = SignedToFloat((sf3[6] >> 14) & 0xFFFF, 16, -5);

  // 32-bit signed, lsb 2^-31 semi-circle.
  u32 = ((sf3[6] << 18) & 0xFF000000) | ((sf3[7] >> 6) & 0x00FFFFFF);  // 32.
  eph->omega = SignedToDouble(u32, 32, -31);

  // 24-bit signed, lsb 2^-43 semi-circle/s.
  eph->omega_dot = SignedToFloat((sf3[8] >> 6) & 0x00FFFFFF, 24, -43);

  eph->iode3 = (sf3[9] >> 22) & 0xFF;  // 8 bits.

  // 14-bit signed, lsb 2^-43 semi-circle/s.
  eph->i_dot = SignedToFloat((sf3[9] >> 8) & 0x3FFF, 14, -43);
}
