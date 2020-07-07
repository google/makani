// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <float.h>
#include <stdint.h>
#include <string.h>

#include <gtest/gtest.h>

extern "C" {

#include "avionics/common/gps_parse.h"
#include "common/macros.h"

}  // extern "C"

TEST(GpsParseSubframe1, NormalPrn3) {
  const uint32_t kSubframe1Prn3[10] = {
    0x22C32300, 0x0E346940, 0x32344000, 0x0866EA40, 0x18F6D900, 0x29BCDC00,
    0x14AF8300, 0x184AFC80, 0x00003740, 0x04BB7B80
  };
  GpsEphemeris eph;
  memset(&eph, 0x5A, sizeof(eph));  // Fill with garbage.
  GpsParseSubframe1(kSubframe1Prn3, &eph);

  // Truth data extracted from RINEX for PRN3 2015-01-13 02:00:00.
  // ftp://www.ngs.noaa.gov/cors/rinex/2015/013/brdc0130.15n.gz.
  // SV clock (line 537).
  EXPECT_EQ(eph.t_oc, 180000);
  EXPECT_NEAR(eph.a_f0, 1.444197259843e-4, 1e-16);   // Field 8.
  EXPECT_NEAR(eph.a_f1, 2.512479113648e-11, 1e-23);  // Field 9.
  EXPECT_NEAR(eph.a_f2, 0.000000000000e0, 1e-12);    // Field 10.
  // Broadcast orbit 5 (line 542).
  EXPECT_EQ(eph.wn, 1827 % 1024);  // Field 3.
  EXPECT_EQ(eph.l2pdata, 0);       // Field 4.
  // Broadcast orbit 6 (line 543).
  EXPECT_EQ(eph.ura, 1);     // Field 1.
  EXPECT_EQ(eph.health, 0);  // Field 2.
  EXPECT_NEAR(eph.t_gd, 5.587935447693e-9, 1e-21);  // Field 3.
  EXPECT_EQ(eph.iodc, 97);   // Field 4.
  // Other.
  EXPECT_EQ(eph.l2_ca_or_p, 1);
}

TEST(GpsParseSubframe2, NormalPrn3) {
  const uint32_t kSubframe2Prn3[10] = {
    0x22C32300, 0x0E348A80, 0x18408F80, 0x0C190100, 0x2D8664C0, 0x00868000,
    0x1F2E3240, 0x04FCE840, 0x038B1F80, 0x0AFC9FC0
  };
  GpsEphemeris eph;
  memset(&eph, 0x5A, sizeof(eph));  // Fill with garbage.
  GpsParseSubframe2(kSubframe2Prn3, &eph);

  // Truth data extracted from RINEX for PRN3 2015-01-13 02:00:00.
  // ftp://www.ngs.noaa.gov/cors/rinex/2015/013/brdc0130.15n.gz.
  // Broadcast orbit 1 (line 538).
  EXPECT_EQ(eph.iode2, 97);  // Field 1.
  EXPECT_NEAR(eph.c_rs, 1.793750000000e1, 1e-11);               // Field 2.
  EXPECT_NEAR(eph.delta_n * GPS_PI, 4.424470011221e-9, 1e-21);  // Field 3.
  EXPECT_NEAR(eph.m_0 * GPS_PI, 1.156333796115e-1, 1e-13);      // Field 4.
  // Broadcast orbit 2 (line 539).
  EXPECT_NEAR(eph.c_uc, 1.002103090286e-6, 1e-18);  // Field 1.
  EXPECT_NEAR(eph.ecc, 9.515519486740e-4, 1e-16);   // Field 2.
  EXPECT_NEAR(eph.c_us, 9.512528777122e-6, 1e-18);  // Field 3.
  EXPECT_NEAR(eph.sqrt_a, 5.153771724701e3, 1e-9);  // Field 4.
  // Broadcast orbit 3 (line 540).
  EXPECT_EQ(eph.t_oe, 180000);  // Field 1.
  // Broadcast orbit 7 (line 544).
  EXPECT_EQ(eph.fit_interval_flag, 0);  // Field 2 (clear indicates 4 hours).
}

TEST(GpsParseSubframe3, NormalPrn3) {
  const uint32_t kSubframe3Prn3[10] = {
    0x22C32300, 0x0E34AB00, 0x00040F00, 0x38DB5840, 0x0002C9C0, 0x04CF0A00,
    0x061CE140, 0x04328080, 0x3FEA34C0, 0x1840BD80
  };
  GpsEphemeris eph;
  memset(&eph, 0x5A, sizeof(eph));  // Fill with garbage.
  GpsParseSubframe3(kSubframe3Prn3, &eph);

  // Truth data extracted from RINEX for PRN3 2015-01-13 02:00:00.
  // ftp://www.ngs.noaa.gov/cors/rinex/2015/013/brdc0130.15n.gz.
  // Broadcast orbit 1 (line 538).
  EXPECT_EQ(eph.iode3, 97);
  // Broadcast orbit 3 (line 540).
  EXPECT_NEAR(eph.c_ic, 2.980232238770e-8, 1e-20);  // Field 2.
  EXPECT_NEAR(eph.omega_0 * GPS_PI, 1.494425871967e0, 1e-12);  // Field 3.
  EXPECT_NEAR(eph.c_is, 2.048909664154e-8, 1e-20);  // Field 4.
  // Broadcast orbit 4 (line 541).
  EXPECT_NEAR(eph.i_0 * GPS_PI, 9.590481427647e-1, 1e-13);         // Field 1.
  EXPECT_NEAR(eph.c_rc, 1.955937500000e2, 1e-10);                  // Field 2.
  EXPECT_NEAR(eph.omega * GPS_PI, -3.017264556426e0, 1e-12);       // Field 3.
  EXPECT_NEAR(eph.omega_dot * GPS_PI, -7.970689154054e-9, 1e-21);  // Field 4.
  // Broadcast orbit 5 (line 542).
  EXPECT_NEAR(eph.i_dot * GPS_PI, 6.750281176306e-11, 1e-23);  // Field 1.
}

TEST(GpsParseSubframe1, NormalPrn7) {
  const uint32_t kSubframe1Prn7[10] = {
    0x22C32300, 0x0E594900, 0x32340000, 0x0A66EA40, 0x18F6D900, 0x29BCDC00,
    0x14AFBA00, 0x14CAFC80, 0x00000780, 0x0E1F8C40
  };
  GpsEphemeris eph;
  memset(&eph, 0x5A, sizeof(eph));  // Fill with garbage.
  GpsParseSubframe1(kSubframe1Prn7, &eph);

  // Truth data extracted from RINEX for PRN7 2015-01-13 02:00:00.
  // ftp://www.ngs.noaa.gov/cors/rinex/2015/013/brdc0130.15n.gz.
  // SV clock (line 569).
  EXPECT_EQ(eph.t_oc, 180000);
  EXPECT_NEAR(eph.a_f0, 4.310067743063e-4, 1e-15);   // Field 8.
  EXPECT_NEAR(eph.a_f1, 3.410605131648e-12, 1e-24);  // Field 9.
  EXPECT_NEAR(eph.a_f2, 0.000000000000e0, 1e-12);    // Field 10.
  // Broadcast orbit 5 (line 574).
  EXPECT_EQ(eph.wn, 1827 % 1024);  // Field 3.
  EXPECT_EQ(eph.l2pdata, 0);       // Field 4.
  // Broadcast orbit 6 (line 575).
  EXPECT_EQ(eph.ura, 0);     // Field 1.
  EXPECT_EQ(eph.health, 0);  // Field 2.
  EXPECT_NEAR(eph.t_gd, -1.117587089539e-8, 1e-20);  // Field 3.
  EXPECT_EQ(eph.iodc, 83);   // Field 4.
  // Other.
  EXPECT_EQ(eph.l2_ca_or_p, 1);
}

TEST(GpsParseSubframe2, NormalPrn7) {
  const uint32_t kSubframe2Prn7[10] = {
    0x22C32300, 0x0E58CA00, 0x14FFD180, 0x0BECE300, 0x38B8B400, 0x3FD08100,
    0x04FED400, 0x0341E840, 0x037C5A40, 0x0AFC9FC0
  };
  GpsEphemeris eph;
  memset(&eph, 0x5A, sizeof(eph));  // Fill with garbage.
  GpsParseSubframe2(kSubframe2Prn7, &eph);

  // Truth data extracted from RINEX for PRN7 2015-01-13 02:00:00.
  // ftp://www.ngs.noaa.gov/cors/rinex/2015/013/brdc0130.15n.gz.
  // Broadcast orbit 1 (line 570).
  EXPECT_EQ(eph.iode2, 83);  // Field 1.
  EXPECT_NEAR(eph.c_rs, -5.812500000000e0, 1e-21);              // Field 2.
  EXPECT_NEAR(eph.delta_n * GPS_PI, 4.361253092268e-9, 1e-21);  // Field 3.
  EXPECT_NEAR(eph.m_0 * GPS_PI, -2.825315920812e0, 1e-12);      // Field 4.
  // Broadcast orbit 2 (line 571).
  EXPECT_NEAR(eph.c_uc, -3.539025783539e-7, 1e-19);  // Field 1.
  EXPECT_NEAR(eph.ecc, 7.964948192239e-3, 1e-15);    // Field 2.
  EXPECT_NEAR(eph.c_us, 6.211921572685e-6, 1e-18);   // Field 3.
  EXPECT_NEAR(eph.sqrt_a, 5.153742876053e3, 1e-7);   // Field 4.
  // Broadcast orbit 3 (line 572).
  EXPECT_EQ(eph.t_oe, 180000);  // Field 1.
  // Broadcast orbit 7 (line 576).
  EXPECT_EQ(eph.fit_interval_flag, 0);  // Field 2 (clear indicates 4 hours).
}

TEST(GpsParseSubframe3, NormalPrn7) {
  const uint32_t kSubframe3Prn7[10] = {
    0x22C32300, 0x0E58EB80, 0x3FF0E4C0, 0x0FCDF200, 0x3FEC09C0, 0x26106800,
    0x085E23C0, 0x3820C900, 0x3FEA3700, 0x14FE0600
  };
  GpsEphemeris eph;
  memset(&eph, 0x5A, sizeof(eph));  // Fill with garbage.
  GpsParseSubframe3(kSubframe3Prn7, &eph);

  // Truth data extracted from RINEX for PRN7 2015-01-13 02:00:00.
  // ftp://www.ngs.noaa.gov/cors/rinex/2015/013/brdc0130.15n.gz.
  // Broadcast orbit 1 (line 570).
  EXPECT_EQ(eph.iode3, 83);
  // Broadcast orbit 3 (line 572).
  EXPECT_NEAR(eph.c_ic, -1.136213541031e-7, 1e-19);             // Field 2.
  EXPECT_NEAR(eph.omega_0 * GPS_PI, -2.669201554250e0, 1e-12);  // Field 3.
  EXPECT_NEAR(eph.c_is, -1.490116119385e-7, 1e-19);             // Field 4.
  // Broadcast orbit 4 (line 573).
  EXPECT_NEAR(eph.i_0 * GPS_PI, 9.718014061493e-1, 1e-13);         // Field 1.
  EXPECT_NEAR(eph.c_rc, 2.677500000000e2, 1e-10);                  // Field 2.
  EXPECT_NEAR(eph.omega * GPS_PI, -2.751912420381e0, 1e-12);       // Field 3.
  EXPECT_NEAR(eph.omega_dot * GPS_PI, -7.967474734446e-9, 1e-21);  // Field 4.
  // Broadcast orbit 5 (line 574).
  EXPECT_NEAR(eph.i_dot * GPS_PI, -1.807218135032e-10, 1e-22);  // Field 1.
}

TEST(GpsParseSubframe1, NormalPrn16) {
  const uint32_t kSubframe1Prn16[10] = {
    0x22C32300, 0x0E58A980, 0x32340000, 0x0A66EA40, 0x18F6D900, 0x29BCDC00,
    0x14AFBA80, 0x034AFC40, 0x00000700, 0x3A942640
  };
  GpsEphemeris eph;
  memset(&eph, 0x5A, sizeof(eph));  // Fill with garbage.
  GpsParseSubframe1(kSubframe1Prn16, &eph);

  // Truth data extracted from RINEX for PRN16 2015-01-13 01:59:44.
  // ftp://www.ngs.noaa.gov/cors/rinex/2015/013/brdc0130.15n.gz.
  // SV clock (line 505).
  EXPECT_EQ(eph.t_oc, 179984);
  EXPECT_NEAR(eph.a_f0, -1.654447987676e-4, 1e-16);  // Field 8.
  EXPECT_NEAR(eph.a_f1, 3.183231456205e-12, 1e-24);  // Field 9.
  EXPECT_NEAR(eph.a_f2, 0.000000000000e0, 1e-12);    // Field 10.
  // Broadcast orbit 5 (line 510).
  EXPECT_EQ(eph.wn, 1827 % 1024);  // Field 3.
  EXPECT_EQ(eph.l2pdata, 0);       // Field 4.
  // Broadcast orbit 6 (line 511).
  EXPECT_EQ(eph.ura, 0);     // Field 1.
  EXPECT_EQ(eph.health, 0);  // Field 2.
  EXPECT_NEAR(eph.t_gd, -1.024454832077e-8, 1e-20);  // Field 3.
  EXPECT_EQ(eph.iodc, 13);   // Field 4.
  // Other.
  EXPECT_EQ(eph.l2_ca_or_p, 1);
}

TEST(GpsParseSubframe2, NormalPrn16) {
  const uint32_t kSubframe2Prn16[10] = {
    0x22C32300, 0x0E58CA00, 0x037F7D80, 0x0A9E88C0, 0x21085EC0, 0x3FAFC100,
    0x038016C0, 0x040F2840, 0x0375BE40, 0x0AFC46C0
  };
  GpsEphemeris eph;
  memset(&eph, 0x5A, sizeof(eph));  // Fill with garbage.
  GpsParseSubframe2(kSubframe2Prn16, &eph);

  // Truth data extracted from RINEX for PRN16 2015-01-13 01:59:44.
  // ftp://www.ngs.noaa.gov/cors/rinex/2015/013/brdc0130.15n.gz.
  // Broadcast orbit 1 (line 506).
  EXPECT_EQ(eph.iode2, 13);  // Field 1.
  EXPECT_NEAR(eph.c_rs, -1.631250000000e1, 1e-11);              // Field 2.
  EXPECT_NEAR(eph.delta_n * GPS_PI, 3.883733201648e-9, 1e-21);  // Field 3.
  EXPECT_NEAR(eph.m_0 * GPS_PI, 8.716971213869e-1, 1e-13);      // Field 4.
  // Broadcast orbit 2 (line 507).
  EXPECT_NEAR(eph.c_uc, -5.979090929031e-7, 1e-19);  // Field 1.
  EXPECT_NEAR(eph.ecc, 7.919322117232e-3, 1e-15);    // Field 2.
  EXPECT_NEAR(eph.c_us, 7.741153240204e-6, 1e-18);   // Field 3.
  EXPECT_NEAR(eph.sqrt_a, 5.153729967117e3, 1e-9);   // Field 4.
  // Broadcast orbit 3 (line 508).
  EXPECT_EQ(eph.t_oe, 179984);  // Field 1.
  // Broadcast orbit 7 (line 512).
  EXPECT_EQ(eph.fit_interval_flag, 0);  // Field 2 (clear indicates 4 hours).
}

TEST(GpsParseSubframe3, NormalPrn16) {
  const uint32_t kSubframe3Prn16[10] = {
    0x22C32300, 0x0E58EB80, 0x00056FC0, 0x1DAF9200, 0x000DCA00, 0x14108B80,
    0x07948240, 0x16B07EC0, 0x3FEA9BC0, 0x037F4E80
  };
  GpsEphemeris eph;
  memset(&eph, 0x5A, sizeof(eph));  // Fill with garbage.
  GpsParseSubframe3(kSubframe3Prn16, &eph);

  // Truth data extracted from RINEX for PRN16 2015-01-13 01:59:44.
  // ftp://www.ngs.noaa.gov/cors/rinex/2015/013/brdc0130.15n.gz.
  // Broadcast orbit 1 (line 506).
  EXPECT_EQ(eph.iode3, 13);
  // Broadcast orbit 3 (line 508).
  EXPECT_NEAR(eph.c_ic, 3.911554813385e-8, 1e-20);              // Field 2.
  EXPECT_NEAR(eph.omega_0 * GPS_PI, -1.583955649425e0, 1e-12);  // Field 3.
  EXPECT_NEAR(eph.c_is, 1.024454832077e-7, 1e-19);              // Field 4.
  // Broadcast orbit 4 (line 509).
  EXPECT_NEAR(eph.i_0 * GPS_PI, 9.894423929443e-1, 1e-13);         // Field 1.
  EXPECT_NEAR(eph.c_rc, 2.425625000000e2, 1e-10);                  // Field 2.
  EXPECT_NEAR(eph.omega * GPS_PI, 2.295945224363e-1, 1e-13);       // Field 3.
  EXPECT_NEAR(eph.omega_dot * GPS_PI, -7.823540167565e-9, 1e-21);  // Field 4.
  // Broadcast orbit 5 (line 510).
  EXPECT_NEAR(eph.i_dot * GPS_PI, -6.357407668690e-11, 1e-23);  // Field 1.
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
