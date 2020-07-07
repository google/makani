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

#include <stdint.h>
#include <string.h>

#include <gtest/gtest.h>

#include <string>

extern "C" {

#include "avionics/common/nmea.h"
#include "avionics/common/strings.h"

}  // extern "C"

namespace {

bool RunTest(bool require_checksum, int32_t length, const uint8_t *data,
             NmeaReceive *nmea) {
  memset(nmea, 0, sizeof(*nmea));
  bool sync = true;
  int32_t parsed = 0;
  for (int32_t i = 0; i < length && sync && parsed == 0; ++i) {
    sync = NmeaParse(require_checksum, i + 1, data, nmea, &parsed);
  }
  return sync && parsed > 0;
}

}  // namespace

TEST(NmeaParse, Normal) {
  NmeaReceive nmea;

  // Parse message with checksum, and require checksum.
  const char gpgga_w_checksum[] =
      "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,"
      "*76\n";
  EXPECT_TRUE(RunTest(true, static_cast<int32_t>(strlen(gpgga_w_checksum)),
                      reinterpret_cast<const uint8_t *>(gpgga_w_checksum),
                      &nmea));
  EXPECT_EQ(nmea.checksum, 0x76);
  EXPECT_EQ(nmea.checksum_index, 68);
  EXPECT_EQ(nmea.fields, 15);
  EXPECT_EQ(nmea.field_delim[0], 6);
  EXPECT_EQ(nmea.field_delim[1], 17);
  EXPECT_EQ(nmea.field_delim[2], 27);
  EXPECT_EQ(nmea.field_delim[3], 29);
  EXPECT_EQ(nmea.field_delim[4], 40);
  EXPECT_EQ(nmea.field_delim[5], 42);
  EXPECT_EQ(nmea.field_delim[6], 44);
  EXPECT_EQ(nmea.field_delim[7], 46);
  EXPECT_EQ(nmea.field_delim[8], 51);
  EXPECT_EQ(nmea.field_delim[9], 56);
  EXPECT_EQ(nmea.field_delim[10], 58);
  EXPECT_EQ(nmea.field_delim[11], 63);
  EXPECT_EQ(nmea.field_delim[12], 65);
  EXPECT_EQ(nmea.field_delim[13], 66);
  EXPECT_EQ(nmea.field_delim[14], 67);

  // Parse message with checksum, and do not require checksum.
  EXPECT_TRUE(RunTest(false, static_cast<int32_t>(strlen(gpgga_w_checksum)),
                      reinterpret_cast<const uint8_t *>(gpgga_w_checksum),
                      &nmea));
  EXPECT_EQ(nmea.checksum, 0x76);
  EXPECT_EQ(nmea.checksum_index, 68);
  EXPECT_EQ(nmea.fields, 15);
  EXPECT_EQ(nmea.field_delim[0], 6);
  EXPECT_EQ(nmea.field_delim[1], 17);
  EXPECT_EQ(nmea.field_delim[2], 27);
  EXPECT_EQ(nmea.field_delim[3], 29);
  EXPECT_EQ(nmea.field_delim[4], 40);
  EXPECT_EQ(nmea.field_delim[5], 42);
  EXPECT_EQ(nmea.field_delim[6], 44);
  EXPECT_EQ(nmea.field_delim[7], 46);
  EXPECT_EQ(nmea.field_delim[8], 51);
  EXPECT_EQ(nmea.field_delim[9], 56);
  EXPECT_EQ(nmea.field_delim[10], 58);
  EXPECT_EQ(nmea.field_delim[11], 63);
  EXPECT_EQ(nmea.field_delim[12], 65);
  EXPECT_EQ(nmea.field_delim[13], 66);
  EXPECT_EQ(nmea.field_delim[14], 67);

  // Parse message without checksum, and require checksum.
  const char gpgga_wo_checksum[] =
      "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,\n";
  EXPECT_FALSE(RunTest(true, static_cast<int32_t>(strlen(gpgga_wo_checksum)),
                       reinterpret_cast<const uint8_t *>(gpgga_wo_checksum),
                       &nmea));

  // Parse message without checksum, and do not require checksum.
  EXPECT_TRUE(RunTest(false, static_cast<int32_t>(strlen(gpgga_wo_checksum)),
                      reinterpret_cast<const uint8_t *>(gpgga_wo_checksum),
                      &nmea));
  EXPECT_EQ(nmea.fields, 15);
  EXPECT_EQ(nmea.field_delim[0], 6);
  EXPECT_EQ(nmea.field_delim[1], 17);
  EXPECT_EQ(nmea.field_delim[2], 27);
  EXPECT_EQ(nmea.field_delim[3], 29);
  EXPECT_EQ(nmea.field_delim[4], 40);
  EXPECT_EQ(nmea.field_delim[5], 42);
  EXPECT_EQ(nmea.field_delim[6], 44);
  EXPECT_EQ(nmea.field_delim[7], 46);
  EXPECT_EQ(nmea.field_delim[8], 51);
  EXPECT_EQ(nmea.field_delim[9], 56);
  EXPECT_EQ(nmea.field_delim[10], 58);
  EXPECT_EQ(nmea.field_delim[11], 63);
  EXPECT_EQ(nmea.field_delim[12], 65);
  EXPECT_EQ(nmea.field_delim[13], 66);
  EXPECT_EQ(nmea.field_delim[14], 67);
}

TEST(NmeaParse, EndOfLine) {
  NmeaReceive nmea;

  // Parse message with \n end of line.
  char gpgga[] =
      "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,"
      "*76\n";
  const size_t gpgga_length = strlen(gpgga);
  EXPECT_TRUE(RunTest(true, static_cast<int32_t>(strlen(gpgga)),
                      reinterpret_cast<const uint8_t *>(gpgga),
                      &nmea));

  // Parse message with \r end of line.
  gpgga[gpgga_length - 1] = '\r';
  EXPECT_TRUE(RunTest(true, static_cast<int32_t>(strlen(gpgga)),
                      reinterpret_cast<const uint8_t *>(gpgga),
                      &nmea));

  // Parse message without end of line.
  gpgga[gpgga_length - 1] = '\0';
  EXPECT_FALSE(RunTest(true, static_cast<int32_t>(strlen(gpgga)),
                       reinterpret_cast<const uint8_t *>(gpgga),
                       &nmea));

  // Parse message with bad end of line.
  gpgga[gpgga_length - 1] = '\t';
  EXPECT_FALSE(RunTest(true, static_cast<int32_t>(strlen(gpgga)),
                       reinterpret_cast<const uint8_t *>(gpgga),
                       &nmea));

  // Parse message with bad end of line.
  gpgga[gpgga_length - 1] = 'X';
  EXPECT_FALSE(RunTest(true, static_cast<int32_t>(strlen(gpgga)),
                       reinterpret_cast<const uint8_t *>(gpgga),
                       &nmea));
}

TEST(NmeaParse, BadChecksum) {
  NmeaReceive nmea;

  // Parse message with valid checksum.
  char gpgga_w_checksum[] =
      "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,"
      "*76\n";
  EXPECT_TRUE(RunTest(true, static_cast<int32_t>(strlen(gpgga_w_checksum)),
                      reinterpret_cast<const uint8_t *>(gpgga_w_checksum),
                      &nmea));

  // Invalidate checksum, and require checksum.
  gpgga_w_checksum[strlen(gpgga_w_checksum) - 2] = '8';

  // Parse message with bad checksum, and require checksum.
  EXPECT_FALSE(RunTest(true, static_cast<int32_t>(strlen(gpgga_w_checksum)),
                       reinterpret_cast<const uint8_t *>(gpgga_w_checksum),
                       &nmea));

  // Parse message with bad checksum, and do not require checksum.
  EXPECT_FALSE(RunTest(false, static_cast<int32_t>(strlen(gpgga_w_checksum)),
                       reinterpret_cast<const uint8_t *>(gpgga_w_checksum),
                       &nmea));
}

TEST(NmeaParse, BadCharacter) {
  NmeaReceive nmea;

  // Parse message with valid checksum.
  char gpgga[] =
      "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,"
      "*76\n";
  EXPECT_TRUE(RunTest(true, static_cast<int32_t>(strlen(gpgga)),
                      reinterpret_cast<const uint8_t *>(gpgga),
                      &nmea));

  // Replace one character with a non-printable character.
  const int32_t index = 10;
  const char ch = gpgga[index];
  gpgga[index] = '\001';

  // Recompute checksum.
  int32_t checksum = nmea.checksum ^ ch ^ gpgga[index];
  WriteHexUint32(checksum & 0xFF, 2, 2, &gpgga[strlen(gpgga) - 3]);

  // Parse message with bad character and valid checksum.
  EXPECT_FALSE(RunTest(true, static_cast<int32_t>(strlen(gpgga)),
                       reinterpret_cast<const uint8_t *>(gpgga),
                       &nmea));
}

TEST(NmeaParse, MaxFields) {
  NmeaReceive nmea;

  // Create long NMEA string: $FIELDS,0,1,2,3,4,...
  const char *prefix = "$FIELDS";
  std::string msg = prefix;
  for (size_t i = 0; i < NMEA_FIELDS_MAX + 1; ++i) {
    msg += "," + std::to_string(i);
  }

  // Add checksum.
  uint8_t checksum = 0;
  size_t checksum_index;
  for (checksum_index = 1; checksum_index < msg.length(); ++checksum_index) {
    checksum = static_cast<uint8_t>(checksum ^ msg[checksum_index]);
  }
  ++checksum_index;
  const char *hex = "0123456789ABCDEF";
  msg += "*";
  msg += hex[(checksum >> 4) & 0x0F];
  msg += hex[checksum & 0x0F];
  msg += "\n";

  // Parse message with more fields than supported.
  EXPECT_TRUE(RunTest(true, static_cast<int32_t>(msg.length()),
                      reinterpret_cast<const uint8_t *>(msg.c_str()),
                      &nmea));
  EXPECT_EQ(nmea.checksum, checksum);
  EXPECT_EQ(nmea.checksum_index, checksum_index);
  EXPECT_EQ(nmea.fields, NMEA_FIELDS_MAX);
  for (size_t i = 0, delim = strlen(prefix); i < NMEA_FIELDS_MAX; ++i) {
    EXPECT_EQ(delim, nmea.field_delim[i]);
    delim += std::to_string(i).length() + 1;
  }
}

TEST(NmeaGetField, Normal) {
  NmeaReceive nmea;

  // Parse message.
  const char gpgga[] =
      "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,"
      "*76\n";
  EXPECT_TRUE(RunTest(true, static_cast<int32_t>(strlen(gpgga)),
                      reinterpret_cast<const uint8_t *>(gpgga),
                      &nmea));
  EXPECT_EQ(nmea.fields, 15);

  // Get first field, $GPGGA.
  int32_t field_length, field_start;
  NmeaGetField(&nmea, 0, &field_length, &field_start);
  EXPECT_EQ(field_length, 6);
  EXPECT_EQ(field_start, 0);

  // Get second field, 092750.000.
  NmeaGetField(&nmea, 1, &field_length, &field_start);
  EXPECT_EQ(field_length, 10);
  EXPECT_EQ(field_start, 7);

  // Get last field, ''.
  NmeaGetField(&nmea, nmea.fields - 1, &field_length, &field_start);
  EXPECT_EQ(field_length, 0);
  EXPECT_EQ(field_start, 67);

  // Get an invalid field.
  NmeaGetField(&nmea, nmea.fields, &field_length, &field_start);
  EXPECT_EQ(field_length, 0);
  EXPECT_EQ(field_start, 0);
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
