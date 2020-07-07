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

#include <gtest/gtest.h>

#include "lib/util/base64.h"
#include "lib/util/test_util.h"

TEST(Base64, Recovery) {
  for (int32_t i = 0; i < 10; ++i) {
    // Produce random binary data and then encode it into Base64.
    uint32_t len_in = rand() % 10000U;  // NOLINT(runtime/threadsafe_fn)
    void *buf_in = (void *)malloc(len_in);
    for (uint32_t j = 0U; j < len_in; ++j) {
      *((char *)buf_in + j) = (char)rand();
    }
    char *buf_64 = Base64Encode(buf_in, len_in);

    // Check that the output consists of only 7-bit ASCII characters.
    for (char *c = buf_64; *c != 0; c ++) {
      EXPECT_EQ('\0', *c & '\x80');
    }

    // Check that the Base64 encoded data, when decoded, matches the
    // original input.
    uint32_t len_out = 0;
    void *buf_out = Base64Decode(buf_64, &len_out);
    EXPECT_EQ(len_out, len_in);
    EXPECT_EQ(memcmp(buf_out, buf_in, len_in), 0);

    // Free the dynamically-allocated buffers.
    Base64Free(buf_out);
    free(buf_in);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
