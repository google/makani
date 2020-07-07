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

#include <netinet/in.h>
#include <netinet/ip.h>
#include <stdint.h>
#include <sys/time.h>

#include <numeric>
#include <vector>

#include "common/macros.h"
#include "lib/pcap_reader/ip_defragmenter.h"

class IpDefragmenterTest : public ::testing::Test {
 public:
  IpDefragmenterTest() : ip_defragmenter_(), test_payload_() {
    for (int32_t i = 0; i < ARRAYSIZE(test_payload_); ++i) {
      test_payload_[i] = static_cast<uint8_t>(i);
    }
  }

 protected:
  IpDefragmenter ip_defragmenter_;
  uint8_t test_payload_[65515];

  // Check that a completed unfragmented packet is handled correctly.
  void DefragmentUnfragmented() {
    struct timeval ts = {0, 0};
    struct iphdr header = {
        0U,
        0U,
        0U,
        htons(static_cast<uint16_t>(sizeof(iphdr) + 22U)),  // tot_len.
        0U,
        0U,
        0U,
        0U,
        0U,
        0U,
        0U};
    const uint8_t *full_payload;
    int32_t length =
        ip_defragmenter_.Defragment(ts, header, test_payload_, &full_payload);
    ASSERT_EQ(22, length);
    ASSERT_EQ(0, memcmp(full_payload, test_payload_, length));
    ASSERT_EQ(0, ip_defragmenter_.Length());
  }

  int32_t DefragmentGeneral(const std::vector<int32_t> &offsets,
                            const std::vector<int32_t> &lengths,
                            const std::vector<int32_t> &orders,
                            const uint8_t **full_payload) {
    struct timeval ts = {0, 0};
    struct iphdr header;
    memset(&header, 0, sizeof(header));

    uint8_t payload_fragment[65515];
    int32_t total_length;
    for (int32_t i : orders) {
      // Prepare IP header.
      if (i < static_cast<int32_t>(lengths.size()) - 1) {
        header.frag_off = htons(IP_MF);  // More fragments.
      } else {
        header.frag_off = 0U;
      }
      header.frag_off =
          header.frag_off | htons(static_cast<uint16_t>(offsets[i] >> 3));
      header.tot_len = htons(static_cast<uint16_t>(sizeof(iphdr) + lengths[i]));

      // Prepare IP payload.
      memcpy(payload_fragment, &test_payload_[offsets[i]], lengths[i]);

      // Defragment and test.
      total_length = ip_defragmenter_.Defragment(ts, header, payload_fragment,
                                                 full_payload);

      if (i != *prev(orders.end())) {
        EXPECT_EQ(total_length, -1)
            << "Defragment did not return -1 on incomplete packet.";
      }
    }

    return total_length;
  }

  void DefragmentCompletePacket(const std::vector<int32_t> &lengths,
                                const std::vector<int32_t> &orders) {
    std::vector<int32_t> offsets(lengths.size() + 1U, 0);
    std::partial_sum(lengths.begin(), lengths.end(), &offsets[1]);

    const uint8_t *full_payload;
    int32_t length = DefragmentGeneral(offsets, lengths, orders, &full_payload);

    // Check that the packet was assembled correctly.
    EXPECT_EQ(offsets.back(), length)
        << "Assembled packet is the wrong length.";
    EXPECT_EQ(0, memcmp(full_payload, test_payload_, length))
        << "Assembled packet is not equivalent to the test packet.";
    EXPECT_EQ(1, ip_defragmenter_.Length())
        << "Wrong number of packets still in the defragmenter.";

    // Check that the previous packet is cleared from the deque after
    // the next packet is processed.
    struct timeval ts = {0, 0};
    struct iphdr header;
    memset(&header, 0, sizeof(header));
    header.tot_len = htons(static_cast<uint16_t>(sizeof(iphdr) + 0U));
    EXPECT_EQ(
        0, ip_defragmenter_.Defragment(ts, header, full_payload, &full_payload))
        << "Defragmented packet is the wrong length.";
    EXPECT_EQ(0, ip_defragmenter_.Length())
        << "Defragmenter did not erase the last complete packet.";
  }
};

TEST_F(IpDefragmenterTest, DefragmentUnfragmented) { DefragmentUnfragmented(); }

TEST_F(IpDefragmenterTest, DefragmentSimpleFragmented) {
  DefragmentCompletePacket({16, 128, 64, 80, 8}, {0, 1, 2, 3, 4});
  DefragmentCompletePacket({0, 8, 0, 80, 256}, {0, 1, 2, 3, 4});
}

TEST_F(IpDefragmenterTest, DefragmentOutOfOrderFragmented) {
  DefragmentCompletePacket({16, 128, 64, 80, 8}, {2, 1, 0, 3, 4});
  DefragmentCompletePacket({0, 128, 0, 80, 8}, {4, 0, 2, 1, 3});
  DefragmentCompletePacket({8, 8, 8, 8, 65483}, {4, 3, 2, 1, 0});
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
