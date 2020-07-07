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

#include "gs/monitor/indicators/indicators_m600_status.h"

#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/build_info.h"
#include "avionics/common/build_info_types.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/switch_links.h"
#include "common/macros.h"
#include "gs/monitor/indicators/system_build.h"
#include "gs/monitor/monitor.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/indicator.h"

#define MAX_LABEL_CHARS 1024

// Specify the number of non-routine packets to allow per second. Set this
// number greater than 0 to ignore the broadcast packets from the Microhard
// modems.
// TODO: Implement packet filtering on switches.
// TODO(b/129549274): Reduce this limit once the ground power traffic is
// dealt with.
#define MAX_NON_ROUTINE_PACKETS 45

// Specify the received signal strength (in dB) to illuminate the warning
// status. The Digi XLR PRO radios have a receive sensitivity of -112 dB.
#define XLR_RSSI_WARNING_THRESHOLD (-112 + 20)

// The Microhard pDDL radios have a receive sensitivity of -98 dBm.
#define PDDL_RSSI_WARNING_THRESHOLD (-98 + 20)

static SystemBuild g_system_build;

static void LevelUp(IndicatorState current, IndicatorState *state) {
  if (current > *state) {
    *state = current;
  }
}

__attribute__((format(printf, 3, 4))) static void PrintText(int32_t *length,
                                                            char *text,
                                                            const char *format,
                                                            ...) {
  if (*length < MAX_LABEL_CHARS) {
    va_list ap;
    va_start(ap, format);
    *length += vsnprintf(&text[*length], (size_t)(MAX_LABEL_CHARS - *length),
                         format, ap);
  }
}

static bool TruncateLines(int32_t min_lines, int32_t max_lines, int32_t *length,
                          char *text) {
  int32_t lines = 0;
  for (int32_t i = 0; i < *length; ++i) {
    lines += text[i] == '\n';
    if (lines >= max_lines) {
      text[i] = '\0';
      *length = i;
      return true;
    }
  }
  if (0 == *length || text[*length - 1] != '\n') {
    ++lines;
  }
  while (lines < min_lines && *length < MAX_LABEL_CHARS - 1) {
    text[*length] = '\n';
    ++(*length);
    text[*length] = '\0';
    ++lines;
  }
  return false;
}

static void NullTerminate(int32_t *length, char *text) {
  // Handles case when length == 0 and we did not initialize text[].
  if (0 > *length) {
    *length = 0;
  } else if (*length > MAX_LABEL_CHARS - 1) {
    *length = MAX_LABEL_CHARS - 1;
  }
  text[*length] = '\0';
}

static void StripLinefeed(int32_t *length, char *text) {
  // Remove trailing linefeed.
  if (0 < *length && *length <= MAX_LABEL_CHARS && text[*length - 1] == '\n') {
    *length -= 1;
  }
  NullTerminate(length, text);
}

static void UpdateIndicator(IndicatorState state, int32_t min_lines,
                            int32_t max_lines, int32_t *length, char *text,
                            Indicator *ind) {
  StripLinefeed(length, text);
  if (TruncateLines(min_lines, max_lines, length, text)) {
    // If we exceed MAX_LABEL_LINES, append "..." to the last line to
    // communicate that more errors/warnings exist.
    PrintText(length, text, "...");
  }
  indicator_set_value(ind, text);
  indicator_set_state(ind, state);
}

static void PrintShortBuildInfo(AioNode node, const BuildInfo *build_info,
                                IndicatorState *level, int32_t *length,
                                char *text) {
  const bool modified = build_info->flags & kBuildStatusModifiedFiles;
  const bool asserts = build_info->flags & kBuildStatusAssertsEnabled;
  PrintText(length, text, "%02x%02x%02x%s| ", build_info->checksum[0],
            build_info->checksum[1], build_info->checksum[2],
            modified | asserts ? "(*)" : "   ");
  if (SystemBuildUpdate(node, build_info, &g_system_build)) {
    LevelUp(INDICATOR_STATE_GOOD, level);
  } else {
    LevelUp(INDICATOR_STATE_WARNING, level);
    PrintText(length, text, "Version does not match system version.\n");
  }
}

static void PrintSystemAsserts(IndicatorState *level, int32_t *length,
                               char *text) {
  const BuildContainer *build;
  if (SystemBuildGetBuild(&g_system_build, &build)) {
    if (build->flags & kBuildStatusAssertsEnabled) {
      LevelUp(INDICATOR_STATE_WARNING, level);
      PrintText(length, text, "Enabled");
    } else {
      LevelUp(INDICATOR_STATE_GOOD, level);
      PrintText(length, text, "Disabled");
    }
  } else {
    LevelUp(INDICATOR_STATE_NONE, level);
  }
}

static void PrintSystemBuild(IndicatorState *level, int32_t *length,
                             char *text) {
  const BuildContainer *build;
  if (SystemBuildGetBuild(&g_system_build, &build)) {
    for (int32_t i = 0; i < ARRAYSIZE(build->checksum); ++i) {
      PrintText(length, text, "%02x", build->checksum[i]);
    }
    if (build->flags & kBuildStatusModifiedFiles) {
      PrintText(length, text, " (modified)");
    }
  }
  if (SystemBuildGetCount(&g_system_build) == 1) {
    LevelUp(INDICATOR_STATE_GOOD, level);
  } else if (SystemBuildGetCount(&g_system_build) > 1) {
    LevelUp(INDICATOR_STATE_WARNING, level);
  } else {
    LevelUp(INDICATOR_STATE_NONE, level);
  }
}

static bool CheckReceiveOnPort(const EthernetStats *stats) {
  return (stats->rx_multicast_packet_rate > 0 || stats->rx_octet_rate > 0 ||
          stats->rx_good_octet_rate > 0);
}

static bool CheckReceiveErrors(const EthernetStats *stats) {
  return (stats->rx_fragment_errors || stats->rx_alignment_errors ||
          stats->rx_fcs_errors || stats->rx_symbol_errors ||
          stats->rx_jabber_errors || stats->rx_in_range_errors);
}

static bool MultipleBitsSet(uint32_t mask) {
  return (mask & (mask - 1U)) != 0U;
}

// Return lowest port set.
static int32_t GetPort(uint32_t mask) {
  int32_t port = 0;
  while ((mask & 0x01) == 0x0) {
    mask >>= 1U;
    ++port;
  }
  return port;
}

static const SwitchLinkInfo *GetLinkFromPort(int32_t num_links,
                                             const SwitchLinkInfo *links,
                                             int32_t port) {
  for (int32_t i = 0; i < num_links; ++i) {
    if (port == links[i].local_port) {
      return &links[i];
    }
  }
  assert(false);
  return NULL;
}

static const SwitchLinkInfo *GetLocalLink(const SwitchLinkMap *map,
                                          uint32_t mask) {
  return GetLinkFromPort(map->num_local_links, map->local_links, GetPort(mask));
}

static const SwitchLinkInfo *GetRemoteLink(const SwitchLinkMap *map,
                                           uint32_t mask) {
  return GetLinkFromPort(map->num_remote_links, map->remote_links,
                         GetPort(mask));
}

static const SwitchLinkInfo *GetLink(const SwitchLinkMap *map,
                                     uint32_t local_mask,
                                     uint32_t remote_mask) {
  if (local_mask != 0x0) {
    return GetLocalLink(map, local_mask);
  } else if (remote_mask != 0x0) {
    return GetRemoteLink(map, remote_mask);
  }
  assert(false);
  return NULL;
}

static void PrintMultiplePorts(const char *preamble, uint32_t mask,
                               const char *postamble, int32_t *length,
                               char *text) {
  if (mask != 0x0) {
    PrintText(length, text, "%s", preamble);
    for (int32_t port = 0; port < (int32_t)sizeof(mask) * 8; ++port) {
      if (mask & (1U << port)) {
        PrintText(length, text, " %d,", port);
      }
    }
    *length -= 1;  // Remove last comma.
    PrintText(length, text, " %s", postamble);
  }
}

static void PrintSwitchStats(const SwitchLinkMap *map,
                             uint32_t link_status_bits,
                             uint32_t segment_status_bits,
                             const EthernetStats *stats, IndicatorState *level,
                             int32_t *length, char *text) {
  // Attempt to provide operator with meaningful information.

  // These ports connect to local devices (e.g., TMS570 or Q7).
  uint32_t local_links = 0x0;
  for (int32_t r = 0; r < map->num_local_links; ++r) {
    local_links |= 1U << map->local_links[r].local_port;
  }

  // These ports connect to remote devices.
  uint32_t remote_links = 0x0;
  uint32_t remote_aio_links = 0x0;
  for (int32_t r = 0; r < map->num_remote_links; ++r) {
    remote_links |= 1U << map->remote_links[r].local_port;
    if (map->remote_links[r].remote_aio) {
      remote_aio_links |= 1U << map->remote_links[r].local_port;
    }
  }

  // All links.
  uint32_t all_links = local_links | remote_links;

  // These links should be connected.
  uint32_t not_connected = ~link_status_bits & all_links;
  if (MultipleBitsSet(not_connected)) {
    LevelUp(INDICATOR_STATE_WARNING, level);
    PrintMultiplePorts("Ports", not_connected, "are not connected.\n", length,
                       text);
  } else if (not_connected != 0x0) {
    LevelUp(INDICATOR_STATE_WARNING, level);
    const SwitchLinkInfo *link =
        GetLink(map, not_connected & local_links, not_connected & remote_links);
    PrintText(length, text, "Port %d should be connected to %s.%d.\n",
              link->local_port, link->remote_name, link->remote_port);
  }

  // Ports 25 and 26 are forced on on the core switches, to make the gigabit
  // ports work.  They show as up even if not connected.
  uint32_t forced_links = 0x3 << 25;

  // These links should not be connected.
  uint32_t bad_connections = link_status_bits & ~all_links & ~forced_links;
  if (bad_connections != 0x0) {
    LevelUp(INDICATOR_STATE_WARNING, level);
    PrintMultiplePorts("Port(s)", bad_connections, "should not be connected.\n",
                       length, text);
  }

  // These links are connected, but not verified.
  uint32_t not_verified =
      ~segment_status_bits & ~not_connected & remote_aio_links;
  if (MultipleBitsSet(not_verified)) {
    LevelUp(INDICATOR_STATE_WARNING, level);
    PrintMultiplePorts("Ports", not_verified, "have not been verified.\n",
                       length, text);
  } else if (not_verified != 0x0) {
    LevelUp(INDICATOR_STATE_WARNING, level);
    const SwitchLinkInfo *link = GetRemoteLink(map, not_verified);
    if (CheckReceiveOnPort(&stats[link->local_port])) {
      PrintText(length, text, "Port %d from %s.%d not verified.\n",
                link->local_port, link->remote_name, link->remote_port);
    } else {
      PrintText(length, text, "Port %d has no communication from %s.%d.\n",
                link->local_port, link->remote_name, link->remote_port);
    }
  }

  // Find receive errors.
  uint32_t receive_errors = 0x0;
  for (int32_t r = 0; r < map->num_local_links; ++r) {
    int32_t local_port = map->local_links[r].local_port;
    if (CheckReceiveErrors(&stats[local_port])) {
      receive_errors |= 1U << local_port;
    }
  }
  for (int32_t r = 0; r < map->num_remote_links; ++r) {
    int32_t local_port = map->remote_links[r].local_port;
    if (CheckReceiveErrors(&stats[local_port])) {
      receive_errors |= 1U << local_port;
    }
  }

  // These links have communication errors.
  if (!not_connected && !bad_connections && !not_verified) {
    if (MultipleBitsSet(receive_errors)) {
      LevelUp(INDICATOR_STATE_WARNING, level);
      PrintMultiplePorts("Ports", receive_errors, "have receive errors.\n",
                         length, text);
    } else if (receive_errors) {
      LevelUp(INDICATOR_STATE_WARNING, level);
      const SwitchLinkInfo *link = GetLink(map, receive_errors & local_links,
                                           receive_errors & remote_links);
      PrintText(length, text, "Port %d has receive errors from %s.%d.\n",
                link->local_port, link->remote_name, link->remote_port);
    }
  }

  // No errors!
  if (!not_connected && !bad_connections && !not_verified && !receive_errors) {
    LevelUp(INDICATOR_STATE_GOOD, level);
  }
}

static void PrintAccessSwitchStats(const SwitchLinkMap *map,
                                   const AccessSwitchStats *as,
                                   IndicatorState *level, int32_t *length,
                                   char *text) {
  PrintSwitchStats(map, as->link_status_bits, as->segment_status_bits,
                   as->stats, level, length, text);
}

static void PrintCoreSwitchStats(const SwitchLinkMap *map,
                                 const CoreSwitchStats *cs,
                                 IndicatorState *level, int32_t *length,
                                 char *text) {
  PrintSwitchStats(map, cs->link_status_bits, cs->segment_status_bits,
                   cs->stats, level, length, text);
}

static void PrintAioStats(const AioStats *stats, IndicatorState *level,
                          int32_t *length, char *text) {
  if (stats->received_valid_aio_packets > 0) {
    LevelUp(INDICATOR_STATE_GOOD, level);
  }
  if (stats->received_invalid_aio_packets > 0) {
    LevelUp(INDICATOR_STATE_WARNING, level);
    PrintText(length, text, "Received %d invalid AIO packets.\n",
              stats->received_invalid_aio_packets);
  } else if (stats->received_non_routine_packets > MAX_NON_ROUTINE_PACKETS) {
    LevelUp(INDICATOR_STATE_WARNING, level);
    PrintText(length, text, "Received %d non-routine packets.\n",
              stats->received_non_routine_packets);
  }
}

static void PrintCvtStats(const CvtStats *stats, IndicatorState *level,
                          int32_t *length, char *text) {
  if (stats->invalid_packets > 0) {
    LevelUp(INDICATOR_STATE_WARNING, level);
    PrintText(length, text, "CVT received %d invalid messages.\n",
              stats->invalid_packets);
  }
}

static void PrintSelfTestResult(const SelfTestMessage *self_test,
                                IndicatorState *level, int32_t *length,
                                char *text) {
  LevelUp(INDICATOR_STATE_ERROR, level);
  PrintText(length, text, "%s\n", self_test->text);
}

static void PrintAccessSwitchNode(AioNode node, IndicatorState *level,
                                  int32_t *length, char *text) {
  const SlowStatusMessage *slow_status;
  if (CheckSlowStatus(node, &slow_status)) {
    PrintShortBuildInfo(node, &slow_status->build_info, level, length, text);
    PrintAccessSwitchStats(GetSwitchLinkMap(node), &slow_status->switch_stats,
                           level, length, text);
    PrintAioStats(&slow_status->network_status.aio_stats, level, length, text);
    PrintCvtStats(&slow_status->network_status.cvt_stats, level, length, text);
  } else {
    SystemBuildTimeout(node, &g_system_build);
  }
}

static void PrintCoreSwitchNode(AioNode node, IndicatorState *level,
                                int32_t *length, char *text) {
  const CoreSwitchSlowStatusMessage *slow_status;
  if (CheckCoreSwitchSlowStatus(node, &slow_status)) {
    PrintShortBuildInfo(node, &slow_status->build_info, level, length, text);
    PrintCoreSwitchStats(GetSwitchLinkMap(node), &slow_status->switch_stats,
                         level, length, text);
    PrintAioStats(&slow_status->network_status.aio_stats, level, length, text);
    PrintCvtStats(&slow_status->network_status.cvt_stats, level, length, text);
  } else {
    SystemBuildTimeout(node, &g_system_build);
  }
}

static void PrintTms570Node(AioNode node, IndicatorState *level,
                            int32_t *length, char *text) {
  const SelfTestMessage *self_test;
  if (CheckSelfTest(node, &self_test)) {
    PrintShortBuildInfo(node, &self_test->build_info, level, length, text);
    PrintSelfTestResult(self_test, level, length, text);
  } else if (IsCoreSwitchNode(node)) {
    PrintCoreSwitchNode(node, level, length, text);
  } else {
    PrintAccessSwitchNode(node, level, length, text);
  }
}

static IndicatorState GetQ7ErrorState(AioNode node) {
  return IsFlightComputerNode(node) ? INDICATOR_STATE_ERROR
                                    : INDICATOR_STATE_WARNING;
}

static const GitHash kAtomBuildWithWorkingGzipAndPdt = {
    0xb0, 0x73, 0xcd, 0x94, 0x6f, 0x42, 0x88, 0x53, 0x34, 0x20,
    0x9a, 0x62, 0x29, 0x9c, 0xe2, 0xbd, 0xa9, 0xda, 0x53, 0xdb};

static const GitHash kAtomBuildWithRecorderSetup = {
    0x11, 0xa3, 0x07, 0x18, 0xe9, 0x09, 0xa8, 0x8e, 0x08, 0x68,
    0x04, 0x12, 0xd9, 0x93, 0x6b, 0xd3, 0x9b, 0xa2, 0x05, 0xed};

static const GitHash kMX6BuildWithNetworkUpdate = {
    0xe2, 0x66, 0xa3, 0x69, 0x21, 0x19, 0x25, 0x16, 0xad, 0x33,
    0xc4, 0xdb, 0x0e, 0x9e, 0x73, 0x7a, 0x9b, 0x49, 0x61, 0x2b};

static const GitHash kMX6BuildWithMessageQueues = {
    0x0e, 0x73, 0xd0, 0x40, 0x3e, 0x40, 0xb1, 0xb3, 0x55, 0x7f,
    0x11, 0x6a, 0x27, 0x91, 0x23, 0x69, 0x4b, 0xe9, 0x84, 0x2b};

static const GitHash *kGoodRecorderHashes[] = {
    &kAtomBuildWithWorkingGzipAndPdt, &kAtomBuildWithRecorderSetup, NULL};

static const GitHash *kGoodControllerHashes[] = {
    &kMX6BuildWithNetworkUpdate, &kMX6BuildWithMessageQueues, NULL};

static const GitHash *kGoodGroundPowerHashes[] = {&kMX6BuildWithMessageQueues,
                                                  NULL};

static bool IsAcceptableBuild(const GitHash git_hash,
                              const GitHash *acceptable_hashes[]) {
  for (int i = 0; acceptable_hashes[i]; ++i) {
    if (!memcmp(git_hash, *acceptable_hashes[i], sizeof(GitHash))) {
      return true;
    }
  }
  return false;
}

static void PrintQ7YoctoGitHashInfo(AioNode node, const GitHash git_hash,
                                    IndicatorState *level, int32_t *length,
                                    char *text) {
  const GitHash **hashes = NULL;
  if (IsRecorderQ7Node(node)) {
    hashes = kGoodRecorderHashes;
  } else if (IsControllerNode(node)) {
    hashes = kGoodControllerHashes;
  } else if (IsGroundPowerQ7Node(node)) {
    hashes = kGoodGroundPowerHashes;
  } else {
    assert(false);
  }
  if (hashes && !IsAcceptableBuild(git_hash, hashes)) {
    LevelUp(GetQ7ErrorState(node), level);
    PrintText(length, text, "%s OS is out of date; hash is:\n",
              AioNodeToShortString(node));
    for (size_t i = 0; i < sizeof(GitHash); ++i) {
      PrintText(length, text, "%02x", git_hash[i]);
    }
  }
}

static void PrintQ7SystemInfo(AioNode node, const SysInfo *sys_info,
                              bool app_is_running, IndicatorState *level,
                              int32_t *length, char *text) {
  if (app_is_running) {
    LevelUp(INDICATOR_STATE_GOOD, level);
  } else {
    LevelUp(GetQ7ErrorState(node), level);
    PrintText(length, text, "Application not running!\n");
  }
  if (sys_info->available_memory <= 1 * 1024 * 1024) {
    LevelUp(GetQ7ErrorState(node), level);
    PrintText(length, text, "Critically low on memory [< 1MB].\n");
  } else if (sys_info->available_memory <= 32 * 1024 * 1024) {
    LevelUp(INDICATOR_STATE_WARNING, level);
    PrintText(length, text, "Low on memory [< 32MB].\n");
  }
  if (sys_info->load_averages[0] / (float)sys_info->num_cpus > 2.0) {
    LevelUp(GetQ7ErrorState(node), level);
    PrintText(length, text, "Very high CPU loads: %0.1f %0.1f %0.1f.\n",
              sys_info->load_averages[0], sys_info->load_averages[1],
              sys_info->load_averages[2]);
  } else if (sys_info->load_averages[0] / (float)sys_info->num_cpus > 1.6) {
    LevelUp(INDICATOR_STATE_WARNING, level);
    PrintText(length, text, "High CPU loads: %0.1f %0.1f %0.1f.\n",
              sys_info->load_averages[0], sys_info->load_averages[1],
              sys_info->load_averages[2]);
  }
  if (sys_info->num_processes > 70) {
    LevelUp(INDICATOR_STATE_WARNING, level);
    PrintText(length, text, "More processes than expected: %d.\n",
              sys_info->num_processes);
  }
}

static void PrintQ7DiskInfo(int32_t num_disks, const DiskInfo *disk_info,
                            IndicatorState *level, int32_t *length,
                            char *text) {
  for (int32_t i = 0; i < num_disks; ++i) {
    const DiskInfo *disk = &disk_info[i];
    if (disk->path[0] != '\0') {
      if (!(disk->flags & kDiskInfoMounted)) {
        LevelUp(INDICATOR_STATE_WARNING, level);
        PrintText(length, text, "Disk failure on %s.\n", disk->path);
        continue;
      }
      if (!strncmp(disk->path, "/logs", ARRAYSIZE(disk->path))) {
        if (!(disk->flags & kDiskInfoWriteable)) {
          LevelUp(INDICATOR_STATE_WARNING, level);
          PrintText(length, text, "SSD %s is not writeable.\n", disk->path);
        }
      } else if (strncmp(disk->path, "/run", ARRAYSIZE(disk->path))) {
        // No filesystem other than /logs and /run should be rw.
        if (disk->flags & kDiskInfoWriteable) {
          LevelUp(INDICATOR_STATE_WARNING, level);
          PrintText(length, text, "%s is mounted writeable.\n", disk->path);
        }
      }
      if (!(disk->flags & kDiskInfoUsageValid)) {
        LevelUp(INDICATOR_STATE_WARNING, level);
        PrintText(length, text, "Disk stat lookup failure on %s.\n",
                  disk->path);
      } else {
        if (!strncmp(disk->path, "/logs", ARRAYSIZE(disk->path))) {
          // Sanity check; we don't use any card smaller than 32GB, and allocate
          // 90% of the SSD to the /logs filesystem.  Make sure we see at least
          // 16 GiB.
          if (disk->total_blocks * disk->block_size < 16 * 1024 * 1024) {
            LevelUp(INDICATOR_STATE_WARNING, level);
            PrintText(length, text,
                      "Logging SSD is reporting bad size: %ld %dKB blocks.\n",
                      disk->total_blocks, disk->block_size);
          } else if ((float)disk->available_blocks / (float)disk->total_blocks <
                     0.05) {
            LevelUp(INDICATOR_STATE_WARNING, level);
            PrintText(length, text,
                      "Logging SSD is > 95%% full: %ld of %ld blocks free.\n",
                      disk->available_blocks, disk->total_blocks);
          } else if ((float)disk->available_blocks / (float)disk->total_blocks <
                     0.15) {
            LevelUp(INDICATOR_STATE_WARNING, level);
            PrintText(length, text,
                      "Logging SSD is > 85%% full: %ld of %ld blocks free.\n",
                      disk->available_blocks, disk->total_blocks);
          }
        } else if (!strncmp(disk->path, "/run", ARRAYSIZE(disk->path))) {
          if (disk->total_blocks * disk->block_size != 256 * 1024) {
            LevelUp(INDICATOR_STATE_WARNING, level);
            PrintText(length, text,
                      "Tempfs %s is reporting unexpected size: "
                      "%ld %dKB blocks.\n",
                      disk->path, disk->total_blocks, disk->block_size);
            if (disk->total_blocks == 0) {
              continue;
            }
          }
          if ((float)disk->available_blocks / (float)disk->total_blocks <
              0.05) {
            LevelUp(INDICATOR_STATE_WARNING, level);
            PrintText(length, text,
                      "Tempfs %s > 95%% full: %ld of %ld "
                      "blocks free.\n",
                      disk->path, disk->available_blocks, disk->total_blocks);
          } else if ((float)disk->available_blocks / (float)disk->total_blocks <
                     0.15) {
            LevelUp(INDICATOR_STATE_WARNING, level);
            PrintText(length, text,
                      "Tempfs %s > 85%% full: %ld of %ld "
                      "blocks free.\n",
                      disk->path, disk->available_blocks, disk->total_blocks);
          }
        }
      }
    }
  }
}

static bool GetMaxValid(bool a_valid, int16_t a, bool b_valid, int16_t b,
                        int16_t *max) {
  if (a_valid && !b_valid) {
    *max = a;
    return true;
  }
  if (b_valid && !a_valid) {
    *max = b;
    return true;
  }
  if (a_valid && b_valid) {
    *max = (a > b) ? a : b;
    return true;
  }
  return false;
}

static void PrintQ7TemperatureInfo(AioNode node,
                                   const TemperatureInfo *temperature_info,
                                   IndicatorState *level, int32_t *length,
                                   char *text) {
  if (temperature_info->flags & kTemperatureInfoFlagSsdValid) {
    if (temperature_info->ssd >= 85) {
      LevelUp(GetQ7ErrorState(node), level);
      PrintText(length, text,
                "SSD, at %dC, is at or over its max rated temperature.\n",
                (int)temperature_info->ssd);
    } else if (temperature_info->ssd > 70) {
      PrintText(length, text,
                "SSD, at %dC, is nearing its max rated temperature.\n",
                (int)temperature_info->ssd);
      LevelUp(INDICATOR_STATE_WARNING, level);
    }
  } else if (IsRecorderQ7Node(node)) {
    PrintText(length, text, "Failed to read SSD temperature.\n");
    LevelUp(INDICATOR_STATE_WARNING, level);
  }
  bool valid_0 = temperature_info->flags & kTemperatureInfoFlagCpuZone0Valid;
  bool valid_1 = temperature_info->flags & kTemperatureInfoFlagCpuZone1Valid;
  int16_t max_temperature;
  if (!GetMaxValid(valid_0, temperature_info->cpu_zone_0, valid_1,
                   temperature_info->cpu_zone_1, &max_temperature)) {
    LevelUp(INDICATOR_STATE_WARNING, level);
    PrintText(length, text, "Failed to read CPU temperature.\n");
  } else if (max_temperature >= 85) {
    LevelUp(GetQ7ErrorState(node), level);
    PrintText(length, text,
              "CPU, at %dC, is at or over its max rated temperature.\n",
              (int)max_temperature);
  } else if (max_temperature > 70) {
    PrintText(length, text,
              "CPU, at %dC, is nearing its max rated temperature.\n",
              (int)max_temperature);
    LevelUp(INDICATOR_STATE_WARNING, level);
  }
}

static void PrintQ7Node(AioNode node, IndicatorState *level, int32_t *length,
                        char *text) {
  const Q7SlowStatusMessage *slow_status;
  if (CheckQ7SlowStatus(node, &slow_status)) {
    PrintQ7YoctoGitHashInfo(node, slow_status->git_hash, level, length, text);
    PrintShortBuildInfo(node, &slow_status->build_info, level, length, text);
    if (!IsControllerNode(node)) {
      PrintQ7SystemInfo(node, &slow_status->sys_info,
                        slow_status->app_is_running, level, length, text);
      PrintQ7TemperatureInfo(node, &slow_status->temperature_info, level,
                             length, text);
      PrintQ7DiskInfo(ARRAYSIZE(slow_status->disk_info), slow_status->disk_info,
                      level, length, text);
    }
  }
}

static bool IsAccessSwitchCommsLinkUp(AioNode access_switch, int32_t port) {
  const SlowStatusMessage *slow_status;
  if (CheckSlowStatus(access_switch, &slow_status)) {
    const AccessSwitchStats *as = &slow_status->switch_stats;

    assert(0 <= port && port < ARRAYSIZE(as->stats));
    return (as->link_status_bits & (1U << port)) &&
           as->stats[port].rx_multicast_packet_rate > 0;
  }
  return false;
}

static bool IsCoreSwitchCommsLinkUp(AioNode core_switch, int32_t port) {
  const CoreSwitchSlowStatusMessage *slow_status;
  if (CheckCoreSwitchSlowStatus(core_switch, &slow_status)) {
    const CoreSwitchStats *cs = &slow_status->switch_stats;

    assert(0 <= port && port < ARRAYSIZE(cs->stats));
    return (cs->link_status_bits & (1U << port)) &&
           cs->stats[port].rx_multicast_packet_rate > 0;
  }
  return false;
}

static void PrintCommsLinkStatus(bool (*const is_link_up)(AioNode node,
                                                          int32_t port),
                                 AioNode node_a, int32_t port_a, AioNode node_b,
                                 int32_t port_b, IndicatorState *level,
                                 int32_t *length, char *text) {
  int32_t total = 0;
  int32_t up = 0;

  if (port_a >= 0) {
    if (is_link_up(node_a, port_a)) {
      PrintText(length, text, "Link A up");
      ++up;
    } else {
      PrintText(length, text, "Link A down");
    }
    ++total;
  }

  if (port_b >= 0) {
    if (total > 0) {
      PrintText(length, text, ", link ");
    } else {
      PrintText(length, text, "Link ");
    }
    if (is_link_up(node_b, port_b)) {
      PrintText(length, text, "B up");
      ++up;
    } else {
      PrintText(length, text, "B down");
    }
    ++total;
  }

  if (up <= 0) {
    LevelUp(INDICATOR_STATE_ERROR, level);
  } else if (up < total) {
    LevelUp(INDICATOR_STATE_WARNING, level);
  } else {
    LevelUp(INDICATOR_STATE_GOOD, level);
  }
}

static void InitWidget(Indicator *ind) {
  gtk_widget_modify_font(GTK_WIDGET(ind->value),
                         pango_font_description_from_string("Mono 8.0"));
}

static void UpdateCommsStatus(bool (*const is_link_up)(AioNode node,
                                                       int32_t port),
                              AioNode node_a, int32_t port_a, AioNode node_b,
                              int32_t port_b, int32_t init, Indicator *ind) {
  if (init) {
    InitWidget(ind);
  }
  if (init || !CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else {
    IndicatorState level = INDICATOR_STATE_NONE;
    int32_t length = 0;
    char text[MAX_LABEL_CHARS] = {0};

    PrintCommsLinkStatus(is_link_up, node_a, port_a, node_b, port_b, &level,
                         &length, text);
    UpdateIndicator(level, 1, 1, &length, text, ind);
  }
}

static void HandleJoystickRadioStatus(MicrohardStatus status, int32_t *length,
                                      char *text, IndicatorState *level) {
  if (!status.connected) {
    PrintText(length, text, "unc");
    LevelUp(INDICATOR_STATE_ERROR, level);
    return;
  }

  PrintText(length, text, "%3d", status.rssi);
  if (status.rssi < PDDL_RSSI_WARNING_THRESHOLD) {
    LevelUp(INDICATOR_STATE_WARNING, level);
  } else {
    LevelUp(INDICATOR_STATE_GOOD, level);
  }
}

void UpdateNodeStatus(Indicator *ind, int32_t init) {
  if (init) {
    InitWidget(ind);
    SystemBuildInit(&g_system_build);
  }
  if (init || !CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else {
    AioNode node = StringToAioNode(indicator_get_label(ind));
    IndicatorState level = INDICATOR_STATE_NONE;
    int32_t length = 0;
    char text[MAX_LABEL_CHARS] = {0};

    if (IsTms570Node(node)) {
      PrintTms570Node(node, &level, &length, text);
    } else if (IsQ7Node(node)) {
      PrintQ7Node(node, &level, &length, text);
    }
    if (level == INDICATOR_STATE_GOOD) {
      PrintText(&length, text, "Healthy");
    }
    UpdateIndicator(level, 2, 2, &length, text, ind);
  }
}

void UpdateSystemAssertStatus(Indicator *ind, int32_t init) {
  if (init) {
    InitWidget(ind);
    SystemBuildInit(&g_system_build);
  }
  if (init || !CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else {
    IndicatorState level = INDICATOR_STATE_NONE;
    int32_t length = 0;
    char text[MAX_LABEL_CHARS] = {0};

    PrintSystemAsserts(&level, &length, text);
    UpdateIndicator(level, 1, 1, &length, text, ind);
  }
}

void UpdateSystemBuildInfoStatus(Indicator *ind, int32_t init) {
  if (init) {
    InitWidget(ind);
    SystemBuildInit(&g_system_build);
  }
  if (init || !CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else {
    IndicatorState level = INDICATOR_STATE_NONE;
    int32_t length = 0;
    char text[MAX_LABEL_CHARS] = {0};

    PrintSystemBuild(&level, &length, text);
    UpdateIndicator(level, 1, 1, &length, text, ind);
  }
}

void UpdateSystemMonitorStatus(Indicator *ind, int32_t init) {
  static BuildInfo build;
  if (init) {
    InitWidget(ind);
    SystemBuildInit(&g_system_build);
    GetBuildInfo(&build);
  }
  if (init || !CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else {
    IndicatorState level = INDICATOR_STATE_NONE;
    int32_t length = 0;
    char text[MAX_LABEL_CHARS] = {0};

    PrintShortBuildInfo(kAioNodeOperator, &build, &level, &length, text);
    if (level == INDICATOR_STATE_GOOD) {
      PrintText(&length, text, "Healthy");
    }
    UpdateIndicator(level, 1, 1, &length, text, ind);
  }
}

void UpdateCommsStatusPoF(Indicator *ind, int32_t init) {
  UpdateCommsStatus(IsCoreSwitchCommsLinkUp, kAioNodeCsGsA, 20, kAioNodeCsGsB,
                    20, init, ind);
}

void UpdateCommsStatusEoP(Indicator *ind, int32_t init) {
  UpdateCommsStatus(IsCoreSwitchCommsLinkUp, kAioNodeCsGsA, -1, kAioNodeCsGsB,
                    -1, init, ind);
}

void UpdateCommsStatusWiFi(Indicator *ind, int32_t init) {
  UpdateCommsStatus(IsCoreSwitchCommsLinkUp, kAioNodeCsGsA, 18, kAioNodeCsGsB,
                    22, init, ind);
}

void UpdateCommsStatusJoystick(Indicator *ind, int32_t init) {
  UpdateCommsStatus(IsAccessSwitchCommsLinkUp, kAioNodeJoystickA, -1,
                    kAioNodeJoystickA, 3, init, ind);
  if (init || !CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else {
    IndicatorState level = INDICATOR_STATE_NONE;
    char text[MAX_LABEL_CHARS] = {0};
    int32_t length = 0;

    PrintText(&length, text, "Link B: ");

    if (CheckJoystickMonitorComms()) {
      HandleJoystickRadioStatus(aio_1->joystick_monitor.microhard_status,
                                &length, text, &level);
    } else {
      PrintText(&length, text, "--");
    }
    PrintText(&length, text, " dBm down, ");

    // TODO: Once this information is in TetherDown, that should be
    // used instead of CoreSwitchStatus.
    if (CheckCsComms(kCoreSwitchB)) {
      HandleJoystickRadioStatus(
          aio_1->core_switch_statuses[kCoreSwitchB].microhard_status, &length,
          text, &level);
    } else {
      PrintText(&length, text, "--");
    }
    PrintText(&length, text, " dBm up");

    UpdateIndicator(level, 1, 1, &length, text, ind);
  }
}

void UpdateCommsStatusLongRange(Indicator *ind, int32_t init) {
  if (init) {
    InitWidget(ind);
  }
  if (init || !CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else {
    IndicatorState level = INDICATOR_STATE_NONE;
    int32_t length = 0;
    char text[MAX_LABEL_CHARS] = {0};

    // We receive TetherDown from CsGsA over the XLR PRO radio.
    const TetherDownMessage *tether_down;
    if (CheckTetherDownComms(kAioNodeCsGsA, &tether_down)) {
      PrintText(&length, text, "Link A: %3d dBm down, %3d dBm up\n",
                tether_down->received_signal_strength,
                tether_down->comms_status.received_signal_strength);
      if (tether_down->received_signal_strength < XLR_RSSI_WARNING_THRESHOLD ||
          tether_down->comms_status.received_signal_strength <
              XLR_RSSI_WARNING_THRESHOLD) {
        LevelUp(INDICATOR_STATE_WARNING, &level);
      } else {
        LevelUp(INDICATOR_STATE_GOOD, &level);
      }
    } else {
      PrintText(&length, text, "Link A down\n");
      LevelUp(INDICATOR_STATE_ERROR, &level);
    }
    UpdateIndicator(level, 1, 1, &length, text, ind);
  }
}
