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

#include <inttypes.h>
#include <mntent.h>
#include <stdio.h>
#include <string.h>
#include <sys/statvfs.h>
#include <sys/sysinfo.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/linux/aio.h"
#include "avionics/linux/q7_slow_status.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"

#define AIO_PERIOD_US (1000 * 1000)

namespace {

bool FindOption(const char *s, const char *opt) {
  const char *cur = s;
  size_t opt_len = strlen(opt);
  if (!opt_len) {
    return false;
  }
  while (*cur) {
    cur = strcasestr(cur, opt);
    if (!cur) {
      return false;
    }
    if ((cur == s || cur[-1] == ',') &&
        (cur[opt_len] == ',' || cur[opt_len] == 0)) {
      return true;
    }
    ++cur;
  }
  return false;
}

bool GetInt32FromCommand(const char *command, int32_t *value) {
  FILE *output = popen(command, "r");
  bool success = false;
  if (output) {
    success = fscanf(output, "%" SCNd32, value) == 1;
    pclose(output);
    if (!success) {
      fprintf(stderr, "fscanf failed.\n");
    }
  } else {
    perror("popen failed.\n");
  }
  return success;
}

// Validate that the supplied path is a mount point, and check whether it's
// writeable.
bool VerifyMountPoint(const char *path, bool *writeable) {
  FILE *mnt_entries = setmntent("/proc/mounts", "r");
  if (!mnt_entries) {
    perror("setmntent failed.\n");
    return false;
  }
  struct mntent *entry = NULL;
  bool found = false;
  while (!found && (entry = getmntent(mnt_entries))) {
    if (!strcmp(entry->mnt_fsname, "rootfs")) {
      // On the MX6, there are two root fs entries; we want the other one.
      continue;
    }
    if (!strcmp(entry->mnt_dir, path)) {
      if (FindOption(entry->mnt_opts, "rw")) {
        *writeable = true;
      } else if (FindOption(entry->mnt_opts, "ro")) {
        *writeable = false;
      } else {
        fprintf(stderr, "Failed to find mount mode in string '%s'.\n",
                entry->mnt_opts);
        *writeable = false;
      }
      found = true;
    }
  }
  endmntent(mnt_entries);
  return found;
}

// Get disk stats for path, IFF it's a mount point.
void GetDiskStats(const char *path, DiskInfo *disk_info) {
  struct statvfs stats;

  strncpy(disk_info->path, path, FS_PATH_LENGTH);
  bool writeable = false;
  if (VerifyMountPoint(path, &writeable)) {
    disk_info->flags |= kDiskInfoMounted;
    if (writeable) {
      disk_info->flags |= kDiskInfoWriteable;
    }
    if (!statvfs(path, &stats)) {
      // Block counts are actually in fragment units, not block units.
      disk_info->block_size = static_cast<int32_t>(stats.f_frsize);
      disk_info->total_blocks = stats.f_blocks;
      disk_info->available_blocks = stats.f_bavail;
      disk_info->total_inodes = stats.f_files;
      disk_info->available_inodes = stats.f_favail;
      disk_info->flags |= kDiskInfoUsageValid;
    }
  }
}

void GetAllDiskStats(AioNode node, DiskInfo disk_info[], size_t max_disks) {
  assert(max_disks >= 4);
  GetDiskStats("/", &disk_info[0]);
  GetDiskStats("/apps", &disk_info[1]);
  GetDiskStats("/run", &disk_info[2]);
  if (IsRecorderQ7Node(node)) {
    GetDiskStats("/logs", &disk_info[3]);
  }
}

bool GetTemperatureFromFile(const char *path, int16_t *temperature) {
  std::ifstream temp_stream(path);
  if (temp_stream.fail()) {
    return false;
  }
  int32_t temp_raw;
  temp_stream >> temp_raw;
  if (temp_raw >= 1000 || temp_raw <= -1000) {
    // Some systems scale by 1000.
    temp_raw = temp_raw / 1000;
  }
  if (temp_raw >= 1000 || temp_raw <= -1000) {
    fprintf(stderr, "temperature %" PRId32 " out of range even scaled down.\n",
            temp_raw);
    return false;
  }
  *temperature = (int16_t)temp_raw;
  return true;
}

bool GetSsdTemperature(int16_t *temperature) {
  const char *command = "/usr/sbin/smartctl -A /dev/sda | "
      "grep Temperature_Celsius | awk '{ print $10 }'";
  int32_t temp;
  if (GetInt32FromCommand(command, &temp)) {
    if (temp >= INT16_MIN && temp <= INT16_MAX) {
      *temperature = static_cast<int16_t>(temp);
      return true;
    }
    fprintf(stderr, "SSD temperature %d out of range.\n", temp);
  }
  return false;
}

void GetTemperatures(AioNode node, TemperatureInfo *temperatures) {
  if (GetTemperatureFromFile("/sys/class/thermal/thermal_zone0/temp",
                             &temperatures->cpu_zone_0)) {
    temperatures->flags |= kTemperatureInfoFlagCpuZone0Valid;
  }

  if (IsRecorderQ7Node(node)) {
    if (GetTemperatureFromFile("/sys/class/thermal/thermal_zone1/temp",
                               &temperatures->cpu_zone_1)) {
      temperatures->flags |= kTemperatureInfoFlagCpuZone1Valid;
    }
    if (GetSsdTemperature(&temperatures->ssd)) {
      temperatures->flags |= kTemperatureInfoFlagSsdValid;
    }
  }
}

bool GetMemAvailable(AioNode node, int32_t *mem_kb) {
  if (IsRecorderQ7Node(node)) {
    const char *command =
        "grep '^MemAvailable:' /proc/meminfo | awk '{ print $2 }'";
    return GetInt32FromCommand(command, mem_kb);
  } else {
    // This is a crude lower bound on free memory, for older kernels such as the
    // one we're running on the MX6 Q7.
    const char *free_command =
        "grep '^MemFree:' /proc/meminfo | awk '{ print $2 }'";
    const char *cached_command =
        "grep '^Cached:' /proc/meminfo | awk '{ print $2 }'";
    int32_t free, cached;
    if (GetInt32FromCommand(free_command, &free) &&
        GetInt32FromCommand(cached_command, &cached)) {
      *mem_kb = free + cached;
      return true;
    }
  }
  return false;
}

void GetSysinfo(AioNode node, SysInfo *sys_info) {
  struct sysinfo info;
  int32_t mem_available_kb;
  if (sysinfo(&info) == 0 && GetMemAvailable(node, &mem_available_kb)) {
    uint32_t mem_shift = 0;
    // This is here to allow us to test this on 64-bit systems.
    while ((static_cast<uint64_t>(info.totalram) >> mem_shift) >=
           (1ULL << 32)) {
      ++mem_shift;
    }
    sys_info->uptime = static_cast<uint32_t>(info.uptime);
    sys_info->load_averages[0] =
        static_cast<float>(info.loads[0]) / (1 << SI_LOAD_SHIFT);
    sys_info->load_averages[1] =
        static_cast<float>(info.loads[1]) / (1 << SI_LOAD_SHIFT);
    sys_info->load_averages[2] =
        static_cast<float>(info.loads[2]) / (1 << SI_LOAD_SHIFT);
    sys_info->total_memory =
        static_cast<uint32_t>(info.totalram >> mem_shift);
    sys_info->mem_units = static_cast<uint32_t>(info.mem_unit << mem_shift);
    sys_info->num_processes = info.procs;
    sys_info->available_memory = mem_available_kb *
        (1024 / sys_info->mem_units);
  }
}

// This excludes our own PID from its check, so if you run it on this
// binary, it will only tell you if an additional copy is running.
// If the system call fails for any reason, IsAppRunning returns false.
bool IsAppRunning(const char *app_name) {
  std::ostringstream string_stream;
  string_stream << "pidof -o " << getpid() << " '" << app_name <<
      "' > /dev/null";
  return !system(string_stream.str().c_str());
}

void GetAppStatus(AioNode node, uint8_t *is_running) {
  if (IsControllerNode(node)) {
    *is_running = IsAppRunning("/apps/control");
  } else if (IsRecorderQ7Node(node)) {
    // TODO: Can we monitor disk usage or log files to see if it's
    // actually working?  We'd need to avoid kill/restart loops if the rest
    // of the wing or the network just isn't up yet.
    *is_running = IsAppRunning("/usr/sbin/tcpdump");
  }
}

bool StartApp(AioNode node) {
  std::ostringstream string_stream;
  string_stream << "nohup ";
  if (IsControllerNode(node)) {
    string_stream << "/apps/control --controller_label=";
    string_stream << static_cast<int>(ControllerAioNodeToControllerLabel(node));
  } else if (IsRecorderQ7Node(node)) {
    string_stream << "/apps/lib/scripts/recorder/recorder start";
  }
  string_stream << " > /dev/null &";

  return !system(string_stream.str().c_str());
}

#define MAX_DEAD_LOOPS 10
void RestartIfNeeded(Q7SlowStatusContext *context, bool is_running) {
  static int counter = MAX_DEAD_LOOPS;

  // If the app is down for N checks in a row, restart it.
  if (!is_running) {
    if (++counter >= MAX_DEAD_LOOPS) {
      // Restart the app in the background, so as not to slow down this loop.
      // Give it N seconds to come up, just in case, to avoid starting more
      // than one.
      if (StartApp(context->node)) {
        counter = 0;
      }
    }
  } else {
    counter = 0;
  }
}

bool LoopFunction(void *arg) {
  Q7SlowStatusContext *context = reinterpret_cast<Q7SlowStatusContext *>(arg);

  Q7SlowStatusMessage message;
  // The message is cleared to all zeroes here, so we only fill in values on
  // success.
  Q7SlowStatusInitMessage(context, &message);

  GetAllDiskStats(context->node, message.disk_info, NUM_DISKS);
  GetSysinfo(context->node, &message.sys_info);
  GetAppStatus(context->node, &message.app_is_running);
  GetTemperatures(context->node, &message.temperature_info);

  RestartIfNeeded(context, message.app_is_running);

  AIO_SEND_PACKED(kMessageTypeQ7SlowStatus, PackQ7SlowStatusMessage,
                  PACK_Q7SLOWSTATUSMESSAGE_SIZE, &message);
  return true;
}

bool SetHostname(const char *name) {
  return !sethostname(name, strlen(name));
}

}  // namespace

int main(int argc, char **argv) {
  if (IsAppRunning(argv[0])) {
    fprintf(stderr, "Another copy is already running.\n");
    return EXIT_FAILURE;
  }
  if (argc != 1) {
    fprintf(stderr, "Warning: ignoring arguments.\n");
  }

  Q7SlowStatusContext context;
  Q7SlowStatusInit(&context);

  if (!IsQ7Node(context.node)) {
    fprintf(stderr, "Host '%s' is not a Q7 node.\n",
            AioNodeToString(context.node));
    return EXIT_FAILURE;
  }

  SetHostname(AioNodeToShortString(context.node));

  AioLoopStart(context.node, UDP_PORT_AIO, NULL, 0,
               LoopFunction, &context, AIO_PERIOD_US);

  AioClose();
  return EXIT_SUCCESS;
}
