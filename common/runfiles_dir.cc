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

#include "common/runfiles_dir.h"

#include <glog/logging.h>
#include <limits.h>
#include <stdlib.h>

DEFINE_string(runfiles_dir, "",
              "Overrides the root directory of the source tree.");

// TODO: Consider renaming this to MakaniHomeDir.
std::string RunfilesDir() {
  if (FLAGS_runfiles_dir.empty()) {
    const char *runfiles_dir = getenv("RUNFILES_DIR");
    if (runfiles_dir == nullptr) {
      LOG(FATAL) << "Neither --runfiles_dir nor RUNFILES_DIR are set.";
    }
    return std::string(runfiles_dir) + "/makani";
  }
  return FLAGS_runfiles_dir;
}

void SetRunfilesDirFromBinaryPath(const char *argv_0) {
  if (!FLAGS_runfiles_dir.empty()) return;
  if (getenv("RUNFILES_DIR") != nullptr) return;

  char target[PATH_MAX];
  CHECK_EQ(realpath(argv_0, target), target);
  FLAGS_runfiles_dir = std::string(target) + ".runfiles/makani";
}
