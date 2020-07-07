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

#include <glog/logging.h>

#include <csignal>

#include "common/macros.h"
#include "lib/logger/logger.h"

int main(int argc, char *argv[]) {
  UNUSED(argc);
  UNUSED(argv);

  auto logger = logger::Logger::GetInstance();

  signal(SIGTERM, logger.HandleStopSignal);
  signal(SIGINT, logger.HandleStopSignal);

  bool success = logger.Start();

  return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
