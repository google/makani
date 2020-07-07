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

#include "lib/logger/logger.h"

#include <glog/logging.h>
#include <stdint.h>

#include <csignal>

#include "avionics/common/avionics_messages.h"
#include "avionics/linux/aio_interface.h"
#include "avionics/recorder/logger_types.h"

namespace logger {

volatile bool logger::Logger::exit_now_ = false;

bool Logger::Start() {
  while (!exit_now_) {
    // TODO: Implement logic here.
  }
  return true;
}

void Logger::HandleStopSignal(int signum) {
  if (signum == SIGTERM || signum == SIGINT) {
    exit_now_ = true;
  }
}

}  // namespace logger
