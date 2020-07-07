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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <locale>
#include <string>

#include "avionics/linux/aio_interface.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_info.h"
#include "avionics/network/message_type.h"
#include "avionics/recorder/logger_types.h"
#include "common/macros.h"

DEFINE_bool(start, false, "Starts the logger.");
DEFINE_string(system, "", "Name of the system.");

DEFINE_bool(save, false, "Saves the log.");
DEFINE_string(tag_name, "", "Name tag of the log.");

DEFINE_bool(stop, false, "Stops the logger.");

class LogCommand : AioInterface {
 public:
  explicit LogCommand(int64_t periodic_interval_usec);
  ~LogCommand(void) {}

  bool StartCommand(std::string /* system */);
  bool SaveCommand(std::string /* tag_name */);
  bool StopCommand(void);

  void OnStart(void) override;
  void OnPeriodic(void) override;

 private:
  MessageType type_;
  AioMessageData data_;

  bool SendLogCommand(LoggerCommandType);
  bool Execute(MessageType type, const AioMessageData *data);
};

LogCommand::LogCommand(int64_t periodic_interval_usec)
    : AioInterface(periodic_interval_usec), type_(), data_() {}

bool LogCommand::StartCommand(std::string system) {
  AioMessageData u;  // Message union.
  memset(&u, 0, sizeof(u));

  LoggerCommandMessage &msg = u.logger_command;
  msg.command = kLoggerCommandStart;
  strncpy(msg.system_name, system.c_str(), system.size());

  return Execute(kMessageTypeLoggerCommand, &u);
}

bool LogCommand::SaveCommand(std::string tag_name) {
  AioMessageData u;  // Message union.
  memset(&u, 0, sizeof(u));

  LoggerCommandMessage &msg = u.logger_command;
  msg.command = kLoggerCommandSave;
  strncpy(msg.flight_name, tag_name.c_str(), tag_name.size());

  return Execute(kMessageTypeLoggerCommand, &u);
}

bool LogCommand::StopCommand() {
  AioMessageData u;  // Message union.
  memset(&u, 0, sizeof(u));

  LoggerCommandMessage &msg = u.logger_command;
  msg.command = kLoggerCommandStop;

  return Execute(kMessageTypeLoggerCommand, &u);
}

void LogCommand::OnStart(void) { LOG(INFO) << "Starting..."; }

bool LogCommand::Execute(MessageType type, const AioMessageData *data) {
  type_ = type;
  data_ = *data;

  SetReuseSocket(true);
  SetJoinAll(true);

  return Run(kAioNodeOperator);
}

void LogCommand::OnPeriodic() {
  LOG(INFO) << "Sending...";
  Send(type_, &data_);
  LOG(INFO) << "Stopping...";
  Stop();
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetUsageMessage("Send commands to the loggers.");

  // TODO(b/147677717): Validate the system name and the tag name using the
  // regexes in the file //lib/log_synchronizer/logsync_config.json

  // Validate flags.
  if (FLAGS_start + FLAGS_save + FLAGS_stop != 1) {
    LOG(ERROR) << "Must specify exactly one logger command.";
    google::ShowUsageWithFlagsRestrict(argv[0], "log.cc");
    return EXIT_FAILURE;
  } else if (FLAGS_start) {
    if (FLAGS_system.empty()) {
      LOG(ERROR) << "System name must be specified using the --system flag.";
      google::ShowUsageWithFlagsRestrict(argv[0], "log.cc");
      return EXIT_FAILURE;
    } else {
      if (FLAGS_system.size() > SYSTEM_NAME_LENGTH) {
        LOG(ERROR) << "System name must be smaller than " << SYSTEM_NAME_LENGTH
                   << " characters.";
        return EXIT_FAILURE;
      }
      for (char const &c : FLAGS_system) {
        if (!isalnum(c) && c != '_' && c != '-') {
          LOG(ERROR) << "System name can only contain alphanumeric characters, "
                        "dashes (-), and underscores (_).";
          return EXIT_FAILURE;
        }
      }
    }
  } else if (FLAGS_save) {
    if (FLAGS_tag_name.empty()) {
      LOG(INFO) << "Saving unnamed log.";
    } else {
      if (FLAGS_tag_name.size() > FLIGHT_NAME_LENGTH) {
        LOG(ERROR) << "Tag name must be smaller than " << FLIGHT_NAME_LENGTH
                   << " characters.";
      }
      for (char const &c : FLAGS_tag_name) {
        if (!isalnum(c) && c != '_' && c != '-') {
          LOG(ERROR) << "Tag name can only contain alphanumeric characters, "
                        "dashes (-), and underscores (_).";
          return EXIT_FAILURE;
        }
      }
    }
  }

  int64_t interval_usec = 10000;
  auto log_command = LogCommand(interval_usec);

  bool success = false;
  if (FLAGS_start + FLAGS_save + FLAGS_stop != 1) {
    LOG(ERROR) << "Must specify exactly one logger command.";
    google::ShowUsageWithFlagsRestrict(argv[0], "log.cc");
    return EXIT_FAILURE;
  } else if (FLAGS_start) {
    success = log_command.StartCommand(FLAGS_system);
  } else if (FLAGS_save) {
    success = log_command.SaveCommand(FLAGS_tag_name);
  } else if (FLAGS_stop) {
    success = log_command.StopCommand();
  } else {
    // Make sure that all commands are taken into account when validating the
    // flags above.
    LOG(ERROR) << "Illegal state error: flag not validated.";
    return EXIT_FAILURE;
  }

  return success ? EXIT_FAILURE : EXIT_SUCCESS;
}
