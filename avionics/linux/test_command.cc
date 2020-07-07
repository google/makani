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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iomanip>
#include <sstream>
#include <string>

#include "avionics/common/avionics_messages.h"
#include "avionics/linux/aio_interface.h"
#include "avionics/linux/clock.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"

DEFINE_string(node, "", "kAioNode name under test.");
DEFINE_string(tests, "*", "List of test suites and test names to execute.");
DEFINE_double(timeout, 5.0,
              "Maximum number of seconds to wait for device to report status.");

class TestCommand : private AioInterface {
 public:
  TestCommand(void);
  ~TestCommand(void) {}

  // Send TestExecute command to hardware, then wait for the results. Argument
  // tests_to_execute should be a comma or space separated list of test suites
  // and suite names to execute. Wildcards are permitted. Valid strings include
  // "Suite.Test", "?uite.T*", "S*.*", and "*". Returns true when all tests
  // pass.
  bool Execute(AioNode node, const std::string &tests_to_execute,
               int64_t node_timeout);

 private:
  // Overloaded functions from AioInterface.
  void OnStart(void) override;
  void OnNewStdioMessage(int64_t timestamp, const AioHeader *header,
                         int32_t length, const char *str) override;
  void OnNewMessage(int64_t timestamp, const AioHeader *header,
                    const AioMessageData *data) override;
  void OnPeriodic(void) override;

  // Current state of the test. Call ChangeState for each state change.
  enum class TestState { kInit, kWaitForHardware, kFinished };
  void ChangeState(TestState new_state);

  // Handle receive messages.
  void HandleTestStatus(const TestStatusMessage &msg);
  void HandleTestStart(const TestStartMessage &msg);
  void HandleTestFailure(const TestFailureMessage &msg);

  // Send start command.
  void SendTestExecute(void);

  // Output information to the console.
  void OutputTestStatus(const TestStatusMessage &msg) const;
  void OutputTestTimeout(void) const;

  AioNode node_under_test_;  // Node to query.
  int64_t node_timeout_;     // Maximum time to wait for the node to respond.

  std::string tests_to_execute_;  // A list of Suite.Test tests to execute.
  bool test_failures_;            // Indicates if a test experienced a failure.

  TestState state_;         // Current processing state.
  int64_t state_end_time_;  // Time to stop execution and abort test.
};

TestCommand::TestCommand(void)
    : AioInterface(10000), node_under_test_(kAioNodeUnknown),
      node_timeout_(0), tests_to_execute_(), test_failures_(false),
      state_(TestState::kInit), state_end_time_(0) {
}

bool TestCommand::Execute(AioNode node, const std::string &tests_to_execute,
                          int64_t node_timeout) {
  node_under_test_ = node;
  node_timeout_ = node_timeout;
  tests_to_execute_ = tests_to_execute;
  test_failures_ = false;
  ChangeState(TestState::kInit);

  Subscribe(kMessageTypeStdio);
  Subscribe(kMessageTypeTestFailure);
  Subscribe(kMessageTypeTestStart);
  Subscribe(kMessageTypeTestStatus);

  // The embedded host executes multiple tests concurrently. We therefore
  // must allow other programs to reuse the port and must handle receiving
  // messages from other nodes.
  SetReuseSocket(true);
  SetJoinAll(false);

  return Run(kAioNodeOperator) && test_failures_;
}

void TestCommand::OnStart(void) {
  LOG(INFO) << "Listening for " << AioNodeToString(node_under_test_) << "...";
}

void TestCommand::OnNewStdioMessage(int64_t /* timestamp */,
                                    const AioHeader *header, int32_t length,
                                    const char *str) {
  if (header->source == node_under_test_) {
    std::string s = std::string(str, length);
    if (s.find("ERROR") != std::string::npos
        || s.find("FAILED") != std::string::npos) {
      LOG(ERROR) << s;
    } else if (s.find("WARNING") != std::string::npos) {
      LOG(WARNING) << s;
    } else {
      LOG(INFO) << s;
    }
  }
}

void TestCommand::OnNewMessage(int64_t /* timestamp */, const AioHeader *header,
                               const AioMessageData *data) {
  if (header->source == node_under_test_) {
    switch (header->type) {
      case kMessageTypeTestStatus:
        HandleTestStatus(data->test_status);
        break;
      case kMessageTypeTestStart:
        HandleTestStart(data->test_start);
        break;
      case kMessageTypeTestFailure:
        HandleTestFailure(data->test_failure);
        break;
      default:
        break;
    }
  }
}

void TestCommand::OnPeriodic(void) {
  if (ClockGetUs() > state_end_time_) {
    test_failures_ = true;
    OutputTestTimeout();
    ChangeState(TestState::kFinished);
  }
  if (state_ == TestState::kFinished) {
    Stop();
  }
}

void TestCommand::ChangeState(TestState new_state) {
  state_ = new_state;
  state_end_time_ = ClockGetUs() + node_timeout_;
}

// Sent between tests and when idle.
void TestCommand::HandleTestStatus(const TestStatusMessage &msg) {
  if (state_ == TestState::kInit) {
    // Wait for initial TestStatusMessage from node.
    OutputTestStatus(msg);
    ChangeState(TestState::kWaitForHardware);
    SendTestExecute();
  } else if (state_ == TestState::kWaitForHardware) {
    // All tests report results with busy=true.
    if (msg.busy) {
      // Accumulate test results.
      CHECK_LE(msg.result.failures, msg.num_failures);
      test_failures_ |= msg.num_failures > 0;
    } else if (msg.result.index > 0) {
      // Wait for all tests to execute before exiting.
      ChangeState(TestState::kFinished);
    } else {
      // Test not executed. We may need to send the execute command multiple
      // times when test_command programs are run in parallel. All test_command
      // programs send the same message from the same source and therefore may
      // overwrite each other in the node's CVT.
      SendTestExecute();
    }
  }
}

// Sent automatically to indicate start of next test.
void TestCommand::HandleTestStart(const TestStartMessage &msg) {
  state_end_time_ = ClockGetUs() + msg.timeout_usec + node_timeout_;
}

// Sent on ASSERT/EXPECT test failures.
void TestCommand::HandleTestFailure(const TestFailureMessage & /* msg */) {
  test_failures_ = true;
}

void TestCommand::SendTestExecute(void) {
  LOG(INFO) << "Execute: " << tests_to_execute_.c_str();

  AioMessageData u;  // Message union.
  memset(&u, 0, sizeof(u));

  TestExecuteMessage &msg = u.test_execute;
  msg.node = node_under_test_;
  strncpy(msg.list, tests_to_execute_.c_str(), ARRAYSIZE(msg.list));
  msg.list[ARRAYSIZE(msg.list) - 1] = '\0';
  Send(kMessageTypeTestExecute, &u);
}

// Called upon reception of initial TestStatusMessage.
void TestCommand::OutputTestStatus(const TestStatusMessage &msg) const {
  std::string aio_node(msg.node, sizeof(msg.node));
  LOG(INFO) << "Aio node: " << aio_node;

  std::ostringstream git_hash;
  for (int32_t i = 0; i < ARRAYSIZE(msg.build_info.checksum); ++i) {
    int c = msg.build_info.checksum[i];
    git_hash << std::hex << std::setw(2) << std::setfill('0') << c;
  }
  if (msg.build_info.flags & kBuildStatusModifiedFiles) {
    git_hash << " (modified)";
  }
  LOG(INFO) << "Git hash: " << git_hash.str() << ".";
}

// Called when device fails to provide status within the timeout period.
void TestCommand::OutputTestTimeout(void) const {
  LOG(ERROR) << "Timeout waiting for device.";
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage("TMS570 unit test interface.");
  google::ParseCommandLineFlags(&argc, &argv, true);

  TestCommand test;
  bool pass = test.Execute(StringToAioNode(FLAGS_node.c_str()), FLAGS_tests,
                           static_cast<int64_t>(FLAGS_timeout * 1e6));

  if (pass) {
    LOG(INFO) << "FAILED.";
  } else {
    LOG(INFO) << "PASSED.";
  }
  return pass ? EXIT_FAILURE : EXIT_SUCCESS;
}
