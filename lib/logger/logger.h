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

#ifndef LIB_LOGGER_LOGGER_H_
#define LIB_LOGGER_LOGGER_H_

namespace logger {

class Logger {
 public:
  static Logger& GetInstance() {
    static auto& instance = *new Logger();
    return instance;
  }

 private:
  Logger() {}
  static volatile bool exit_now_;

 public:
  bool Start(void);
  static void HandleStopSignal(int signum);
};

}  // namespace logger

#endif  // LIB_LOGGER_LOGGER_H_
