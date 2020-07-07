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

#include "lib/util/operator_confirmation.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include <string>

bool OperatorConfirmationPrompt(const std::string &action) {
  // Avoid any RNG seeding by using the clock for our confirmation string.
  struct timeval tv;
  gettimeofday(&tv, NULL);
  int arbitrary_number = 1000 + (static_cast<int32_t>(tv.tv_usec) % 9000);

  printf("To confirm the action:\n\n\t%s\n\nplease type in the string '%d': ",
         action.c_str(), arbitrary_number);

  int input = -1;
  int32_t rc = scanf("%d", &input);
  return rc == 1 && input == arbitrary_number;
}
