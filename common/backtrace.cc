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

#include "common/backtrace.h"

#include <cxxabi.h>
#define UNW_LOCAL_ONLY  // Only need local unwinding.
#include <libunwind.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// This is taken from
// http://eli.thegreenplace.net/2015/programmatic-access-to-the-call-stack-in-c/
void PrintBacktrace() {
  unw_cursor_t cursor;
  unw_context_t context;

  // Initialize cursor to current frame for local unwinding.
  unw_getcontext(&context);
  unw_init_local(&cursor, &context);

  // Unwind frames one by one, going up the frame stack.
  while (unw_step(&cursor) > 0) {
    unw_word_t offset, pc;
    unw_get_reg(&cursor, UNW_REG_IP, &pc);
    if (pc == 0) {
      break;
    }
    printf("0x%lx:", pc);

    char sym[256];
    if (unw_get_proc_name(&cursor, sym, sizeof(sym), &offset) == 0) {
      char* nameptr = sym;
      int status;
      char* demangled = abi::__cxa_demangle(sym, nullptr, nullptr, &status);
      if (status == 0) {
        nameptr = demangled;
      }
      printf(" %s+0x%lx\n", nameptr, offset);
      free(demangled);
    } else {
      printf(" -- error: unable to obtain symbol name for this frame\n");
    }
  }
}

static void BacktraceHandler(int sig) {
  printf("%s\n", strsignal(sig));
  PrintBacktrace();
  exit(1);
}

void InstallBacktraceHandler(const std::vector<int>& signals) {
  for (int sig : signals) {
    signal(sig, BacktraceHandler);
  }
}
