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

#ifndef COMMON_BARRIER_H_
#define COMMON_BARRIER_H_

// Prevent reordering of instructions by the compiler in either direction around
// this function call.
static inline void MemoryBarrier(void) {
  // This is a typical GCC memory barrier; details may be found via a Google
  // search. To summarize, the inline assembly syntax takes the form:
  //
  //   __asm__(string of assembly instructions
  //           : output operands
  //           : input operands
  //           : list of clobbered registers);
  //
  // In this case, we specify no assembly instructions and no input or output
  // operands. However, the keyword "memory" indicates that all memory may be
  // modified by this inline assembly code which prevents the compiler from
  // keeping values cached in registers and reordering operations across this
  // assembly code. Volatile is added to prevent this line from being optimized
  // out. Note that no assembly instruction is given which makes this valid for
  // all architectures and no CPU cycles are directly consumed.
  __asm__ __volatile__("" : : : "memory");
}

#endif  // COMMON_BARRIER_H_
