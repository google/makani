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

#ifndef AVIONICS_FIRMWARE_DRIVERS_LOG_H_
#define AVIONICS_FIRMWARE_DRIVERS_LOG_H_

#include <stdbool.h>
#include <stddef.h>  // For size_t.
#include <stdint.h>

// This module implements a printf style circular buffer that stores only the
// parameter arguments rather than the rendered string output. To render the
// output strings, the consumer must have knowledge of the format strings
// (stored in .const memory).

// Implement printf-like functionality. On a 32-bit machine, this function can
// store four char/int types, or two float/double types. Format must reside
// in .const memory.
#ifndef NDEBUG
#define LOG_PRINTF(format, ...) LogPut("" format, ## __VA_ARGS__)
#else
#define LOG_PRINTF(format, ...) do {} while (0)
#endif

// Store call trace to aid debugging. Sprinkle LOG_TRACE() calls throughout
// your code to understand the call history.
#ifndef NDEBUG
#define LOG_TRACE() LOG_PRINTF(__FILE__ ":%ld %s\n", __LINE__, __func__)
#else
#define LOG_TRACE() do {} while (0)
#endif

// Call on boot.
void LogInit(void);

// Disable logging by ignoring calls to LogPut().
void LogDisable(void);

// Enable logging.
void LogEnable(void);

// Put printf format string into head position of circular buffer.
// Thread safe: can call from both interrupt and idle loop functions.
void LogPut(const char *format, ...) __attribute__((format(printf, 1, 2)));

// Get printf format arguments from tail position of circular buffer.
// Thread safe: can call from both interrupt and idle loop functions.
bool LogGet(int32_t *index, uint32_t *cycles, const char **format,
            size_t *arg0, size_t *arg1, size_t *arg2, size_t *arg3);

// Get rendered printf string from tail position of circular buffer.
// Call from idle loop. Returns number of bytes written to out.
int32_t LogRead(int32_t max_length, char *out);

// Render printf string from tail position of circular buffer to stdout.
// Call from idle loop. Returns false after exhausting the log buffer.
bool LogToStdout(void);

#endif  // AVIONICS_FIRMWARE_DRIVERS_LOG_H_
