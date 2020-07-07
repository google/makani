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

#ifndef AVIONICS_FIRMWARE_STARTUP_NEWLIB_H_
#define AVIONICS_FIRMWARE_STARTUP_NEWLIB_H_

#include <sys/stat.h>

int _close(int fd);
int _fstat(int fd, struct stat *st);
int _getpid(void);
int _isatty(int fd);
int _kill(int pid, int sig);
off_t _lseek(int fd, off_t offset, int whence);
int _read(int fd, void *buf, size_t len);
void *_sbrk(int incr);
int _write(int fd, const void *buf, size_t len);

#endif  // AVIONICS_FIRMWARE_STARTUP_NEWLIB_H_
