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

// Minimal newlib syscall stubs with basic stdout and stderr support.

#include <errno.h>
#undef errno
extern int errno;

#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "avionics/common/network_config.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/startup/ldscript.h"
#include "avionics/firmware/startup/newlib.h"

void _exit(int status) {
  (void)status;
  while (1) {}
}

int _close(int fd) {
  (void)fd;
  errno = EBADF;
  return -1;
}

int _fstat(int fd, struct stat *st) {
  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
    memset(st, 0, sizeof(*st));
    st->st_mode = S_IFCHR;
    return 0;
  } else {
    errno = EBADF;
    return -1;
  }
}

int _getpid(void) {
  return 1;
}

int _isatty(int fd) {
  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
    return 1;
  } else {
    errno = EBADF;
    return 0;
  }
}

int _kill(int pid, int sig) {
  (void)pid;
  (void)sig;
  errno = EINVAL;
  return -1;
}

off_t _lseek(int fd, off_t offset, int whence) {
  (void)offset;
  (void)whence;
  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
    errno = ESPIPE;
    return -1;
  } else {
    errno = EBADF;
    return -1;
  }
}

int _read(int fd, void *buf, size_t len) {
  (void)fd;
  (void)buf;
  (void)len;
  errno = EBADF;
  return -1;
}

void *_sbrk(int incr) {
  static uint8_t *brk = ldscript_heap_begin;
  if (brk + incr <= ldscript_heap_end) {
    uint8_t *pbrk = brk;
    brk += incr;
    return pbrk;
  } else {
    errno = ENOMEM;
    return (void *)-1;
  }
}

int _write(int fd, const void *buf, size_t len) {
  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
#ifdef NDEBUG
    (void)buf;
    (void)len;
    errno = EIO;
    return -1;
#else
    if (NetSendAio(kMessageTypeStdio, buf, len)) {
      return len;
    } else {
      errno = EIO;
      return -1;
    }
#endif
  } else {
    errno = EBADF;
    return -1;
  }
}
