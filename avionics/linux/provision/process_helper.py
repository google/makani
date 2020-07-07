# Copyright 2020 Makani Technologies LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A subproccess handling helper."""

import logging
import os
import select
import signal
import subprocess


def RunProcessRetry(retries=3, no_exception=False):
  """Decorator for running a function multiple times upon "failure".

  Run the function as many as retries times as long as the function returns a
  non-zero return_code. The decorated function must return an iterable of
  (return_code, stdout_string, stderr_string).

  Args:
    retries: The number of attempts to make at running the decorated function.
    no_exception: Disables raising RuntimeError exceptions.

  Returns:
    A function which returns a tuple of: return_code, stdout, stderr.

  Raises:
    RuntimeError upon returning non-zero return_code retries times.
  """
  # pylint: disable=missing-docstring
  def Wrapper(func):
    # pylint: disable=missing-docstring
    def Call(*args, **kwargs):
      retry_count = 0
      local_logger = logging.getLogger(func.__name__)
      while retry_count < retries:
        return_code, stdout, stderr = func(*args, **kwargs)
        retry_count += 1
        if return_code == 0:
          break
        elif retry_count < retries:
          local_logger.warn('%s exited with return code "%i", trying '
                            '%i more times.', func.__name__, return_code,
                            retries-retry_count)
        else:
          local_logger.warn('%s exited with return code "%i" %i times, '
                            'giving up.', func.__name__, return_code, retries)
          local_logger.warn('Last attempt returned:')
          error_logger = local_logger.getChild('stdout')
          for line in stdout.split('\n'):
            error_logger.warn(line)
          error_logger = local_logger.getChild('stderr')
          for line in stderr.split('\n'):
            error_logger.warn(line)
          if not no_exception:
            raise RuntimeError('{} returned a non-zero return code {} '
                               'times.'.format(func.__name__, retries))
      return return_code, stdout, stderr
    return Call
  return Wrapper


class NonBlockingReader(object):
  """A non-blocking stream reader for processing lines.

  Implements a non-blocking stream reader which will read as much as possible
  from a stream and will call line_func on each completed line of input. The
  .readline() method will block, and using O_NONBLOCK does not work with select
  methods for polling. This provides a non-blocking equivalent of:
    for line in fd: line_func(line)
  while simultaneously storing the lines in a string accessible by calling the
  __str__() method.

  Args:
    fd: A file-handle which implements fileno().
    line_func: A function which will be called on each complete line of input
      read from fd.
    chunk_size: The size of reads (in bytes) attempted on fd using os.read().
  """

  def __init__(self, fd, line_func=None, chunk_size=1024):
    self._buffer = ''
    self._line_buffer = ''
    self._size = chunk_size
    self._fd = fd
    self._line_func = line_func

  def Slurp(self):
    """Slurp as much as possible from fd, call line_func on complete lines."""
    while True:
      buf = os.read(self._fd.fileno(), self._size)
      self._buffer += buf
      self._line_buffer += buf
      if self._line_func:
        while '\n' in self._line_buffer:
          newline_index = self._line_buffer.index('\n') + 1
          self._line_func(self._line_buffer[:newline_index])
          self._line_buffer = self._line_buffer[newline_index:]
      if len(buf) < self._size:
        break

  def __str__(self):
    return self._buffer

  def __repr__(self):
    return str(self)


def RunProcess(proc_exec, cwd='.', parse_stdout=None, parse_stderr=None,
               filter_stderr=None, timeout_stdio=10, stdin_pipe=None):
  """Run a process from the command line, stream stdout and stderr.

  The run_process method is a wrapper for the subprocess Popen. The stdout and
  stderr streams are split into lines and passed to functions if provided. A
  timeout will monitor the stdout and stderr and trigger a process kill if
  timeout_stdio seconds have passed without any input or output.

  Args:
    proc_exec: The list of the executable and arguments.
    cwd: A string of the working directory with which to execute the process.
    parse_stdout: An optional method which will be called for each line of
        stdout.
    parse_stderr: An optional method which will be called for each line of
        stderr.
    filter_stderr: A method for filtering stderr. The method should accept a
    list of lines and return only the lines which are truly errors.
    timeout_stdio: The number of seconds to wait for more output from stdout.
       The process is killed after timeout_stdio seconds have elapsed since last
       stdout.
    stdin_pipe: An I/O stream to be piped in to the process.

  Returns:
    A tuple of: (return code of process, stdout string, stderr string)

  Raises:

  """

  logging.debug('proc: %s cwd: %s', ' '.join(proc_exec), cwd)
  if proc_exec[0] == 'python':
    proc_exec.insert(1, '-u')
  proc = subprocess.Popen(
      proc_exec,
      stdin=stdin_pipe,
      stdout=subprocess.PIPE, stderr=subprocess.PIPE,
      cwd=cwd,
      shell=False, preexec_fn=os.setsid,
      bufsize=0)
  stdin_string = None
  if stdin_pipe:
    stdin_pipe_offset = stdin_pipe.tell()
    stdin_string = repr(stdin_pipe.read())
    stdin_pipe.seek(stdin_pipe_offset)
  logging.debug('Executing \'%s\' with cwd=\'%s\' stdin=%s',
                ' '.join(proc_exec), cwd, stdin_string)
  name = os.path.basename(' '.join(proc_exec))
  try:
    with proc.stderr, proc.stdout:
      stdout_processor = NonBlockingReader(proc.stdout, parse_stdout)
      stderr_processor = NonBlockingReader(proc.stderr, parse_stderr)

      while proc.poll() is None:
        # Select is used to get whichever file descriptor is ready first.
        (waitable_filenos, _, _) = select.select(
            [proc.stderr.fileno(), proc.stdout.fileno()],
            [],
            [],
            timeout_stdio)
        if len(waitable_filenos) < 1:
          logging.warning('%s process timed out, killing %i', name, proc.pid)
          os.killpg(proc.pid, signal.SIGHUP)
        for fileno in waitable_filenos:
          if fileno == proc.stderr.fileno():
            stderr_processor.Slurp()
          elif fileno == proc.stdout.fileno():
            stdout_processor.Slurp()

    logging.debug('%s process returned: %i', name, proc.poll())
  except:
    proc.terminate()
    raise
  stdout = str(stdout_processor)
  stderr = str(stderr_processor)
  if filter_stderr:
    stderr = '\n'.join(filter_stderr(stderr.split('\n')))
  return (proc.poll(), stdout, stderr)
