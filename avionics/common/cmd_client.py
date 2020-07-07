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

"""Common code for wing commandline clients."""

import cmd
import functools
import glob
import os
import threading

from makani.avionics.common import aio


class WingClientError(Exception):
  pass


def SelectArgs(args, candidates, require_some=True, require_all=False,
               require_one=False, select_all=False):
  """Matches the tokens in `args` to a collection of `candidates`.

  Matches are case-insensitive.  Partial matches are allowed as long as only one
  element of `candidates` is matched.

  Args:
    args: The line whose arguments are matched and selected.
    candidates: Iterable of possible values.
    require_some: Require at least one element of `args` to match.
    require_all: Require all elements of `args` to match.
    require_one: Require exactly one element of `args` to match.
    select_all: Allow the keyword 'All' to include all of `candidates`.

  Returns:
    (selected, remainder), where `selected` is the set of matched `candidates`,
        and `remainder` is the remaining arguments.

  Raises:
    WingClientError: If `args` is empty, if its first word is ambiguous,
        or if there are no matches.
  """
  selected = set()
  args_remain = []
  for arg in args:
    if select_all and arg.upper() == 'ALL':
      selected.update(candidates)
    else:
      match = None
      conflict = False
      for c in candidates:
        if c.upper() == arg.upper():
          match = c
          conflict = False
          break
        elif c.upper().find(arg.upper()) == 0:
          if match:
            conflict = True
          match = c
      if conflict:
        raise WingClientError('Ambiguous argument "%s". Must match one of:'
                              ' %s.' % (arg, ', '.join(candidates)))
      if match:
        selected.add(match)
      else:
        args_remain.append(arg)
  if (require_all or (require_some and not selected)) and args_remain:
    raise WingClientError('Unrecognized argument "%s". Must match one of: %s.'
                          % (args_remain[0], ', '.join(candidates)))
  if require_some and not selected:
    raise WingClientError('No valid arguments selected.')
  if require_one and len(selected) != 1:
    raise WingClientError('Multiple arguments selected.')
  if require_one:
    return list(selected)[0], args_remain
  else:
    return selected, args_remain


def CompleteFile(path):  # pylint: disable=invalid-name
  """Completes file paths command.

  Searches for file or directories matching the signature path.

  Args:
    path: A string containing a partially or fully complete path to a file.

  Returns:
    A list containing suggestions of directories and files that match the string
    in path. The character '/' is appended to directories. If there is only a
    single suggestion and it matches path, an empty list is returned.
  """
  matches = glob.glob(path + '*')
  suggestions = [os.path.basename(x) + '/' if os.path.isdir(x) else
                 os.path.basename(x) for x in matches]

  if len(suggestions) == 1 and suggestions[0] == path:
    return []

  return suggestions


class AioThread(threading.Thread):
  """A joint interface to AioClient and threading.Thread."""

  def __init__(self, message_types, **aio_client_kwargs):
    super(AioThread, self).__init__()
    self._client = aio.AioClient(message_types, **aio_client_kwargs)
    self._should_exit = False

  def run(self):
    while not self._should_exit:
      self._RunOnce()

  def Exit(self):
    self._should_exit = True

  def TryStop(self):
    if self.is_alive():
      self.Exit()
      self.join(1)
      if self.is_alive():
        raise WingClientError('Could not terminate thread.')


class _SourceErrorTracker(object):
  """Tracks errors encountered while running WingCommandClient.do_source."""

  def __init__(self):
    self._line_numbers = []
    self._error_lists = []
    self.any_errors = False

  def SetLineNumber(self, line_number):
    self._line_numbers.append(line_number)
    self._error_lists.append([])

  def AddError(self, exception):
    assert self._line_numbers
    self.any_errors = True
    self._error_lists[-1].append(exception)

  def PrintErrors(self):
    if not self.any_errors:
      return

    for line_number, error_list in zip(self._line_numbers, self._error_lists):
      if error_list:
        print 'Line %d' % line_number
        for error in error_list:
          print 'Error: %s' % error


def Command(num_args=None):
  """Creates a decorator for WingCommandClient command.

  Imposes common checks and supports error tracking with the "source" command.

  Args:
    num_args: The expected number of arguments to the command.  Use None
        (default) if this shouldn't be checked.  Multiple options can be passed
        in a list.

  Returns:
    Decorator.
  """
  # pylint: disable=missing-docstring
  def _Decorator(method):
    @functools.wraps(method)
    def _Wrapper(self, line):  # `self` refers to the CommandClient.
      # pylint: disable=protected-access
      try:
        if num_args is not None:
          actual_num_args = len(line.split())
          if isinstance(num_args, list) and actual_num_args not in num_args:
            raise WingClientError('Wrong number of arguments; expected a '
                                  'number in the set %s; got %d' %
                                  (num_args, actual_num_args))
          elif not isinstance(num_args, list) and actual_num_args != num_args:
            raise WingClientError('Wrong number of arguments; expected %s, '
                                  'got %d.' % (num_args, actual_num_args))
        method(self, line)
      except WingClientError as e:
        if self._source_error_tracker is not None:
          self._source_error_tracker.AddError(e)
        print e
      return False

    return _Wrapper

  return _Decorator


class WingCommandClient(cmd.Cmd):
  """Generic command line client for M600."""

  prompt = '(wing_client) '

  def __init__(self, *args, **kwargs):
    cmd.Cmd.__init__(self, *args, **kwargs)

    # Used to record exceptions when running source files.
    self._source_error_tracker = None

  def TryStopThreads(self):
    pass

  def _CompleteArg(self, text, matches):
    if text:
      return [p for p in matches
              if p.lower().startswith(text.lower())]
    else:
      return matches

  def emptyline(self):
    pass

  def do_EOF(self, unused_line):  # pylint: disable=invalid-name
    print 'Got EOF; exiting.'
    self.TryStopThreads()
    return True

  def do_quit(self, line):  # pylint: disable=invalid-name
    return self.do_EOF(line)

  @Command()
  def do_source(self, line):  # pylint: disable=invalid-name
    """Runs commands from a source file.

    Only "set_param"-like commands are allowed.

    Args:
      line: Command to this function.

    Raises:
      WingClientError: The file failed to open, or a command other than a
          "set_param"-like one was found in the file.
    """
    file_name = line.strip()
    try:
      with open(file_name, 'r') as f:
        # Pre-check that only "set_param"-like commands are used.
        for file_line in f:
          parts = file_line.split(None, 1)
          if (parts and not parts[0].startswith('#') and parts[0] not in
              ['set_param', 'set_param_dyno']):
            raise WingClientError('Only "set_param"-like commands may be '
                                  'included in a source file.')

        f.seek(0)
        print 'Executing file: ' + file_name
        self._source_error_tracker = _SourceErrorTracker()
        for i, file_line in enumerate(f):
          stripped = file_line.strip()
          if not stripped or stripped.startswith('#'):
            continue
          self._source_error_tracker.SetLineNumber(i + 1)
          self.onecmd(file_line)

        if self._source_error_tracker.any_errors:
          print 'Errors encountered while processing file %s.' % file_name
          self._source_error_tracker.PrintErrors()
        else:
          print 'All commands successful (file %s).' % file_name
        self._source_error_tracker = None
    except IOError:
      raise WingClientError('Failed to open file: %s.' % file_name)

  def complete_source(self, _, line, *unused_args):  # pylint: disable=invalid-name
    """Completes arguments for the "source" command."""
    args = line.split(None, 1)
    if len(args) <= 1:
      path = ''
    else:
      path = args[1]

    return CompleteFile(path)

  def __del__(self):
    """Stop running threads.

    This is a last resort to prevent the a program from hanging in case of an
    abnormal exit.
    """
    self.TryStopThreads()
