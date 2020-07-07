#!/usr/bin/python -u
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


r"""Bootload multiple targets in parallel with cool progress-bar display.

   Usage: bazel run //avionics/bootloader:parallel_bootloader -- \
                      "bootloader_client --target target1 [type] [binary1]" \
                      "bootloader_client --target target2 [type] [binary2]" \
                      ...
                      "bootloader_client --target targetN [type] [binaryN]"

   Press ESC to exit (if using the curses interface), or Ctrl-C.  The
   bootloading subprocesses will be terminated.

   Examples:

          bazel run //avionics/bootloader:parallel_bootloader -- \
                      "bootloader_client --target servo_a1" \
                      "bootloader_client --target servo_a2" \
                      "bootloader_client --target servo_a4" \
                      "bootloader_client --target servo_e1" \
                      "bootloader_client --target servo_e2" \
                      "bootloader_client --target servo_r1" \
                      "bootloader_client --target servo_r2" \

"""
import argparse
import curses
import datetime
import errno
import os
import re
import select
import signal
import subprocess
import sys

from makani.avionics.network import network_config
from makani.avionics.network import network_util

# TODO: Improve separation between user interface and functionality.
# TODO: Formalize state machine of "Task" objects.
# TODO: Check for target liveness using ping.
# TODO: Add timeouts and retries.
# TODO: Allow user-commanded retries.

_MAX_CONCURRENT_TASKS = 5


# We receive output byte-by-byte from the bootloader subprocesses,
# which we need to parse line-by-line.
class StringBuffer(object):
  """Allow line buffering of a buffer received byte-by-byte."""

  def __init__(self):
    self._buffer = ''

  def AddChars(self, chars):
    self._buffer += chars

  def IsLineAvailable(self):
    return '\n' in self._buffer

  def GetLine(self):
    assert self.IsLineAvailable()
    index = self._buffer.find('\n') + 1
    message = self._buffer[:index]
    self._buffer = self._buffer[index:]
    return message


class BootloadTask(object):
  """Keep track of one bootloader target."""

  def __init__(self, bootload_args):
    self._bootload_args = bootload_args
    if len(bootload_args) < 3:
      raise ValueError('Expect at least two arguments to a command '
                       '(such as bootloader_client --target motor_sbo).')

    # Boolean indicating whether the bootloader terminated successfully.
    self._bootloader_declared_success = False
    # Buffer in which to collect the stdout of the bootloader process.
    self._buffer = StringBuffer()
    # File descriptor of the bootloader process's stdout.
    self._fd = None
    # Index of this task (used for screen positioning in curses display).
    self._index = None
    # Last string received from the bootloader process.
    self._last_message = ''
    # Process object corresponding to the bootloader process.
    self._process = None
    # Boolean indicating whether the file descriptor is registered in the poll.
    self._registered = False
    # Total number of payload bytes sent by the bootloader to the target.
    self._sent_size = 0
    # Boolean indicating whether the bootloader client has been started.
    self._started = False
    # Time at which the last message was read from the bootloader client.
    self._t_last_heard = None
    # Time at which the bootloader client was started.
    self._t_start = None
    # Name of the target to bootload.
    self._target = bootload_args[2].lower()
    # Boolean indicating whether we've received acknowledgement from the target.
    self._target_ack = False
    # IP address of the target, as reported by the bootloader client.
    self._target_ip = ''
    # Set of tasks that must be complete before this one can start.
    self._tasks_to_wait_for = set()
    # Total size of the payload binary to bootload (see also _sent_size).
    self._total_size = 0
    # Concatenated unexpected output from the bootloader client.
    self._unparsed_messages = ''
    # Weight of the task in number of dependant tasks.
    self._weight = 0

  def GetTarget(self):
    return self._target

  def GetTaskName(self):
    """Get a short name describing this task."""
    if self.IsApplicationSegment():
      seg = 'boot_app' if self.IsBootloaderApplication() else 'app'
    elif self.IsBootloaderSegment():
      seg = 'boot'
    elif self.IsConfigSegment():
      seg = 'config'
    elif self.IsCalibSegment():
      seg = 'calib'
    return '%s.%s' % (self.GetTarget(), seg)

  def SetIndex(self, index):
    self._index = index

  def GetIndex(self):
    return self._index

  def GetProcess(self):
    return self._process

  def Start(self):
    """Start this bootloader task."""
    assert self.IsStartable() and not self.IsStarted()
    args = self._bootload_args

    try:
      self._process = subprocess.Popen(args,
                                       stderr=subprocess.STDOUT,
                                       stdout=subprocess.PIPE)
    except OSError as e:
      if use_curses:
        curses.endwin()
      print 'Could not execute bootloader client.  Command line was:'
      print ' '.join(args)
      raise e

    now = datetime.datetime.now()
    self._fd = self._process.stdout.fileno()
    self._t_start = now
    self._t_last_heard = now
    self._started = True
    return self._process

  def IsBootloaderApplication(self):
    return self.IsApplicationSegment() and any([
        'bootloader_application' in arg for arg in self._bootload_args])

  def IsApplicationSegment(self):
    """Return true iff we are programming the application segment."""
    return not (self.IsConfigSegment() or self.IsCalibSegment()
                or self.IsBootloaderSegment())

  def IsCalibSegment(self):
    """Return true iff we are programming the calibration."""
    return '--calib' in self._bootload_args

  def IsConfigSegment(self):
    """Return true iff we are programming the configuration segment."""
    return '--config' in self._bootload_args

  def IsBootloaderSegment(self):
    return '--bootloader' in self._bootload_args

  def IsComplete(self):
    """Is this job done?"""
    # When _process.poll() gives us a value other than None, we know
    # that the bootloader client has terminated.  We wait a few
    # additional seconds before indicating completion to allow the
    # target to reboot.
    return (self._process is not None
            and self._process.poll() is not None
            and (datetime.datetime.now() - self._t_last_heard
                 > datetime.timedelta(seconds=7.0)))

  def IsRunning(self):
    """Is this job currently running?"""
    return self._process is not None and self._process.poll() is None

  def IsStartable(self):
    """Is this job ready to be started?"""
    if self.IsStarted() or self.IsComplete():
      return False
    return all({task.IsComplete() for task in self._tasks_to_wait_for})

  def IsStarted(self):
    return self._started

  def Poll(self, fd):
    """Process any output from the underlying bootloader process."""
    assert self.IsStarted() and fd == self._fd
    read_buffer_size = 100
    chars = os.read(self._fd, read_buffer_size)
    self._t_last_heard = datetime.datetime.now()
    self._buffer.AddChars(chars)
    while self._buffer.IsLineAvailable():
      message = self._buffer.GetLine()
      self._ParseMessage(message)
      if use_curses:
        self.DisplayStatus()
      else:
        print '%s: %s' % (self._target, message),

  def _ParseMessage(self, message):
    """Parse output from the underlying bootloader process."""
    # Record this message in the object state for debugging purposes.
    self._last_message = message
    # Attempt to parse this message by matching it against expected
    # messages and recording pertinent state information.
    m = re.match('INFO: Attempting to flash (.*) segment on target'
                 ' (.*) \\[(.*), .*\\]\\.', message)
    if m:
      self._target_ip = m.group(3)
      return

    m = re.match('INFO: Binary size: (.*) bytes; target IP: (.*)\\.', message)
    if m:
      self._total_size = int(m.group(1))
      self._target_ip = m.group(2)
      return

    m = re.match('INFO: Sent (.*) bytes\\.\\.\\.\n', message)
    if m:
      self._sent_size = int(m.group(1))
      return

    m = re.match('INFO: Got an acknowledgement from target; starting upload\\.',
                 message)
    if m:
      self._target_ack = True
      return

    m = re.match('INFO: Successfully transferred (.*) bytes; cleaning up\\.',
                 message)
    if m:
      self._bootloader_declared_success = True
      self._sent_size = int(m.group(1))
      return

    # Messages that we simply ignore.
    if re.match('INFO: Target hardware type: (.*)\\.', message): return
    if re.match('INFO: Flashing file (.*)\\.', message): return
    if re.match('Another Bazel command .*\\.\\.\\.', message): return

    self._unparsed_messages += message

  def _DrawProgressBar(self, length, fraction):
    """Draw a progress bar."""
    complete_chars = int((length-2) * fraction)
    stdscr.addstr('[' + 'X' * complete_chars +
                  ' ' * (length - complete_chars - 2) + ']')

  def DisplayStatus(self):
    """Display the task's status at the position given by index."""
    if not use_curses:
      return
    num_top_vert_spaces = 2
    try:
      stdscr.move(self._index + num_top_vert_spaces, 0)
      start_row = stdscr.getyx()[0]
      stdscr.addstr('  %-30s ' % self.GetTaskName(),
                    curses.color_pair(4) if not self.IsApplicationSegment()
                    else curses.color_pair(3))
      stdscr.addstr('%-16s ' % self._target_ip,
                    curses.color_pair(2 if self._target_ack else 1))
      if not self.IsStarted():
        waiting_tasks = {task.GetTarget() for task in self._tasks_to_wait_for
                         if not task.IsComplete()}
        if waiting_tasks:
          stdscr.addstr('Waiting for: ' + ', '.join(waiting_tasks) + '.')
        else:
          stdscr.addstr('Waiting to start...')
        stdscr.clrtoeol()
      elif self.IsComplete():
        if self._process.poll() == 0:
          stdscr.addstr('Completed successfully.')
        else:
          stdscr.addstr('Failed.')
        stdscr.clrtoeol()
      else:
        progress = ((float(self._sent_size) / self._total_size)
                    if self._total_size > 0 else 0)
        stdscr.addstr('%3d%% ' % (progress * 100))
        progress_width = stdscr.getmaxyx()[1] - stdscr.getyx()[1] - 1
        if progress_width > 3 and start_row == stdscr.getyx()[0]:
          self._DrawProgressBar(progress_width, progress)
    except curses.error:
      pass
    stdscr.refresh()

  def Terminate(self, wait):
    """Terminate the underlying bootloader process."""
    if self._process is None:
      return
    try:
      # This can fail if, for example, the process has already died on its own.
      self._process.terminate()
      if wait:
        self._process.wait()
    except OSError:
      pass

  def PrintStatusAndTerminate(self):
    """Print any queued output from the underlying process and terminate."""
    if self._unparsed_messages:
      print 'Bootloading %s had the following unparsed messages:' % (
          self._target)
      print self._unparsed_messages
    if self.IsRunning():
      print 'Terminating PID %d (bootloading %s) ... ' % (
          self._process.pid, self._target)
    self.Terminate(True)

  def AddPrerequisiteTask(self, task_to_wait_for):
    self._tasks_to_wait_for.add(task_to_wait_for)

  def IncrementWeight(self):
    self._weight += 1

  def Weight(self):
    return self._weight


class BootloadTaskManager(object):
  """The list of all bootload tasks."""

  def __init__(self):
    # Whether the task manager should refresh the display.
    self._do_refresh_display = True
    # List of all tasks.
    self._tasks = []
    # Dict mapping file descriptors to indicies of task objects in _tasks.
    self._fd_to_index = dict()
    # Dict mapping target names to lists of task objects.
    self._target_name_to_tasks = dict()
    # Dict mapping tasks to booleans indicating whether they are
    # registered in our _poll object.
    self._registered_tasks = set()
    # Whether the user has pressed Ctrl-C.
    self._termination_requested = False
    # Create a poll object that will be used to listen for output from
    # the bootloader subprocesses.
    self._poll = select.poll()

  def AddTask(self, task):
    """Add a bootloader task to the task manager."""
    assert task not in self._tasks
    task.SetIndex(len(self._tasks))
    self._tasks.append(task)
    target = task.GetTarget()
    if target not in self._target_name_to_tasks:
      self._target_name_to_tasks[target] = []
    self._target_name_to_tasks[target] += [task]

  def GetTasks(self):
    return self._tasks

  def _CountRunningTasks(self):
    return len([task for task in self.GetTasks()
                if task.IsStarted() and task.IsRunning()])

  def SignalHandler(self, sig, _):
    if sig == signal.SIGINT:
      print 'Received SIGINT.  Cleaning up and terminating early.'
      self._termination_requested = True

  def StartSingleTask(self, task):
    """Start an individual task."""
    p = task.Start()
    fd = p.stdout.fileno()
    self._poll.register(fd, select.POLLIN)
    self._registered_tasks.add(task)
    self._fd_to_index[fd] = task.GetIndex()
    task.DisplayStatus()

  def StartStartableTasks(self):
    """Start all tasks that are startable."""
    started_something = False
    running_tasks = self._CountRunningTasks()
    weighted_tasks = sorted(self._tasks,
                            key=lambda task: -task.Weight())
    for task in weighted_tasks:
      if running_tasks >= _MAX_CONCURRENT_TASKS:
        break
      if task.IsStartable() and not task.IsStarted():
        self.StartSingleTask(task)
        running_tasks += 1
        started_something = True
    return started_something

  def IsComplete(self):
    """Are we done?"""
    return all([task.IsComplete() for task in self._tasks])

  def UnregisterTask(self, task):
    """Unregister a task, for instance if it has died."""
    self._poll.unregister(task.GetProcess().stdout.fileno())
    self._registered_tasks.remove(task)

  def Poll(self):
    """Poll all the tasks for waiting output or termination status."""
    poll_timeout_ms = 100     # Poll timeout [ms]

    # First poll the tasks to see if they have output pending for us to read.
    try:
      results = self._poll.poll(poll_timeout_ms)
      for result in results:
        (fd, event) = result
        if event & select.POLLIN:
          # There is output waiting from this file descriptor.  Look up
          # which task it belongs to.
          index = self._fd_to_index[fd]
          task = self._tasks[index]
          task.Poll(fd)
    except select.error as e:
      # Curses events (such as resizing the terminal) will interrupt
      # the system call behind the poll().  Catch this here to avoid
      # crashing the program.
      if e.args[0] != errno.EINTR:
        raise

    # Check for tasks who have completed.
    for task in self._tasks:
      if task.IsComplete():
        if task in self._registered_tasks:
          self.UnregisterTask(task)
          self._do_refresh_display = True

  def Run(self):
    """Run the task manager."""
    cycle = 0
    spinner = ['|', '/', '-', '\\']
    while not self.IsComplete():
      if self._do_refresh_display:
        for task in self._tasks:
          task.DisplayStatus()
        self._do_refresh_display = False

      cycle = (cycle + 1) % len(spinner)
      if use_curses:
        try:
          stdscr.move(len(self._tasks)+3, 2)
          stdscr.addstr('Bootloading... %c' % spinner[cycle])
        except curses.error:
          pass
        stdscr.refresh()
      self.Poll()
      self.StartStartableTasks()
      # Terminate early if the user presses escape (in curses mode) or
      # Ctrl-C.  Redraw the display if it has been resized and curses
      # is in use.

      if use_curses:
        ch = stdscr.getch()
        if ch == 27:
          self._termination_requested = True
        elif curses.KEY_RESIZE:
          self._do_refresh_display = True
      if self._termination_requested:
        return False
    # Catch any stragglers who terminated after the last display update.
    for task in self._tasks:
      task.DisplayStatus()
    return True

  def PrintStatusAndTerminate(self):
    """Print the status of all tasks, and then kill them."""
    for task in self._tasks:
      task.PrintStatusAndTerminate()

  def GetTasksByTargetName(self, target_name):
    """Return a list of tasks corresponding to the given target name."""
    #  A particular target name will have multiple tasks associated with
    #  it if we are also programming the configuration parameters.
    assert isinstance(target_name, str)
    if target_name in self._target_name_to_tasks:
      return self._target_name_to_tasks[target_name]
    else:
      return []

  def AddDependency(self, target, prereq_target):
    assert isinstance(target, str)
    assert isinstance(target, str)
    target_tasks = self.GetTasksByTargetName(target)
    prereq_tasks = self.GetTasksByTargetName(prereq_target)
    for target_task in target_tasks:
      for prereq_task in prereq_tasks:
        target_task.AddPrerequisiteTask(prereq_task)
        prereq_task.IncrementWeight()

  def AddDependencies(self, target, prerequisites):
    assert isinstance(target, str)
    for prereq in prerequisites:
      self.AddDependency(target, prereq)

  def AddInternalDependencies(self, target):
    """Enforce program order: bootloader_app, bootloader, app, config, calib."""
    def GetTaskPriority(task):
      if task.IsBootloaderSegment():
        return 1
      if task.IsApplicationSegment():
        return 0 if task.IsBootloaderApplication() else 2
      if task.IsConfigSegment():
        return 3
      if task.IsCalibSegment():
        return 4
    task_order = [None] * 5
    for task in self.GetTasksByTargetName(target):
      priority = GetTaskPriority(task)
      assert not task_order[priority], 'Duplicate task type for target.'
      task_order[priority] = task
    task_order = [task for task in task_order if task]
    for i in range(1, len(task_order)):
      task_order[i].AddPrerequisiteTask(task_order[i - 1])


def ParseArguments(argv):
  """Parse the command line arguments."""
  parser = argparse.ArgumentParser(
      description='Bootload several targets in parallel.')
  parser.add_argument('target', nargs='+')
  group = parser.add_mutually_exclusive_group()
  group.add_argument('--curses', dest='use_curses', action='store_true')
  group.add_argument('--no_curses', dest='use_curses', action='store_false')
  group.set_defaults(use_curses=True)

  args = parser.parse_args(argv)
  return (args.target, args.use_curses)


def AddDependencies(task_mgr):
  """Add bootload dependencies given unicast routes in the network config."""

  config = network_config.NetworkConfig()
  path_finder = network_util.PathFinder(config.GetSwitches(),
                                        config.all_messages)
  destinations = ['aio_nodes.' + t.GetTarget() for t in task_mgr.GetTasks()]
  paths = path_finder.GetHops(None, 'aio_nodes.operator', destinations,
                              unicast=True)
  for path in paths:
    for ingress_index, _ in enumerate(path[:-1]):
      ingress_all = path_finder.GetAttachedNodes(path[ingress_index])
      ingress_tms570 = [n for n in ingress_all
                        if config.GetAioNode(n).tms570_node]
      for egress_index in range(ingress_index + 1, len(path)):
        egress_all = path_finder.GetAttachedNodes(path[egress_index])
        egress_tms570 = [n for n in egress_all
                         if config.GetAioNode(n).tms570_node]
        for egress in egress_tms570:
          for ingress in ingress_tms570:
            task_mgr.AddDependency(egress, ingress)

  # Configuration tasks depend on the corresponding application tasks.
  for target in [task.GetTarget() for task in task_mgr.GetTasks()]:
    task_mgr.AddInternalDependencies(target)


def Main(argv):
  """Update multiple TMS570 boards with the supplied binaries."""
  global use_curses
  global stdscr

  all_complete = False

  (targets, use_curses) = ParseArguments(argv)
  tasks = BootloadTaskManager()

  for target in targets:
    tasks.AddTask(BootloadTask(target.split()))

  # Set up the graph of bootloading dependencies.
  AddDependencies(tasks)

  # Set up a handler for Ctrl-C.
  signal.signal(signal.SIGINT, tasks.SignalHandler)

  try:
    if use_curses:
      stdscr = curses.initscr()
      curses.start_color()
      curses.use_default_colors()
      for i in range(0, curses.COLORS):
        curses.init_pair(i, i, -1)
      curses.curs_set(0)
      stdscr.nodelay(True)
      stdscr.keypad(True)

    all_complete = tasks.Run()
  finally:
    if use_curses:
      curses.endwin()
    if all_complete:
      print 'Bootloading finished.  All processes have terminated.'
    if tasks:
      tasks.PrintStatusAndTerminate()

  # Exit with a nonzero exit code if we did not succeed in bootloading
  # all the targets.
  sys.exit(0 if all_complete else 1)

if __name__ == '__main__':
  # TODO: Abstract away the output routines into a
  # StatusUpdater class, to allow for clean implementation of
  # different kinds of user-interfaces (i.e. one child class for
  # curses, one for plain standard output).  In the meantime, we use
  # these two global variables to get the job done.
  stdscr = None
  use_curses = False
  Main(sys.argv[1:])
