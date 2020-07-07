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

"""A wrapper for a dialog-driven interface using dialog or the command-line."""

import locale
import logging

import dialog


class Gauge(object):
  """A dialog gauge instance which maintains state between updates.

  A gauge stores the percentage complete and the textual description of the
  current state of a process or anything.
  """

  def __init__(self, dialog_instance, init_text='', title=''):
    self._dialog = dialog_instance
    self._percent = 0
    self._text = init_text
    if title and not init_text:
      self._text = '{}...'.format(title)
    self._title = title

  def __enter__(self):
    self._dialog.gauge_start(text=self._text, width=70, title=self._title)
    return self

  def __exit__(self, exception_type, exception_value, traceback):
    self._dialog.gauge_stop()

  def Update(self, percent=0, text=None):
    """Update the gauge's progress bar percent and text."""
    if percent:
      self._percent = percent
    if text:
      self._text = text
    self._dialog.gauge_update(percent=self._percent, text=self._text,
                              update_text=1)


class ProgressParser(object):
  """Stores and updates the progress of a task."""

  def __init__(self, text_init='', complete=None, update_callback=None,
               logger=None):
    self._complete = complete
    self._percent = 0
    self._text = text_init
    self._logger = logger
    self._update_callback = update_callback

  def Update(self, line):
    """Processes a string and calls the update_callback method."""
    if self._logger:
      self._logger.debug(line.rstrip('\n'))
    if self._update_callback:
      self._update_callback(*self.GetProgress())

  def GetProgress(self):
    return self._percent, self._text


class ProgressParserBootloader(ProgressParser):
  """Stores and updates the progress of the bootloader_client.py script."""

  def Update(self, line):
    if 'Sent' in line:
      self._text = line
      line_arr = line[line.index('Sent'):].split(' ')
      if len(line_arr) > 1 and self._complete > 0:
        self._percent = int(100 * float(line_arr[1]) / self._complete)
    elif 'Binary size:' in line:
      line_arr = line[line.index('Binary size'):].split(' ')
      if len(line_arr) > 2:
        self._complete = int(line_arr[2])
    return ProgressParser.Update(self, line)


class ProgressParserTiJtag(ProgressParser):
  """Stores and updates the progress of the CCS loadti script."""

  def __init__(self, file_size=0x40000, *args, **kwargs):
    super(ProgressParserTiJtag, self).__init__(*args, **kwargs)
    self._file_size = file_size

  def Update(self, line):
    if line.startswith('CortexR4:'):
      line_arr = line.split(' ')
      if 'Writing Flash' in line:
        mem_addr = line_arr[5]
        mem_len = line_arr[8]
        self._percent = int(
            100.0 * (int(mem_addr, 16) + float(int(mem_len, 16) / 2))
            / self._file_size)
    return ProgressParser.Update(self, line)


class UserInterface(object):
  """A wrapper for dialog which supports a command-line driven interface."""

  def __init__(self, mode='dialog'):
    assert mode in ('dialog', 'cmdline', 'noninteractive')
    self.mode = mode
    if self.mode == 'dialog':
      # Set locale (recommended for dialog)
      locale.setlocale(locale.LC_ALL, '')
      self.dialog_instance = dialog.Dialog(dialog='dialog', autowidgetsize=True)
      self.dialog_instance.add_persistent_args(['--colors', '--no-collapse'])

  def Info(self, text, title='', **kwargs):
    """Presents text to the user, no interaction is necessary.

    Args:
      text: A string which will be presented to the user.
      title: A string for the dialog box title.
      **kwargs: Keyword arguments will be passed to dialog function.
    """
    if self.mode == 'dialog':
      self.dialog_instance.infobox(text=text, title=title, **kwargs)
    else:
      print text

  def Message(self, text, title='', **kwargs):
    """Presents text to the user with an OK button.

    Args:
      text: A string which will be presented to the user with an OK button.
      title: A string for the dialog box title.
      **kwargs: Keyword arguments will be passed to dialog function.

    Returns:
      True if the user selected "OK", else False.
    """
    if self.mode == 'dialog':
      code = self.dialog_instance.msgbox(text=text, title=title, **kwargs)
      return code == self.dialog_instance.DIALOG_OK
    else:
      print text
      return True

  def YesNo(self, text, title='', default_no=False, **kwargs):
    """Prompts the user for a yes or no answer.

    Args:
      text: A string which will be presented to the user with the yes and no
          options.
      title: A string for the dialog box title.
      default_no: Set the default answer to "No."
      **kwargs: Keyword arguments will be passed to dialog function.

    Returns:
      The user-selected choice or the default choice when in noninteractive
      mode.
    """
    if self.mode == 'dialog':
      code = self.dialog_instance.yesno(text=text, title=title,
                                        defaultno=default_no, **kwargs)
      return code == self.dialog_instance.DIALOG_OK
    elif self.mode == 'cmdline':
      if title:
        print '({})'.format(title),
      answer = ''
      while answer.lower() not in ('y', 'n'):
        print
        answer = raw_input('{} (y\\n): '.format(text))
      return answer.lower() == 'y'
    else:
      print '{} ...Assuming ',
      if default_no:
        print '\'No\' ...'
      else:
        print '\'Yes\' ...'
      return not default_no

  def Input(self, text, input_init='', **kwargs):
    """Prompts the user for text input.

    Args:
      text: A string which will be presented to the user with the text input
          box.
      input_init: The default string to place in the input box.
      **kwargs: Keyword arguments will be passed to dialog function.

    Returns:
      The exit status (OK: True, Cancel: None) and the string entered by the
      user.

    """
    if self.mode == 'dialog':
      code, output = self.dialog_instance.inputbox(text=text, init=input_init,
                                                   **kwargs)
      return (code == self.dialog_instance.DIALOG_OK, output)
    elif self.mode == 'cmdline':
      print text,
      if input_init:
        print ' (default fill: {})'.format(input_init),
      print
      answer = raw_input('Answer: ')
      print
      return (True, answer)
    else:
      return None, None

  def _GetNumberFromUser(self, min_input, max_input):
    num = raw_input('Please make a selection: ')
    while not (num.isdigit() and int(num) >= min_input
               and int(num) <= max_input):
      print 'Invalid selection: {}'.format(num)
      num = raw_input('Please make a selection: ')
    return int(num)

  def Checklist(self, text, options, **kwargs):
    """Presents a menu of checkbox items.

    Args:
      text: A string which will be presented to the user with the checkboxes.
      options: A tuple of tuples in which each sub-tuple is of the form:
          (codename, description, status). status determines the initial default
          "checked" state and should be a string equal to "on" or "off."
      **kwargs: Keyword arguments will be passed to dialog function.

    Returns:
      The exit status (OK: True, Cancel: False) and a list of selected codenames
      or False and an empty list if the user cancelled.
    """
    if self.mode == 'dialog':
      code, choice = self.dialog_instance.checklist(text, choices=options,
                                                    **kwargs)
      return (code == self.dialog_instance.DIALOG_OK, choice)
    elif self.mode == 'cmdline':
      num = 0
      # Result stores the selection state, it is a Boolean.
      result = [item[2] == 'on' for item in options]
      while num != len(options) + 1:
        print text
        print '    {}: {}'.format(0, 'Cancel')
        for i, item in enumerate(options):
          print '[{}] {}: {} ({})'.format('X' if result[i] else ' ', i + 1,
                                          item[1], item[0])
        print '    {}: {}'.format(len(options) + 1, 'Done')
        num = self._GetNumberFromUser(0, len(options) + 1)
        print
        if num == 0:
          return False, []
        if num > 0 and num <= len(options):
          # Update the selection by inverting the selected entry.
          result[num - 1] = not result[num - 1]
      return True, [options[num][0] for num, res in enumerate(result) if res]
    else:
      return False, []

  def Menu(self, text, options, auto=True, **kwargs):
    """Presents a menu of checkbox items.

    Args:
      text: A string which will be presented to the user with the menu.
      options: A tuple of tuples in which each sub-tuple is of the form:
          (codename, description).
      auto: An option to enable auto-choose mode which will select the only menu
          option without user-interaction if only one option is to be offered.
      **kwargs: Keyword arguments will be passed to dialog function.

    Returns:
      The exit status (OK: True, Cancel: False) and the selected codename or an
      empty string.
    """
    if auto and len(options) == 1:
      logging.info('Auto selecting %s', options[0][0])
      self.Info('Auto selecting {}'.format(options[0][0]), title=text, height=3)
      return True, options[0][0]
    if self.mode == 'dialog':
      code, choice = self.dialog_instance.menu(text, choices=options, **kwargs)
      return (code == self.dialog_instance.DIALOG_OK, choice)
    elif self.mode == 'cmdline':
      print text
      print '{}: {}'.format(0, 'Cancel')
      for num, item in enumerate(options, 1):
        print '{}: {} ({})'.format(num, item[1], item[0])
      num = self._GetNumberFromUser(0, len(options) + 1)
      if num == 0:
        return False, ''
      else:
        return True, options[num - 1][0]
    else:
      return None, None
