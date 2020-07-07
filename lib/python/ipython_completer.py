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

"""Tab-completion of dictionary keys and ndarray fields in IPython.

To install, first create an IPython configuration file (if you don't
already have one) by running:

  ipython profile create

Then, add the following line to the file, which is usually located in
~/.config/ipython/profile_default/ipython_config.py:

  c.InteractiveShellApp.extensions = ['makani.lib.python.ipython_completer']

"""
import re

_MATCH_DICT_COMPLETION = re.compile(r"""(?x)
.*?([\.\w\[\]'"]+)
\[
(?:
  '([^']*) | "([^"]*)
)
$
""")


# pylint: disable=eval-used
def _CompleteDictOrNdarray(context, event):
  """Tab-completion for dictionary or ndarray.

  Args:
    context: IPython.terminal.interactiveshell.TerminalInteractiveShell object
        that contains information about the user's namespace.
    event: IPython.core.completer.Bunch object that contains
        information about the current line.

  Returns:
    List of strings that represent the keys or fields of the dictionary
        or ndarray.
  """
  try:
    obj_name = _MATCH_DICT_COMPLETION.split(event.line)[1]
    obj = eval(obj_name, context.user_ns)

    if hasattr(obj, 'keys'):
      return obj.keys()
    elif hasattr(obj, 'dtype'):
      return obj.dtype.names
    else:
      return []
  # Some versions of ipython will crash the completer if an exception slips
  # through, requiring a restart of ipython.
  except Exception as e:  # pylint: disable=broad-except
    print e


# pylint: disable=invalid-name
def load_ipython_extension(ipython=None):
  ipython.set_hook('complete_command', _CompleteDictOrNdarray,
                   re_key=_MATCH_DICT_COMPLETION)
