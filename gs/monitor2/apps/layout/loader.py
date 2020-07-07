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

"""Detect and loads layouts."""
import importlib
import logging
import os
import sys

from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.plugins import layouts
from makani.gs.monitor2.project import settings


class LayoutLoader(object):
  """A class that detects and load layouts."""

  def __init__(self):
    layouts_dir = os.path.join(settings.MONITOR_PATH, 'apps/plugins/layouts')
    filenames = os.listdir(layouts_dir)
    # Scavenge for layout definitions.
    for filename in filenames:
      if filename.endswith(base.BaseLayout.SUFFIX + '.py'):
        module_name = filename[:-3]
        module_path = '%s.%s' % (layouts.__name__, module_name)
        try:
          importlib.import_module(module_path)
        except ImportError as e:
          logging.error(e.message)
          continue
        class_name = ''.join([item.capitalize()
                              for item in module_name.split('_')])
        cls = getattr(sys.modules[module_path], class_name)
        cls.Register()

  def GetLayoutByName(self, name):
    """Get a layout object associated with the layout name."""
    all_layouts = base.BaseLayout.Layouts()
    if name in all_layouts:
      layout_cls = all_layouts[name]
      return layout_cls()
    else:
      return None

  def GetLayoutByModuleName(self, module):
    """Get a layout object associated with the module name."""
    names_by_module_name = base.BaseLayout.NamesByModuleName()
    if module in names_by_module_name:
      all_layouts = base.BaseLayout.Layouts()
      name = names_by_module_name[module]
      if name in all_layouts:
        layout_cls = all_layouts[name]
        return layout_cls()
    return None

  def ModuleName(self, layout_name):
    for module, name in base.BaseLayout.NamesByModuleName().iteritems():
      if layout_name == name:
        return module
    return None

  def ModuleNames(self):
    return base.BaseLayout.NamesByModuleName().keys()

  def Names(self):
    """Get all layout names."""
    all_layouts = base.BaseLayout.Layouts()
    return all_layouts.keys()
