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

"""List the available layouts in web monitor."""

from makani.gs.monitor2.apps.layout import loader


# TODO: Merge this command into the web monitor binary.
def main():
  print '---------------------------'
  print 'Layouts in the web monitor:'
  print '---------------------------'
  layout_loader = loader.LayoutLoader()
  module_names = layout_loader.ModuleNames()
  for module_name in sorted(module_names):
    print '{:>16}:     {:60}'.format(
        module_name, layout_loader.GetLayoutByModuleName(module_name).Name())


if __name__ == '__main__':
  main()
