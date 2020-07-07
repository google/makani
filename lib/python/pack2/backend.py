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

"""Abstract base class describing the backend interface."""

import abc


class Backend(object):
  """Abstract base class describing the backend interface."""
  __metaclass__ = abc.ABCMeta

  @abc.abstractmethod
  def AddInclude(self, path):
    pass

  @abc.abstractmethod
  def AddBitfield(self, bitfield):
    pass

  @abc.abstractmethod
  def AddEnum(self, endum):
    pass

  @abc.abstractmethod
  def AddScaled(self, scaled):
    pass

  @abc.abstractmethod
  def AddStruct(self, struct):
    pass

  @abc.abstractmethod
  def AddHeader(self, config):
    pass

  @abc.abstractmethod
  def AddParam(self, param):
    pass

  @abc.abstractmethod
  def Finalize(self):
    pass

  @abc.abstractmethod
  def GetSourceString(self, name):
    pass

  @classmethod
  def __subclasshook__(cls, c):
    methods = ['AddInclude', 'AddBitfield', 'AddEnum', 'AddScaled', 'AddStruct',
               'AddHeader', 'AddParam', 'Finalize', 'GetSourceString']
    if cls is not Backend:
      return NotImplemented

    for method in methods:
      if not any(method in b.__dict__ for b in c.__mro__):
        return NotImplemented

    return True
