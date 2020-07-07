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

"""Utility to convert a CType object into a Python object."""

from __future__ import absolute_import
import ctypes


def Attributes(obj):
  return [x[0] for x in obj._fields_]  # pylint: disable=protected-access


def CTypeToPython(obj):
  """Convert a CType object to Python."""

  if isinstance(obj, ctypes.Structure):
    return {name: CTypeToPython(getattr(obj, name)) for name in Attributes(obj)
            if not name.startswith('PADDING_')}
  elif isinstance(obj, ctypes.Array):
    return [CTypeToPython(item) for item in obj]
  else:
    return obj


def TypeCast(obj, target_class):
  """Return obj in target_class."""
  return CastPointer(obj, target_class).contents


def CastPointer(obj, target_class):
  """Return pointer to the obj in target_class."""
  return ctypes.cast(ctypes.pointer(obj), ctypes.POINTER(target_class))


def SizelessArray(obj):
  """Return an array obj to sizeless array object (E.g. int a[8] to int a[])."""
  return TypeCast(obj, type(obj)._type_ * 0)  # pylint: disable=protected-access


def Vec3ToList(vec3):
  return [vec3.x, vec3.y, vec3.z]
