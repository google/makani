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

"""Pack2 code generation engine."""

from makani.lib.python.pack2 import metadata


class Generator(object):
  """The pack2 code generation engine."""

  def __init__(self, backend):
    self.needed_types = []
    self.needed_type_names = set()
    self.backend = backend

  def _AddType(self, type_obj):
    if type_obj.__class__ == metadata.StructType:
      self.backend.AddStruct(type_obj)
    elif type_obj.__class__ == metadata.EnumType:
      self.backend.AddEnum(type_obj)
    elif type_obj.__class__ == metadata.BitfieldType:
      self.backend.AddBitfield(type_obj)
    elif type_obj.__class__ == metadata.ScaledType:
      self.backend.AddScaled(type_obj)
    elif type_obj.__class__ == metadata.Header:
      self.backend.AddHeader(type_obj)
    elif type_obj.__class__ == metadata.Param:
      self.backend.AddParam(type_obj)
    elif type_obj.__class__ == metadata.PrimitiveType:
      pass  # Primitive types handled natively by backend.
    else:
      raise NotImplementedError('Unknown type object %s.'
                                % type_obj.__class__.__name__)

  def Generate(self, meta):
    """Generate code.

    Generates code for objects in meta using the backend specified during
    construction.

    Args:
      meta: The Metadata object containing the information to process.
    """

    for include in sorted(meta.includes):
      self.backend.AddInclude(include)

    for t in meta.types:
      # Don't generate code for types included form other p2 files.
      if not t.included:
        self._AddType(t)

    self.backend.Finalize()

  def GetSourceString(self, name):
    return self.backend.GetSourceString(name)

  def WriteSourceFile(self, name, path):
    contents = self.GetSourceString(name)
    with open(path, 'w') as f:
      f.write(contents)
