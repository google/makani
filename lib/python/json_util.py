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

"""Utilities to read/process JSON."""

import json
import numpy


class JsonNumpyEncoder(json.JSONEncoder):
  """A JSON encoder that's compatible with NumPy arrays."""

  def default(self, obj):
    if isinstance(obj, numpy.integer):
      return int(obj)
    elif isinstance(obj, numpy.floating):
      return float(obj)
    elif isinstance(obj, numpy.bool_):
      return bool(obj)
    elif isinstance(obj, numpy.ndarray):
      return obj.tolist()
    else:
      return super(JsonNumpyEncoder, self).default(obj)

