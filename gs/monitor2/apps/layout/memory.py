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

"""Utilities to store local memories."""

import expiringdict
from makani.gs.monitor2.project import settings

# A dictionary of local memories for each tab.
_memories = expiringdict.ExpiringDict(
    max_len=settings.MAX_CLIENT_COUNT,
    max_age_seconds=settings.MEMORY_STALE_TIMEOUT_S)


def GetMemory(client_id, create_if_none):
  if client_id in _memories:
    return _memories[client_id]
  elif create_if_none:
    _memories[client_id] = {}
    return _memories[client_id]
  else:
    return None
