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

"""Utilities for selecting GPS indicators."""

from makani.avionics.network import aio_node
from makani.lib.python import c_helpers
from makani.system import labels as system_labels
from makani.system import labels_util

_WING_GPS_HELPER = c_helpers.EnumHelper(
    'WingGpsReceiverLabel', system_labels, prefix='kWingGpsReceiver')
_AIO_NODE_HELPER = c_helpers.EnumHelper('AioNode', aio_node)


def GpsSelector():
  for gps_label in _WING_GPS_HELPER.Values():
    gps_type = labels_util.WingGpsReceiverLabelToGpsReceiverType(gps_label)
    fc_name = _AIO_NODE_HELPER.ShortName(
        labels_util.WingGpsReceiverLabelToAioNode(gps_label))
    yield gps_type, fc_name
