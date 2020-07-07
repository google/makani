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

"""Urls to monitor pages."""
from django.conf import urls

urlpatterns = urls.patterns(
    # Receive commands to execute locally. Currently, only 'ls' is allowed.
    'makani.gs.monitor2.apps.dashboard.views',
    urls.url(r'^console/(?P<command>[\w_\-]+)/(?P<args>.+)$', 'Console',
             name='console'),
    # Select all logs.
    urls.url(r'^select_logs/$', 'SelectAllLogs', name='select_logs'),
    # User defined layouts.
    urls.url(r'^aio/layout/(?P<layout_name>[\w\s\(\)\[\]]+)$', 'ViewAioLayout',
             name='view_aio_layout'),
    # Data poll from a layout.
    urls.url(r'^periodic/layout/(?P<client_id>\w+)/'
             r'(?P<layout_name>[\w\s\(\)\[\]]+)$',
             'PeriodicDataPoll',
             name='periodic_layout_sync'),
    # Browse log, triggered when a node is clicked, returns child nodes.
    urls.url(r'^log/browse/(?P<path>.*)$',
             'BrowseLog',
             name='browse_log'),
    # Show the page that visualizes the log structure.
    urls.url(r'^log/structure/(?P<paths>(.|\n)+)$',
             'ViewLogStructure',
             name='view_log_structure'),
    # Get values of the requested fields in the log.
    urls.url(
        r'^log/data/(?P<mode>\w+)/(?P<fields>(.|\n)+)$', 'GetLogData',
        name='get_log_data'),
    # Get raw values to download.
    urls.url(
        r'^log/raw/(?P<fields>(.|\n)+)$', 'GetRawLogData',
        name='get_raw_log_data'),
    # Auto generate layouts for a message type.
    urls.url(r'^update_message_options/(?P<client_id>\w+)',
             'UpdateMessageOptions',
             name='update_msg_options'),
    urls.url(r'^view_message_type/(?P<client_id>\w+)/(?P<message_type>\w+)/',
             'ViewMessageType',
             name='view_msg_type'),
    urls.url(r'^periodic/msg_enum/(?P<client_id>\w+)/(?P<message_type>\w+)$',
             'PeriodicMessagePoll',
             name='periodic_message_type'),
    # Get message snapshot.
    urls.url(
        r'^debug/snapshot/(?P<client_id>\w+)/(?P<title>.+)$',
        'GetMessageSnapshot', name='get_message_snapshot'),
)
