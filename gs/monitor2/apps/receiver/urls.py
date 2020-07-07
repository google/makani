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

"""Urls to handle receivers."""

from django.conf import urls

urlpatterns = urls.patterns(
    'makani.gs.monitor2.apps.receiver.views',
    urls.url(r'^start_aio/(?P<client_id>\w+)$', 'StartAioReceiver',
             name='start_aio'),
    urls.url(r'^stop_aio/(?P<client_id>\w+)$', 'StopReceiver', name='stop_aio'),
    urls.url(r'^start_log/(?P<client_id>\w+)/(?P<message_type>\w+)/'
             r'(?P<source>\w+)/(?P<log_path>.*)$',
             'StartLogReceiver', name='start_log'),
    urls.url(r'^stop_log/(?P<client_id>\w+)$', 'StopReceiver', name='stop_log'),
)
