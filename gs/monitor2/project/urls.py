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

"""The root url patterns."""

from django.conf import urls
from django.contrib import admin

admin.autodiscover()

APPS_PATH = 'makani.gs.monitor2.apps'

urlpatterns = urls.patterns(
    '',
    urls.url(r'^$', '%s.dashboard.views.Home' % APPS_PATH, name='home'),
    urls.url(r'^dashboard/', urls.include('%s.dashboard.urls' % APPS_PATH)),
    urls.url(r'^receiver/', urls.include('%s.receiver.urls' % APPS_PATH)),
)
