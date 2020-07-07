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

"""Constant values used by the Google Client API.

The API relies on a whole bunch of strings, the documentation of which is
fragmented.  This module reduces the need to clutter other code with string
literals and serves as a central reference point.
"""

API_VERSION = 'v1'

GCE_API_URL = 'https://www.googleapis.com/compute/%s/projects' % API_VERSION

# Used by GCE instances to obtain access tokens for OAuth2 authentication.
# In general, the URI is of the form <metadata_server>/<service_account>/token.
GCE_TOKEN_URI = ('http://metadata/computeMetadata/v1/instance/service-accounts'
                 '/default/token')

# OAuth2 uses authorization scopes to control access to resources.  The list
# below is only a small subset of all scopes.
#
# See:
#   https://developers.google.com/compute/docs/api/how-tos/authorization
_SCOPE_NAMES = [
    'compute',
    'compute.readonly',
    'devstorage.full_control',
    'devstorage.read_only',
    'devstorage.read_write',
    'devstorage.write_only',
    'gerritcodereview',
]

SCOPES = {n: 'https://www.googleapis.com/auth/' + n for n in _SCOPE_NAMES}

# See https://developers.google.com/compute/docs/zones.
ZONES = [
    'asia-east1-a',
    'asia-east1-b',
    'asia-east1-c',
    'europe-west1-a',
    'europe-west1-b',
    'us-central1-a',
    'us-central1-b',
    'us-central1-f',
]

# See https://developers.google.com/compute/docs/machine-types.
MACHINE_TYPES = [
    'f1-micro',
    'g1-small',
    'n1-highcpu-2',
    'n1-highcpu-4',
    'n1-highcpu-8',
    'n1-highcpu-16',
    'n1-highmem-2',
    'n1-highmem-4',
    'n1-highmem-8',
    'n1-highmem-16',
    'n1-standard-1',
    'n1-standard-2',
    'n1-standard-4',
    'n1-standard-8',
    'n1-standard-16',
]

# Client ID for OAuth2.  This identifies locally-stored credentials. Each client
# ID requires separate approval.
OAUTH2_CLIENT_ID = 'makani-app'

# Some services require a user agent.  This is strictly for identification and,
# AFAICT, we can choose anything we like.
USER_AGENT = 'makani-user-agent/0.1'
