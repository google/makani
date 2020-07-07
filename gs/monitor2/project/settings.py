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

"""Django settings for the project."""

import logging
import os

import makani
from makani.avionics.network import network_config
from makani.config import mconfig


# Makani configurations.

mconfig.WING_MODEL = 'm600'
MONITOR_PATH = os.path.join(makani.HOME, 'gs/monitor2')

# TODO: Figure out how to use the config system with overrides in the
# webmonitor, so this can be specified in MonitorParams. Getting this right is
# tricky because overrides need to propagate to all of the wrapped C/C++
# libraries.
POPULATE_MESSAGES_FROM_SIM = (
    os.environ['MAKANI_WEBMON_FROM_SIM'] == 'True'
    if 'MAKANI_WEBMON_FROM_SIM' in os.environ else False)


# A third-party CSS/JS library divides a webpage into logical a grid, where we
# can put views and widgets inside a block of grid cells.
# We use a setup that has 12 columns.
CSS_GRID_COLUMNS = 12

# Number of bits used for AIO message sequence.
AIO_MESSAGE_SEQUENCE_BITS = 16
# Path to the network configuration file.
NETWORK_YAML = os.path.join(makani.HOME, 'avionics/network/network.yaml')
NETWORK_CONFIGS = network_config.NetworkConfig(NETWORK_YAML)

# Number of seconds a layout's memory gets cached without further access.
# TODO: Develop a better way to handle stale data.
MEMORY_STALE_TIMEOUT_S = 36000
# Maximum number of clients.
MAX_CLIENT_COUNT = 100
# The max number of data points per line per plot.
MAX_DATA_POINTS_PER_LOG_FIELD = 6000

# Server configurations.
logging.basicConfig(level=logging.ERROR)

DEBUG = True

ALLOWED_HOSTS = ['*']
TEMPLATE_DEBUG = DEBUG

MONITOR_PATH = os.path.join(makani.HOME, 'gs/monitor2')

ADMINS = (
    # ('Your Name', 'your_email@example.com'),
)

MANAGERS = ADMINS

DATABASES = {
    'default': {
        # Add 'postgresql_psycopg2', 'mysql', 'sqlite3' or 'oracle'.
        'ENGINE': 'django.db.backends.sqlite3',
        # Path to database file if using sqlite3.
        'NAME': '/tmp/database',
        # Not used with sqlite3.
        'USER': '',
        # Not used with sqlite3.
        'PASSWORD': '',
        # Set to empty string for localhost. Not used with sqlite3.
        'HOST': '',
        # Set to empty string for default. Not used with sqlite3.
        'PORT': '',
    }
}

# Local time zone for this installation. Choices can be found here:
# http://en.wikipedia.org/wiki/List_of_tz_zones_by_name
# although not all choices may be available on all operating systems.
# On Unix systems, a value of None will cause Django to use the same
# timezone as the operating system.
# If running in a Windows environment this must be set to the same as your
# system time zone.
TIME_ZONE = None

# Language code for this installation. All choices can be found here:
# http://www.i18nguy.com/unicode/language-identifiers.html
LANGUAGE_CODE = 'en-us'

SITE_ID = 1

# If you set this to False, Django will make some optimizations so as not
# to load the internationalization machinery.
USE_I18N = True

# If you set this to False, Django will not format dates, numbers and
# calendars according to the current locale.
USE_L10N = True

# If you set this to False, Django will not use timezone-aware datetimes.
USE_TZ = True

# Absolute filesystem path to the directory that will hold user-uploaded files.
# Example: "/home/media/media.lawrence.com/media/"
MEDIA_ROOT = ''

# URL that handles the media served from MEDIA_ROOT. Make sure to use a
# trailing slash.
# Examples: "http://media.lawrence.com/media/", "http://example.com/media/"
MEDIA_URL = ''

# Absolute path to the directory static files should be collected to.
# Don't put anything in this directory yourself; store your static files
# in apps' "static/" subdirectories and in STATICFILES_DIRS.
# Example: "/home/media/media.lawrence.com/static/"
STATIC_ROOT = os.path.join(MONITOR_PATH, 'assets', 'static')

# URL prefix for static files.
# Example: "http://media.lawrence.com/static/"
STATIC_URL = '/static/'

# Additional locations of static files
STATICFILES_DIRS = (
    # Put strings here, like "/home/html/static" or "C:/www/django/static".
    # Always use forward slashes, even on Windows.
    # Don't forget to use absolute paths, not relative paths.
    os.path.join(MONITOR_PATH, 'UI', 'static'),
    os.path.join(makani.HOME, 'third_party/javascript')
)

# List of finder classes that know how to find static files in
# various locations.
STATICFILES_FINDERS = (
    'django.contrib.staticfiles.finders.FileSystemFinder',
    'django.contrib.staticfiles.finders.AppDirectoriesFinder',
    # 'dajaxice.finders.DajaxiceFinder',
    # 'django.contrib.staticfiles.finders.DefaultStorageFinder',
)

# Make this unique, and don't share it with anybody.
SECRET_KEY = '+s6*yy3i$1qg@c2b8g-jld-)9y!7o4hr6c*4-rsz9thr)y^l*#'

# List of callables that know how to import templates from various sources.
TEMPLATE_LOADERS = (
    'django.template.loaders.filesystem.Loader',
    'django.template.loaders.app_directories.Loader',
    # 'django.template.loaders.eggs.Loader',
)

MIDDLEWARE_CLASSES = (
    'django.middleware.common.CommonMiddleware',
    'django.contrib.sessions.middleware.SessionMiddleware',
    'django.middleware.csrf.CsrfViewMiddleware',
    'django.contrib.auth.middleware.AuthenticationMiddleware',
    'django.contrib.messages.middleware.MessageMiddleware',
    # Uncomment the next line for simple clickjacking protection:
    # 'django.middleware.clickjacking.XFrameOptionsMiddleware',
)

SESSION_SERIALIZER = 'django.contrib.sessions.serializers.PickleSerializer'

ROOT_URLCONF = 'makani.gs.monitor2.project.urls'

# Python dotted path to the WSGI application used by Django's runserver.
WSGI_APPLICATION = 'makani.gs.monitor2.project.wsgi.application'

# Absolute paths to HTML template files.
TEMPLATE_DIRS = (
    os.path.join(MONITOR_PATH, 'UI', 'templates'),
)

PROJECT_APPS = (
    'apps.dashboard',
    'apps.receiver',
    'apps.plugins',
)

INSTALLED_APPS = PROJECT_APPS + (
    'django.contrib.auth',
    'django.contrib.contenttypes',
    'django.contrib.sessions',
    'django.contrib.sites',
    'django.contrib.messages',
    'django.contrib.staticfiles',
    # Uncomment the next line to enable the admin:
    'django.contrib.admin',
    # Uncomment the next line to enable admin documentation:
    'django.contrib.admindocs',
    'django_extensions',
)

# A sample logging configuration. The only tangible logging
# performed by this configuration is to send an email to
# the site admins on every HTTP 500 error when DEBUG=False.
# See http://docs.djangoproject.com/en/dev/topics/logging for
# more details on how to customize your logging configuration.
LOGGING = {
    'version': 1,
    'disable_existing_loggers': False,
    'filters': {
        'require_debug_false': {
            '()': 'django.utils.log.RequireDebugFalse'
        }
    },
    'handlers': {
        'mail_admins': {
            'level': 'ERROR',
            'filters': ['require_debug_false'],
            'class': 'django.utils.log.AdminEmailHandler'
        }
    },
    'loggers': {
        'django.request': {
            'handlers': ['mail_admins'],
            'level': 'ERROR',
            'propagate': True,
        },
    }
}
