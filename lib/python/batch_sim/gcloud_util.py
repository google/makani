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

"""Utilities for working with Google Cloud Platform.

This module creates an internal API with which to access Makani's resources on
Google Cloud Platform.  OAuth2 authentication should be hidden behind these APIs
to the greatest extent possible
"""

import argparse
import httplib
import json
import logging
import os
import random
import socket
import sys
import textwrap
import time
import urllib2

import gflags
from googleapiclient import discovery as gapi_discovery
from googleapiclient import errors as gapi_errors
from googleapiclient import http as gapi_http
import httplib2
from makani.lib.python.batch_sim import gcloud_constants
from oauth2client import client as oauth2_client
from oauth2client import tools as oauth2_tools
from oauth2client.contrib import multiprocess_file_storage


gflags.DEFINE_string('gcloud_credentials_dir',
                     os.path.join(os.environ.get('HOME', ''), '.makani.d'),
                     'If using client secrets to authenticate, this directory '
                     'must hold a suitable client_secrets.json file, and it '
                     'will be used to cache credentials once they are '
                     'obtained.')

# TODO: Find out if there's a way to auto-detect whether we're running
# on GCE, so this choice can be automated.
gflags.DEFINE_enum('gcloud_authentication_method', 'client_secrets',
                   ['client_secrets', 'service_account'],
                   'Method with which to authenticate.')

gflags.DEFINE_integer('gcloud_num_retries', 5,
                      'Number of times we retry executing an HTTP request.')

FLAGS = gflags.FLAGS

_OAUTH2_FLAGS = None


def _GetOAuth2Flags():
  if not _OAUTH2_FLAGS:
    raise _AuthenticatorError('OAuth2 flags are not populated. Please '
                              'call InitializeFlagsWithOAuth2(argv) first.')
  return _OAUTH2_FLAGS


def GetAccessTokenContent():
  """Returns an access token from the metadata server.

  The function obtains an access token and it's expiration time fro the GCE
  metadata server. If the request is unsuccessful or if this is called outside
  of GCE, the function returns None. If the request is successful, is returns a
  dict with the following entries:

  - access_token: The access token.
  - expires_in: Number of seconds before the token expires.
  """
  request = urllib2.Request(gcloud_constants.GCE_TOKEN_URI)
  request.add_header('X-Google-Metadata-Request', 'True')

  try:
    response = urllib2.urlopen(request)
  except urllib2.HTTPError as e:
    logging.error('Request for access token was unsuccessful: %s', e)
    return None

  content = json.loads(response.read())
  return content


def GcsPath(*path_components):
  """Implement the path joining function for Cloud Storage paths.

  The function assumes linux-style path syntax. Example:
  GcsPath(*['gcs://bucket', 'file', 'path']) => 'gcs://bucket/file/path'

  Args:
    *path_components: Path components to be joined.

  Returns:
    A string for the merged path.
  """
  return os.path.join(*path_components)


def GcsDirname(path):
  """Get the path to the directory of a Cloud Storage path.

  Example:
      GcsDirname('gcs://bucket/dir/file') => 'gcs://bucket/dir'

  Args:
    path: A Cloud Storage path.

  Returns:
    Path to the Cloud Storage directory containing `path`.
  """
  return os.path.dirname(path)


def GcsBasename(path):
  """Get the base path of a Cloud Storage path.

  Example:
      GcsBasename('gcs://bucket/dir/file.ext') => 'file.ext'

  Args:
    path: A Cloud Storage path.

  Returns:
    Path without the directory part.
  """
  return os.path.basename(path)


def InitializeFlagsWithOAuth2(argv):
  """Initialze flags when OAuth2 is in use.

  This is meant to be called from main() instead of the usual gflags parsing
  logic.  It uses the argparse.ArgumentParser defined by oauth2client.tools to
  parse flags out of argv and store them in this module.  All remaining flags
  are parsed by gflags.

  Unfortunately, the -h/--help can only display help for one of the sets of
  flags, which should naturally be gflags in our context.  If this proves to be
  a significant problem, we can look for a way to expose both.

  Args:
    argv: sys.argv or similar.

  Returns:
    <list> argv with parsed flags removed.
  """

  # Parse OAuth2 flags using argparse, and store them in _OAUTH2_FLAGS.
  global _OAUTH2_FLAGS
  parser = argparse.ArgumentParser(
      description=__doc__,
      add_help=False,  # Leave -h/--help to gflags.
      formatter_class=argparse.RawDescriptionHelpFormatter,
      parents=[oauth2_tools.argparser])
  _OAUTH2_FLAGS, remaining_argv = parser.parse_known_args(argv[1:])

  # Ick.  argparse removes the executable name from argv, while gflags expects
  # it to be left in.
  remaining_argv.insert(0, argv[0])

  # Parse remaining flags with gflags.
  try:
    remaining_argv = FLAGS(remaining_argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, argv[0], FLAGS)
    sys.exit(1)

  return remaining_argv


class _AuthorizableHttp(object):
  """Wraps httplib2.Http so we can record when it is authorized.

  Attributes:
    http: <httplib2.Http>
    refresh_number: Refresh number of the authenticator that last updated this
        authorized this object.  None if never authorized.
  """

  def __init__(self, http=None, refresh_number=None):
    self.Reset(http, refresh_number)

  def Reset(self, http, refresh_number):
    self.http = http
    self.refresh_number = refresh_number


class _AuthenticatorError(Exception):
  pass


class _Authenticator(object):
  """Base class for an object that generates OAuth2 credentials."""

  def __init__(self, scope):
    """Initializes the _Authenticator.

    Args:
      scope: String or iterable of strings; authorization scopes requested.
    """
    self._scope = scope
    self._refresh_number = 0

  def _Refresh(self):
    """If necessary, refreshes credentials and increments self._refresh_number.

    Returns:
      Valid OAuth2 credentials.
    """
    raise NotImplementedError

  def Authorize(self, auth_http):
    """Ensures that auth_http is authorized.

    If auth_http.refresh_number matches our refresh number, then authorization
    is not necessary.

    Args:
      auth_http: <_AuthorizableHttp>

    Returns:
      True, if authorization was performed.  False, if none was needed.

    Raises:
      _AuthenticatorError: A request to the metadata server failed.
    """

    credentials = self._Refresh()
    if auth_http.refresh_number != self._refresh_number:
      auth_http.Reset(credentials.authorize(httplib2.Http()),
                      self._refresh_number)
      return True
    else:
      return False


class _ClientSecretsAuthenticator(_Authenticator):
  """Generates OAuth2 credentials from a client secrets file.

  This is the authenticator to use when running scripts as a live human being.
  Credentials are cached in a local file.

  If credentials have not been previously cached, then GetCredentials() will
  open a browser window to authorize the default service account to act with
  privileges determined by the authenticator's scope.
  """

  def __init__(self, scope):
    super(_ClientSecretsAuthenticator, self).__init__(scope)

    self._client_secrets_file = os.path.join(
        FLAGS.gcloud_credentials_dir, 'client_secrets.json')

    # Ensure that the client secrets file exists.  This also verifies that that
    # the credentials directory exists.
    if not os.path.isfile(self._client_secrets_file):
      raise _AuthenticatorError(self._HelpMessage())

    # Name of file used to cache credentials locally.
    self._credentials_filename = os.path.join(
        FLAGS.gcloud_credentials_dir, 'google_cloud_credentials.dat')

    # The second argument is a "client_id" that is only used for keying
    # credentials in the multistore.
    key = '{}-{}-{}'.format(gcloud_constants.OAUTH2_CLIENT_ID,
                            gcloud_constants.USER_AGENT,
                            self._scope)
    self._storage = multiprocess_file_storage.MultiprocessFileStorage(
        self._credentials_filename, key)

    self._flow = oauth2_client.flow_from_clientsecrets(
        self._client_secrets_file, self._scope, message=self._HelpMessage())

  def _HelpMessage(self):
    return textwrap.dedent(
        """
        Authentication depends on a client secrets file.  You can download this
        file from
            https://console.developers.google.com/project/apps~google.com:makani/apiui/credential
        Find an entry for "Client ID for native application", click "Download
        JSON", and move the downloaded file to %s."""
        % self._client_secrets_file)

  def _Refresh(self):
    credentials = self._storage.get()
    if credentials is None or credentials.invalid:
      credentials = oauth2_tools.run_flow(self._flow, self._storage,
                                          _GetOAuth2Flags())
      self._refresh_number += 1

    return credentials


class _ServiceAccountAuthenticator(_Authenticator):
  """Generates OAuth2 credentials for a service account.

  This is the authenticator to use when running scripts on Compute Engine.
  """

  # Request a new access token from the metadata server when our token is within
  # this many seconds of expiration.  This gives us wiggle room in our time
  # calculations, which naturally can't be 100% sharp.
  #
  # As of writing, the metadata server itself will create a new token when the
  # previous one is within 5 minutes of expiration, so that's the natural value
  # to choose.
  #
  # Note that the metadata server only provides an access token, which is used
  # by the `oauth2_client.AccessTokenCredentials` class to initialize a
  # credentials object. However, the credentials object does not contain a
  # refresh token that can be used by the `googleapiclient.http` library to
  # refresh the access token. Therefore, if an upload takes longer than an hour
  # to complete, it will fail. If we ever plan on supporting large file uploads,
  # we should refactor this class to use the
  # `oauth2client.service_account.ServiceAccountCredentials class` instead.
  _BUFFER_SECONDS = 5 * 60

  def __init__(self, scope):
    super(_ServiceAccountAuthenticator, self).__init__(scope)

    self._access_token = None
    self._credentials = None
    self._expiration_seconds_utc = 0.0

  def _Refresh(self):
    # If our access token is close to expiring, request a new one and generate
    # new credentials.
    if self._expiration_seconds_utc <= time.time() + self._BUFFER_SECONDS:
      content = GetAccessTokenContent()
      if not content:
        raise _AuthenticatorError('Request for access token was unsuccessful')
      self._expiration_seconds_utc = time.time() + content['expires_in']
      self._access_token = content['access_token']
      self._credentials = oauth2_client.AccessTokenCredentials(
          self._access_token, gcloud_constants.USER_AGENT)
      self._refresh_number += 1

    return self._credentials


def _GetAuthenticator(scope):
  """Returns an _Authenticator based on --gcloud_authentication_method."""

  if FLAGS.gcloud_authentication_method == 'client_secrets':
    return _ClientSecretsAuthenticator(scope)
  elif FLAGS.gcloud_authentication_method == 'service_account':
    return _ServiceAccountAuthenticator(scope)
  else:
    raise _AuthenticatorError(
        'Invalid value "%s" for --gcloud_authentication_method.  Why wasn''t '
        'this properly validated?' % FLAGS.gcloud_authentication_method)


class CloudApi(object):
  """Base class for APIs to Google Cloud services.

  This class assists with maintaining a service object from the Google Client
  API with up-to-date OAuth2 credentials.
  """

  def __init__(self, service_name, scope_id, authenticator=None):
    self._service_name = service_name
    self._authenticator = authenticator
    if self._authenticator is None:
      self._authenticator = _GetAuthenticator(gcloud_constants.SCOPES[scope_id])
    self._auth_http = _AuthorizableHttp()

    # Cached service object to be accessed by the _service property.
    self.__service = None

  @property
  def _service(self):
    if self._authenticator.Authorize(self._auth_http) or not self.__service:
      # The Authorize method returns True if the authenticator gets refreshed.
      # We instantiate a new service object if either the authenticator has been
      # refreshed or if the service object has not been initialized.
      self.__service = gapi_discovery.build(self._service_name,
                                            gcloud_constants.API_VERSION,
                                            http=self._auth_http.http,
                                            cache_discovery=False)
    return self.__service

  def _CommonErrorMatcher(self, error):
    """Determine if an error might be fixed by retrying a request."""
    if isinstance(error, httplib.BadStatusLine):
      return True  # Bad status from server.
    if isinstance(error, socket.error):
      return True  # "Connection reset by peer"
    if isinstance(error, gapi_errors.HttpError):
      if error.resp:
        if (error.resp.status in (408,  # Broken connection.
                                  410)  # Backend error.
            or error.resp.status >= 500):  # Server-side errors.
          return True
    return False

  def _RunWithRetries(self, callback, error_matcher):
    """Execute an HTTP request with retries.

    Methods such as googleapiclient.http.HttpRequest.execute support retries for
    status codes in the 500s.  However, they don't catch BadStatusLine
    exceptions, which may occur if a server closes a connection before sending a
    valid response.  Nor do they retry 404s, which Google Cloud services
    produce intermittently.

    This method wraps such calls, staying as consistent as possible with the
    existing retry logic.

    Args:
      callback: Callback to execute.
      error_matcher: Function that takes an exception generated by the callback
          and returns True if the callback should be retried.

    Returns:
      Response to successfully-executed method.
    """
    for i in xrange(FLAGS.gcloud_num_retries):
      try:
        return callback()
      except Exception as e:  # pylint: disable=broad-except
        if not error_matcher(e):
          raise
        # Use randomized exponential backoff, like methods in
        # googleapiclient.http.
        retry_seconds = random.random() * 2**(i + 1)
        logging.warning('Request raised an error: %s\n'
                        'Will retry in %f seconds.', e, retry_seconds)
        time.sleep(retry_seconds)

    return callback()


class CloudStorageApiError(Exception):
  pass


# Sample usage: Uploading a local file testdata.txt to
# gs://makani/misc/testdata.txt, then dowloading that same file and viewing the
# contents:
#   import io
#   import sys
#
#   from makani.lib.python.batch_sim import gcloud_util
#
#   gcloud_util.InitializeFlagsWithOAuth2(sys.argv)
#   bucket = 'makani'
#   gcs = gcloud_util.CloudStorageApi(bucket)
#   open('testdata.txt', 'w').write('THIS IS A TEST')
#   gcs.UploadFile('testdata.txt', 'misc/testdata.txt')
#
#   stream = io.BytesIO()
#   gcs.DownloadFile('misc/testdata.txt', stream)
#   print stream.getvalue()
class CloudStorageApi(CloudApi):
  """Interface to a bucket in Google Cloud Storage (GCS)."""

  def __init__(self, bucket=None, **kwargs):
    """Initialize the CloudStorageApi.

    Args:
      bucket: Default name of the bucket on Cloud Storage.
              If specified, file names supplied to the operations can just
              contain the file path in that bucket.
              If None, file names supplied to the operations must contain
              the bucket name (e.g., gs://bucket_name/file_path).
      **kwargs: A dictionary of arguments needed by CloudApi.
    """
    self._bucket = bucket
    super(CloudStorageApi, self).__init__('storage', 'devstorage.read_write',
                                          **kwargs)

  def DownloadFile(self, gcs_file_name, io_base):
    """Download a file from GCS.

    Args:
      gcs_file_name: Name of the file on GCS.
      io_base: Any <io.IOBase> object to store the downloaded contents.

    Raises:
      CloudStorageApiError: If object metadata is malformed.
    """
    bucket, bucket_path = self._ParseBucketAndPath(gcs_file_name)

    # Check the size of the remote file. If it's empty, we have to return early
    # because the chunked downloader will crash. There aren't any contents to
    # retrieve in that case, anyway.
    object_data = self._RunWithRetries(
        self._service.objects().get(bucket=bucket, object=bucket_path).execute,
        self._CommonErrorMatcher)
    if ('name' not in object_data or object_data['name'] != bucket_path
        or 'size' not in object_data):
      raise CloudStorageApiError('Object data for %s is malformed.' %
                                 GcsPath(bucket, bucket_path))
    if int(object_data['size']) == 0:
      return

    request = self._service.objects().get_media(bucket=bucket,
                                                object=bucket_path)
    downloader = gapi_http.MediaIoBaseDownload(
        io_base, request, chunksize=1024*1024)
    done = False
    while not done:
      # The first return value indicates download progress, which we won't do
      # anything fancy with for now.
      _, done = self._RunWithRetries(downloader.next_chunk,
                                     self._CommonErrorMatcher)

  def _UploadWithProgressInternal(self, media, gcs_file_name):
    """Uploads an object while logging progress."""
    bucket, bucket_path = self._ParseBucketAndPath(gcs_file_name)
    request = self._service.objects().insert(bucket=bucket,
                                             media_body=media,
                                             name=bucket_path)
    if media._size == 0:  # pylint: disable=protected-access
      return self._RunWithRetries(request.execute, self._CommonErrorMatcher)

    response = None
    logged_percent_done = 0
    while response is None:
      status, response = self._RunWithRetries(request.next_chunk,
                                              self._CommonErrorMatcher)
      if status:
        percent_done = int(status.progress() * 100)
        if percent_done - logged_percent_done >= 5:
          logging.info('Uploading to gs://%s/%s: %d%% complete.',
                       bucket,
                       bucket_path,
                       int(status.progress() * 100))
          logged_percent_done = percent_done
    return response

  def _UploadWithProgress(self, *args):
    """Retry-wrapper of _UploadWithProgressInternal."""

    def ErrorMatcher(error):
      """Match errors for _UploadWithProgress."""

      # Retry all common errors.
      if self._CommonErrorMatcher(error):
        return True

      # Retry when we encounter a KeyError on the key 'range'. This appears to
      # be due to a bug in googleapiclient version 1.4.1.
      if isinstance(error, KeyError) and error.message == 'range':
        return True

      # A 400 indicates a malformed request. If it's our fault, the request will
      # continue to be malformed, and we'll find out soon enough. However,
      # this error has also occurred partway through an upload (see b/32096351),
      # in which case the bad request may have been populated by data from the
      # remote service.
      if isinstance(error, gapi_errors.HttpError) and error.resp.status == 400:
        return True

      return False

    return self._RunWithRetries(
        lambda: self._UploadWithProgressInternal(*args), ErrorMatcher)

  def _ParseBucketAndPath(self, gcs_path):
    try:
      return ParseBucketAndPath(gcs_path, self._bucket)
    except ValueError, e:
      raise CloudStorageApiError(e.message)

  def UploadFile(self, local_file_name, gcs_file_name,
                 mimetype='application/octet-stream'):
    """Upload a file to GCS.

    Args:
      local_file_name: Path to the file on the client's machine.
      gcs_file_name: Name with which to store the file on GCS.
                     It can be a full path (e.g., gs://makani/file), or a path
                     relative to the preset default bucket.
      mimetype: MIME type of the object.

    Returns:
      Response to the Google Client API's download request.
    """
    resumable = os.stat(local_file_name).st_size > 0
    media = gapi_http.MediaFileUpload(local_file_name,
                                      mimetype=mimetype,
                                      resumable=resumable)

    # gsutil's code suggests that 404s and 410s are retryable for resumable
    # uploads (see ResumableUploadStartOverException).
    def _ErrorMatcher(error):
      return (self._CommonErrorMatcher(error)
              or (isinstance(error, gapi_errors.HttpError)
                  and error.resp.status in (404, 410)))

    return self._RunWithRetries(
        lambda: self._UploadWithProgress(media, gcs_file_name),
        _ErrorMatcher)

  def UploadStream(self, io_base, gcs_file_name,
                   mimetype='application/octet-stream'):
    # Only make the request resumable if the stream is nonempty.
    position = io_base.tell()
    resumable = bool(io_base.readline(1))
    io_base.seek(position)
    media = gapi_http.MediaIoBaseUpload(io_base, mimetype, chunksize=1024*1024,
                                        resumable=resumable)
    return self._UploadWithProgress(media, gcs_file_name)

  def DeleteFile(self, gcs_file_name):
    bucket, bucket_path = self._ParseBucketAndPath(gcs_file_name)
    request = self._service.objects().delete(bucket=bucket,
                                             object=bucket_path)
    return self._RunWithRetries(request.execute, self._CommonErrorMatcher)

  def _DeletePrefixInternal(self, prefix):
    """Corresponds to a single attempt of DeletePrefix."""
    def _HandleResponse(unused_request_id, unused_response, exception):
      """Handler with format dictated by BatchHttpRequest."""
      if exception is not None:
        raise exception

    bucket, _ = self._ParseBucketAndPath(prefix)
    # The file names are returned as paths relative to the bucket.
    file_names = self.List(prefix)

    # TODO(b/123649389): With a request over 1000 objects we have started
    # getting a BatchError.  This seems to be new behavior and is undocumented.
    # Working around it for now.
    max_batch_size = 1000
    if file_names:
      for i in range(0, len(file_names), max_batch_size):
        batch = self._service.new_batch_http_request()
        for name in file_names[i:min(i + max_batch_size, len(file_names))]:
          batch.add(
              self._service.objects().delete(bucket=bucket, object=name),
              callback=_HandleResponse)
        batch.execute()

  def DeletePrefix(self, prefix):
    """Delete all files with a certain prefix."""
    # We'll retry both the List and the batch deletion in the event of an
    # error. This lets us catch the "GCS says files are there when they really
    # aren't" problem.
    self._RunWithRetries(lambda: self._DeletePrefixInternal(prefix),
                         self._CommonErrorMatcher)

  def List(self, prefix=''):
    """List bucket contents with optional prefix.

    Args:
      prefix: Prefix for paths of listed files.

    Returns:
      List of file names.
    """

    bucket, bucket_path = self._ParseBucketAndPath(prefix)
    names = []
    request = self._service.objects().list(bucket=bucket, prefix=bucket_path)
    response = self._RunWithRetries(request.execute, self._CommonErrorMatcher)

    while response:
      if 'items' in response:
        names += [item['name'] for item in response['items']]

      if 'nextPageToken' in response:
        request = self._service.objects().list(
            bucket=bucket, prefix=bucket_path,
            pageToken=response['nextPageToken'])
        response = self._RunWithRetries(request.execute,
                                        self._CommonErrorMatcher)
      else:
        response = None

    return names


class ComputeEngineApiError(Exception):
  pass


# Sample usage: Create and then delete an instance, printing responses for each
# completed operation.  (Try to avoid leaving instances running unintentionally
# for long periods, as they incur charges.)
#   import sys
#
#   from makani.lib.python.batch_sim import gcloud_util
#
#   gcloud_util.InitializeFlagsWithOAuth2(sys.argv)
#   gce = gcloud_util.ComputeEngineApi()
#
#   instance_name = 'internal-api-test'
#
#   response = gce.CreateInstance(instance_name, 'ubuntu-trusty-server')
#   response = gce.WaitForCompletion(response)
#   gce.CheckSuccess(response)
#   print response
#
#   response = gce.DeleteInstance(instance_name)
#   response = gce.WaitForCompletion(response)
#   gce.CheckSuccess(response)
#   print response
class ComputeEngineApi(CloudApi):
  """Interface to Google Compute Engine.

  This is specialized to the project 'google.com:makani', but it could easily be
  generalized if needed.
  """

  _PROJECT_ID = 'google.com:makani'
  _PROJECT_URL = '%s/%s' % (gcloud_constants.GCE_API_URL, _PROJECT_ID)

  _DEFAULT_ZONE = 'us-central1-f'
  _DEFAULT_MACHINE_TYPE = 'n1-standard-1'
  _DEFAULT_NETWORK = 'default'

  _DEFAULT_SCOPES = (gcloud_constants.SCOPES['compute'],
                     gcloud_constants.SCOPES['devstorage.read_write'])

  _SERVICE_EMAIL = 'default'

  def __init__(self, **kwargs):
    super(ComputeEngineApi, self).__init__('compute', 'compute', **kwargs)

  def CreateInstance(self, instance_name, image_name,
                     image_project=_PROJECT_ID,
                     machine_type=_DEFAULT_MACHINE_TYPE,
                     metadata=None,
                     network=_DEFAULT_NETWORK,
                     scopes=_DEFAULT_SCOPES,
                     use_ssd=False,
                     disk_size_gb=20.0,
                     zone=_DEFAULT_ZONE):
    """Creates a new VM instance.

    Args:
      instance_name: Name of the instance.
      image_name: Name of the disk image.
      image_project: Name of the project that owns the disk image.  (E.g., we
          might want to use one of the canned Debian instances owned by
          debian-cloud.)
      machine_type: Name of the machine type.
      metadata: Custom instance metadata.  A list of dictionaries of the form
          {'key': <key>, 'value': <value>}, or None for no metadata.
      network: Name of the network.
      scopes: Authorization scopes made available to the instance's service
          account.
      use_ssd: <bool>; indicates whether workers should use SSDs.
      disk_size_gb: <float> Size of the worker disk in GB.
      zone: Name of the zone in which the instance is created.

    Returns:
      <dict> Response to the initial insertion request, likely still with
          PENDING status.

    Raises:
      ComputeEngineApiError: The request failed because an instance with the
          same name exists.
    """

    # Before attempting to create the instance, verify that it does not exist.
    try:
      self.GetInstance(instance_name, zone=zone)
    except gapi_errors.HttpError as e:
      if e.resp and e.resp.status != 404:
        # If we get an unexpected error, raise it.
        raise e
      # If the error was 404, then we just exit this try-except-else and
      # continue.
    else:
      # If no error is thrown, that means that the instance exists, therefore
      # we raise an error.
      raise ComputeEngineApiError('The instance "%s" already exists.'
                                  % instance_name)

    zone_url = '%s/zones/%s' % (self._PROJECT_URL, zone)
    machine_type_url = '%s/machineTypes/%s' % (zone_url, machine_type)
    network_url = '%s/global/networks/%s' % (self._PROJECT_URL, network)
    image_url = '%s/%s/global/images/%s' % (gcloud_constants.GCE_API_URL,
                                            image_project, image_name)

    disk_type = 'pd-ssd' if use_ssd else 'pd-standard'
    disk_type_url = '%s/diskTypes/%s' % (zone_url, disk_type)

    if metadata is None:
      metadata = {}

    instance_descriptor = {
        'name': instance_name,
        'machineType': machine_type_url,
        'disks': [{
            'autoDelete': 'true',
            'boot': 'true',
            'type': 'PERSISTENT',
            'initializeParams': {
                'diskName': instance_name,
                'diskSizeGb': disk_size_gb,
                'diskType': disk_type_url,
                'sourceImage': image_url
            }
        }],
        'networkInterfaces': [{
            'accessConfigs': [{
                'type': 'ONE_TO_ONE_NAT',
                'name': 'External NAT'
            }],
            'network': network_url,
        }],
        'serviceAccounts': [{
            'email': self._SERVICE_EMAIL,
            'scopes': scopes
        }],
        'metadata': metadata,
    }

    request = self._service.instances().insert(
        project=self._PROJECT_ID, body=instance_descriptor, zone=zone)

    try:
      return self._RunWithRetries(request.execute, self._CommonErrorMatcher)
    except gapi_errors.HttpError as e:
      if e.resp and e.resp.status == 409:
        return self.GetInstance(instance_name, zone=zone)
      else:
        raise e

  def GetInstance(self, instance_name, zone=_DEFAULT_ZONE):
    request = self._service.instances().get(
        project=self._PROJECT_ID, instance=instance_name, zone=zone)
    return self._RunWithRetries(request.execute, self._CommonErrorMatcher)

  def ListInstances(self, zone=_DEFAULT_ZONE):
    request = self._service.instances().list(project=self._PROJECT_ID,
                                             zone=zone)
    response = self._RunWithRetries(request.execute, self._CommonErrorMatcher)
    if 'items' in response:
      return [item['name'] for item in response['items']]
    else:
      return []

  def DeleteInstance(self, instance_name, zone=_DEFAULT_ZONE):
    """Deletes the specified VM instance.

    Args:
      instance_name: Name of the instance.
      zone: Zone in which the instance lives.

    Returns:
      <dict> Response to the initial deletion request, likely still with PENDING
          status.
    """

    request = self._service.instances().delete(
        project=self._PROJECT_ID, instance=instance_name, zone=zone)
    return self._RunWithRetries(request.execute, self._CommonErrorMatcher)

  def WaitForCompletion(self, response,
                        wait_seconds=1,
                        timeout_seconds=300):
    """Waits until the operation status is 'DONE' for the given operation.

    This is accomplished by querying for operation status until the operation is
    complete.

    Args:
      response: <dict> Response corresponding to the operation.
      wait_seconds: Number of seconds between queries for operation status.
      timeout_seconds: Number of seconds after which to time out.

    Returns:
      <dict> Final response with status 'DONE'.  (This will be the initial
          response if its status was already 'DONE'.)

    Raises:
      ComputeEngineApiError: Timeout was reached before the operation was
          complete.
    """
    # This is adapted from the Python API tutorial:
    # https://developers.google.com/compute/docs/api/python-guide.

    start_time = time.time()

    status = response['status']
    while status != 'DONE' and response:
      time.sleep(wait_seconds)
      if time.time() > start_time + timeout_seconds:
        raise ComputeEngineApiError(
            'Operation failed to complete before timeout (%s seconds).  '
            'Last response:\n%s' % (timeout_seconds, response))
      operation_id = response['name']

      if 'zone' in response:
        zone_id = response['zone'].split('/')[-1]
        request = self._service.zoneOperations().get(project=self._PROJECT_ID,
                                                     operation=operation_id,
                                                     zone=zone_id)
      else:
        request = self._service.globalOperations().get(
            project=self._PROJECT_ID, operation=operation_id)

      response = self._RunWithRetries(request.execute, self._CommonErrorMatcher)
      if response:
        status = response['status']
    return response

  def CheckSuccess(self, response):
    """Checks that an operation completed successfully.

    Args:
      response: <dict> Response corresponding to the operation.

    Raises:
      ComputeEngineApiError: The response failed.
    """
    if 'error' in response:
      raise ComputeEngineApiError('An operation completed with errors:\n%s'
                                  % response)


def ParseBucketAndPath(gcs_path, default_bucket=None):
  """Parse a Cloud Storage path.

  If bucket name is missing, the default bucket name is used.

  Args:
    gcs_path: A path to a location on the Cloud Storage.
    default_bucket: The default bucket name.

  Returns:
    bucket: Name of the Cloud Storage bucket.
    path: Path within the bucket.

  Raises:
    ValueError: Raised when the bucket name is missing.
  """
  gcs_path = gcs_path.strip()
  gs_prefix = 'gs://'
  if default_bucket is not None and not gcs_path.startswith(gs_prefix):
    return default_bucket, gcs_path

  if gcs_path.startswith(gs_prefix):
    full_path = gcs_path[len(gs_prefix):]
    path_parts = full_path.split('/', 1)
    if len(path_parts) == 2 and path_parts[0]:
      bucket, path = path_parts
    else:
      raise ValueError('Cannot identify the bucket name in %s.'
                       % gcs_path)
  else:
    raise ValueError('Path "%s" ' % gcs_path
                     + 'requires a default bucket that is missing.')
  return bucket, path
