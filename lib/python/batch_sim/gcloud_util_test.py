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

"""Tests for the gcloud_util module.

These tests are somewhat regression flavored -- constructing them involves a lot
of manual inspection of the URLs involved and tweaking responses as needed to
satisfy the Google Client API.

The URIs are part of the official API specification (up to argument order, at
least; we can make the matching more robust later if needed).  Some of the other
details, particularly when it comes to resumable requests, are uncomfortably
close to implementation details of the Client API.  Hopefully they'll provide
some better testing infrastructure in the meantime, but for now, at least we've
got test coverage.
"""

import httplib
import io
import json
import os
import sys
import tempfile
import unittest

import gflags
from googleapiclient import errors as gapi_errors
import httplib2
from makani.lib.python import test_util
from makani.lib.python.batch_sim import gcloud_util
import mock

BUCKET = 'makani'


class HttpMockSequence(object):

  def __init__(self, responses):
    """Sequence of mock HTTP responses.

    This is a variant on googleapiclient.http.HttpMockSequence, which does not
    support URI validation.

    Args:
      responses: List of tuples of the form:
          (expected_uri, response_headers, response_body).
    """
    self._responses = responses

  # This function's signature must match httplib2.Http.request.
  # pylint: disable=invalid-name,unused-argument
  def request(self, uri,
              method='GET',
              body=None,
              headers=None,
              redirections=1,
              connection_type=None):
    assert self._responses, ('No response available for: '
                             'URL: %s\nHeaders: %s\nBody: %s'
                             % (uri, headers, body))

    expected_uri, response_headers, content = self._responses.pop(0)
    if isinstance(response_headers, Exception):
      raise response_headers

    expected_uri = 'https://www.googleapis.com/' + expected_uri
    assert uri == expected_uri, ('Actual URI does not match expected URI: '
                                 '%s vs. %s.' % (uri, expected_uri))
    return httplib2.Response(response_headers), content


class FakeAuthenticator(gcloud_util._Authenticator):
  """Performs "authentication".

  By which I mean "injection of an HttpMockSequence" with which to test
  expectations.
  """

  def __init__(self, http_sequence):
    self._http_sequence = http_sequence

  def Authorize(self, auth_http):
    if auth_http.refresh_number is None:
      auth_http.Reset(self._http_sequence, 1)
      return True
    else:
      return False


class StorageApiExceptionTest(unittest.TestCase):

  def GetService(self, http_sequence, bucket=None):
    gcs = gcloud_util.CloudStorageApi(
        bucket=bucket,
        authenticator=FakeAuthenticator(http_sequence))
    return gcs

  def testBadFilePaths(self):
    sequence = HttpMockSequence([])
    gcs = self.GetService(sequence, BUCKET)
    gcs_path = 'gs://'
    with self.assertRaises(gcloud_util.CloudStorageApiError):
      gcs.List(gcs_path)

    gcs_path = 'gs:///extra_slash'
    with self.assertRaises(gcloud_util.CloudStorageApiError):
      gcs.List(gcs_path)

    gcs = self.GetService(sequence)
    gcs_path = 'test/path'
    with self.assertRaises(gcloud_util.CloudStorageApiError):
      gcs.List(gcs_path)


class StorageApiAbsPathTest(unittest.TestCase):

  DISCOVERY = open(os.path.join(os.path.dirname(__file__),
                                'testdata/storage-discovery.json'),
                   'r').read()

  def Path(self, rel_path):
    """Format the remote file path.

    Args:
      rel_path: The file path relative to the bucket.

    Returns:
      The formated path.
    """
    return 'gs://%s/%s' % (BUCKET, rel_path)

  def GetService(self, http_sequence):
    gcs = gcloud_util.CloudStorageApi(
        bucket=None,
        authenticator=FakeAuthenticator(http_sequence))
    return gcs

  def testList(self):
    sequence = HttpMockSequence([
        ('discovery/v1/apis/storage/v1/rest',
         {'status': '200'},
         self.DISCOVERY),
        ('storage/v1/b/makani/o?prefix=misc&alt=json',
         {'status': '200'},
         json.dumps({'items': [{'name': 'a.txt'}, {'name': 'b.txt'}],
                     'nextPageToken': 'Token'})),
        ('storage/v1/b/makani/o?pageToken=Token&prefix=misc&alt=json',
         {'status': '200'},
         json.dumps({'items': [{'name': 'c.txt'}]})),
    ])
    gcs = self.GetService(sequence)
    names = gcs.List(self.Path('misc'))
    self.assertEqual(['a.txt', 'b.txt', 'c.txt'], names)

  def testDownloadFile(self):
    def GetSequence(file_contents):
      return  HttpMockSequence([
          ('discovery/v1/apis/storage/v1/rest',
           {'status': '200'},
           self.DISCOVERY),
          ('storage/v1/b/makani/o/primate.txt?alt=json',
           {'status': '200'},
           json.dumps({'name': 'primate.txt',
                       'size': str(len(file_contents))})),
          ('storage/v1/b/makani/o/primate.txt?alt=media',
           {'status': '200', 'content-range': 'foo/%d' % len(file_contents)},
           file_contents),
      ])

    # Empty and nonempty files are handled differently.
    for file_contents in ('', 'gibbon'):
      gcs = self.GetService(GetSequence(file_contents))
      stream = io.BytesIO()
      gcs.DownloadFile(self.Path('primate.txt'), stream)
      self.assertEqual(file_contents, stream.getvalue())

  def testUploadMethods(self):
    def GetSequence():
      return HttpMockSequence([
          ('discovery/v1/apis/storage/v1/rest',
           {'status': '200'},
           self.DISCOVERY),
          ('upload/storage/v1/b/makani/o?uploadType=resumable'
           '&alt=json&name=primate.txt',
           {'status': '200',
            'location': 'https://www.googleapis.com/fake_resumable_uri'},
           ''),
          ('fake_resumable_uri',
           {'status': '200', 'location': 'https://fake_resumable_uri'},
           '{}'),
      ])

    file_contents = 'marmoset'

    # Test UploadStream.
    gcs = self.GetService(GetSequence())
    stream = io.BytesIO(file_contents)
    gcs.UploadStream(stream, self.Path('primate.txt'))

    # Test UploadFile.
    gcs = self.GetService(GetSequence())
    with tempfile.NamedTemporaryFile() as local_file:
      local_file.write(file_contents)
      local_file.flush()
      gcs.UploadFile(local_file.name, self.Path('primate.txt'))

  def testRetryUploadOnKeyErrorRange(self):
    def GetSequence(error_key):
      return HttpMockSequence([
          ('discovery/v1/apis/storage/v1/rest',
           {'status': '200'},
           self.DISCOVERY),
          ('upload/storage/v1/b/makani/o?uploadType=resumable'
           '&alt=json&name=primate.txt',
           {'status': '200',
            'location': 'https://www.googleapis.com/fake_resumable_uri'},
           ''),
          ('placeholder_uri', KeyError(error_key), '{}'),
          ('upload/storage/v1/b/makani/o?uploadType=resumable'
           '&alt=json&name=primate.txt',
           {'status': '200',
            'location': 'https://www.googleapis.com/fake_resumable_uri'},
           ''),
          ('fake_resumable_uri',
           {'status': '200', 'location': 'https://fake_resumable_uri'},
           '{}'),
      ])

    file_contents = 'marmoset'

    # Retry on KeyError('range').
    gcs = self.GetService(GetSequence('range'))
    stream = io.BytesIO(file_contents)
    gcs.UploadStream(stream, self.Path('primate.txt'))

    # Don't retry on other KeyErrors.
    with self.assertRaises(KeyError):
      gcs = self.GetService(GetSequence('orangutan'))
      stream = io.BytesIO(file_contents)
      gcs.UploadStream(stream, self.Path('primate.txt'))

  def testUploadMethodsWithEmptyContents(self):
    def GetSequence():
      return HttpMockSequence([
          ('discovery/v1/apis/storage/v1/rest',
           {'status': '200'},
           self.DISCOVERY),
          ('upload/storage/v1/b/makani/o?uploadType=media'
           '&alt=json&name=empty_file.txt',
           {'status': '200'},
           '{}'),
      ])

    file_contents = ''

    # Test UploadStream.
    gcs = self.GetService(GetSequence())
    stream = io.BytesIO(file_contents)
    gcs.UploadStream(stream, self.Path('empty_file.txt'))

    # Test UploadFile.
    gcs = self.GetService(GetSequence())
    with tempfile.NamedTemporaryFile() as local_file:
      local_file.write(file_contents)
      local_file.flush()
      gcs.UploadFile(local_file.name, self.Path('empty_file.txt'))

  def testDeletePrefix(self):
    # Shamelessly stolen from:
    #     https://github.com/google/google-api-python-client/blob/master/tests/test_http.py  pylint: disable=line-too-long
    # This can't be left-flushed with textwrap.dedent due to the carriage
    # returns.
    batch_response = """--batch_foobarbaz
        Content-Type: application/http
        Content-Transfer-Encoding: binary
        Content-ID: <randomness + 1>

        HTTP/1.1 200 OK
        Content-Type: application/json
        Content-Length: 14
        ETag: "etag/pony"\r\n\r\n{"foo": 42}

        --batch_foobarbaz
        Content-Type: application/http
        Content-Transfer-Encoding: binary
        Content-ID: <randomness + 2>

        HTTP/1.1 200 OK
        Content-Type: application/json
        Content-Length: 14
        ETag: "etag/sheep"\r\n\r\n{"baz": "qux"}
        --batch_foobarbaz--""".replace(' ' * 8, '')

    sequence = HttpMockSequence([
        ('discovery/v1/apis/storage/v1/rest',
         {'status': '200'},
         self.DISCOVERY),
        ('storage/v1/b/makani/o?prefix=misc&alt=json',
         {'status': '200'},
         json.dumps(
             {'items': [{'name': 'misc/a.txt'}, {'name': 'misc/b.txt'}]})),
        ('batch',
         {'status': '200',
          'content-type': 'multipart/mixed; boundary="batch_foobarbaz"'},
         batch_response)
    ])
    gcs = self.GetService(sequence)
    gcs.DeletePrefix(self.Path('misc'))

  def testRetryBadStatusLine(self):
    sequence = HttpMockSequence([
        ('discovery/v1/apis/storage/v1/rest',
         {'status': '200'},
         self.DISCOVERY),
        ('storage/v1/b/makani/o?prefix=misc&alt=json',
         httplib.BadStatusLine('junk'),
         None),
        ('storage/v1/b/makani/o?prefix=misc&alt=json',
         httplib.BadStatusLine('junk'),
         None),
        ('storage/v1/b/makani/o?prefix=misc&alt=json',
         {'status': '200'},
         json.dumps({'items': [{'name': 'a.txt'}]})),
    ])
    mock_sleep = mock.MagicMock()

    gcs = self.GetService(sequence)
    with mock.patch('time.sleep', mock_sleep), test_util.DisableWarnings():
      names = gcs.List(self.Path('misc'))

    self.assertEqual(2, mock_sleep.call_count)
    self.assertEqual(['a.txt'], names)

  def testRetry500(self):
    sequence = HttpMockSequence([
        ('discovery/v1/apis/storage/v1/rest',
         {'status': '200'},
         self.DISCOVERY),
        ('storage/v1/b/makani/o?prefix=misc&alt=json',
         gapi_errors.HttpError(httplib2.Response({'status': '500'}),
                               '{"code": 500, "message": "Backend Error"}'),
         None),
        ('storage/v1/b/makani/o?prefix=misc&alt=json',
         gapi_errors.HttpError(httplib2.Response({'status': '501'}),
                               '{"code": 500, "message": "Backend Error"}'),
         None),
        ('storage/v1/b/makani/o?prefix=misc&alt=json',
         {'status': '200'},
         json.dumps({'items': [{'name': 'a.txt'}]})),
    ])
    mock_sleep = mock.MagicMock()

    gcs = self.GetService(sequence)
    with mock.patch('time.sleep', mock_sleep), test_util.DisableWarnings():
      names = gcs.List(self.Path('misc'))

    self.assertEqual(2, mock_sleep.call_count)
    self.assertEqual(['a.txt'], names)


class StorageApiDefaultBucketAbsPathTest(StorageApiAbsPathTest):

  def GetService(self, http_sequence):
    gcs = gcloud_util.CloudStorageApi(
        bucket=BUCKET,
        authenticator=FakeAuthenticator(http_sequence))
    return gcs


class StorageApiDiffBucketAbsPathTest(StorageApiAbsPathTest):

  def GetService(self, http_sequence):
    other_bucket = '_nonexist_bucket_'
    gcs = gcloud_util.CloudStorageApi(
        bucket=other_bucket,
        authenticator=FakeAuthenticator(http_sequence))
    return gcs


class StorageApiDefaultBucketRelPathTest(StorageApiDefaultBucketAbsPathTest):

  def Path(self, rel_path):
    return rel_path


class ComputeEngineApiTest(unittest.TestCase):

  DISCOVERY = open(os.path.join(os.path.dirname(__file__),
                                'testdata/compute-discovery.json'),
                   'r').read()

  def GetService(self, http_sequence):
    return gcloud_util.ComputeEngineApi(
        authenticator=FakeAuthenticator(http_sequence))

  def testCreateInstance(self):
    instance_name = 'foo_instance'
    sequence = HttpMockSequence([
        ('discovery/v1/apis/compute/v1/rest',
         {'status': '200'},
         self.DISCOVERY),
        ('compute/v1/projects/google.com%3Amakani/zones/outer_space/'
         'instances/' + instance_name + '?alt=json',
         gapi_errors.HttpError(httplib2.Response({'status': '404'}), (
             '{"code": 404, "message": "The resource ' + instance_name +
             ' was not found."')),
         '{}'),
        ('compute/v1/projects/google.com%3Amakani/zones/outer_space/'
         'instances?alt=json',
         {'status': '200'},
         '{}'),
    ])
    gce = self.GetService(sequence)

    gce.CreateInstance('foo_instance', 'foo_image',
                       image_project='some_project',
                       zone='outer_space',
                       machine_type='super_awesome',
                       network='some_network',
                       scopes=['foo', 'bar'],
                       metadata=[{'key': 'color', 'value': 'red'}])

  def testCreateInstance_BadGatewayAlreadyExistsIssue(self):
    # In this test case, the create instance call initially fails with a 500
    # bad gateway error, but the instance does get created in Google's server,
    # therefore the retry call fails with an already exists error.
    # For more information, see b/136171159
    instance_name = 'foo_instance'
    zone = 'outer_space'
    sequence = HttpMockSequence([
        ('discovery/v1/apis/compute/v1/rest',
         {'status': '200'},
         self.DISCOVERY),
        ('compute/v1/projects/google.com%3Amakani/zones/outer_space/'
         'instances/' + instance_name + '?alt=json',
         gapi_errors.HttpError(httplib2.Response({'status': '404'}), (
             '{"code": 404, "message": "The resource ' + instance_name +
             ' was not found."')),
         '{}'),
        ('compute/v1/projects/google.com%3Amakani/zones/' + zone + '/'
         'instances?alt=json',
         gapi_errors.HttpError(httplib2.Response({'status': '502'}), (
             '{"code": 502, "message": "Bad Gateway"')),
         '{}'),
        ('compute/v1/projects/google.com%3Amakani/zones/' + zone + '/'
         'instances?alt=json',
         gapi_errors.HttpError(httplib2.Response({'status': '409'}), (
             '{"code": 409, "message": "The resource ' + instance_name +
             ' already exists."')),
         '{}'),
        ('compute/v1/projects/google.com%3Amakani/zones/' + zone + '/'
         'instances/' + instance_name + '?alt=json',
         {'status': '200'},
         '{}')
    ])

    gce = self.GetService(sequence)

    gce.CreateInstance(instance_name, 'foo_image',
                       image_project='some_project',
                       zone=zone,
                       machine_type='super_awesome',
                       network='some_network',
                       scopes=['foo', 'bar'],
                       metadata=[{'key': 'color', 'value': 'red'}])

  def testCreateInstance_FailIfAlreadyExists(self):
    instance_name = 'foo_instance'
    zone = 'outer_space'
    sequence = HttpMockSequence([
        ('discovery/v1/apis/compute/v1/rest',
         {'status': '200'},
         self.DISCOVERY),
        ('compute/v1/projects/google.com%3Amakani/zones/' + zone + '/'
         'instances?alt=json',
         gapi_errors.HttpError(httplib2.Response({'status': '409'}), (
             '{"code": 409, "message": "The resource ' + instance_name +
             ' already exists."')),
         '{}')
    ])

    gce = self.GetService(sequence)

    with self.assertRaises(gapi_errors.HttpError):
      gce.CreateInstance(instance_name, 'foo_image',
                         image_project='some_project',
                         zone=zone,
                         machine_type='super_awesome',
                         network='some_network',
                         scopes=['foo', 'bar'],
                         metadata=[{'key': 'color', 'value': 'red'}])

  def testGetInstance(self):
    instance_name = 'foo_instance'
    zone = 'outer_space'
    sequence = HttpMockSequence([
        ('discovery/v1/apis/compute/v1/rest',
         {'status': '200'},
         self.DISCOVERY),
        ('compute/v1/projects/google.com%3Amakani/zones/' + zone + '/'
         'instances/' + instance_name + '?alt=json',
         {'status': '200'},
         '{}'),
    ])
    gce = self.GetService(sequence)

    gce.GetInstance(instance_name, zone=zone)

  def testDeleteInstance(self):
    sequence = HttpMockSequence([
        ('discovery/v1/apis/compute/v1/rest',
         {'status': '200'},
         self.DISCOVERY),
        ('compute/v1/projects/google.com%3Amakani/zones/outer_space/instances/'
         'foo_instance?alt=json',
         {'status': '200'},
         '{}'),
    ])
    gce = self.GetService(sequence)

    gce.DeleteInstance('foo_instance', 'outer_space')

  def testWaitForCompletion(self):
    sequence = HttpMockSequence([
        ('discovery/v1/apis/compute/v1/rest',
         {'status': '200'},
         self.DISCOVERY),
        ('compute/v1/projects/google.com%3Amakani/zones/outer_space/operations/'
         'fake_operation_id?alt=json',
         {'status': '200'},
         json.dumps({'name': 'fake_operation_id',
                     'status': 'PENDING',
                     'zone': 'outer_space'})),
        ('compute/v1/projects/google.com%3Amakani/zones/outer_space/operations/'
         'fake_operation_id?alt=json',
         {'status': '200'},
         json.dumps({'name': 'fake_operation_id',
                     'status': 'DONE',
                     'zone': 'outer_space'})),
    ])
    gce = self.GetService(sequence)

    first_response = {'name': 'fake_operation_id',
                      'status': 'PENDING',
                      'zone': 'outer_space'}
    gce.WaitForCompletion(first_response, wait_seconds=0.01)

  def testWaitForCompletionGlobal(self):
    sequence = HttpMockSequence([
        ('discovery/v1/apis/compute/v1/rest',
         {'status': '200'},
         self.DISCOVERY),
        ('compute/v1/projects/google.com%3Amakani/global/operations/'
         'fake_operation_id?alt=json',
         {'status': '200'},
         json.dumps({'name': 'fake_operation_id',
                     'status': 'PENDING'})),
        ('compute/v1/projects/google.com%3Amakani/global/operations/'
         'fake_operation_id?alt=json',
         {'status': '200'},
         json.dumps({'name': 'fake_operation_id',
                     'status': 'DONE'})),
    ])
    gce = self.GetService(sequence)

    pending_response = {'name': 'fake_operation_id',
                        'status': 'PENDING'}
    gce.WaitForCompletion(pending_response, wait_seconds=0.01)

  def testWaitForCompletionWithTimeout(self):
    sequence = HttpMockSequence([
        ('discovery/v1/apis/compute/v1/rest',
         {'status': '200'},
         self.DISCOVERY),
        ('compute/v1/projects/google.com%3Amakani/zones/outer_space/operations/'
         'fake_operation_id?alt=json',
         {'status': '200'},
         json.dumps({'name': 'fake_operation_id',
                     'status': 'PENDING',
                     'zone': 'outer_space'})),
    ])
    gce = self.GetService(sequence)

    first_response = {'name': 'fake_operation_id',
                      'status': 'PENDING',
                      'zone': 'outer_space'}
    with self.assertRaises(gcloud_util.ComputeEngineApiError):
      gce.WaitForCompletion(first_response, wait_seconds=0.01,
                            timeout_seconds=0.015)

  def testCheckSuccess(self):
    sequence = HttpMockSequence([
        ('discovery/v1/apis/compute/v1/rest',
         {'status': '200'},
         self.DISCOVERY),
    ])
    gce = self.GetService(sequence)

    gce.CheckSuccess({'status': '200'})
    with self.assertRaises(gcloud_util.ComputeEngineApiError):
      gce.CheckSuccess({'status': '404', 'error': 'Not Found'})


if __name__ == '__main__':
  gflags.FLAGS(sys.argv)
  unittest.main()
