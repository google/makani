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

"""This module provides a simple Sqlite3 database interface."""

import os
import sqlite3


def _DictFactory(cursor, row):
  d = {}
  for idx, col in enumerate(cursor.description):
    d[col[0]] = row[idx]
  return d


class DatabaseInterface(object):
  """Implements a simple database wrapper."""

  def __init__(self, name):
    if not name:
      name = ':memory:'
    dirname = os.path.dirname(name)
    if dirname and not os.path.isdir(dirname):
      os.makedirs(dirname)
    self._conn = sqlite3.connect(database=name)
    self._conn.row_factory = _DictFactory
    self.SqlExecute('PRAGMA foreign_keys = ON')
    self.SqlExecute('PRAGMA busy_timeout = 30000')

  def SqlSelectOne(self, *args):
    """Execute an SQL SELECT query and return one result."""
    with self._conn:
      cursor = self._conn.execute(*args)
      result = cursor.fetchone()
    return result

  def SqlSelectMany(self, count, *args):
    """Execute an SQL SELECT query and return multiple results."""
    with self._conn:
      cursor = self._conn.execute(*args)
      result = cursor.fetchmany(count)
    return result

  def SqlSelectAll(self, *args):
    """Execute an SQL SELECT query and return all results."""
    with self._conn:
      cursor = self._conn.execute(*args)
      result = cursor.fetchall()
    return result

  def SqlExecute(self, *args):
    """Execute an SQL query.

    Args:
      *args: Arguments to pass to SQL execute function.

    Returns:
      The last row id.
    """
    try:
      with self._conn:
        cursor = self._conn.execute(*args)
        result = cursor.lastrowid
      return result
    except sqlite3.IntegrityError as e:
      print 'Sqlite3 Error: {} with command "{}".'.format(e, args)
      raise
    except sqlite3.OperationalError as e:
      print 'Sqlite3 Error: {} with command "{}".'.format(e, args)
