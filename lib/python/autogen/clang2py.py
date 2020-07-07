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

"""This script wraps clang2py so it can be used as a binary.

We would prefer to call the clang2py module directly, but it does not properly
accept arguments passed in through main().  The project has not been updated in
some time, but it's possible a future release will include this feature.
"""

from __future__ import absolute_import
import cStringIO
import ctypes.util
import sys

import clang.cindex
import ctypeslib.clang2py

if __name__ == "__main__":
  # Set the clang library to clang-7, since clang2py will not look for it.
  for version in ["libclang", "clang", "clang-7"]:
    libclang = ctypes.util.find_library(version)
    if libclang is not None:
      clang.cindex.Config.set_library_file(libclang)

  # Capture stderr and stdout to look for clang error message, since this does
  # not result in an exception and a nonzero return code.
  real_stderr = sys.stderr
  real_stdout = sys.stdout
  try:
    local_stderr = cStringIO.StringIO()
    local_stdout = cStringIO.StringIO()
    sys.stderr = local_stderr
    sys.stdout = local_stdout

    ctypeslib.clang2py.main()

    if ("crash detected" in local_stderr.getvalue() or
        "crash detected" in local_stdout.getvalue()):
      sys.stderr.write("Failure due to libclang crash.\n")
      sys.exit(-1)
  finally:
    sys.stderr = real_stderr
    sys.stderr.write(local_stderr.getvalue())
    sys.stdout = real_stdout
    sys.stdout.write(local_stdout.getvalue())
