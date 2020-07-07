#!/bin/bash -e
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


source "${MAKANI_HOME}/lib/scripts/mbash.sh"

if [[ -d /opt/gsl ]]; then
  if [[ ! "$(grep "PACKAGE_VERSION='1.16'" /opt/gsl/configure)" ]]; then
    echo 'You have an older version of GSL installed; please remove /opt/gsl' \
      'and try again.'
    exit 1
  fi
  echo "The correct version of gsl is already installed."
  exit 0
fi

wget 'http://archive.ubuntu.com/ubuntu/pool/main/g/gsl/gsl_1.16+dfsg.orig.tar.gz'
tar xfvz gsl_1.16+dfsg.orig.tar.gz

cd gsl-1.16+dfsg

# Not sure why this is necessary, but debian/rules copies a version of this file
# into place before configuring.
mkdir -p doc
touch doc/Makefile.in

# This hack lets the configure script skip the checks where it
# tries to make an executable.
sed -i 's/| \*\.obj //g' configure

# Setup GSL library for ARM architecture.
echo 'Configuring GSL for the M600 ARM architecture...'

# Run configure using the ARM compiler.  Treat warnings as errors.
# Suppress the warnings about unreachable lines, which the
# configure script likes to use.
./configure --host=arm CC=arm-linux-gnueabi-gcc CFLAGS=''

# Instead of not defining HAVE_IEEEFP_H, we define it to be 0,
# which gets rid of some warnings from the ARM compiler.
sed -i 's@/\* #undef HAVE_IEEEFP_H \*/@#define HAVE_IEEEFP_H 0@' config.h

mkdir -p makani_config/arm
cp config.h templates_on.h templates_off.h build.h makani_config/arm

# Make GSL header file links.
cd gsl
make
cd ..

# Older versions of the code look for config.h in /opt/gsl to compile for
# W7, so we delete the config.h used by M600.
rm config.h

cd ..

sudo mv gsl-1.16+dfsg /opt/gsl
sudo chown -R root.root /opt/gsl
sudo chmod -R a+r /opt/gsl
sudo find /opt/gsl -type d -exec chmod 755 {} \;
rm gsl_1.16+dfsg.orig.tar.gz
