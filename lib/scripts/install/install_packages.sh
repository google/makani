#!/bin/bash -ex
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

#
# Installs all the packages and peripheral software necessary to build
# and run the simulator and controller.

if [[ "${USER}" == 'root' ]]; then
  echo 'Do not run install_packages.sh as root.'
  exit 1
fi

# Make sure that files are accessible to all users.
umask 022

readonly MAKANI_OPT='/opt/makani'
readonly MAKANI_THIRD_PARTY="${MAKANI_OPT}/third_party"
sudo mkdir -p "${MAKANI_OPT}"
sudo chown "${USER}:$(id -gn)" "${MAKANI_OPT}"

# Change to the directory where the install script is (in case it was
# run from elsewhere).
readonly INSTALL_SCRIPT_DIR="$(cd $(dirname ${BASH_SOURCE[0]}) && pwd)"
cd "${INSTALL_SCRIPT_DIR}"

# Grabs directory that this script is in and goes up three
# directories to determine MAKANI_HOME.
export MAKANI_HOME="$(cd ${INSTALL_SCRIPT_DIR} && cd ../../../ && pwd)"

# Create ~/.makani.d directory if it doesn't exist already.
mkdir -p ~/.makani.d

# Generate the makani_bashrc file and place it in ~/.makani.d/
NEW_PATH="${MAKANI_HOME}/lib/scripts/operator:${MAKANI_HOME}/lib/scripts/developer"
cat > ~/.makani.d/makani_bashrc <<EOL
################################################################################
# GENERATED FILE, DO NOT EDIT!
# This file is generated when running //lib/scripts/install/install_packages.sh
################################################################################

# Makani bash configuration
# This file is sourced in ~/.bashrc
export MAKANI_HOME=${MAKANI_HOME}
export PATH=\$PATH:${NEW_PATH}
EOL

# Source makani_bashrc in ~/.bashrc
# Note that this is the only place where we modify ~/.bashrc
if ! grep -q "~/.makani.d/makani_bashrc" ~/.bashrc; then
  echo "source ~/.makani.d/makani_bashrc" >> ~/.bashrc
fi

source "${MAKANI_HOME}/lib/scripts/mbash.sh"
source "${MAKANI_HOME}/lib/scripts/system.sh"

# Override Debian environment and profile files to allow access to sbin dirs.
if ($(system::is_stretch)); then
  sudo cp "${MAKANI_HOME}/lib/scripts/install/data/etc_environment" \
       /etc/environment
  sudo cp "${MAKANI_HOME}/lib/scripts/install/data/etc_profile" \
       /etc/profile
fi

if !($(system::is_supported_linux)); then
  echo 'System not supported. Supported distributions: Debian Stretch.'
  exit 1
fi

# Installation starts here!
sudo apt-get -y update

# Install pip for Python package management.
sudo apt-get -y install python-pip python3-pip

sudo apt-get -y install python-dev python3-dev

sudo pip2 install setuptools==40.6.3

# GSUtil does not properly declare its dependence on pyasn1.
sudo pip2 install gsutil==4.35 google-apitools==0.5.26 pyasn1==0.4.5

# TODO: likely remove, or come up with alternative source for binaries
MAKANI_PIP_REPO=$MAKANI_OPT/pip
mkdir -p $MAKANI_PIP_REPO
gsutil -m rsync -r -d "gs://makani_pub/machine_setup/pip" $MAKANI_PIP_REPO

# Install compilers, coverage analyzers, and common numerical
# libraries.
sudo apt-get -y install g++ lcov gsl-bin libgsl0-dev make libblas-dev \
     liblapack-dev gfortran
sudo apt-get -y install clang-7

if $(system::is_stretch); then
  sudo apt-get -y install clang-tools-7
  sudo ln -s /usr/bin/clangd-7 /usr/bin/clangd || true
fi

# Install FreeGLUT, which is used by the simple visualizer.
sudo apt-get -y install freeglut3-dev

# Install Modbus libraries, which are used for the GS winch drivers.
sudo apt-get -y install libmodbus5 libmodbus-dev

# Basic utilities (psmisc includes killall).  These may not be included with
# minimial installations, like those used by GCE instances.
sudo apt-get -y install psmisc unzip

# Install tools for saving PNG files with Latex text
# (e.g. analysis/iec_cases/publish_iec_cases.py).
sudo apt-get -y install dvipng

# Use for compiling for ARM Linux targets.
sudo apt-get -y install gcc-arm-linux-gnueabi g++-arm-linux-gnueabi

# Use for JSON output for saving state in sim and control.
sudo apt-get -y install libjansson4 libjansson-dev

# Use for base64 encoding in saving sim and control.
sudo apt-get -y install libglib2.0-0 libglib2.0-dev

# Use for our code "linting" scripts.
sudo apt-get -y install splint astyle colordiff xemacs21 valgrind

# Snappy is used for compressing data over the remote telemetry link.
sudo apt-get -y install libsnappy-dev

sudo apt-get -y install tcpdump dnsutils

# Install net-tools for ifconfig and route (legacy tools).
sudo apt-get -y install net-tools

# TODO: Migrate or remove
CLANG_FORMAT_VERSION="3.8.1"
readonly CLANG_PREBUILT="clang+llvm-${CLANG_FORMAT_VERSION}-x86_64-linux-gnu-debian8"

echo "Obtaining clang-format from ${CLANG_PREBUILT}."
gsutil -m cp \
    "gs://makani_pub/machine_setup/${CLANG_PREBUILT}.tar.xz" .
tar -xvf "${CLANG_PREBUILT}.tar.xz" "${CLANG_PREBUILT}/bin/clang-format"
sudo cp "${CLANG_PREBUILT}/bin/clang-format" \
    "/usr/bin/clang-format-${CLANG_FORMAT_VERSION}"
sudo chmod +x "/usr/bin/clang-format-${CLANG_FORMAT_VERSION}"
rm -r "${CLANG_PREBUILT}"
rm "${CLANG_PREBUILT}.tar.xz"

# Used for python GUIs.
sudo apt-get -y install wxglade libwebkit2gtk-4.0-dev libsdl2-dev

# Used for legacy monitor.
sudo apt-get -y install libgtk2.0-dev

# For static builds involving gflags and glog.
sudo apt-get -y install libunwind-dev
sudo apt-get -y install liblzma-dev

# Install Google flags and logging libraries.
sudo apt-get -y install libgoogle-glog0v5 libgflags2v5
sudo apt-get -y install libgoogle-glog-dev libgoogle-glog-doc \
    libgflags-dev libgflags-doc

# Install libraries used for C header to Python conversion.
sudo apt-get -y install libclang-7-dev
sudo apt-get -y install castxml

# Install HDF5 1.8.11 and pcap library for data recorder.
sudo apt-get -y install libhdf5-100 libhdf5-dev libpcap-dev libcap2-bin

# Install tools for generating data for test scripts.
sudo apt-get -y --allow-downgrades install octave

# Install Boost library for using odeint.
sudo apt-get -y install libboost1.62-dev

# Install SWIG to wrap C/C++ functions in Python.
sudo apt-get -y install swig3.0

# Install packages for Python control design.
sudo apt-get -y install liblapack-dev gfortran python-tk

# Install Wine for running TurbSim.
sudo apt-get -y install wine

# Install python wrapper for dialog used by ui_helper.
sudo apt-get -y install dialog

# Install mysql client for Log Analyzer service.
sudo apt-get -y install default-libmysqlclient-dev

# Install libusb-1.0-0 for labjack module compilation.
sudo apt-get -y install libusb-1.0-0-dev

# Install libqt4 for PySide.
sudo apt-get -y install libqt4-opengl libqt4-svg

# Include python dependencies for bazel pip installation.
# TODO(b/140763177): Installing numpy may be unnecessary if slycot is upgraded.
# Currently slycot installation in bazel pip fails due to missing numpy.
pip2 install --user -I numpy==1.14.6

# Create makani python virtual environment.
pip2 install --user virtualenv==16.6.2
sudo apt-get -y install python3-venv
PY2_VENV="/opt/makani/python2"
if [[ ! -d "${PY2_VENV}" ]]; then
  python2 -m virtualenv "${PY2_VENV}"
fi
PY3_VENV="/opt/makani/python3"
if [[ ! -d "${PY3_VENV}" ]]; then
  python3 -m venv "${PY3_VENV}"
fi

# Install shflags library in /opt.
if [[ ! -d /opt/shflags-1.0.3 ]]; then
  wget -nH https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/shflags/shflags-1.0.3.tgz
  tar xzfp shflags-1.0.3.tgz
  rm shflags-1.0.3.tgz
  sudo mkdir -p /opt
  sudo mv shflags-1.0.3 /opt
  sudo chown -R root.root /opt/shflags-1.0.3
fi

# Install odeint as an alternative to GSL's ODE solver.
if [[ ! -d /opt/odeint-v2 ]]; then
  git clone git://github.com/headmyshoulder/odeint-v2
  cd odeint-v2
  git checkout -b odeint_v24 tags/v2.4 > /dev/null
  cd ..
  sudo mv odeint-v2 /opt
  sudo chown -R root.root /opt/odeint-v2
  sudo find /opt/odeint-v2 -exec chmod 755 {} \;
fi

# Fix dynamic links for access to shared libraries.
sudo ldconfig

# Sometimes ttyACM* (for the joystick) requires root access.
sudo adduser "${USER}" dialout

# Set up permissions to allow using tcpdump without being root.
sudo groupadd -f tcpdump
sudo addgroup "${USER}" tcpdump
sudo chown root.tcpdump /usr/sbin/tcpdump
sudo chmod 0750 /usr/sbin/tcpdump
sudo setcap 'CAP_NET_RAW+eip' /usr/sbin/tcpdump

# Add logs directory.
mkdir -p "${MAKANI_HOME}/logs"

# Install GSL source for compiling for the M600.
./install_gsl.sh

# TODO: likely remove
# Install sysctl igmp_max_memberships script.
readonly SYSCTL_SCRIPT_DIR='/etc/sysctl.d'
if [[ -d "${SYSCTL_SCRIPT_DIR}" ]]; then
  sudo cp "${MAKANI_HOME}/lib/scripts/install/data/99-igmp-max-memberships.conf" \
    "${SYSCTL_SCRIPT_DIR}/99-igmp-max-memberships.conf"
  sudo chown root.root "${SYSCTL_SCRIPT_DIR}/99-igmp-max-memberships.conf"
  sudo chmod 644 "${SYSCTL_SCRIPT_DIR}/99-igmp-max-memberships.conf"
else
  echo 'The sysctl configuration directory does not exist; '
  echo 'not installing IGMP max memberships script.'
fi

# TODO: likely remove
# Install aio_network_impl script.
sudo cp "${MAKANI_HOME}/lib/scripts/install/data/aio_network_impl" \
     "/usr/sbin/aio_network_impl"
sudo chown root.root "/usr/sbin/aio_network_impl"
sudo chmod 755 "/usr/sbin/aio_network_impl"

# TODO: remove
# Install the disable_apparmor command, used by log.
sudo cp "${MAKANI_HOME}/lib/scripts/install/data/disable_apparmor" \
     "/usr/sbin/disable_apparmor"
sudo chown root.root "/usr/sbin/disable_apparmor"
sudo chmod 755 "/usr/sbin/disable_apparmor"

# Install Bazel.
./install_bazel.sh

# Install gtk+extra (used for new data monitor).
sudo apt-get -y install libgtkextra-3.0 libgtkextra-dev

echo 'Installation complete.'
