#!/bin/bash -xe
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
# Startup script for producing a batch sim worker image, for use in conjunction
# with make_worker_image.sh.

readonly ZONE="$(curl -H 'Metadata-Flavor: Google' \
  "http://metadata.google.internal/computeMetadata/v1/instance/zone" \
   | awk -F'/' '{print $NF}')"

function set_status() {
  gcloud compute instances add-metadata "$(hostname)" --zone="${ZONE}" \
    --metadata custom_status="${1}"
}

trap 'set_status ERROR; exit 1' ERR

# Configure network for multicast.
sudo echo "
# The loopback network interface
auto lo
iface lo inet loopback
post-up route add -net 239.0.0.0 netmask 255.0.0.0 dev lo
" >> /etc/network/interfaces

# Initiailize the installers. Note that PIP needs to update itself to pick up bug fixes
# since the apt-installed version.
sudo apt-get -y update
sudo apt-get -y install python-dev
sudo apt-get -y install python-pip
sudo pip2 install --upgrade pip
sudo pip2 install virtualenv==16.6.2
python2 -m virtualenv "/opt/makani/python2"
sudo apt-get -y install python3-venv
python3 -m venv "/opt/makani/python3"

sudo apt-get -y install gcc

sudo apt-get -y install libhdf5-100 libopenblas-dev libblas-dev liblapack-dev \
     gfortran libquadmath0 libstdc++6 libc6 libgcc1

# These packages are for the controller analysis scripts and the MX
# optimizer worker.
sudo apt-get -y install python-tk

sudo apt-get -y install tcpdump
sudo echo "export PATH=\$PATH:/usr/sbin" >> /etc/bash.bashrc

set_status SUCCESS
