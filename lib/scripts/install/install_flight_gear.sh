#!/bin/bash
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
# Installs FlightGear from source (this takes a really long time!).

# The flight gear files are located on tau in
# \\tau\network\Kenny\makani_v2_files\
# TODO: Automatically get these from the network.

readonly FLIGHT_GEAR_HOME='/usr/local/share/flightgear'

sudo apt-get -y install automake libboost-all-dev libboost-graph-dev \
  libopenal-dev libalut-dev libopenscenegraph-dev libjpeg62-dev libplib-dev \
  zlib1g-dev

# Install SimGear.
tar xvfz simgear-2.4.0.tar.gz
cd simgear-2.4.0
./configure --with-jpeg-factory
make -j10
sudo make install
cd ..
sudo rm -rf simgear-2.4.0
# TODO: Once this script becomes more solid, we can
# automatically delete .tar.gz files.
# rm simgear-2.4.0.tar.gz

# Install FlightGear.
bunzip2 flightgear-2.4.0.tar.bz2
tar xvf flightgear-2.4.0.tar
cd flightgear-2.4.0
./configure
make -j10
sudo make install
cd ..
sudo rm -rf flightgear-2.4.0
# rm flightgear-2.4.0.tar

# Install FlightGear base.
tar xvfz fgfs-base_2.4.0.orig.tar.gz
sudo mv data "${FLIGHT_GEAR_HOME}"

# Install Makani FlightGear files.
cd "${MAKANI_HOME}/lib/flight_gear"
sudo cp -r w7 "${FLIGHT_GEAR_HOME}/Aircraft/"
sudo cp with_tether_scenario.xml "${FLIGHT_GEAR_HOME}/AI/"
sudo cp wing_and_tether_prot.xml "${FLIGHT_GEAR_HOME}/Protocol/"
