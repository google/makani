/*
 * Copyright 2020 Makani Technologies LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "avionics/firmware/gps/septentrio_config.h"

#include <stdint.h>

#include "avionics/firmware/cpu/sci.h"
#include "avionics/firmware/drivers/gps_device.h"
#include "avionics/firmware/drivers/septentrio.h"
#include "common/macros.h"


// See AsteRx-m Firmware v3.3.0 Command Line Interface Reference Guide. Rover
// initialization strings correspond to the wing flight computers.
static const char *kRoverInitStrings[] = {
  "getReceiverCapabilities\n",
  "setDataInOut,COM1,auto,SBF\n",
  "setDataInOut,COM2,auto,none\n",
  "setElevationMask,PVT,5\n",
  "setAntennaOffset,Main,0,0,0,\"ACCG5ANT_2AT1   NONE\",ROVER,0\n",
  "setReceiverDynamics,High,UAV\n",
  "setPVTMode,Rover,all\n",
  "setRTCMv3Usage,all\n",
  "setSatelliteTracking,all\n",
  "setSatelliteUsage,GPS+SBAS+GLONASS\n",
  // Output debugging information (one shot).
  "lif,Identification\n",
  "lif,Debug\n",
  "lif,Error\n",
  // Receiver disables RTK when requesting data rates above 10 Hz.
  "setSBFOutput,Stream1,COM1,MeasEpoch+MeasExtra+EndOfMeas+PVTCart,msec100\n",
  // Debug output.
  "setSBFOutput,Stream2,COM1,OutputLink+InputLink+ChannelStatus+ReceiverStatus"
  "+ReceiverSetup+Commands+PVTSupport+QualityInd+DiskStatus+DiffCorrIn,sec1\n",
  // GNSS constellation data.
  "setSBFOutput,Stream3,COM1,GEORawL1+GPSNav+GPSAlm+GPSIon+GPSUtc"
  "+GLONav+GLOAlm+GLOTime+GEONav,sec1\n",
};

const SeptentrioDevice kSeptentrioRover = {
  .hardware = &kGps,
  .this_port = &kSci2Interrupt,
  .other_port = &kSci1Interrupt,
  .init_string = kRoverInitStrings,
  .num_init_strings = ARRAYSIZE(kRoverInitStrings)
};

// See AsteRx-m Firmware v3.3.0 Command Line Interface Reference Guide. Base
// station initialization strings correspond to the ground station.
static const char *kBaseInitStrings[] = {
  "getReceiverCapabilities\n",
  "setDataInOut,COM1,auto,SBF\n",
  "setDataInOut,COM2,auto,RTCMv3\n",
  "setElevationMask,PVT,5\n",
  "setAntennaOffset,Main,0,0,0,\"ACCG5ANT_2AT1   NONE\",BASE,0\n",
  "setReceiverDynamics,Low,Static\n",
  "setPVTMode,Rover,all\n",
  "setRTCMv3Formatting,123\n",
  "setClockSyncThreshold,usec500\n",
  "setRTCMv3Output,COM2,RTCM1004+RTCM1006+RTCM1008+RTCM1012+RTCM1013+RTCM1033"
  "\n",
  "setRTCMv3Interval,RTCM1003|4+RTCM1011|12,0.1\n",
  "setRTCMv3Interval,RTCM1005|6+RTCM1007|8+RTCM1013+RTCM1033,1.0\n",
  "setSatelliteTracking,all\n",
  "setSBFOutput,Stream1,COM1,MeasEpoch+GPSNav+GPSIon+GPSUtc+PVTCart,msec50\n"
  // Output receiver version.
  "setSBFOutput,Stream2,COM1,ReceiverSetup,min5\n",
};

const SeptentrioDevice kSeptentrioBase = {
  .hardware = &kGps,
  .this_port = &kSci2Interrupt,
  .other_port = &kSci1Interrupt,
  .init_string = kBaseInitStrings,
  .num_init_strings = ARRAYSIZE(kBaseInitStrings)
};
