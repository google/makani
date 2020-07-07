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

#include "avionics/firmware/gps/novatel_config.h"

#include "avionics/firmware/cpu/sci.h"
#include "avionics/firmware/drivers/novatel.h"
#include "common/macros.h"

static const char *kRoverInitStrings[] = {
  "unlogall",
  "dynamics air",
  "rtkdynamics dynamic",
  // Initialize antennas.
  "antennapower on",
  "dualantennapower off",
  "dualantennaalign disable",
  // INTERFACEMODE [port] rxtype txtype [responses].
  "interfacemode com1 novatel novatel on",
  "interfacemode com2 rtcmv3 none",
  // Use EXTRA_SAFE RTK mode to prevent the receiver from converging to an
  // incorrect NARROW_INT solution when it does not have a clear view of the
  // sky.
  "rtkqualitylevel extra_safe",
  // LOG [port] message [trigger [period [offset [hold]]]].
  "log com1 bestxyzb ontime 0.05",
  "log com1 ionutcb onnew",
  "log com1 rangeb ontime 0.05",
  "log com1 rawephemb onnew",
  "log com1 rxstatusb ontime 10",
  "log com1 hwmonitorb ontime 10",
  // Output receiver version and configuration. We do not request rxconfig
  // periodically because it ties up the GPS receiver and serial communications
  // for a couple seconds and degrades performance. http://b/64680344
  "log com1 versionb once",
  "log com1 rxconfigb once",
  "log com1 validmodels once",
  "log com1 authcodes once",
  "log com1 version once",
};

const NovAtelDevice kNovAtelRover = {
  .this_port = &kSci2Interrupt,
  .other_port = &kSci1Interrupt,
  .init_string = kRoverInitStrings,
  .num_init_strings = ARRAYSIZE(kRoverInitStrings)
};

static const char *kBaseInitStrings[] = {
  "unlogall",
  "dynamics foot",
  "rtkdynamics auto",
  // INTERFACEMODE [port] rxtype txtype [responses].
  "interfacemode com1 novatel novatel on",
  "interfacemode com2 none rtcmv3",
  // Initialize attitude engine.
  "antennapower on",
  "dualantennapower on",
  "dualantennaalign enable 20 20",
  // LOG [port] message [trigger [period [offset [hold]]]].
  "log com1 bestxyzb ontime 0.05",
  "log com1 ionutcb onnew",
  "log com1 rangeb ontime 0.05",
  "log com1 rawephemb onnew",
  "log com1 headingb onnew",
  "log com1 headingrateb onnew",
  "posave on 0.10 0 0",
  // High bandwidth RTK observables.
  "log com2 rtcm1074 ontime 0.1",  // MSM4 Full GPS observables.
  "log com2 rtcm1084 ontime 0.1",  // MSM4 Full GLONASS observables.
  // Common RTK messages.
  "log com2 rtcm1006 ontime 1.0",  // Stationary antenna reference w/ height.
  "log com2 rtcm1033 ontime 1.0",  // Receiver and antenna descriptors.
  "log com2 rtcm1230 ontime 1.0",  // GLONASS L1 & L2 code phase biases.
  // Receiver hardware status and confguration.
  "log com1 rxstatusb ontime 10",
  "log com1 hwmonitorb ontime 10",
  // Output receiver version and configuration. We do not request rxconfig
  // periodically because it ties up the GPS receiver and serial communications
  // for a couple seconds and degrades performance. http://b/64680344
  "log com1 versionb once",
  "log com1 rxconfigb once",
  "log com1 validmodels once",
  "log com1 authcodes once",
  "log com1 version once",
};

const NovAtelDevice kNovAtelBase = {
  .this_port = &kSci2Interrupt,
  .other_port = &kSci1Interrupt,
  .init_string = kBaseInitStrings,
  .num_init_strings = ARRAYSIZE(kBaseInitStrings)
};
