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

#include "avionics/platform/firmware/weather.h"

#include <stdbool.h>

#include "avionics/common/gill_types.h"
#include "avionics/firmware/cpu/sci.h"
#include "avionics/firmware/cpu/spi_pin.h"
#include "avionics/firmware/drivers/metpak.h"
#include "avionics/firmware/drivers/windmaster.h"
#include "common/macros.h"

static const MetPakConfig kMetPakConfig = {
  .device = &kSci2Interrupt,
};

static const WindmasterConfig kWindmasterConfig = {
  .device = &kSci1BufferedHalfDuplex,
};

void WeatherInit(void) {
  // Select RS-422.
  SpiPinConfigureAsOutputPushPull(kSpi3PinScs1, false);  // UART1_TYPE.
  SpiPinConfigureAsOutputPushPull(kSpi3PinScs4, false);  // UART2_TYPE.
  MetPakInit();
  WindmasterInit();
}

bool WeatherPollMetPak(GillDataMetPakFull *out) {
  GillData gill;
  if (MetPakPoll(&kMetPakConfig, &gill) && gill.id == kGillDataIdMetPakFull) {
    *out = gill.u.metpak_full;

    // We were not able to disable the wind sensor in the MetPak configuration,
    // so we ignore the wind sensor fault status here. We assume the MetPak
    // does not have any other faults when it reports a wind sensor fault.
    // TODO: Discuss configuration with Gill Instruments.
    if (out->status == kGillMetPakStatusWindSensorFailed) {
      out->status = kGillMetPakStatusOk;
    }
    return true;
  }
  return false;
}

bool WeatherPollWind(GillDataWindmasterUvw *out) {
  GillData gill;
  if (WindmasterPoll(&kWindmasterConfig, &gill)
      && gill.id == kGillDataIdWindmasterUvw) {
    *out = gill.u.windmaster_uvw;
    return true;
  }
  return false;
}
