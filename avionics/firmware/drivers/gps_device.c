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

#include "avionics/firmware/drivers/gps_device.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>  // For NULL.

#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/registers.h"


static void Init(void) {
  // GPS hardware configuration.
  //
  // TMS570 Pin  Signal    Type
  // ----------  --------  ----------
  // N2HET1[16]  GPS_nRST  Open-drain
  // GIOA6       GPS_PPS   Input
  // CAN2RX      GPS_PV    Open-drain

  // See Table 4-21 from TMS570 Reference Manual Rev A (TI P/N SPNU515A).
  // Select H2HET1[16] (default function).
  IommClearPinmux(34, 0);
  // Select GIOA6 (default function).
  IommClearPinmux(3, 16);

  // Place GPS into reset during initialization.
  N2HET(1).HETDIR.HETDIR16 = 1;    // 1 = Output.
  N2HET(1).HETPDR.HETPDR16 = 1;    // 1 = Open-drain.
  N2HET(1).HETDOUT.HETDOUT16 = 0;  // 0 = Logic low.

  // Pull GIO module out of reset.
  GIO.GCR0.RESET = 1;

  // Configure GPS 1-PPS interrupt.
  GIO.DIRA.DIR6 = 0;            // 0 = Input.
  GIO.PULDISA.PULDIS6 = 0;      // 0 = Enable pulling.
  GIO.PSLA.PSL6 = 1;            // 1 = Pull up.
  GIO.ENACLR.ENACLRA6 = 1;      // 1 = Disable interrupts.
  GIO.INTDET.INTDETA6 = 0;      // 0 = Either falling or rising edge.
  GIO.POL.POLA6 = 0;            // 0 = Trigger on falling edge.
  GIO.FLG.raw = GIO_FLG_FLGA6;  // 1 = Clear PPS interrupt flag.
}

static void SetReset(void) {
  // Safest to reinitialize all hardware.
  Init();
}

static void ClearReset(void) {
  // Bring GPS out of reset.
  N2HET(1).HETDOUT.HETDOUT16 = 1;  // 1 = High impedance.
}

static bool PollPps(void) {
  if (GIO.FLG.FLGA6) {
    GIO.FLG.raw = GIO_FLG_FLGA6;
    return true;
  }
  return false;
}

void GpsDeviceInit(const GpsDevice *dev) {
  assert(dev != NULL && dev->init != NULL);
  dev->init();
}

void GpsDeviceSetReset(const GpsDevice *dev) {
  assert(dev != NULL && dev->set_reset != NULL);
  dev->set_reset();
}

void GpsDeviceClearReset(const GpsDevice *dev) {
  assert(dev != NULL && dev->clear_reset != NULL);
  dev->clear_reset();
}

bool GpsDevicePollPps(const GpsDevice *dev) {
  assert(dev != NULL && dev->poll_pps != NULL);
  return dev->poll_pps();
}

const GpsDevice kGps = {
  .init = Init,
  .set_reset = SetReset,
  .clear_reset = ClearReset,
  .poll_pps = PollPps
};
