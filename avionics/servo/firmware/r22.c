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

#include "avionics/servo/firmware/r22.h"

#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/cpu/sci.h"
#include "avionics/servo/firmware/r22_can.h"
#include "avionics/servo/firmware/r22_def.h"

void R22Init(void) {
  // Initialize N2HET1 for GPIO functionality.
  //
  // N2HET1 Pin  R22 Pin          Type
  // ----------  ---------------  ----------
  // N2HET1[04]  R22_IN2          Out
  // N2HET1[14]  R22_IN3          Out
  // N2HET1[16]  LIGHT_OUT        Out
  // N2HET1[18]  CLAMP_ENABLE     Out
  // N2HET1[20]  EXT_NDISABLE     In
  // N2HET1[22]  R22_ENABLE       Out

  // Enable clocks.
  PeripheralEnable(kPeripheralN2Het1);
  PeripheralEnable(kPeripheralGio);

  // Set output pins.
  N2HET(1).HETDIR.HETDIR4  = 1;
  N2HET(1).HETDIR.HETDIR14 = 1;
  N2HET(1).HETDIR.HETDIR16 = 1;
  N2HET(1).HETDIR.HETDIR18 = 1;
  N2HET(1).HETDIR.HETDIR22 = 1;

  // Set input pins.
  N2HET(1).HETDIR.HETDIR20 = 0;

  // Initialize GIO.
  //
  // GIO Pin  R22 Pin          Type
  // -------  ---------------  ----------
  // GIOA[2]  R22_FAULT        In
  // GIOA[6]  R22_OUT2         In
  // GIOA[7]  CLAMP_DRIVE_MON  In

  // Reset.
  GIO.GCR0.RESET = 0;
  GIO.GCR0.RESET = 1;

  // Set input pins.
  GIO.DIRA.DIR2 = 0;
  GIO.DIRA.DIR6 = 0;
  GIO.DIRA.DIR7 = 0;

  // Enable R22.
  N2HET(1).HETDOUT.HETDOUT22 = 1;

  // Initialize SCI to interface with the R22.
  SciInit(&R22_SCI, R22_DEFAULT_BAUD_RATE);

  // Initialize R22 CAN interface.
  R22CanInit();
}

bool R22GetFaultMon(void) {
  return GIO.DINA.DIN2 != 0;
}

bool R22GetClampMon(void) {
  return GIO.DINA.DIN7 != 0;
}

void R22EnableClamp(void) {
  N2HET(1).HETDOUT.HETDOUT18 = 0;
}

void R22DisableClamp(void) {
  N2HET(1).HETDOUT.HETDOUT18 = 1;
}
