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

#ifndef AVIONICS_FIRMWARE_IDENTITY_IDENTITY_TYPES_H_
#define AVIONICS_FIRMWARE_IDENTITY_IDENTITY_TYPES_H_

// This enumeration specifies a wire format. To maintain compatibility with all
// devices, please do not renumber.  This specifies the TMS570 main board
// hardware, and doesn't specify any daughter board(s), so e.g.  kHardwareTypeFc
// is used for old recorder boards as well.
typedef enum {
  kHardwareTypeUnknown = -1,  // Used for board bringup.
  kHardwareTypeCs      =  0,
  kHardwareTypeFc      =  1,
  kHardwareTypeMotor   =  2,
  kHardwareTypeServo   =  3,
  kHardwareTypeAio     =  4
} HardwareType;

typedef enum {
  kCarrierHardwareTypeUnknown        = -1,
  kCarrierHardwareTypeFc             = 0,
  kCarrierHardwareTypeCs             = 1,
  kCarrierHardwareTypeServo          = 2,
  kCarrierHardwareTypeMotor          = 3,
  kCarrierHardwareTypeShortStack     = 4,
  kCarrierHardwareTypeRecorder       = 5,
  kCarrierHardwareTypeLoadcell       = 6,
  kCarrierHardwareTypeGroundIo       = 7,
  kCarrierHardwareTypeBattery        = 8,
  kCarrierHardwareTypeMvLv           = 9,
  kCarrierHardwareTypeFaultInjection = 10,
  kCarrierHardwareTypeJoystick       = 11,
  kCarrierHardwareTypeBreakout       = 12,
  kCarrierHardwareTypeUnused13       = 13,
  kCarrierHardwareTypeUnused14       = 14,
  kCarrierHardwareTypeNone           = 15,
} CarrierHardwareType;

#endif  // AVIONICS_FIRMWARE_IDENTITY_IDENTITY_TYPES_H_
