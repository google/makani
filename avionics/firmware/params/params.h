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

#ifndef AVIONICS_FIRMWARE_PARAMS_PARAMS_H_
#define AVIONICS_FIRMWARE_PARAMS_PARAMS_H_

#include "avionics/firmware/params/i2c.h"
#include "avionics/firmware/params/flash.h"

// This header exists as a convenience to access all the raw param interfaces
// from one place.  These functions are of the form:
//    const void *Get<SECTION>ParamsRaw(uint32_t *version_number);
//
// If the param section header and data are valid, these functions return a
// pointer to the parameter data and the version number (crc) of param format.
// If either the header or data is invalid, they return NULL.
//
// Use of these functions generally not needed.  Instead, for config and calib
// params use the auto-genrated code from pack2.  For serial params use the
// library functionality in avionics/firmware/serial.

#endif  // AVIONICS_FIRMWARE_PARAMS_PARAMS_H_
