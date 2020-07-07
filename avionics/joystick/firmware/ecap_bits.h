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

// Definition of tms570 eCAP register bit fields.
//
// These come from the eCAP application section of the tms570 TRM.
#ifndef AVIONICS_JOYSTICK_FIRMWARE_ECAP_BITS_H_
#define AVIONICS_JOYSTICK_FIRMWARE_ECAP_BITS_H_

// CAPxPOL bits.
#define EC_RISING     0x0
#define EC_FALLING    0x1

// CTRRSTx bits.
#define EC_ABS_MODE   0x0
#define EC_DELTA_MODE 0x1

// PRESCALE bits.
#define EC_BYPASS     0x0
#define EC_DIV1       0x0
#define EC_DIV2       0x1
#define EC_DIV4       0x2
#define EC_DIV6       0x3
#define EC_DIV8       0x4
#define EC_DIV10      0x5

// ECCTL2 (ECAP Control Reg 2.)
// ============================
// CONT/ONESHOT bit.
#define EC_CONTINUOUS 0x0
#define EC_ONESHOT    0x1

// STOPVALUE bit.
#define EC_EVENT1     0x0
#define EC_EVENT2     0x1
#define EC_EVENT3     0x2
#define EC_EVENT4     0x3

// RE-ARM bit.
#define EC_ARM        0x1

// TSCTRSTOP bit.
#define EC_FREEZE     0x0
#define EC_RUN        0x1

// SYNCO_SEL bit.
#define EC_SYNCIN     0x0
#define EC_CTR_PRD    0x1
#define EC_SYNCO_DIS  0x2

// CAP/APWM mode bit.
#define EC_CAP_MODE   0x0
#define EC_APWM_MODE  0x1

// APWMPOL bit.
#define EC_ACTV_HI    0x0
#define EC_ACTV_LO    0x1

#endif  // AVIONICS_JOYSTICK_FIRMWARE_ECAP_BITS_H_
