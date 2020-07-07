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

#include "avionics/firmware/startup/clock_tms570_config.h"

#include "common/macros.h"

// See TMS570LS1227 Table 3-1 "Clock Domain Timing Specifications".
COMPILE_ASSERT(CLOCK_HCLK_KHZ <= 160000,
               CLOCK_HCLK_KHZ_exceeds_maximum);
COMPILE_ASSERT(CLOCK_GCLK_KHZ <= CLOCK_HCLK_KHZ,
               CLOCK_GCLK_KHZ_exceeds_maximum);
COMPILE_ASSERT(CLOCK_VCLK_KHZ <= 100000,
               CLOCK_VCLK_KHZ_exceeds_maximum);
COMPILE_ASSERT(CLOCK_VCLK2_KHZ <= 100000,
               CLOCK_VCLK2_KHZ_exceeds_maximum);
COMPILE_ASSERT(CLOCK_VCLK3_KHZ <= 100000,
               CLOCK_VCLK3_KHZ_exceeds_maximum);
COMPILE_ASSERT(CLOCK_VCLKA1_KHZ <= 100000,
               CLOCK_VCLKA1_KHZ_exceeds_maximum);
COMPILE_ASSERT(CLOCK_VCLKA2_KHZ <= 100000,
               CLOCK_VCLKA2_KHZ_exceeds_maximum);
COMPILE_ASSERT(CLOCK_VCLKA3_KHZ <= 100000,
               CLOCK_VCLKA3_KHZ_exceeds_maximum);
COMPILE_ASSERT(CLOCK_VCLKA4_KHZ <= 100000,
               CLOCK_VCLKA4_KHZ_exceeds_maximum);

// EMAC requirements.
COMPILE_ASSERT(CLOCK_VCLKA4_DIVR_KHZ == 25000,
               CLOCK_VCLKA4_DIVR_KHZ_must_equal_25_MHz_for_EMAC);
