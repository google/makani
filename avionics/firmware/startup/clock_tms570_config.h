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

#ifndef AVIONICS_FIRMWARE_STARTUP_CLOCK_TMS570_CONFIG_H_
#define AVIONICS_FIRMWARE_STARTUP_CLOCK_TMS570_CONFIG_H_

// Input clocks.
#define CLOCK_PLL1_KHZ 160000
#define CLOCK_PLL2_KHZ 100000

// System and peripheral clocks.
#define CLOCK_GCLK_KHZ        CLOCK_PLL1_KHZ
#define CLOCK_HCLK_KHZ        CLOCK_GCLK_KHZ
#define CLOCK_VCLK_DIVR       2
#define CLOCK_VCLK_KHZ        (CLOCK_HCLK_KHZ / CLOCK_VCLK_DIVR)
#define CLOCK_VCLK2_DIVR      2
#define CLOCK_VCLK2_KHZ       (CLOCK_HCLK_KHZ / CLOCK_VCLK2_DIVR)
#define CLOCK_VCLK3_DIVR      2
#define CLOCK_VCLK3_KHZ       (CLOCK_HCLK_KHZ / CLOCK_VCLK3_DIVR)
#define CLOCK_VCLKA1_KHZ      CLOCK_PLL2_KHZ
#define CLOCK_VCLKA2_KHZ      CLOCK_PLL2_KHZ
#define CLOCK_VCLKA3_KHZ      CLOCK_PLL2_KHZ
#define CLOCK_VCLKA3_DIVR     4
#define CLOCK_VCLKA3_DIVR_KHZ (CLOCK_VCLKA3_KHZ / CLOCK_VCLKA3_DIVR)
#define CLOCK_VCLKA4_KHZ      CLOCK_PLL2_KHZ
#define CLOCK_VCLKA4_DIVR     4
#define CLOCK_VCLKA4_DIVR_KHZ (CLOCK_VCLKA4_KHZ / CLOCK_VCLKA4_DIVR)

#endif  // AVIONICS_FIRMWARE_STARTUP_CLOCK_TMS570_CONFIG_H_
