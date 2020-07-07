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

#include "avionics/firmware/cpu/adc_test.h"

#include <stdint.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/adc.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/test/test.h"

#define ADC_MODULE 1

static void TestSetup(void) {
  AdcInit(ADC_MODULE);
}

static void TestTeardown(void) {
}

static void TestCalibrate(void) {
  float scale = AdcCalibrate(ADC_MODULE);
  EXPECT_NEAR(1.0f, scale, 0.05);

  int32_t offset = SignExtend(ADC(ADC_MODULE).ALR.ADCALR, ADC_RESOLUTION);
  EXPECT_NEAR(0.0f, (float)offset, (float)(1 << 2));
}

static const TestConfig kAdcTests[] = {
  TEST_CONFIG_INIT(TestCalibrate, 10000),
};

const TestSuite kAdcTest = TEST_SUITE_INIT(kAdcTests, TestSetup, TestTeardown);
