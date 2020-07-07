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

#include "avionics/firmware/cpu/io_test.h"

#include "avionics/firmware/cpu/dcan.h"
#include "avionics/firmware/cpu/gio.h"
#include "avionics/firmware/cpu/io.h"
#include "avionics/firmware/cpu/n2het.h"
#include "avionics/firmware/cpu/spi.h"
#include "avionics/firmware/test/test.h"

static void TestSetup(void) {
  IoInit();
}

static void TestTeardown(void) {
}

static void TestIoPinIsUnmappedPin(void) {
  // Set to zero to match designated initializer default.
  EXPECT_EQ(0, kIoUnmappedPin);

  EXPECT_TRUE(IoPinIsUnmappedPin(kIoUnmappedPin));
  EXPECT_FALSE(IoPinIsUnmappedPin(kIoDcan1PinTx));
}

static void TestIoPinIsDcanPin(void) {
  EXPECT_TRUE(IoPinIsDcanPin(kIoDcan1PinRx));
  EXPECT_TRUE(IoPinIsDcanPin(kIoDcan1PinTx));
  EXPECT_TRUE(IoPinIsDcanPin(kIoDcan2PinRx));
  EXPECT_TRUE(IoPinIsDcanPin(kIoDcan2PinTx));
  EXPECT_TRUE(IoPinIsDcanPin(kIoDcan3PinTx));
  EXPECT_TRUE(IoPinIsDcanPin(kIoDcan3PinRx));

  EXPECT_FALSE(IoPinIsDcanPin(kIoGioPinA0));
  EXPECT_FALSE(IoPinIsDcanPin(kIoN2het2Pin0));
  EXPECT_FALSE(IoPinIsDcanPin(kIoSpi3PinMosi0));
}

static void TestIoPinIsGioPin(void) {
  EXPECT_TRUE(IoPinIsGioPin(kIoGioPinA0));
  EXPECT_TRUE(IoPinIsGioPin(kIoGioPinA7));
  EXPECT_TRUE(IoPinIsGioPin(kIoGioPinB0));
  EXPECT_TRUE(IoPinIsGioPin(kIoGioPinB7));

  EXPECT_FALSE(IoPinIsGioPin(kIoDcan1PinRx));
  EXPECT_FALSE(IoPinIsGioPin(kIoN2het2Pin0));
  EXPECT_FALSE(IoPinIsGioPin(kIoSpi3PinMosi0));
}

static void TestIoPinIsN2hetPin(void) {
  EXPECT_TRUE(IoPinIsN2hetPin(kIoN2het1Pin0));
  EXPECT_TRUE(IoPinIsN2hetPin(kIoN2het1Pin31));
  EXPECT_TRUE(IoPinIsN2hetPin(kIoN2het2Pin0));
  EXPECT_TRUE(IoPinIsN2hetPin(kIoN2het2Pin31));

  EXPECT_FALSE(IoPinIsN2hetPin(kIoDcan1PinRx));
  EXPECT_FALSE(IoPinIsN2hetPin(kIoGioPinA0));
  EXPECT_FALSE(IoPinIsN2hetPin(kIoSpi3PinMosi0));
}

static void TestIoPinIsSpiPin(void) {
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi1PinClk));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi1PinEna));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi1PinMiso0));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi1PinMosi0));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi1PinScs0));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi1PinScs1));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi1PinScs2));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi1PinScs3));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi1PinScs4));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi1PinScs5));

  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi3PinClk));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi3PinEna));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi3PinMiso0));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi3PinMosi0));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi3PinScs0));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi3PinScs1));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi3PinScs2));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi3PinScs3));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi3PinScs4));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi3PinScs5));

  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi4PinClk));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi4PinEna));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi4PinMiso0));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi4PinMosi0));
  EXPECT_TRUE(IoPinIsSpiPin(kIoSpi4PinScs0));

  EXPECT_FALSE(IoPinIsSpiPin(kIoDcan1PinRx));
  EXPECT_FALSE(IoPinIsSpiPin(kIoGioPinA0));
  EXPECT_FALSE(IoPinIsSpiPin(kIoN2het2Pin0));
}

static void TestIoPinToDcanPin(void) {
  EXPECT_EQ(kDcan1PinRx, IoPinToDcanPin(kIoDcan1PinRx));
  EXPECT_EQ(kDcan1PinTx, IoPinToDcanPin(kIoDcan1PinTx));
  EXPECT_EQ(kDcan2PinRx, IoPinToDcanPin(kIoDcan2PinRx));
  EXPECT_EQ(kDcan2PinTx, IoPinToDcanPin(kIoDcan2PinTx));
  EXPECT_EQ(kDcan3PinRx, IoPinToDcanPin(kIoDcan3PinRx));
  EXPECT_EQ(kDcan3PinTx, IoPinToDcanPin(kIoDcan3PinTx));
}

static void TestIoPinToGioPin(void) {
  EXPECT_EQ(kGioPinA0, IoPinToGioPin(kIoGioPinA0));
  EXPECT_EQ(kGioPinA7, IoPinToGioPin(kIoGioPinA7));
  EXPECT_EQ(kGioPinB0, IoPinToGioPin(kIoGioPinB0));
  EXPECT_EQ(kGioPinB7, IoPinToGioPin(kIoGioPinB7));
}

static void TestIoPinToN2hetPin(void) {
  EXPECT_EQ(kN2het1Pin0, IoPinToN2hetPin(kIoN2het1Pin0));
  EXPECT_EQ(kN2het1Pin31, IoPinToN2hetPin(kIoN2het1Pin31));
  EXPECT_EQ(kN2het2Pin0, IoPinToN2hetPin(kIoN2het2Pin0));
  EXPECT_EQ(kN2het2Pin31, IoPinToN2hetPin(kIoN2het2Pin31));
}

static void TestIoPinToSpiPin(void) {
  EXPECT_EQ(kSpi1PinClk, IoPinToSpiPin(kIoSpi1PinClk));
  EXPECT_EQ(kSpi1PinEna, IoPinToSpiPin(kIoSpi1PinEna));
  EXPECT_EQ(kSpi1PinMiso0, IoPinToSpiPin(kIoSpi1PinMiso0));
  EXPECT_EQ(kSpi1PinMosi0, IoPinToSpiPin(kIoSpi1PinMosi0));
  EXPECT_EQ(kSpi1PinScs0, IoPinToSpiPin(kIoSpi1PinScs0));
  EXPECT_EQ(kSpi1PinScs1, IoPinToSpiPin(kIoSpi1PinScs1));
  EXPECT_EQ(kSpi1PinScs2, IoPinToSpiPin(kIoSpi1PinScs2));
  EXPECT_EQ(kSpi1PinScs3, IoPinToSpiPin(kIoSpi1PinScs3));
  EXPECT_EQ(kSpi1PinScs4, IoPinToSpiPin(kIoSpi1PinScs4));
  EXPECT_EQ(kSpi1PinScs5, IoPinToSpiPin(kIoSpi1PinScs5));

  EXPECT_EQ(kSpi3PinClk, IoPinToSpiPin(kIoSpi3PinClk));
  EXPECT_EQ(kSpi3PinEna, IoPinToSpiPin(kIoSpi3PinEna));
  EXPECT_EQ(kSpi3PinMiso0, IoPinToSpiPin(kIoSpi3PinMiso0));
  EXPECT_EQ(kSpi3PinMosi0, IoPinToSpiPin(kIoSpi3PinMosi0));
  EXPECT_EQ(kSpi3PinScs0, IoPinToSpiPin(kIoSpi3PinScs0));
  EXPECT_EQ(kSpi3PinScs1, IoPinToSpiPin(kIoSpi3PinScs1));
  EXPECT_EQ(kSpi3PinScs2, IoPinToSpiPin(kIoSpi3PinScs2));
  EXPECT_EQ(kSpi3PinScs3, IoPinToSpiPin(kIoSpi3PinScs3));
  EXPECT_EQ(kSpi3PinScs4, IoPinToSpiPin(kIoSpi3PinScs4));
  EXPECT_EQ(kSpi3PinScs5, IoPinToSpiPin(kIoSpi3PinScs5));

  EXPECT_EQ(kSpi4PinClk, IoPinToSpiPin(kIoSpi4PinClk));
  EXPECT_EQ(kSpi4PinEna, IoPinToSpiPin(kIoSpi4PinEna));
  EXPECT_EQ(kSpi4PinMiso0, IoPinToSpiPin(kIoSpi4PinMiso0));
  EXPECT_EQ(kSpi4PinMosi0, IoPinToSpiPin(kIoSpi4PinMosi0));
  EXPECT_EQ(kSpi4PinScs0, IoPinToSpiPin(kIoSpi4PinScs0));
}

static void TestIoConfigurePinAsOutputPushPull(void) {
  // This test may fail if an external device shorts the pin to ground or Vcc.
  // Configure pin as an input after test in case we're driving something we
  // should not drive.
  // TODO: Evaluate best pins for each board.
  IoConfigureAsOutputPushPull(kIoDcan1PinTx, 0);
  EXPECT_EQ(0, IoGetValue(kIoDcan1PinTx));
  IoConfigureAsOutputPushPull(kIoDcan1PinTx, 1);
  EXPECT_EQ(1, IoGetValue(kIoDcan1PinTx));
  IoConfigureAsInput(kIoDcan1PinTx);

  IoConfigureAsOutputPushPull(kIoGioPinA0, 0);
  EXPECT_EQ(0, IoGetValue(kIoGioPinA0));
  IoConfigureAsOutputPushPull(kIoGioPinA0, 1);
  EXPECT_EQ(1, IoGetValue(kIoGioPinA0));
  IoConfigureAsInput(kIoGioPinA0);

  IoConfigureAsOutputPushPull(kIoN2het1Pin0, 0);
  EXPECT_EQ(0, IoGetValue(kIoN2het1Pin0));
  IoConfigureAsOutputPushPull(kIoN2het1Pin0, 1);
  EXPECT_EQ(1, IoGetValue(kIoN2het1Pin0));
  IoConfigureAsInput(kIoN2het1Pin0);

  IoConfigureAsOutputPushPull(kIoSpi3PinClk, 0);
  EXPECT_EQ(0, IoGetValue(kIoSpi3PinClk));
  IoConfigureAsOutputPushPull(kIoSpi3PinClk, 1);
  EXPECT_EQ(1, IoGetValue(kIoSpi3PinClk));
  IoConfigureAsInput(kIoSpi3PinClk);
}

static const TestConfig kIoTests[] = {
  TEST_CONFIG_INIT(TestIoPinIsUnmappedPin, 1000),
  TEST_CONFIG_INIT(TestIoPinIsDcanPin, 1000),
  TEST_CONFIG_INIT(TestIoPinIsGioPin, 1000),
  TEST_CONFIG_INIT(TestIoPinIsN2hetPin, 1000),
  TEST_CONFIG_INIT(TestIoPinIsSpiPin, 1000),
  TEST_CONFIG_INIT(TestIoPinToDcanPin, 1000),
  TEST_CONFIG_INIT(TestIoPinToGioPin, 1000),
  TEST_CONFIG_INIT(TestIoPinToN2hetPin, 1000),
  TEST_CONFIG_INIT(TestIoPinToSpiPin, 1000),
  TEST_CONFIG_INIT(TestIoConfigurePinAsOutputPushPull, 1000),
};

const TestSuite kIoTest = TEST_SUITE_INIT(kIoTests, TestSetup, TestTeardown);
