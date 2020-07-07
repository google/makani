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

#include "avionics/firmware/drivers/hsc.h"

#include <assert.h>
#include <float.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/drivers/hsc_model.h"
#include "common/macros.h"

#define HSC_SPI             3
#define HSC_SPI_FREQ_HZ     800000  // 50 kHz <= freq <= 800 kHz.
#define HSC_SPI_WDELAY_NS   2000    // Minimum of 2 usec.
#define HSC_SPI_CHARLEN     8
#define HSC_TRANSFER_LENGTH 4
#define HSC_PRESSURE_COUNTS (1 << 14)

#define PASCALS_PER_PSI 6894.75729f
#define PASCALS_PER_MBAR 100.0f

typedef struct {
  float pressure_min;  // [Pa]
  float pressure_max;  // [Pa]
  float output_min;    // [counts]
  float output_max;    // [counts]
} HscConfig;

static const HscConfig kConfig[] = {
  [kHscModelNumber001Pdsa3] = {
    .pressure_min = -1.0f * PASCALS_PER_PSI,
    .pressure_max = 1.0f * PASCALS_PER_PSI,
    .output_min = 0.10f * HSC_PRESSURE_COUNTS,
    .output_max = 0.90f * HSC_PRESSURE_COUNTS,
  },
  [kHscModelNumber015Pasa3] = {
    .pressure_min = 0.0f,
    .pressure_max = 15.0f * PASCALS_PER_PSI,
    .output_min = 0.10f * HSC_PRESSURE_COUNTS,
    .output_max = 0.90f * HSC_PRESSURE_COUNTS,
  },
  [kHscModelNumber006Mdsa3] = {
    .pressure_min = -6.0f * PASCALS_PER_MBAR,
    .pressure_max = 6.0f * PASCALS_PER_MBAR,
    .output_min = 0.10f * HSC_PRESSURE_COUNTS,
    .output_max = 0.90f * HSC_PRESSURE_COUNTS,
  },
};

static int32_t g_device = 0;

// Flight computer revision AVCC00000BB adds a SN74LV4051A decoder that decodes
// three chip select lines into eight chip select lines. Prior flight computer
// revisions are not supported.

// Specify chip select (nCS) inputs to the SN74LV4051A decoder.
#define PITOT_A0_CS 1
#define PITOT_A1_CS 4
#define PITOT_A2_CS 5
#define PITOT_A(X) ((((X) >> 0) & 0x01) << PITOT_A0_CS          \
                    | (((X) >> 1) & 0x01) << PITOT_A1_CS        \
                    | (((X) >> 2) & 0x01) << PITOT_A2_CS)

// Specify SN74LV4051A decoder outputs for nCS. Output Y7 asserts when all
// input signals (SPI chip selects) are high, thus we cannot use it as a
// valid pitot chip select.
#define PITOT_Y(X) PITOT_A(~(X) & 0x07)

static const uint8_t kChipSelect[kNumHscDevices] = {
  [kHscDeviceAltitude] = PITOT_Y(2),
  [kHscDevicePitch]    = PITOT_Y(4),
  [kHscDeviceSpeed]    = PITOT_Y(3),
  [kHscDeviceYaw]      = PITOT_Y(5),
};

void HscInit(void) {
  // HSC hardware configuration.
  //
  // TMS570 Pin  HSC Pin     Type
  // ----------  ----------  ----
  // SPI3_CLK    PITOT_SCLK  Out
  // SPI3_MISO   PITOT_MISO  In
  // SPI3_NCS1   PITOT_A0    Out
  // SPI3_NCS4   PITOT_A1    Out
  // SPI3_NCS5   PITOT_A2    Out

  // Configure SPI.
  uint8_t cs = 0x0;
  for (HscDevice dev = 0; dev < kNumHscDevices; ++dev) {
    cs |= kChipSelect[dev];
  }
  g_device = MibSPIAddDevice(HSC_SPI, cs, HSC_SPI_FREQ_HZ, HSC_SPI_WDELAY_NS,
                             false, true, HSC_SPI_CHARLEN, HSC_TRANSFER_LENGTH);

  // Prepare transfer.
  MibSPIWriteConst(HSC_SPI, g_device, 1, HSC_TRANSFER_LENGTH, 0);
}

static void SelectDevice(HscDevice dev) {
  assert(kHscDeviceInvalid < dev && dev < kNumHscDevices);

  // Reconfigure SPI chip select.
  MibSPISetChipSelect(HSC_SPI, g_device, kChipSelect[dev], true, 0,
                      HSC_TRANSFER_LENGTH);
}

// Trigger SPI transfer.
bool HscTrigger(HscDevice dev) {
  assert(kHscDeviceInvalid < dev && dev < kNumHscDevices);

  SelectDevice(dev);
  return MibSPITriggerBySoftware(HSC_SPI, g_device);
}

static void ParseData(const uint8_t *in, HscData *out) {
  uint16_t a16, b16;
  int32_t o = 0;
  o += ReadUint16Be(&in[o], &a16);
  out->status = (a16 >> 14) & 0x3;
  out->pressure = a16 & 0x3FFF;
  o += ReadUint16Be(&in[o], &b16);
  out->temperature = b16 >> 5;
  out->invalid = (a16 == 0xFFFF && b16 == 0xFFFF);
  assert(o == HSC_TRANSFER_LENGTH);
}

// Read result of SPI transfer.
bool HscRead(HscData *data) {
  assert(data != NULL);

  uint8_t miso[HSC_TRANSFER_LENGTH];
  if (MibSPIReadUint8(HSC_SPI, g_device, 1, HSC_TRANSFER_LENGTH, miso)) {
    ParseData(miso, data);
    return true;
  }
  return false;
}

float HscPressureRawToPa(HscModel model, const HscData *raw) {
  assert(0 <= model && model < ARRAYSIZE(kConfig));

  // See Honeywell HSC SPI communications datasheet, equation 2.
  float out = (float)raw->pressure;
  float out_max = kConfig[model].output_max;
  float out_min = kConfig[model].output_min;
  float p_max = kConfig[model].pressure_max;
  float p_min = kConfig[model].pressure_min;
  assert(out_max - out_min > FLT_EPSILON);

  return (out - out_min) * (p_max - p_min) / (out_max - out_min) + p_min;
}

float HscTemperatureRawToC(const HscData *raw) {
  // See Honeywell HSC SPI communications datasheet, equation 3.
  return 200.0f * (float)raw->temperature / 2047.0f - 50.0f;
}

// Blocking function.
void HscSample(HscDevice dev, HscData *data) {
  while (!HscTrigger(dev)) {}
  while (!HscRead(data)) {}
}
