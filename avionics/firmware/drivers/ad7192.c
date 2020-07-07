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

#include "avionics/firmware/drivers/ad7192.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/io.h"
#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/util/state_machine.h"
#include "common/macros.h"

#define SPI_BUS 3
#define SYNC_PIN kIoGioPinA2

typedef enum {
  kRegAddrComm = 0b000,
  kRegAddrStatus = 0b000,
  kRegAddrMode = 0b001,
  kRegAddrConfig = 0b010,
  kRegAddrData = 0b011,
  kRegAddrId = 0b100,
  kRegAddrGpocon = 0b101,
  kRegAddrOffset = 0b110,
  kRegAddrFullScale = 0b111,
} RegAddr;

static const int8_t kRegSize[] = {
  [kRegAddrComm] = 1,  // kRegAddrStatus is also address 0 and size 1.
  [kRegAddrMode] = 3,
  [kRegAddrConfig] = 3,
  [kRegAddrData] = 4,  // Assume data + status mode.
  [kRegAddrId] = 1,
  [kRegAddrGpocon] = 1,
  [kRegAddrOffset] = 3,
  [kRegAddrFullScale] = 3,
};

typedef union {
  struct {
    unsigned int RDYn:1;
    unsigned int ERR:1;
    unsigned int NOREF:1;
    unsigned int PARITY:1;
    unsigned int :1;
    unsigned int CHD:3;
  };
  uint8_t raw;
} RegStatus;

typedef union {
  struct {
    unsigned int :8;
    unsigned int MD:3;
    unsigned int DAT_STA:1;
    unsigned int CLK:2;
    unsigned int :2;
    unsigned int SINC3:1;
    unsigned int :1;
    unsigned int ENPAR:1;
    unsigned int CLK_DIV:1;
    unsigned int SINGLE:1;
    unsigned int REJ60:1;
    unsigned int FS:10;
  };
  uint32_t raw;
} RegMode;

typedef union {
  struct {
    unsigned int :8;
    unsigned int CHOP:1;
    unsigned int :2;
    unsigned int REFSEL:1;
    unsigned int :4;
    unsigned int CH:8;
    unsigned int BURN:1;
    unsigned int REFDET:1;
    unsigned int :1;
    unsigned int BUF:1;
    unsigned int UBn:1;
    unsigned int G:3;
  };
  uint32_t raw;
} RegConfig;

typedef enum {
  kAd7192ModeContinuousConversion = 0b000,
  kAd7192ModeSingleConversion = 0b001,
  kAd7192ModeIdle = 0b010,
  kAd7192ModePowerDown = 0b011,
  kAd7192ModeInternalZeroCalibration = 0b100,
  kAd7192ModeInternalFullScaleCalibration = 0b101,
  kAd7192ModeSystemZeroCalibration = 0b110,
  kAd7192ModeSystemFullScaleCalibration = 0b111,
} Ad7192Mode;

static Ad7192 *GetDevice(const FsmState *state) {
  assert(GetStateUserData(state) != NULL);
  return (Ad7192 *)GetStateUserData(state);
}

static bool SpiTransferAsync(const Ad7192 *device, int32_t len, void *buf) {
  return MibSPITransferUint8Async(SPI_BUS, device->spi_device,
                                  1, len, buf, buf);
}

static bool AdcRead(const Ad7192 *device, RegAddr reg, uint32_t *value) {
  uint8_t buf[5];
  buf[0] = 0x40 | (reg << 3);
  WriteUint32Be(0xFFFFFFFFl, &buf[1]);
  if (SpiTransferAsync(device, kRegSize[reg] + 1, buf)) {
    ReadUint32Be(&buf[1], value);
    *value >>= (32 - 8 * kRegSize[reg]);
    return true;
  }
  return false;
}

static bool AdcWrite(const Ad7192 *device, RegAddr reg, uint32_t value) {
  uint8_t buf[5];
  buf[0] = (reg << 3);
  WriteUint32Be(value << (32 - 8 * kRegSize[reg]), &buf[1]);
  if (SpiTransferAsync(device, kRegSize[reg] + 1, buf)) {
    return true;
  }
  return false;
}

static bool GetOddParity(uint32_t value) {
  bool parity = false;
  while (value != 0) {
    if (value & 1) parity = !parity;
    value >>= 1;
  }
  return parity;
}

typedef enum {
  kStateInit,
  kStateConfig,
  kStateMode,
  kStateRead,
  kStateReadyWait,
  kStateZeroCal,
  kStateFullCal,
  kStateSync,
} Ad7192State;

static bool StateInit(FsmState *state) {
  Ad7192 *device = GetDevice(state);
  device->new_sample = false;
  memset(&device->sample, 0, sizeof(device->sample));
  device->run_calibration = false;

  // AD7192 hardware configuration.
  //
  // TMS570 Pin  ADIS16488 Pin   Type
  // ----------  --------------  ----------
  // SPI3_CLK    ADC_SCLK        Out
  // SPI3_MISO   ADC_MISO        In
  // SPI3_MOSI   ADC_MOSI        Out
  // SPI3_NCS0   ADC_NCS         Out
  // GIOA[2]     ADC_NSYNC       Out

  if (device->config->configure_sync) {
    IoConfigureAsOutputPushPull(SYNC_PIN, 1);
  }

  // Enable SPI communications.
  device->spi_device = MibSPIAddDevice(SPI_BUS, 1 << device->config->cs_pin,
                                       1e6, 0, true, true, 8, 5);

  return true;
}

static bool VerifyWriteReg(bool first_entry, RegAddr reg, uint32_t value,
                           Ad7192 *device) {
  if (first_entry) device->write_reg = false;
  if (device->write_reg) {
    if (AdcWrite(device, reg, value)) device->write_reg = false;
  } else {
    uint32_t read_value;
    if (AdcRead(device, reg, &read_value)) {
      if (read_value == value) {
        return true;
      } else {
        device->write_reg = true;
      }
    }
  }
  return false;
}

static bool StateConfig(FsmState *state) {
  const Ad7192Config *config = GetDevice(state)->config;
  RegConfig config_reg;
  config_reg.raw = 0;
  config_reg.CHOP = config->chop;
  config_reg.REFSEL = config->ref;
  config_reg.CH = config->input_select;
  config_reg.BURN = config->burn;
  config_reg.REFDET = config->ref_detect;
  config_reg.BUF = config->buffer;
  config_reg.UBn = config->unipolar;
  config_reg.G = config->gain;
  return VerifyWriteReg(state->first_entry, kRegAddrConfig, config_reg.raw,
                        GetDevice(state));
}

static bool StateMode(FsmState *state) {
  const Ad7192Config *config = GetDevice(state)->config;
  RegMode mode_reg;
  mode_reg.raw = 0;
  mode_reg.MD = kAd7192ModeContinuousConversion;
  mode_reg.DAT_STA = 0x1;
  mode_reg.CLK = config->clock;
  mode_reg.SINC3 = config->sinc3;
  mode_reg.ENPAR = 0x1;
  mode_reg.CLK_DIV = config->clock_divide;
  mode_reg.SINGLE = config->single_cycle;
  mode_reg.REJ60 = config->reject_60hz;
  mode_reg.FS = config->filter_rate;
  return VerifyWriteReg(state->first_entry, kRegAddrMode, mode_reg.raw,
                        GetDevice(state));
}

static bool StateReadyWait(FsmState *state) {
  // TODO: Read falling edge from MISO with NCS low and advance state.
  uint32_t data;
  if (!AdcRead(GetDevice(state), kRegAddrStatus, &data)) {
    return false;
  }
  RegStatus status;
  status.raw = data & 0xFF;
  return !status.RDYn;
}

static bool StateRead(FsmState *state) {
  uint32_t data;
  Ad7192 *device = GetDevice(state);
  if (AdcRead(device, kRegAddrData, &data)) {
    RegStatus status;
    status.raw = data & 0xFF;
    int32_t sample = data >> 8;
    if (!status.RDYn) {
      device->new_sample = true;
      device->sample.input = status.CHD;
      device->sample.value_raw = sample;
      device->sample.valid_parity = (status.PARITY == GetOddParity(sample));
      device->sample.error = status.ERR;
    }
    return true;
  }
  return false;
}

static bool StateZeroCal(FsmState *state) {
  // TODO: Implement calibration.
  (void)state;
  return true;
}

static bool StateFullCal(FsmState *state) {
  // TODO: Implement calibration.
  (void)state;
  return true;
}

static bool StateSync(FsmState *state) {
  if (!GetDevice(state)->config->configure_sync) {
    return true;
  }
  if (state->first_entry) {
    IoSetValue(SYNC_PIN, 0);
  }
  if (CLOCK32_GE(Clock32GetCycles(),
                 state->entry_cycles + CLOCK32_USEC_TO_CYCLES(10))) {
    IoSetValue(SYNC_PIN, 1);
    return true;
  }
  return false;
}

static const StateTransition kFsmTrans[] = {
  STATE_TRANS_DEFAULT(kStateInit, StateInit, kStateConfig),
  STATE_TRANS_DEFAULT(kStateConfig, StateConfig, kStateMode),
  STATE_TRANS_DEFAULT(kStateMode, StateMode, kStateReadyWait),
  STATE_TRANS_DEFAULT(kStateReadyWait, StateReadyWait, kStateRead),
  STATE_TRANS_DEFAULT(kStateRead, StateRead, kStateReadyWait),
  STATE_TRANS_DEFAULT(kStateZeroCal, StateZeroCal, kStateFullCal),
  STATE_TRANS_DEFAULT(kStateFullCal, StateFullCal, kStateReadyWait),
  STATE_TRANS_DEFAULT(kStateSync, StateSync, kStateReadyWait),
};

static const StateMachine kFsm = {
  .num_states = ARRAYSIZE(kFsmTrans),
  .init_state = kStateInit,
  .states = kFsmTrans,
};

void Ad7192Init(Ad7192 *device) {
  memset(device, 0, sizeof(*device));
  StateMachineInit(&kFsm, &device->state);
  SetStateUserData(device, &device->state);
}

void Ad7192Poll(const Ad7192Config *config, Ad7192 *device) {
  assert(config != NULL);
  device->config = config;
  if (GetState(&device->state) == kStateReadyWait && device->run_calibration) {
    device->run_calibration = false;
    SetState(kStateZeroCal, &device->state);
  }
  StateMachinePoll(&kFsm, &device->state);
}

bool Ad7192Ready(const Ad7192 *device) {
  int32_t state = GetState(&device->state);
  return state == kStateReadyWait || state == kStateRead;
}

bool Ad7192Read(Ad7192 *device, const Ad7192Data **data) {
  assert(data != NULL);
  if (device->new_sample) {
    device->new_sample = false;
    *data = &device->sample;
    return true;
  }
  return false;
}

bool Ad7192Sync(Ad7192 *device) {
  if (Ad7192Ready(device)) {
    SetState(kStateSync, &device->state);
    device->new_sample = false;
    return true;
  }
  return false;
}

void Ad7192RunCalibration(Ad7192 *device) {
  device->run_calibration = true;
}
