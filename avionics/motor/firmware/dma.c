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

#include "avionics/motor/firmware/dma.h"

#include <stddef.h>

#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/motor/firmware/adc.h"
#include "avionics/motor/firmware/isr.h"
#include "avionics/firmware/serial/motor_serial_params.h"

void MotorDmaInit(MotorHardware motor_controller_type) {
  // Reset DMA module.
  DMA.GCTRL.DMARES = 1;
  DMA.GCTRL.DMARES = 0;

  int32_t num_adc_channels
      = motor_controller_type == kMotorHardwareOzoneA1 ? 5 : 4;

  // Enable DMA Channel 10 for ADC1.
  DMACP(10).ISADDR.ISADDR = (uint32_t)ADCRAM(1).RAM;
  DMACP(10).IDADDR.IDADDR = (uint32_t)&g_adc_1_buffer;
  DMACP(10).ITCOUNT.IFTCOUNT = NUM_ADC_SAMPLES;
  DMACP(10).ITCOUNT.IETCOUNT = num_adc_channels;
  DMACP(10).CHCTRL.CHAIN = 11 + 1;  // Trigger channel 11 next.
  DMACP(10).CHCTRL.RES = 2;    // 32-bit read element size.
  DMACP(10).CHCTRL.WES = 2;    // 32-bit write element size.
  DMACP(10).CHCTRL.TTYPE = 1;  // Hardware request triggers one block transfer.
  DMACP(10).CHCTRL.ADDMR = 1;  // Addressing mode read: post-increment.
  DMACP(10).CHCTRL.ADDMW = 3;  // Addressing mode write: indexed.
  DMACP(10).CHCTRL.AIM = 1;    // Auto-initiation mode is enabled.
  DMACP(10).EIOFF.EIDXD = NUM_ADC_SAMPLES * sizeof(union ADCRAM_RAM);
  DMACP(10).EIOFF.EIDXS = 0;   // Post-increment mode; element size is used.
  DMACP(10).FIOFF.FIDXD = 1 * sizeof(union ADCRAM_RAM);
  DMACP(10).FIOFF.FIDXS = 0;   // Post-increment mode; element size is used.
  DMA.HWCHENAS.HWCHENA10 = 1;  // Enable channel 10 for hardware triggering.
  DMA.PAR1.CH10PA = 4;         // Assign channel 10 to port B.
  DMA.CHPRIOS.CPS10 = 1;       // Assign channel 10 to high priority queue.

  // Enable DMA Channel 11 for ADC2.
  DMACP(11).ISADDR.ISADDR = (uint32_t)ADCRAM(2).RAM;
  DMACP(11).IDADDR.IDADDR = (uint32_t)&g_adc_2_buffer;
  DMACP(11).ITCOUNT.IFTCOUNT = NUM_ADC_SAMPLES;
  DMACP(11).ITCOUNT.IETCOUNT = num_adc_channels;
  DMACP(11).CHCTRL.CHAIN = 0;  // No channel is selected to trigger next.
  DMACP(11).CHCTRL.RES = 2;
  DMACP(11).CHCTRL.WES = 2;
  DMACP(11).CHCTRL.TTYPE = 1;
  DMACP(11).CHCTRL.ADDMR = 1;
  DMACP(11).CHCTRL.ADDMW = 3;
  DMACP(11).CHCTRL.AIM = 1;
  DMACP(11).EIOFF.EIDXD = NUM_ADC_SAMPLES * sizeof(union ADCRAM_RAM);
  DMACP(11).EIOFF.EIDXS = 0;
  DMACP(11).FIOFF.FIDXD = 1 * sizeof(union ADCRAM_RAM);
  DMACP(11).FIOFF.FIDXS = 0;
  DMA.HWCHENAS.HWCHENA11 = 1;
  DMA.PAR1.CH11PA = 4;
  DMA.CHPRIOS.CPS11 = 1;
  DMA.GCHIENAS.GCHIE11 = 1;    // Enable channel 11 for interrupts.
  DMA.BTCINTENAS.BTCINTENA11 = 1;  // Enable channel 11 block transfer int.

  // Setup DMA request from N2HET request line 4 -> DMA request line 20.
  DMA.DREQASI2.CH10ASI = 20;

  // Setup ADC block end interrupt.
  VimRegisterIrq(kVimChannelDmaBtca, MotorIsrControlLoop);
  VimEnableInterrupt(kVimChannelDmaBtca);

  // Enable DMA.
  DMA.GCTRL.DMAEN = 1;
}
