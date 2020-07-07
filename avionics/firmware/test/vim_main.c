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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/network/switch_info.h"
#include "common/macros.h"

#define RUN_TIME_CYCLES CLOCK32_SEC_TO_CYCLES(10)

/* This file measures the latency between an event and handling of that event
 * in an interrupt function. We configure ePWM2 to count up and produce an
 * interrupt when the timer rolls over to 0. We then sample the timer upon
 * execution of the interrupt handler. The value of the timer represents the
 * latency since we're sampling the number of cycles since 0 (the time of
 * interrupt). We expect the interrupt latency to be around 800--2000 ns
 * depending on the IRQ/FIQ trampoline.
 *
 * This file also confirms that FIQ interrupt functions preempt IRQ interrupt
 * functions. When the ePWM interrupt handler executes, it triggers an ADC
 * start of conversion and then waits for the end of conversion. The FIQ
 * interrupt should execute the ADC interrupt handler while the IRQ interrupt
 * function waits.
 */

struct {
  int32_t sum;
  int32_t count;
  uint32_t max;
  bool preempted;
} g_epwm;

struct {
  int32_t count;
  volatile bool ran;
} g_adc;

static void EPwmHandler(void) {
  // Compute average and maximum latency.
  uint32_t sample = EPWM(2).TBCTR.raw;
  g_epwm.sum += sample;
  if (sample > g_epwm.max) {
    g_epwm.max = sample;
  }
  ++g_epwm.count;

  // The ADC EOC interrupt should preempt this interrupt routine.
  g_adc.ran = false;
  ADC(1).G1SEL.G1_SEL = 0x1;
  while (!ADC(1).G1SR.G1_END && !g_adc.ran) {}
  g_epwm.preempted = g_adc.ran;

  // Clear ePWM interrupt.
  EPWM(2).ETCLR.raw = EPWM_ETCLR_INT;
}

static void AdcHandler(void) {
  g_adc.ran = true;
  ++g_adc.count;
  ADC(1).G1INTFLG.raw = ADC_G1INTFLG_G1_END;
}

static void StartPwm(void) {
  // Synchronize all time bases (TBCLKSYNC).
  IommSetPinmux(37, 1);

  // Enable eTPWM2A clock via PINMMR37[16].
  IommSetPinmux(37, 16);

  // Disable while configuring.
  EPWM(2).TBCTL.CTRMODE = 3;

  // Disable ePWM interrupts.
  EPWM(2).ETSEL.INTEN = 0;
  EPWM(2).TZEINT.raw = 0x0;

  // Scale TBCLK relative to system clock (VCLK4) at 80 MHz.
  EPWM(2).TBCTL.CLKDIV = 0;
  EPWM(2).TBCTL.HSPCLKDIV = 0;
  EPWM(2).TBCTL.FREE_SOFT = 2;

  // Configure PWM period.
  EPWM(2).TBPRD.TBPRD = UINT16_MAX;
  EPWM(2).CMPA.CMPA = EPWM(2).TBPRD.TBPRD / 2;  // 50% duty cycle.

  // Mode.
  EPWM(2).TBCTL.PHSEN = 0;     // 0 = Disabled.
  EPWM(2).TBCTL.PRDLD = 0;     // 0 = Shadow mode.
  EPWM(2).TBCTL.SYNCOSEL = 3;  // 3 = Disable.

  // Compare control.
  EPWM(2).CMPCTL.SHDWAMODE = 0;  // Shadow mode.
  EPWM(2).CMPCTL.SHDWBMODE = 0;  // Shadow mode.
  EPWM(2).CMPCTL.LOADAMODE = 0;  // load on CTR = Zero.
  EPWM(2).CMPCTL.LOADBMODE = 0;  // load on CTR = Zero.

  // Action qualifier.
  EPWM(2).AQCTLA.CAU = 1;  // Force EPWM2A output low on CMPA and incrementing.
  EPWM(2).AQCTLA.ZRO = 2;  // Force EPWM2A output high on reload.

  // Trigger interrupt/flag when time-base counter equals CMPA.
  EPWM(2).ETPS.INTPRD = 1;   // 1h = Generate interrupt on first count.
  EPWM(2).ETSEL.INTSEL = 1;  // 1h = Generate interrupt when time-base == 0.
  EPWM(2).ETSEL.INTEN = 1;

  // Clear spurious interrupts.
  EPWM(2).ETCLR.raw = 0xFFFF;
  EPWM(2).TZCLR.raw = 0xFFFF;

  // Enable.
  EPWM(2).TBCTR.TBCTR = 0;
  EPWM(2).TBCTL.CTRMODE = 0;  // 0 = Count up.
}

static void StopPwm(void) {
  // Disable counter.
  EPWM(2).TBCTL.CTRMODE = 3;

  // Disable ePWM interrupts.
  EPWM(2).ETSEL.INTEN = 0;
  EPWM(2).TZEINT.raw = 0x0;
}

static void SetupAdc(void) {
  // Enable peripheral clocks.
  PeripheralEnable(kPeripheralMibAdc1);

  // Reset ADC modules.
  ADC(1).RSTCR.RESET = 1;
  ADC(1).RSTCR.RESET = 0;

  // Enable ADCs state machine in 12-bit mode.
  ADC(1).OPMODECR._10_12_BIT = 1;
  ADC(1).OPMODECR.ADC_EN = 1;

  // Set clock prescale and sampling window.
  ADC(1).CLOCKCR.PS = 2;
  ADC(1).G1SAMP.G1_ACQ = 4;
  ADC(1).G2SAMP.G2_ACQ = 4;

  // Set conversion memory boundaries.
  ADC(1).BNDCR.BNDA = 0;
  ADC(1).BNDCR.BNDB = 1;
  ADC(1).BNDEND.BNDEND = 2;  // 2 = 64 words.

  // Enable interrupts.
  ADC(1).G1INTFLG.G1_END = 1;
  ADC(1).G1INTENA.G1_END_INT_EN = 1;
}

static void RunVim(void) {
  memset(&g_epwm, 0, sizeof(g_epwm));
  memset(&g_adc, 0, sizeof(g_adc));
  SetupAdc();
  VimRegisterIrq(kVimChannelEPwm2, EPwmHandler);
  VimEnableInterrupt(kVimChannelEPwm2);
  VimRegisterFiq(kVimChannelAdc1SwGroup, AdcHandler);
  VimEnableInterrupt(kVimChannelAdc1SwGroup);
  VimEnableIrq();
  VimEnableFiq();
  StartPwm();
  Clock32WaitCycles(RUN_TIME_CYCLES);
  StopPwm();
  VimDisableInterrupt(kVimChannelEPwm2);
  VimDisableInterrupt(kVimChannelAdc1SwGroup);
  VimDisableIrq();
}

int main(void) {
  // Initialize network.
  MibSPIInit(1, kSpiPinmuxAll);
  Bcm53101Init(true);
  // TODO: Remove this delay if/when initialization is asynchronous.
  while (!Bcm53101Ready()) {
    Bcm53101Poll(NULL);
  }
  NetInit(AppConfigGetAioNode());

  while (true) {
    // TODO: Compute actual clock rate.
    float ns_per_tick = 1e9f / 80e6f;  // Assume 80 MHz clock rate.
    float avg_ns, max_ns;

    printf("Running latency test...\n");
    RunVim();

    avg_ns = ns_per_tick * (float)g_epwm.sum / (float)g_epwm.count;
    max_ns = ns_per_tick * g_epwm.max;
    printf("  avg=%g ns, max=%g ns, samples=%ld, preempted=%d, adc count=%ld\n",
           avg_ns, max_ns, g_epwm.count, g_epwm.preempted, g_adc.count);
  }
  return 0;
}
