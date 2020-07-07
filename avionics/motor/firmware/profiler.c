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

#include "avionics/motor/firmware/profiler.h"

#include <signal.h>
#include <stdint.h>

#include "avionics/common/motor_profiler_types.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/common/fast_math/fast_math.h"
#include "avionics/common/fast_math/filter.h"
#include "common/barrier.h"

typedef struct {
  uint32_t isr_start;
  uint32_t loop_start;
  uint32_t netpoll_count;
  ProfilerOutput output[2];
  FirstOrderFilterState isr_filter_state;
  FirstOrderFilterState loop_filter_state;
  FirstOrderFilterState netpoll_filter_state;
  FirstOrderFilterParams isr_filter_params;
  FirstOrderFilterParams loop_filter_params;
  FirstOrderFilterParams netpoll_filter_params;
} ProfilerState;

static ProfilerState g_state;
static volatile sig_atomic_t g_output_ind = 0;

void ProfilerInit(void) {
  uint32_t now = Clock32GetCycles();

  g_state.isr_start = now;
  g_state.loop_start = now;
  g_state.netpoll_count = 0;

  for (uint32_t i = 0; i < 2; ++i) {
    g_state.output[i].isr_mean = 0;
    g_state.output[i].isr_max = -INFINITY;
    g_state.output[i].isr_min = INFINITY;
    g_state.output[i].loop_mean = 0;
    g_state.output[i].loop_max = -INFINITY;
    g_state.output[i].loop_min = INFINITY;
    g_state.output[i].netpoll_mean = 0;
    g_state.output[i].netpoll_max = -INFINITY;
    g_state.output[i].netpoll_min = INFINITY;
  }

  // Initialize for 10Hz first-order filter.
  // ISR runs at 15kHz.  Loop and netpoll run at 1kHz.
  FirstOrderFilterInit(1.0f, -2.0f * 15e3f, -2.0f * PI_F * 10.0f, 1.0f / 15e3f,
                       0.0f, &g_state.isr_filter_params,
                       &g_state.isr_filter_state);
  FirstOrderFilterInit(1.0f, -2.0f * 1e3f, -2.0f * PI_F * 10.0f, 1.0f / 1e3f,
                       0.0f, &g_state.loop_filter_params,
                       &g_state.loop_filter_state);
  FirstOrderFilterInit(1.0f, -2.0f * 1e3f, -2.0f * PI_F * 10.0f, 1.0f / 1e3f,
                       0.0f, &g_state.netpoll_filter_params,
                       &g_state.netpoll_filter_state);
}

void ProfilerIsrTic(void) {
  g_state.isr_start = Clock32GetCycles();
}

void ProfilerIsrToc(void) {
  int32_t delta_cycles = CLOCK32_SUBTRACT(Clock32GetCycles(),
                                          g_state.isr_start);
  float delta = CLOCK32_CYCLES_TO_USEC_F(delta_cycles);

  // Cancel time spent in ISR from loop delta.
  g_state.loop_start += delta_cycles;

  ProfilerOutput *output = &g_state.output[g_output_ind];

  output->isr_mean = FirstOrderFilter(delta, &g_state.isr_filter_params,
                                      &g_state.isr_filter_state);
  output->isr_max = Maxf(output->isr_max, delta);
  output->isr_min = Minf(output->isr_min, delta);
}

void ProfilerLoopTic(void) {
  g_state.loop_start = Clock32GetCycles();
}

void ProfilerLoopToc(void) {
  float delta = CLOCK32_CYCLES_TO_USEC_F(CLOCK32_SUBTRACT(Clock32GetCycles(),
                                                          g_state.loop_start));

  ProfilerOutput *output = &g_state.output[g_output_ind];

  output->loop_mean = FirstOrderFilter(delta, &g_state.loop_filter_params,
                                       &g_state.loop_filter_state);
  output->loop_max = Maxf(output->loop_max, delta);
  output->loop_min = Minf(output->loop_min, delta);
}

void ProfilerNetPollCount(void) {
  g_state.netpoll_count++;
}

void ProfilerNetPollCountSample(void) {
  ProfilerOutput *output = &g_state.output[g_output_ind];

  output->netpoll_mean = FirstOrderFilter((float)g_state.netpoll_count,
                                          &g_state.netpoll_filter_params,
                                          &g_state.netpoll_filter_state);
  output->netpoll_max = Maxf(output->netpoll_max,
                             (float)g_state.netpoll_count);
  output->netpoll_min = Minf(output->netpoll_min,
                             (float)g_state.netpoll_count);

  g_state.netpoll_count = 0;
}

ProfilerOutput *ProfilerGetOutput(void) {
  volatile sig_atomic_t next_ind = (g_output_ind + 1) % 2;

  g_state.output[next_ind].isr_max = -INFINITY;
  g_state.output[next_ind].isr_min = INFINITY;
  g_state.output[next_ind].loop_max = -INFINITY;
  g_state.output[next_ind].loop_min = INFINITY;
  g_state.output[next_ind].netpoll_max = -INFINITY;
  g_state.output[next_ind].netpoll_min = INFINITY;

  volatile sig_atomic_t old_ind = g_output_ind;

  MemoryBarrier();
  g_output_ind = next_ind;

  return &g_state.output[old_ind];
}
