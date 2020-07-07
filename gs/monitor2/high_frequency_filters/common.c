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

#include <assert.h>
#include <stdint.h>

#include "common/macros.h"
#include "gs/monitor2/high_frequency_filters/common.h"
#include "gs/monitor2/high_frequency_filters/filter_handlers.h"

// Array for initialization callbacks.
static void (*filter_initializers[])(FilterState *, FilteredData *) = {
        [kHighFreqFilterFilterFrequency] = InitFilterFrequencyState,
        [kHighFreqFilterMergeTetherDown] = InitMergeTetherDownState,
        [kHighFreqFilterWindGust] = InitWindGustState,
        [kHighFreqFilterLoopCount] = InitLoopCountState,
};

COMPILE_ASSERT(ARRAYSIZE(filter_initializers) == kNumHighFreqFilters,
               every_high_frequency_filter_must_have_one_initializer);

// Array for tear-down callbacks.
static void (*filter_teardowns[])(FilterState *, FilteredData *) = {
        [kHighFreqFilterFilterFrequency] = NULL,
        [kHighFreqFilterMergeTetherDown] = NULL,
        [kHighFreqFilterWindGust] = NULL, [kHighFreqFilterLoopCount] = NULL,
};

COMPILE_ASSERT(ARRAYSIZE(filter_teardowns) == kNumHighFreqFilters,
               every_high_frequency_filter_must_have_one_teardown_function);

// Array to store the filter handlers. The order of the filter handlers in the
// array is defined by the enum value for that filter.
static void (*filter_handlers[])(FilterState *, FilteredData *) = {
        [kHighFreqFilterFilterFrequency] = FilterFrequencyHandler,
        [kHighFreqFilterMergeTetherDown] = MergeTetherDownHandler,
        [kHighFreqFilterWindGust] = WindGustHandler,
        [kHighFreqFilterLoopCount] = LoopCountHandler,
};

COMPILE_ASSERT(ARRAYSIZE(filter_handlers) == kNumHighFreqFilters,
               every_high_frequency_filter_must_have_one_handler);

// Data structure to store the filtered data.
static FilterState g_filter_state;
static FilteredData g_filtered_data_table;

void InitFilters(void) {
  for (int32_t i = 0; i < kNumHighFreqFilters; ++i) {
    if (filter_initializers[i] != NULL) {
      filter_initializers[i](&g_filter_state, &g_filtered_data_table);
    }
  }
}

void TearDownFilters(void) {
  for (int32_t i = 0; i < kNumHighFreqFilters; ++i) {
    if (filter_teardowns[i] != NULL) {
      filter_teardowns[i](&g_filter_state, &g_filtered_data_table);
    }
  }
}

void RunAllFilters(void) {
  for (int32_t i = 0; i < kNumHighFreqFilters; ++i) {
    assert(filter_handlers[i] != NULL);
    if (filter_handlers[i] != NULL) {
      filter_handlers[i](&g_filter_state, &g_filtered_data_table);
    }
  }
}

void *GetFilteredData(uint32_t *length) {
  *length = sizeof(FilteredData);
  return (void *)&g_filtered_data_table;
}
