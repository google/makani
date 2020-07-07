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

// Common functions for running high frequency filters.

#ifndef GS_MONITOR2_HIGH_FREQUENCY_FILTERS_COMMON_H_
#define GS_MONITOR2_HIGH_FREQUENCY_FILTERS_COMMON_H_

#include <stdint.h>

#include "gs/monitor2/high_frequency_filters/filter_types.h"

// Initialize all of the filter states.
void InitFilters(void);

// Tear down all of the filter states.
void TearDownFilters(void);

// Run all of the filter handlers.
void RunAllFilters(void);

// Returns a pointer to g_filtered_data_table, and puts the size of the
// FilteredData struct in length. The length is used to convert the SWIG
// object to a ctypes object, so it can be used in Python.
void *GetFilteredData(uint32_t *length);

#endif  // GS_MONITOR2_HIGH_FREQUENCY_FILTERS_COMMON_H_
