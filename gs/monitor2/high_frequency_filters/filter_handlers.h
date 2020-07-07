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

// High Frequency Filters for the Webmonitor.
//
// Processes incoming messages and stores filtered data to provide
// to the webmonitor server.

#ifndef GS_MONITOR2_HIGH_FREQUENCY_FILTERS_FILTER_HANDLERS_H_
#define GS_MONITOR2_HIGH_FREQUENCY_FILTERS_FILTER_HANDLERS_H_
#define MIN_WIND_SPEED_FOR_EFFECTIVE_GUST 3.0

#include "gs/monitor2/high_frequency_filters/filter_types.h"

// Declare all of the filter initializers.
void InitFilterFrequencyState(FilterState *filter_state,
                              FilteredData *filtered_data);
void InitMergeTetherDownState(FilterState *filter_state,
                              FilteredData *filtered_data);
void InitWindGustState(FilterState *filter_state, FilteredData *filtered_data);
void InitLoopCountState(FilterState *filter_state, FilteredData *filtered_data);

// Declare all of the filter handlers.
void FilterFrequencyHandler(FilterState *filter_state,
                            FilteredData *filtered_data);
void MergeTetherDownHandler(FilterState *filter_state,
                            FilteredData *filtered_data);
void WindGustHandler(FilterState *filter_state, FilteredData *filtered_data);
void LoopCountHandler(FilterState *filter_state, FilteredData *filtered_data);

#endif  // GS_MONITOR2_HIGH_FREQUENCY_FILTERS_FILTER_HANDLERS_H_
