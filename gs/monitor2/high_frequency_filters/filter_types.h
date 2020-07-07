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

// Filter Types.
//
// This file defines the Filter enum, the FilteredData struct, and the
// return types for each of the filters.

#ifndef GS_MONITOR2_HIGH_FREQUENCY_FILTERS_FILTER_TYPES_H_
#define GS_MONITOR2_HIGH_FREQUENCY_FILTERS_FILTER_TYPES_H_

#include <stdbool.h>
#include <stdint.h>
#include "common/c_math/filter.h"

#include "avionics/common/avionics_messages.h"
#include "avionics/common/tether_message_types.h"
#include "avionics/network/aio_node.h"

// Filter enum.
// One entry for each filter. The enum value is equal to the index of the
// filter's handler in the global filter_handlers array.
typedef enum {
  kHighFreqFilterForceSigned = -1,
  kHighFreqFilterFilterFrequency,
  kHighFreqFilterMergeTetherDown,
  kHighFreqFilterWindGust,
  kHighFreqFilterLoopCount,
  kNumHighFreqFilters,
} HighFreqFilter;

// Declare the return type of each filter, using a struct with a required field
// 'valid' that indicates whether the data has been updated (vs only containing
// the default values.) The struct may contain any number of additional fields
// of arbitrary type.

// Store the past 50 interval times, should correspond to the past 1 second.
#define FILTER_FREQUENCY_WINDOW_SIZE 50

typedef struct {
  CircularAveragingBuffer buffer;
  double intervals[FILTER_FREQUENCY_WINDOW_SIZE];
  int64_t prev_time_us;
} FilterFrequencyState;

// Return type of kHighFreqFilterFilterFrequency.
typedef struct {
  bool valid;
  double frequency;
} FilterFrequencyFilteredData;

// Get wind gust tke over the past 5 minutes, and keep track of average
// sampling rate over past 1 minute.
#define WIND_GUST_WINDOW_SIZE 6000
#define WIND_GUST_INTERVAL_WINDOW_SIZE 1200

typedef struct {
  double desired_period;
  double timeout;
  int64_t prev_sample_time_us;
  int64_t prev_data_time_us;
  double intervals[WIND_GUST_INTERVAL_WINDOW_SIZE];
  double u_samples[WIND_GUST_WINDOW_SIZE];
  double v_samples[WIND_GUST_WINDOW_SIZE];
  double speed_samples[WIND_GUST_WINDOW_SIZE];
  double u_squared[WIND_GUST_WINDOW_SIZE];
  double v_squared[WIND_GUST_WINDOW_SIZE];
  double speed_squared[WIND_GUST_WINDOW_SIZE];
  // A circular averaging buffer to keep track of the time between successive
  // samples of wind velocity. We buffer the past minute of intervals and
  // calculate an average sampling frequency from those intervals.
  CircularAveragingBuffer interval_buffer;
  // A circular averaging buffer for the u and v components of wind velocity,
  // and for the squared values of these components (to make computing the
  // variance more efficient.)
  CircularAveragingBuffer u_buffer, v_buffer, speed_buffer, u_squared_buffer,
      v_squared_buffer, speed_squared_buffer;
  uint16_t sequence;
} WindGustState;

// Return type of kHighFreqFilterWindGust.
typedef struct {
  bool valid;
  double frequency;
  // tke = 'turbulent kinetic energy'
  double tke;
  // turbulence_intensity = std(wind_speed) / avg(wind_speed)
  double turbulence_intensity;
} WindGustFilteredData;

#define NODE_STATUS_EXPIRATION_TIME_US 6000000
#define MAX_NODE_STATUS_NO_UPDATE_COUNT 32

typedef struct {
  TetherDownMessage tether_down;
  TetherCommsStatus comms_status[kNumTetherDownSources];
  bool comms_status_valid[kNumTetherDownSources];
  TetherNodeStatus node_status[kNumAioNodes];
  bool node_status_valid[kNumAioNodes];
  double max_loop_time;
  bool valid;
  // Timestamp of the last valid merged TetherDown message.
  double timestamp_sec;
} MergeTetherDownFilteredData;

typedef struct {
  int64_t latest_node_status_timestamp_us[kNumAioNodes];
  TetherDownMergeState merge_state;
} MergeTetherDownState;

typedef struct {
  double last_angle;
  // Tells whether the recorded loop angle has been initialized in the current
  // crosswind flight, so that calculation of a difference with last_angle
  // is valid.
  bool initialized_in_crosswind;
  // Tells whether the kite has flown a big portion of the flight circle
  // in order to start the loop counting. If false, the kite may have just
  // trans-in'ed before it wraps around the loop angle, and in this case
  // the loop count should not be incremented.
  bool first_loop_section_complete;
} LoopCountState;

// Return type of kHighFreqFilterFilterFrequency.
typedef struct {
  int32_t current_rev_count;
  int32_t total_rev_count;
} LoopCountFilteredData;

typedef struct {
  FilterFrequencyState filter_frequency;
  MergeTetherDownState merge_tether_down;
  WindGustState wind_gust;
  LoopCountState loop_count;
} FilterState;

// FilteredData struct.
// Contains exactly one field for each filter. Each filter defines its own
// return type, and the filter's entry in FilteredData is a pointer to an
// instance of that type.
typedef struct {
  FilterFrequencyFilteredData filter_frequency;
  MergeTetherDownFilteredData merge_tether_down;
  WindGustFilteredData wind_gust;
  LoopCountFilteredData loop_count;
} FilteredData;

#endif  // GS_MONITOR2_HIGH_FREQUENCY_FILTERS_FILTER_TYPES_H_
