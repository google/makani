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

#ifndef CONTROL_CROSSWIND_CROSSWIND_PLAYBOOK_TYPES_H_
#define CONTROL_CROSSWIND_CROSSWIND_PLAYBOOK_TYPES_H_

#define CROSSWIND_SCHEDULE_TABLE_LENGTH 50
typedef struct {
  double wind_speed;
  double alpha_lookup[CROSSWIND_SCHEDULE_TABLE_LENGTH];
  double beta_lookup[CROSSWIND_SCHEDULE_TABLE_LENGTH];
  double airspeed_lookup[CROSSWIND_SCHEDULE_TABLE_LENGTH];
  double transout_airspeed_lookup[CROSSWIND_SCHEDULE_TABLE_LENGTH];
  double path_radius_target;
  double elevation;
  double azi_offset;
  double lookup_loop_angle[CROSSWIND_SCHEDULE_TABLE_LENGTH];
} PlaybookEntry;

#define NUM_PLAYBOOK_ENTRIES 15
typedef struct {
  int num_entries;
  PlaybookEntry entries[NUM_PLAYBOOK_ENTRIES];
} Playbook;

typedef struct {
  PlaybookEntry entry;
  double crossfade_rate;
  double crossfade_throttle;
} PlaybookFallbackParams;

#endif  // CONTROL_CROSSWIND_CROSSWIND_PLAYBOOK_TYPES_H_
