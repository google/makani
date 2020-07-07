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

#include <math.h>

#include "control/crosswind/crosswind_playbook.h"
#include "control/crosswind/crosswind_playbook_types.h"
#include "control/system_params.h"
#include "control/system_types.h"

#include "common/c_math/util.h"
#include "common/c_math/vec3.h"

// Returns an azimuth offset from the path center for the edges of the loop,
// taking into account the effect of elevation. Half cone angle is the angle
// from an edge of a circular path to the path center.
//
// At high elevations, the assuption that the left and right side of the
// path are the furthest azimuths is increasingly not true, but is a
// reasonable assumption for typical elevations.
double AdjustHalfConeAziOffsetForElevation(const double half_cone_angle,
                                           const double elevation) {
  return atan2(sin(half_cone_angle), cos(elevation) * cos(half_cone_angle));
}

static void CrossfadePlaybookEntries(const PlaybookEntry *pb_entry_lower,
                                     const PlaybookEntry *pb_entry_upper,
                                     double x, double x_lower, double x_upper,
                                     PlaybookEntry *pb_entry_interp) {
  assert(pb_entry_lower != NULL && pb_entry_upper != NULL);
  assert(pb_entry_interp != NULL);

  // Check to make sure lookup loop angles are the same, then copy
  for (int i = 0; i < CROSSWIND_SCHEDULE_TABLE_LENGTH; i++) {
    assert(pb_entry_lower->lookup_loop_angle[i] ==
           pb_entry_upper->lookup_loop_angle[i]);
    pb_entry_interp->lookup_loop_angle[i] =
        pb_entry_lower->lookup_loop_angle[i];
  }

  CrossfadeArray(pb_entry_lower->alpha_lookup, pb_entry_upper->alpha_lookup,
                 CROSSWIND_SCHEDULE_TABLE_LENGTH, x, x_lower, x_upper,
                 pb_entry_interp->alpha_lookup);

  CrossfadeArray(pb_entry_lower->beta_lookup, pb_entry_upper->beta_lookup,
                 CROSSWIND_SCHEDULE_TABLE_LENGTH, x, x_lower, x_upper,
                 pb_entry_interp->beta_lookup);

  CrossfadeArray(pb_entry_lower->airspeed_lookup,
                 pb_entry_upper->airspeed_lookup,
                 CROSSWIND_SCHEDULE_TABLE_LENGTH, x, x_lower, x_upper,
                 pb_entry_interp->airspeed_lookup);

  CrossfadeArray(pb_entry_lower->transout_airspeed_lookup,
                 pb_entry_upper->transout_airspeed_lookup,
                 CROSSWIND_SCHEDULE_TABLE_LENGTH, x, x_lower, x_upper,
                 pb_entry_interp->transout_airspeed_lookup);

  pb_entry_interp->path_radius_target =
      Crossfade(pb_entry_lower->path_radius_target,
                pb_entry_upper->path_radius_target, x, x_lower, x_upper);

  pb_entry_interp->elevation =
      Crossfade(pb_entry_lower->elevation, pb_entry_upper->elevation, x,
                x_lower, x_upper);

  pb_entry_interp->azi_offset =
      Crossfade(pb_entry_lower->azi_offset, pb_entry_upper->azi_offset, x,
                x_lower, x_upper);

  // Wind speed is set to zero because it is not necessarily well defined.  A
  // caller should set it to an appropriate value if there is one and a value
  // is needed.
  pb_entry_interp->wind_speed = 0.0;
}

// Given the playbook and a wind speed [m/s], interpolate between playbook
// entries to find the flight parameters for the current wind speed.
static void GetNominalPlaybookEntry(const Playbook *playbook, double wind_speed,
                                    PlaybookEntry *pb_entry_interp) {
  assert(playbook != NULL);
  assert(pb_entry_interp != NULL);
  const PlaybookEntry *pb_entry_lower, *pb_entry_upper;

  int wind_lower_index = 0, wind_upper_index = 0;
  for (int i = 0; i < playbook->num_entries; i++) {
    if (wind_speed >= playbook->entries[i].wind_speed) {
      wind_lower_index = i;
      wind_upper_index = i + 1;
    }
  }

  if (wind_upper_index >= playbook->num_entries) {
    wind_upper_index = wind_lower_index;
  }

  double wind_lower = playbook->entries[wind_lower_index].wind_speed;
  double wind_upper = playbook->entries[wind_upper_index].wind_speed;

  pb_entry_lower = &playbook->entries[wind_lower_index];
  pb_entry_upper = &playbook->entries[wind_upper_index];

  CrossfadePlaybookEntries(pb_entry_lower, pb_entry_upper, wind_speed,
                           wind_lower, wind_upper, pb_entry_interp);

  pb_entry_interp->wind_speed = wind_speed;
}

void GetPlaybookEntry(const Playbook *playbook, const PlaybookEntry *fallback,
                      double wind_speed, double fallback_crossfade,
                      PlaybookEntry *pb_entry_interp) {
  assert(playbook != NULL);
  assert(pb_entry_interp != NULL);

  PlaybookEntry pb_entry_wind_interp;
  if (fallback_crossfade == 0.0) {
    GetNominalPlaybookEntry(playbook, wind_speed, pb_entry_interp);
  } else {
    assert(fallback != NULL);
    GetNominalPlaybookEntry(playbook, wind_speed, &pb_entry_wind_interp);
    CrossfadePlaybookEntries(&pb_entry_wind_interp, fallback,
                             fallback_crossfade, 0.0, 1.0, pb_entry_interp);
    pb_entry_interp->wind_speed = wind_speed;
  }
}

// Returns a crosswind path center azimuth such that the entire path will not
// violate azimuth limits.
double GetPlaybookEntryAzimuthWithLimits(double wind_dir,
                                         const PlaybookEntry *playbook_entry) {
  double azi_path_center = Wrap(wind_dir + playbook_entry->azi_offset, -PI, PI);

  double half_loop_angle =
      Asin(playbook_entry->path_radius_target / g_sys.tether->length);

  double azi_allow_start = GetSystemParams()->test_site_params.azi_allow_start;
  double azi_allow_end = GetSystemParams()->test_site_params.azi_allow_end;

  // Make sure the allowable size is large enough for full path to fit.
  assert(GetSystemParams()->test_site_params.azi_no_go_size <=
         2.0 * PI - 2.0 * half_loop_angle);

  // We step the half_cone from 0 to half_loop_angle, and clip at each step.
  // This slowly kicks the center out of the no-go zone to the closest side, and
  // handles edge cases where the full half_loop could straddle the no-go zone,
  // or one side is closer to the opposite edge of the no-go.
  int32_t num_steps =
      (int32_t)ceil(half_loop_angle / (MIN_AZI_NO_GO_SIZE / 2.0));
  double step_size = half_loop_angle / num_steps;

  for (int32_t i = 0; i <= num_steps; ++i) {
    double half_angle_temp = i * step_size;

    double azi_half_offset = AdjustHalfConeAziOffsetForElevation(
        half_angle_temp, playbook_entry->elevation);

    double azi_left = Wrap(azi_path_center - azi_half_offset, -PI, PI);
    double azi_right = Wrap(azi_path_center + azi_half_offset, -PI, PI);

    double azi_left_clip =
        SaturateWrapped(azi_left, azi_allow_start, azi_allow_end, -PI, PI);
    double azi_right_clip =
        SaturateWrapped(azi_right, azi_allow_start, azi_allow_end, -PI, PI);

    // Figure out which side is clipped the most (ie, furthest in the no-go
    // zone, if both of them are) and move the path center by that amount.
    double azi_adjust;
    if (fabs(Wrap(azi_left - azi_left_clip, -PI, PI)) >=
        fabs(Wrap(azi_right - azi_right_clip, -PI, PI))) {
      azi_adjust = azi_left - azi_left_clip;
    } else {
      azi_adjust = azi_right - azi_right_clip;
    }

    azi_path_center = Wrap(azi_path_center - azi_adjust, -PI, PI);
  }

  return azi_path_center;
}
