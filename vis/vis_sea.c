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

#include "vis/vis_sea.h"
#include <math.h>
#include "common/c_math/util.h"
#include "control/system_params.h"
#include "sim/sim_params.h"
#include "vis/vis_util.h"

// Number of segments of the sea to draw in each dimension.
#define N_DRAW_SEGMENTS 100

#define SEA_VERTEX_COUNT (N_DRAW_SEGMENTS * N_DRAW_SEGMENTS)
#define SEA_INDEX_COUNT ((N_DRAW_SEGMENTS - 1) * (N_DRAW_SEGMENTS - 1) * 3 * 2)

void DrawSea(const double *wave_transl_coord, const double *wave_elev_g) {
  double grid_dx;  // Grid spacing in X direction [m].
  double grid_dy;  // Grid spacing in Y direction [m].
  double grid_radius = GetSimParams()->sea_sim.grid_half_length;
  double grid_x_min = -grid_radius / sqrt(2.0);
  double grid_y_min = -grid_radius / sqrt(2.0);
  grid_dx = 2.0 * grid_radius / sqrt(2.0) / N_DRAW_SEGMENTS;
  grid_dy = 2.0 * grid_radius / sqrt(2.0) / N_DRAW_SEGMENTS;
  double wave_heading_g = GetSimParams()->sea_sim.wave_heading_ned -
                          GetSystemParams()->ground_frame.heading;

  float sea_vertices[SEA_VERTEX_COUNT * 3];
  int sea_indices[SEA_INDEX_COUNT];

  int v = 0;
  for (int i = 0; i < N_DRAW_SEGMENTS; i++) {
    for (int j = 0; j < N_DRAW_SEGMENTS; j++) {
      double x = grid_x_min + i * grid_dx;
      double y = grid_y_min + j * grid_dy;
      double h = Interp1(wave_transl_coord, wave_elev_g, SEA_N_GRID_SEGMENTS,
                         x * cos(wave_heading_g) + y * sin(wave_heading_g),
                         kInterpOptionDefault);
      sea_vertices[v++] = (float)x;
      sea_vertices[v++] = (float)y;
      sea_vertices[v++] = (float)h;
    }
  }

  // TODO: The following is only needed once. Move to initialization.
  int d = 0;
  for (int i = 0; i < N_DRAW_SEGMENTS - 1; i++) {
    for (int j = 0; j < N_DRAW_SEGMENTS - 1; j++) {
      sea_indices[d++] = i * N_DRAW_SEGMENTS + j;
      sea_indices[d++] = i * N_DRAW_SEGMENTS + j + 1;
      sea_indices[d++] = (i + 1) * (N_DRAW_SEGMENTS) + j;
      sea_indices[d++] = (i + 1) * (N_DRAW_SEGMENTS) + j;
      sea_indices[d++] = i * N_DRAW_SEGMENTS + j + 1;
      sea_indices[d++] = (i + 1) * (N_DRAW_SEGMENTS) + j + 1;
    }
  }

  glPushMatrix();
  glLineWidth(1.5f);
  glColor4f(0.5f, 0.5f, 0.9f, 0.7f);  // Blu-ish.
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_FLOAT, 0, sea_vertices);
  glDrawElements(GL_TRIANGLES, SEA_INDEX_COUNT, GL_UNSIGNED_INT, sea_indices);
  glDisableClientState(GL_VERTEX_ARRAY);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glPopMatrix();
}
