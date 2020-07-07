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

#include "common/c_math/util/linalg_io.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "common/c_math/linalg.h"

void VecDisp(const Vec *v) {
  printf("[");
  for (int32_t i = 0; i < v->length; ++i) {
    printf("% lf", v->d[i]);
    if (i < v->length - 1) {
      printf(", ");
    }
  }
  printf("]");
}

void VecScan(FILE *f, Vec *v) {
  // Note that we load the file in a column-major order.
  for (int32_t j = 0; j < v->length; ++j) {
    int32_t read = fscanf(f, "%lf", &v->d[j]);
    assert(read == 1);
  }
}

static void MatArrDisp(const double *d, int32_t nr, int32_t nc) {
  printf("[");
  for (int32_t i = 0; i < nr; ++i) {
    for (int32_t j = 0; j < nc; ++j) {
      printf("%lf", d[i * nc + j]);
      if (j < nc - 1) {
        printf(", ");
      } else if (i < nr - 1) {
        printf(";\n");
      }
    }
  }
  printf("]");
}

void MatDisp(const Mat *m) { MatArrDisp(m->d, m->nr, m->nc); }

void MatScan(FILE *f, Mat *m) {
  // Note that we load the file in a column-major order.
  for (int32_t j = 0; j < m->nc; ++j) {
    for (int32_t i = 0; i < m->nr; ++i) {
      int32_t read = fscanf(f, "%lf", &m->d[i * m->nc + j]);
      assert(read == 1);
    }
  }
}
