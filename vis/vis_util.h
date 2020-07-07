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

#ifndef VIS_VIS_UTIL_H_
#define VIS_VIS_UTIL_H_

#include <GL/gl.h>
#include <GL/glu.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"

typedef enum { kVisXyPlane, kVisXzPlane, kVisYzPlane } VisPlaneType;

#ifdef __cplusplus
extern "C" {
#endif

void DrawPoint(const Vec3 *point);
void DrawCoordSystem(double scale);
void DrawCylinder(GLUquadric *quadratic, const Vec3 *pos, double radius,
                  double height, VisPlaneType plane);
void DrawQuad(const Vec3 *v0, const Vec3 *v1, const Vec3 *v2, const Vec3 *v3);

// Draws a rectangular cuboid with a given center and dimensions.
void DrawRectangularCuboid(const Vec3 *center, const Vec3 *dimensions);

// Draws a line between two points.
void DrawLine(const Vec3 *start, const Vec3 *end);

// Draws a circle specified by a center, radius, and normal vector.
void DrawCircle(const Vec3 *center, double radius, const Vec3 *normal);

void DcmToGlMatrixf(const Mat3 *dcm, GLfloat m[16]);

void CalcNormals(int num_vertices, const Vec3 *vertices, Vec3 *normals);

void InitModel(int num_vertices, Vec3 *vertices, GLuint *GLList);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // VIS_VIS_UTIL_H_
