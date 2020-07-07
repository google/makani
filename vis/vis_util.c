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

#include "vis/vis_util.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>
#include <stdint.h>

#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"

void DrawPoint(const Vec3 *point) {
  glPointSize(5.0f);
  glColor4d(1.0, 0.0, 0.0, 0.7);
  glBegin(GL_POINTS);
  glVertex3d(point->x, point->y, point->z);
  glEnd();
}

void DrawCoordSystem(double scale) {
  glLineWidth(3.0f);
  glColor4d(0.0, 0.0, 1.0, 1.0);
  glBegin(GL_LINE_STRIP);
  glVertex3d(0.0, 0.0, 0.0);
  glVertex3d(scale, 0.0, 0.0);
  glEnd();

  glColor4d(0.0, 1.0, 0.0, 1.0);
  glBegin(GL_LINE_STRIP);
  glVertex3d(0.0, 0.0, 0.0);
  glVertex3d(0.0, scale, 0.0);
  glEnd();

  glColor4d(1.0, 0.0, 0.0, 1.0);
  glBegin(GL_LINE_STRIP);
  glVertex3d(0.0, 0.0, 0.0);
  glVertex3d(0.0, 0.0, scale);
  glEnd();
}

void DrawCylinder(GLUquadric *quadratic, const Vec3 *pos, double radius,
                  double height, VisPlaneType plane) {
  glPushMatrix();
  glTranslated(pos->x, pos->y, pos->z);
  switch (plane) {
    default:
    case kVisXyPlane:
      break;
    case kVisXzPlane:
      glRotated(90.0, 0.0, 1.0, 0.0);
      break;
    case kVisYzPlane:
      glRotated(90.0, 1.0, 0.0, 0.0);
      break;
  }
  gluCylinder(quadratic, radius, radius, height, 32, 32);
  glPopMatrix();
}

void DrawQuad(const Vec3 *v0, const Vec3 *v1, const Vec3 *v2, const Vec3 *v3) {
  glBegin(GL_QUADS);
  glVertex3d(v0->x, v0->y, v0->z);
  glVertex3d(v1->x, v1->y, v1->z);
  glVertex3d(v2->x, v2->y, v2->z);
  glVertex3d(v3->x, v3->y, v3->z);
  glEnd();
}

void DrawRectangularCuboid(const Vec3 *center, const Vec3 *dimensions) {
  // Draw -x face.
  glBegin(GL_QUADS);
  glVertex3d(center->x - dimensions->x / 2.0, center->y - dimensions->y / 2.0,
             center->z - dimensions->z / 2.0);
  glVertex3d(center->x - dimensions->x / 2.0, center->y - dimensions->y / 2.0,
             center->z + dimensions->z / 2.0);
  glVertex3d(center->x - dimensions->x / 2.0, center->y + dimensions->y / 2.0,
             center->z + dimensions->z / 2.0);
  glVertex3d(center->x - dimensions->x / 2.0, center->y + dimensions->y / 2.0,
             center->z - dimensions->z / 2.0);
  glEnd();

  // Draw +x face.
  glBegin(GL_QUADS);
  glVertex3d(center->x + dimensions->x / 2.0, center->y - dimensions->y / 2.0,
             center->z - dimensions->z / 2.0);
  glVertex3d(center->x + dimensions->x / 2.0, center->y + dimensions->y / 2.0,
             center->z - dimensions->z / 2.0);
  glVertex3d(center->x + dimensions->x / 2.0, center->y + dimensions->y / 2.0,
             center->z + dimensions->z / 2.0);
  glVertex3d(center->x + dimensions->x / 2.0, center->y - dimensions->y / 2.0,
             center->z + dimensions->z / 2.0);
  glEnd();

  // Draw -y face.
  glBegin(GL_QUADS);
  glVertex3d(center->x - dimensions->x / 2.0, center->y - dimensions->y / 2.0,
             center->z - dimensions->z / 2.0);
  glVertex3d(center->x + dimensions->x / 2.0, center->y - dimensions->y / 2.0,
             center->z - dimensions->z / 2.0);
  glVertex3d(center->x + dimensions->x / 2.0, center->y - dimensions->y / 2.0,
             center->z + dimensions->z / 2.0);
  glVertex3d(center->x - dimensions->x / 2.0, center->y - dimensions->y / 2.0,
             center->z + dimensions->z / 2.0);
  glEnd();

  // Draw +y face.
  glBegin(GL_QUADS);
  glVertex3d(center->x - dimensions->x / 2.0, center->y + dimensions->y / 2.0,
             center->z - dimensions->z / 2.0);
  glVertex3d(center->x - dimensions->x / 2.0, center->y + dimensions->y / 2.0,
             center->z + dimensions->z / 2.0);
  glVertex3d(center->x + dimensions->x / 2.0, center->y + dimensions->y / 2.0,
             center->z + dimensions->z / 2.0);
  glVertex3d(center->x + dimensions->x / 2.0, center->y + dimensions->y / 2.0,
             center->z - dimensions->z / 2.0);
  glEnd();

  // Draw -z face.
  glBegin(GL_QUADS);
  glVertex3d(center->x - dimensions->x / 2.0, center->y - dimensions->y / 2.0,
             center->z - dimensions->z / 2.0);
  glVertex3d(center->x - dimensions->x / 2.0, center->y + dimensions->y / 2.0,
             center->z - dimensions->z / 2.0);
  glVertex3d(center->x + dimensions->x / 2.0, center->y + dimensions->y / 2.0,
             center->z - dimensions->z / 2.0);
  glVertex3d(center->x + dimensions->x / 2.0, center->y - dimensions->y / 2.0,
             center->z - dimensions->z / 2.0);
  glEnd();

  // Draw +z face.
  glBegin(GL_QUADS);
  glVertex3d(center->x - dimensions->x / 2.0, center->y - dimensions->y / 2.0,
             center->z + dimensions->z / 2.0);
  glVertex3d(center->x + dimensions->x / 2.0, center->y - dimensions->y / 2.0,
             center->z + dimensions->z / 2.0);
  glVertex3d(center->x + dimensions->x / 2.0, center->y + dimensions->y / 2.0,
             center->z + dimensions->z / 2.0);
  glVertex3d(center->x - dimensions->x / 2.0, center->y + dimensions->y / 2.0,
             center->z + dimensions->z / 2.0);
  glEnd();
}

void DrawLine(const Vec3 *start, const Vec3 *end) {
  glBegin(GL_LINES);
  glVertex3d(start->x, start->y, start->z);
  glVertex3d(end->x, end->y, end->z);
  glEnd();
}

void DrawCircle(const Vec3 *center, double radius, const Vec3 *normal) {
  glBegin(GL_LINE_LOOP);

  // Find a vector that is orthogonal to normal.
  Vec3 x_unit;
  if (fabs(Vec3Dot(normal, &kVec3X)) > fabs(Vec3Dot(normal, &kVec3Y))) {
    Vec3Add(normal, &kVec3Y, &x_unit);
  } else {
    Vec3Add(normal, &kVec3X, &x_unit);
  }
  Vec3Cross(&x_unit, normal, &x_unit);
  Vec3Normalize(&x_unit, &x_unit);

  Vec3 y_unit;
  Vec3Cross(normal, &x_unit, &y_unit);
  Vec3Normalize(&y_unit, &y_unit);

  for (double angle = 0.0; angle < 2.0 * PI; angle += 0.1) {
    Vec3 point, x, y;
    Vec3Scale(&x_unit, radius * cos(angle), &x);
    Vec3Scale(&y_unit, radius * sin(angle), &y);
    Vec3Add3(center, &x, &y, &point);
    glVertex3d(point.x, point.y, point.z);
  }
  glEnd();
}

void DcmToGlMatrixf(const Mat3 *dcm, GLfloat m[16]) {
  m[3] = m[7] = m[11] = m[12] = m[13] = m[14] = 0.0f;
  m[15] = 1.0f;
  for (int32_t i = 0; i < 3; ++i) {
    for (int32_t j = 0; j < 3; ++j) {
      m[4 * i + j] = (float)dcm->d[i][j];
    }
  }
}

void CalcNormals(int num_vertices, const Vec3 *vertices, Vec3 *normals) {
  // It is assumed that the vertices are grouped as a list of
  // triangles.
  assert(num_vertices % 3 == 0);
  int num_triangles = num_vertices / 3;

  // This struct definition is private to this function as it is not
  // used elsewhere.
  const struct triangle {
    Vec3 v1, v2, v3;
  } *triangles = (const struct triangle *)vertices;

  // OpenGL requires that we define a normal vector at each
  // vertex. For smooth shading, one typically averages the normal
  // vectors for each facet that meets at a particular vertex, to get
  // the normal for that vertex.  Here we simply set the normal for
  // each vertex of a triangle to the normal vector of the
  // surface. This produces a slightly more "faceted" shading.
  for (int i = 0; i < num_triangles; i++) {
    const struct triangle *triangle = &triangles[i];
    Vec3 v12, v23, normal;
    Vec3Sub(&triangle->v1, &triangle->v2, &v12);
    Vec3Sub(&triangle->v2, &triangle->v3, &v23);
    Vec3Cross(&v12, &v23, &normal);
    Vec3Normalize(&normal, &normal);

    ((struct triangle *)normals)[i].v1 = normal;
    ((struct triangle *)normals)[i].v2 = normal;
    ((struct triangle *)normals)[i].v3 = normal;
  }
}

void InitModel(int num_vertices, Vec3 *vertices, GLuint *GLList) {
  // Calculate the normals.
  Vec3 *normals;
  normals = (Vec3 *)malloc((unsigned)num_vertices * sizeof(Vec3));
  CalcNormals(num_vertices, vertices, normals);

  // Compile a "display list" used to draw the wing later.
  *GLList = glGenLists(1);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_NORMAL_ARRAY);
  glNewList(*GLList, GL_COMPILE);
  glVertexPointer(3, GL_DOUBLE, 0, vertices);
  glNormalPointer(GL_DOUBLE, 0, normals);
  glDrawArrays(GL_TRIANGLES, 0, num_vertices);
  glEndList();
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_NORMAL_ARRAY);

  free(normals);
}
