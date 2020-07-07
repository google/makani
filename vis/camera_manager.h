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

#ifndef VIS_CAMERA_MANAGER_H_
#define VIS_CAMERA_MANAGER_H_

#include <stdbool.h>

#include "common/c_math/vec3.h"
#include "vis/vis_main.h"

typedef struct CameraPose {
  double phi;
  double theta;
  double rho;
  double focus_x;
  double focus_y;
  double focus_z;
  float field_of_view_y;
} CameraPose;

extern CameraPose view_cross;
extern CameraPose view_loop;
extern CameraPose view_x;
extern CameraPose view_y;
extern CameraPose view_z;
extern CameraPose view_ch_y;
extern CameraPose view_gs;

extern CameraPose camera_pose;

#ifdef __cplusplus
extern "C" {
#endif

// Sets the camera orientation and focus.
void SetCamera(const CameraPose *new_cam);

// Sets the camera orientation and focus for a view of the perch.
void SetPerchView(void);

// Sets the camera orientation and focus for a narrow command center view.
void SetCommandCenterNarrowView(void);

// Sets the camera orientation and focus for a wide command center view.
void SetCommandCenterWideView(void);

// Sets the camera orientation and focus for tether view.
void SetTetherView(void);

// Sets whether the focus is on the wing or at a fixed location.
void SetFollowWing(bool follow_wing);

// Sets whether the camera is on the wing and rotates with it or fixed location.
void SetFlyWithWing(bool fly_with_wing);

// Updates the OpenGL projection matrix based on the current view
// angle and focal point.
void UpdateCamera(const Vec3 *Xg, const Vec3 *eulers);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // VIS_CAMERA_MANAGER_H_
