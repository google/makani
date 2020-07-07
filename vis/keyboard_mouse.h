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

#ifndef VIS_KEYBOARD_MOUSE_H_
#define VIS_KEYBOARD_MOUSE_H_

#include <stdint.h>

#include "vis/camera_manager.h"

typedef struct MouseRotation {
  int32_t button;
  int32_t x0;
  int32_t y0;
} MouseRotation;

#ifdef __cplusplus
extern "C" {
#endif

void Zoom(double drho);
void Rotate(double dphi, double dtheta);
void Pan(double dx, double dy, double dz);
void KeyPressed(uint8_t key, int32_t x, int32_t y);
void ArrowKeyPressed(int32_t key, int32_t x, int32_t y);
void MouseClicked(int32_t button, int32_t state, int32_t x, int32_t y);
void MouseMoved(int32_t x, int32_t y);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // VIS_KEYBOARD_MOUSE_H_
