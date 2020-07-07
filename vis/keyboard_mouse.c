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

#include "vis/keyboard_mouse.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// usleep is not part of c99, so we bring it in by defining __USE_MISC.
#ifndef __USE_MISC
#define __USE_MISC
#include <unistd.h>
#undef __USE_MISC
#endif

#include <GL/gl.h>
#include <GL/glut.h>

#include "common/c_math/util.h"
#include "vis/camera_manager.h"

extern bool g_lighting_enabled;

const double kZoomFactorMouse = 0.9;
const double kZoomFactorKey = 0.5;
const double kPanFactorMouse = 0.002;
const double kPanFactorKey = 0.1;

MouseRotation mouse_rotation = {.x0 = 0, .y0 = 0, .button = 0};

inline void Zoom(double drho) {
  camera_pose.rho = fmax(camera_pose.rho * drho, 0.1);
}

inline void Rotate(double dphi, double dtheta) {
  camera_pose.phi += dphi;
  camera_pose.theta = Saturate(camera_pose.theta + dtheta, -PI / 2.0, PI / 2.0);
}

inline void Pan(double dx, double dy, double dz) {
  dx = dx * camera_pose.rho;
  dy = dy * camera_pose.rho;
  dz = dz * camera_pose.rho;

  camera_pose.focus_x += sin(camera_pose.phi) * dx + cos(camera_pose.phi) * dy;
  camera_pose.focus_y += -cos(camera_pose.phi) * dx + sin(camera_pose.phi) * dy;
  camera_pose.focus_z += dz;
}

void KeyPressed(uint8_t key, int32_t x __attribute__((unused)),
                int32_t y __attribute__((unused))) {
  usleep(100);

  switch (key) {
    case '+':
      Zoom(kZoomFactorKey);
      break;
    case '-':
      Zoom(1.0 / kZoomFactorKey);
      break;
    case 'u':
      Pan(0.0, 0.0, -kPanFactorKey);
      break;
    case 'd':
      Pan(0.0, 0.0, kPanFactorKey);
      break;
    case '1':
      SetPerchView();
      break;
    case '2':
      SetCamera(&view_cross);
      break;
    case '3':
      SetCamera(&view_loop);
      break;
    case '4':
      SetFlyWithWing(false);
      SetFollowWing(true);
      break;
    case '5':
      SetCommandCenterNarrowView();
      break;
    case '6':
      SetCommandCenterWideView();
      break;
    case '7':
      SetCamera(&view_gs);
      break;
    case '8':
      SetTetherView();
      break;
    case 'f':
    case 'F':
      SetFollowWing(false);
      SetFlyWithWing(true);
      break;
    case 'l':
    case 'L':
      g_lighting_enabled = !g_lighting_enabled;
      break;
    case 'x':
      SetCamera(&view_x);
      break;
    case 'y':
      SetCamera(&view_y);
      break;
    case 'Y':
      SetCamera(&view_ch_y);
      break;
    case 'z':
      SetCamera(&view_z);
      break;
    default:
      break;
  }
}

void ArrowKeyPressed(int32_t key, int32_t x __attribute__((unused)),
                     int32_t y __attribute__((unused))) {
  if (glutGetModifiers() & GLUT_ACTIVE_SHIFT) {
    switch (key) {
      case GLUT_KEY_UP:
        Rotate(0.0, -0.1);
        break;
      case GLUT_KEY_DOWN:
        Rotate(0.0, 0.1);
        break;
      case GLUT_KEY_LEFT:
        Rotate(-0.1, 0.0);
        break;
      case GLUT_KEY_RIGHT:
        Rotate(0.1, 0.0);
        break;
      default:
        break;
    }
  } else {
    switch (key) {
      case GLUT_KEY_UP:
        Pan(0.0, kPanFactorKey, 0.0);
        break;
      case GLUT_KEY_DOWN:
        Pan(0.0, -kPanFactorKey, 0.0);
        break;
      case GLUT_KEY_LEFT:
        Pan(kPanFactorKey, 0.0, 0.0);
        break;
      case GLUT_KEY_RIGHT:
        Pan(-kPanFactorKey, 0.0, 0.0);
        break;
      default:
        break;
    }
  }
}

void MouseClicked(int32_t button, int32_t state, int32_t x, int32_t y) {
  usleep(100);
  mouse_rotation.button = button;
  mouse_rotation.x0 = x;
  mouse_rotation.y0 = y;

  // state is GLUT_DOWN at start of button click and GLUT_UP at release
  if (state == GLUT_DOWN) {
    switch (mouse_rotation.button) {
      case 3:  // Scroll wheel up (no #DEFINE)
        Zoom(kZoomFactorMouse);
        break;
      case 4:  // Scroll wheel down (no #DEFINE)
        Zoom(1.0 / kZoomFactorMouse);
        break;
      default:
        break;
    }
  }
}

void MouseMoved(int32_t x, int32_t y) {
  usleep(100);

  switch (mouse_rotation.button) {
    case GLUT_LEFT_BUTTON:
      Rotate(2.0 * PI * (x - mouse_rotation.x0) / window_width,
             2.0 * PI * (y - mouse_rotation.y0) / window_height);
      break;
    case GLUT_RIGHT_BUTTON:
      Pan((x - mouse_rotation.x0) * kPanFactorMouse,
          (y - mouse_rotation.y0) * kPanFactorMouse, 0.0);
      break;
    case GLUT_MIDDLE_BUTTON:
      Pan(0.0, 0.0, (mouse_rotation.y0 - y) * kPanFactorMouse);
      break;
    default:
      break;
  }

  mouse_rotation.x0 = x;
  mouse_rotation.y0 = y;
}
