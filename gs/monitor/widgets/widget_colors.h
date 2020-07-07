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

#ifndef GS_MONITOR_WIDGETS_WIDGET_COLORS_H_
#define GS_MONITOR_WIDGETS_WIDGET_COLORS_H_

#include <gtk/gtk.h>

static const GdkColor RED = {0, 0xffff, 0x0000, 0x0000};
static const GdkColor GREEN = {1, 0x0000, 0x8888, 0x0000};
static const GdkColor BLUE = {2, 0x0000, 0x0000, 0xffff};
static const GdkColor CYAN = {3, 0x0000, 0xffff, 0xffff};
static const GdkColor MAGENTA = {4, 0xffff, 0x0000, 0xffff};
static const GdkColor YELLOW = {5, 0xffff, 0xffff, 0x0000};

static const GdkColor BLACK = {6, 0x0000, 0x0000, 0x0000};
static const GdkColor GRAY = {7, 0x8888, 0x8888, 0x8888};
static const GdkColor WHITE = {8, 0xffff, 0xffff, 0xffff};

static const GdkColor *COLORS[8] = {&BLUE,    &GREEN,  &RED,   &CYAN,
                                    &MAGENTA, &YELLOW, &BLACK, &GRAY};

#endif  // GS_MONITOR_WIDGETS_WIDGET_COLORS_H_
