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

#ifndef GS_MONITOR_WIDGETS_INDICATOR_H_
#define GS_MONITOR_WIDGETS_INDICATOR_H_

#include <glib-object.h>
#include <glib.h>

#include <gtk/gtkeventbox.h>
#include <gtk/gtkhbox.h>
#include <gtk/gtklabel.h>
#include <gtk/gtkvbox.h>

G_BEGIN_DECLS

#define INDICATOR_TYPE (indicator_get_type())
#define INDICATOR(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), INDICATOR_TYPE, Indicator))
#define INDICATOR_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), INDICATOR_TYPE, IndicatorClass))
#define IS_INDICATOR(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), INDICATOR_TYPE))
#define IS_INDICATOR_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass), INDICATOR_TYPE))

#define INDICATOR_MAX_VALUE_LEN 2048

// The stoplights are flight critical because they cue the
// (human) flight operator to take a specific action. Therefore,
// monitor indicators should follow these guidelines.
//
// Error (Red)
//   Meaning: A specific check indicates it is not safe to continue
//            a flight.
//   Action: If in-flight, end the flight as soon and as
//           safely as possible.
//   Note: - Use red sparingly and beware of false positives.
//         - This condition means that we may often see red indicators
//           during ground testing.
//
// Warning (Yellow)
//   Meaning: A specific check indicates that this system
//            is not operating nominally.
//   Action: If in-flight, follow-up with test engineers to
//           understand what is causing this warning.
//
// Good (Green)
//   Meaning: The monitor is receiving data for at least part of this system,
//            and the data received passes nominal operation checks if they
//            exist.
//   Action: It's ok to ignore green lights.
//   Note: A green light does not guarantee that the system is operating
//         nominally.
//
// None (Gray)
//   Meaning: The monitor does not have access to any data needed to update
//            this indicator.

typedef enum {
  INDICATOR_STATE_EMPTY,
  INDICATOR_STATE_NONE,
  INDICATOR_STATE_GOOD,
  INDICATOR_STATE_WARNING,
  INDICATOR_STATE_ERROR,
  INDICATOR_STATE_ERROR_SPECIAL
} IndicatorState;

typedef enum { INDICATOR_STYLE_NORMAL, INDICATOR_STYLE_COMPACT } IndicatorStyle;

typedef struct _Indicator Indicator;
typedef struct _IndicatorClass IndicatorClass;

struct _Indicator {
  GtkHBox box;
  GtkVBox *vbox;
  GtkLabel *label;
  GtkEventBox *label_color_box;
  GtkLabel *value;
  GtkEventBox *value_color_box;
  IndicatorState state;

  char last_value_str[INDICATOR_MAX_VALUE_LEN];
};

struct _IndicatorClass {
  GtkHBoxClass parent_class;
  void (*indicator)(Indicator *indicator);
};

typedef void (*const IndicatorUpdateFunction)(Indicator *ind, int32_t init);

GType indicator_get_type(void);
GtkWidget *indicator_new(const char *label_str);
GtkWidget *indicator_new_with_style(const char *label_str,
                                    IndicatorStyle style);

void indicator_set_style(Indicator *ind, IndicatorStyle style);
void indicator_set_label(Indicator *ind, const char *label_str);
const char *indicator_get_label(Indicator *ind);
void indicator_set_value(Indicator *ind, const char *value_str);
void indicator_set_state(Indicator *ind, IndicatorState state);

G_END_DECLS

#endif  // GS_MONITOR_WIDGETS_INDICATOR_H_
