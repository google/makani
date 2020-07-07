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

#include "avionics/firmware/test/selection.h"

#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/strings.h"

static bool IsTestDelimiter(char c) {
  return c == ',' || c == ' ';
}

static bool IsFieldDelimiter(char c) {
  return c == '.';
}

// Search for SuiteName.TestName, SuiteName, or TestName in comma or space
// delimited list.
bool IsTestSelected(const char *selection, const char *suite_name,
                    const char *test_name) {
  assert(selection != NULL);
  assert(suite_name != NULL);
  assert(test_name != NULL);

  int32_t suite_length = (int32_t)strlen(suite_name);
  int32_t test_length = (int32_t)strlen(test_name);
  for (const char *c = selection; *c != '\0'; ) {
    // Skip test delimiters.
    while (IsTestDelimiter(*c)) {
      ++c;
    }

    // Acquire suite name field.
    const char *s_str = c;
    const char *t_str = c;
    while (!IsTestDelimiter(*c) && !IsFieldDelimiter(*c) && *c != '\0') {
      ++c;
    }
    int32_t s_len = (int32_t)(c - s_str);

    // Acquire field delimiter.
    if (IsFieldDelimiter(*c)) {
      ++c;
      t_str = c;
    }

    // Acquire test name field.
    while (!IsTestDelimiter(*c) && *c != '\0') {
      ++c;
    }
    int32_t t_len = (int32_t)(c - t_str);

    // Check match.
    if (t_len == 0
        && WildCompare(suite_length, suite_name, s_len, s_str)) {
      return true;  // Match suite.
    } else if (s_len == 0
               && WildCompare(test_length, test_name, t_len, t_str)) {
      return true;  // Match test.
    } else if (s_str == t_str && s_len == t_len
               && (WildCompare(suite_length, suite_name, s_len, s_str)
                   || WildCompare(test_length, test_name, s_len, s_str))) {
      return true;  // Match suite or test.
    } else if (s_str != t_str
               && WildCompare(suite_length, suite_name, s_len, s_str)
               && WildCompare(test_length, test_name, t_len, t_str)) {
      return true;  // Match suite and test.
    }
  }
  return false;  // No match.
}
