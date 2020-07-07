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

#include "avionics/common/strings.h"

#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/macros.h"

static const int32_t kInt32Power10[] = {
  1,
  10,
  100,
  1000,
  10000,
  100000,
  1000000,
  10000000,
  100000000,
  1000000000
};

static const char kAsciiNumbers[] = {
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
  'A', 'B', 'C', 'D', 'E', 'F'
};

int32_t ReadDecInt32(const char *buf, int32_t max_length, int32_t *value) {
  assert(buf != NULL);
  assert(max_length >= 0);
  assert(value != NULL);

  uint32_t u32;
  int32_t length;
  *value = 0;  // Default return value when length equals 0.
  if (max_length > 0 && buf[0] == '-') {
    length = ReadDecUint32(&buf[1], max_length - 1, &u32);
    if (length > 0) {
      *value = (u32 > INT32_MAX) ? INT32_MIN : -(int32_t)u32;
      ++length;
    }
  } else if (max_length > 0 && buf[0] == '+') {
    length = ReadDecUint32(&buf[1], max_length - 1, &u32);
    *value = (u32 > INT32_MAX) ? INT32_MAX : (int32_t)u32;
    if (length > 0) {
      ++length;
    }
  } else {
    length = ReadDecUint32(buf, max_length, &u32);
    *value = (u32 > INT32_MAX) ? INT32_MAX : (int32_t)u32;
  }
  return length;
}

int32_t ReadDecUint32(const char *buf, int32_t max_length, uint32_t *value) {
  assert(buf != NULL);
  assert(max_length >= 0);
  assert(value != NULL);

  int32_t length = 0;
  *value = 0U;  // Default return value when length equals 0.
  while (length < max_length && '0' <= buf[length] && buf[length] <= '9') {
    uint32_t prev_value = *value;
    *value *= 10U;
    *value += (uint32_t)(buf[length] - '0');

    // Saturate at maximum value.
    if (*value < prev_value) {
      *value = 0xFFFFFFFF;
    }
    ++length;
  }
  return length;
}

int32_t ReadDecFloat(const char *buf, int32_t max_length, float *value) {
  assert(buf != NULL);
  assert(max_length >= 0);
  assert(value != NULL);

  bool negative = false;
  int32_t length;

  // Determine integer portion of string.

  // Prefix.
  if (max_length > 0 && buf[0] == '-') {
    negative = true;
    length = 1;
  } else if (max_length > 0 && buf[0] == '+') {
    length = 1;
  } else {
    length = 0;
  }

  // Number.
  *value = 0.0f;  // Default return value when length equals 0.
  while (length < max_length && '0' <= buf[length] && buf[length] <= '9') {
    *value *= 10.0f;
    *value += (float)(buf[length] - '0');
    ++length;
  }

  // Determine fraction portion of string.
  if (length < max_length && buf[length] == '.') {
    ++length;

    // Determine maximum precision. Note that although an int32_t can only
    // store 9 full digits of precision, it is greater precision than
    // FLT_EPSILON.
    uint32_t u32;
    int32_t precision = max_length - length;
    if (precision > ARRAYSIZE(kInt32Power10) - 1) {
      precision = ARRAYSIZE(kInt32Power10) - 1;
    }
    precision = ReadDecUint32(&buf[length], precision, &u32);

    // Compute fraction.
    *value += (float)u32 / (float)kInt32Power10[precision];

    // Compute length.
    length += precision;
    while (length < max_length && '0' <= buf[length] && buf[length] <= '9') {
      ++length;
    }
  }

  // Handle negative values.
  if (negative) {
    *value = -*value;
  }

  // Handle incomplete failures.
  if (length == 1 && (buf[0] == '-' || buf[0] == '+')) {
    length = 0;
  }
  return length;
}

int32_t ReadHexUint8(const char *buf, int32_t max_length, uint8_t *value) {
  assert(buf != NULL);
  assert(max_length >= 0);
  assert(value != NULL);
  if (max_length > 2) {
    max_length = 2;
  }

  uint32_t dword;
  int32_t length = ReadHexUint32(buf, max_length, &dword);
  *value = (uint8_t)dword;

  return length;
}

int32_t ReadHexUint32(const char *buf, int32_t max_length, uint32_t *value) {
  assert(buf != NULL);
  assert(max_length >= 0);
  assert(value != NULL);
  if (max_length > 8) {
    max_length = 8;
  }

  int32_t length;
  *value = 0U;
  for (length = 0; length < max_length; ++length) {
    if ('0' <= buf[length] && buf[length] <= '9') {
      *value *= 16U;
      *value += (uint32_t)(buf[length] - '0');
    } else if ('a' <= buf[length] && buf[length] <= 'f') {
      *value *= 16U;
      *value += (uint32_t)(10 + (buf[length] - 'a'));
    } else if ('A' <= buf[length] && buf[length] <= 'F') {
      *value *= 16U;
      *value += (uint32_t)(10 + (buf[length] - 'A'));
    } else {
      break;
    }
  }
  return length;
}

int32_t WriteDecInt32(int32_t value, int32_t max_length, char *buf) {
  assert(max_length >= 0);
  assert(buf != NULL);

  int32_t length;
  if (value >= 0) {
    length = WriteDecUint32((uint32_t)value, max_length, buf);
  } else if (max_length) {
    buf[0] = '-';
    length = WriteDecUint32((uint32_t)(-value), max_length - 1, &buf[1]) + 1;
  } else {
    length = 0;
  }
  return length;
}

int32_t WriteDecUint32(uint32_t value, int32_t max_length, char *buf) {
  assert(max_length >= 0);
  assert(buf != NULL);

  // Count digits since we need to write right to left.
  // The maximum value of uint32(4294967295) has 10 digits.
  int32_t length = 1;
  for (uint32_t v = 10U; v <= value && length < 10; v *= 10U, ++length) {}

  // Truncation likely indicates a bug elsewhere.
  assert(length <= max_length);

  // Truncate output for insufficient data.
  uint32_t output = value;
  while (length > max_length) {
    --length;
    output /= 10U;
  }
  for (int32_t i = length; i; ) {
    buf[--i] = kAsciiNumbers[output % 10];
    output /= 10U;
  }
  if (length < max_length) {
    buf[length] = '\0';
  }
  return length;
}

int32_t WriteHexUint32(uint32_t value, int32_t min_length, int32_t max_length,
                       char *buf) {
  assert(min_length >= 0);
  assert(max_length >= min_length);
  assert(buf != NULL);

  // Count digits since we need to write right to left.
  int32_t length = 1;
  for (uint32_t v = value >> 4; v; v >>= 4, ++length) {}
  if (length < min_length) {
    length = min_length;
  }

  // Truncation likely indicates a bug elsewhere.
  assert(length <= max_length);

  // Truncate output for insufficient data.
  uint32_t output = value;
  while (length > max_length) {
    --length;
    output >>= 4;
  }
  for (int32_t i = length; i; ) {
    buf[--i] = kAsciiNumbers[output & 0x0F];
    output >>= 4;
  }
  if (length < max_length) {
    buf[length] = '\0';
  }
  return length;
}

int32_t WriteString(const char *str, int32_t max_length, char *buf) {
  assert(str != NULL);
  assert(max_length >= 0);
  assert(buf != NULL);

  int32_t length;
  for (length = 0; length < max_length && str[length]; ++length) {
    buf[length] = str[length];
  }
  if (length < max_length) {
    buf[length] = '\0';
  }
  // Truncation likely indicates a bug elsewhere.
  assert(length < max_length);
  return length;
}

// Find a substring (needle) within another string (haystack). Returns starting
// address of matching substring in string or NULL if not found.
const char *FindString(int32_t haystack_length, const char *haystack,
                       int32_t needle_length, const char *needle) {
  const char *sub = NULL;
  int32_t length = haystack_length - needle_length + 1;
  if (length < 0 || haystack_length < 0 || needle_length < 0
      || haystack == NULL || needle == NULL) {
    return NULL;
  }
  for (int32_t i = 0; i < length && sub == NULL; ++i) {
    sub = &haystack[i];
    for (int32_t j = 0; j < needle_length; ++j) {
      assert(sub[j] != '\0');
      if (sub[j] != needle[j]) {
        sub = NULL;
        break;
      }
    }
  }
  return sub;
}

bool WildCompare(int32_t ref_length, const char *ref, int32_t wild_length,
                 const char *wild) {
  assert(ref != NULL);
  assert(wild != NULL);

  // Here we attempt to match a reference string with another string that may
  // contain wildcards '*' and '?'. For each '*' wildcard, we save the current
  // processing index + 1, and then search forward to the next wildcard. If we
  // encounter a mismatch, then we restore to the previously saved index and
  // re-evaluate. Wildcard '?' compares to any single character.
  int32_t r = 0;  // Current test index in ref[].
  int32_t w = 0;  // Current test index in wild[].
  int32_t r_save = 0;  // Saved index in ref[] of last '*' wildcard + 1.
  int32_t w_save = 0;  // Saved index in wild[] of last '*' wildcard + 1.

  // Terminate when we reach the end of reference string.
  while (r < ref_length && ref[r] != '\0') {
    if (w >= wild_length || wild[w] == '\0') {
      // End of wild string reached. Restore to saved state and re-evaluate
      // next possible match.
      if (w_save <= 0) {
        return false;  // No restore point saved.
      }
      w = w_save;
      r = r_save;
      ++r_save;

    } else if (wild[w] == '*') {
      // Set restore point to next index.
      ++w;
      w_save = w;
      r_save = r + 1;

      // Match remainder of string.
      if (w == wild_length || wild[w] == '\0') {
        return true;
      }
    } else if (wild[w] == ref[r] || wild[w] == '?') {
      // Single character match.
      ++r;
      ++w;
    } else {
      // End of current wildcard match reached. Restore to saved state and
      // re-evaluate next possible match.
      w = w_save;
      r = r_save;
      ++r_save;
    }
  }

  // Handle total match with trailing '*' wildcards.
  while (w < wild_length && wild[w] == '*') {
    ++w;
  }

  // True when we've reached the end of both strings.
  return (r == ref_length || ref[r] == '\0')
      && (w == wild_length || wild[w] == '\0');
}

bool WildCompareString(const char *ref, const char *wild) {
  return WildCompare((int32_t)strlen(ref), ref, (int32_t)strlen(wild), wild);
}

bool IsNumeric(const char *str) {
  int32_t sign_count = 0;
  int32_t decimal_count = 0;

  for (size_t i = 0; i < (size_t)strlen(str); ++i) {
    // Cast each array element to unsigned char to avoid compiler warning
    // [-Wchar-subscripts] with the use of `isdigit` from <ctype.h>.
    unsigned char elem = (unsigned char)str[i];

    // Handle non-digit elements.
    if (!isdigit(elem)) {
      if (i == 0 && (elem == '-' || elem == '+')) {
        ++sign_count;
      } else if (elem == '.' || elem == ',') {
        ++decimal_count;
      } else {
        return false;
      }
    }
  }

  // Handle extraneous signs and decimal points.
  if (sign_count > 1 || decimal_count > 1)
    return false;

  return true;
}

// Prototype made to match memset() to avoid confusion.
void vmemset(volatile void *s, int c, size_t n) {
  volatile uint8_t *s8 = (volatile uint8_t *)s;
  for (size_t i = 0; i < n; ++i) {
    s8[i] = (uint8_t)c;
  }
}

// Prototype made to match memcpy() to avoid confusion.
void vmemcpy(volatile void *dst, const volatile void *src, size_t n) {
  volatile uint8_t *dst8 = (volatile uint8_t *)dst;
  const volatile uint8_t *src8 = (const volatile uint8_t *)src;
  for (size_t i = 0; i < n; ++i) {
    dst8[i] = src8[i];
  }
}
