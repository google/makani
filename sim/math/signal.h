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

#ifndef SIM_MATH_SIGNAL_H_
#define SIM_MATH_SIGNAL_H_

#include <stdint.h>

#include <vector>

#include "sim/math/util.h"

namespace sim {
namespace signal {

// These functions are specialized for types such as Vec3 and double to
// allow for templated signal processing classes such as Measurement
// (see sim/models/signals/measurement.h).
//
// In most cases, the result of a calculation is stored in a pointer
// given given as the last argument, and a reference to the pointee
// is returned.

// Add two values.
template <typename T>
const T &Add(const T &x, const T &y, T *result);

// Return a "zero" element.
template <typename T>
const T &GetAddIdentity();

// Multiply two values element-wise.
template <typename T>
const T &Mult(const T &x, const T &y, T *result);

// Get a "one" element.
template <typename T>
const T &GetMultIdentity();

// Scale an element by a double.
template <typename T>
const T &Scale(const T &x, double y, T *result);

// Rounds x down to a multiple of the quantization. If the quantization is 0.0,
// then x is returned unmodified.
template <typename T>
const T &Quantize(const T &x, const T &quantization, T *result);

// Fill a value with standard normal random variables.
template <typename T>
const T &GetRandNormal(NamedRandomNumberGenerator *rng, T *result);

// Saturate a value element-wise.
template <typename T>
const T &Saturate(const T &value, const T &bound_low, const T &bound_high,
                  T *result);

// Get the number of doubles required to store this type.
template <typename T>
uint32_t GetSize();

// Store a value to an vector of doubles.
template <typename T>
void ToVector(const T &value, std::vector<double> *vec);

// Set a value from a vector of doubles.
template <typename T>
const T &FromVector(const std::vector<double> &vec, T *value);

// This is a testing utility to compare two values.
template <typename T>
bool IsNear(const T &expected, const T &actual, double tol);

}  // namespace signal
}  // namespace sim

#endif  // SIM_MATH_SIGNAL_H_
