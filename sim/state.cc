// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sim/state.h"

#include <stdint.h>
#include <stdio.h>

#include <memory>

#include "common/c_math/force_moment.h"
#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "control/system_types.h"

template <>
void State<double>::Disp() const {
  if (updated_) {
    printf("\n%s = %f", name().c_str(), val_);
  } else {
    DispNotUpdated();
  }
}

template <>
void State<uint8_t>::Disp() const {
  if (updated_) {
    printf("\n%s = %d", name().c_str(), val_);
  } else {
    DispNotUpdated();
  }
}

template <>
void State<int16_t>::Disp() const {
  if (updated_) {
    printf("\n%s = %d", name().c_str(), val_);
  } else {
    DispNotUpdated();
  }
}

template <>
void State<int32_t>::Disp() const {
  if (updated_) {
    printf("\n%s = %d", name().c_str(), val_);
  } else {
    DispNotUpdated();
  }
}

template <>
void State<bool>::Disp() const {
  if (updated_) {
    if (val_) {
      printf("\n%s = TRUE", name().c_str());
    } else {
      printf("\n%s = FALSE", name().c_str());
    }
  } else {
    DispNotUpdated();
  }
}

template <>
void State<Vec3>::Disp() const {
  if (updated_) {
    printf("\n%s = [%.12f, %.12f, %.12f]'", name().c_str(), val_.x, val_.y,
           val_.z);
  } else {
    DispNotUpdated();
  }
}

template <>
void State<ForceMomentPos>::Disp() const {
  if (updated_) {
    printf("\n%s.force = [%.12f, %.12f, %.12f]'", name().c_str(), val_.force.x,
           val_.force.y, val_.force.z);
    printf("\n%s.moment = [%.12f, %.12f, %.12f]'", name().c_str(),
           val_.moment.x, val_.moment.y, val_.moment.z);
    printf("\n%s.pos = [%.12f, %.12f, %.12f]'", name().c_str(), val_.pos.x,
           val_.pos.y, val_.pos.z);
  } else {
    DispNotUpdated();
  }
}

template <>
int32_t State<bool>::Save(DictFormatterInterface *out) const {
  int32_t error = out->Set(name(), val_);
  if (error) return -1;

  return 0;
}

template <>
int32_t State<bool>::Load(const DictFormatterInterface &in) {
  bool value;
  int32_t error = in.Get(name(), &value);
  if (error) return -1;

  set_val(value);
  return 0;
}

template <>
int32_t State<Vec2>::Save(DictFormatterInterface *out) const {
  double val_array[2] = {val_.x, val_.y};
  return out->Set(name(), val_array, 2);
}

template <>
int32_t State<Vec2>::Load(const DictFormatterInterface &in) {
  double val_array[2];
  int32_t error = in.Get(name(), val_array, 2);
  if (error) return -1;

  Vec2 val__;
  val__.x = val_array[0];
  val__.y = val_array[1];
  set_val(val__);

  return 0;
}

template <>
int32_t State<Vec3>::Save(DictFormatterInterface *out) const {
  double val_array[3] = {val_.x, val_.y, val_.z};
  return out->Set(name(), val_array, 3);
}

template <>
int32_t State<Vec3>::Load(const DictFormatterInterface &in) {
  double val_array[3];
  int32_t error = in.Get(name(), val_array, 3);
  if (error) return -1;

  Vec3 val__ = {val_array[0], val_array[1], val_array[2]};
  set_val(val__);

  return 0;
}

template <>
int32_t State<Quat>::Save(DictFormatterInterface *out) const {
  double val_array[4] = {val_.q0, val_.q1, val_.q2, val_.q3};
  return out->Set(name(), val_array, 4);
}

template <>
int32_t State<Quat>::Load(const DictFormatterInterface &in) {
  double val_array[4];
  int32_t error = in.Get(name(), val_array, 4);
  if (error) return -1;

  Quat val__ = {val_array[0], val_array[1], val_array[2], val_array[3]};
  set_val(val__);

  return 0;
}

// TODO: Switch matrices to be stored vectors of vectors.
template <>
int32_t State<Mat3>::Save(DictFormatterInterface *out) const {
  double val_array[9];
  for (uint32_t i = 0; i < 3; ++i)
    for (uint32_t j = 0; j < 3; ++j) val_array[i * 3 + j] = val_.d[i][j];

  return out->Set(name(), val_array, 9);
}

template <>
int32_t State<Mat3>::Load(const DictFormatterInterface &in) {
  double val_array[9];
  int32_t error = in.Get(name(), val_array, 9);
  if (error) return -1;

  Mat3 val__;
  for (uint32_t i = 0; i < 3; ++i)
    for (uint32_t j = 0; j < 3; ++j) val__.d[i][j] = val_array[i * 3 + j];
  set_val(val__);

  return 0;
}
