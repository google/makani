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

#ifndef SIM_STATE_H_
#define SIM_STATE_H_

#include <float.h>
#include <glog/logging.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <limits>
#include <memory>
#include <string>

#include "lib/util/base64.h"
#include "sim/interfaces.h"

// Forward declarations.
struct ForceMomentPos;
struct Quat;
struct Vec2;
struct Vec3;

namespace internal {

template <typename T>
inline bool IsSerializable() {
  return false;
}
template <>
inline bool IsSerializable<double>() {
  return true;
}
template <>
inline bool IsSerializable<struct ForceMomentPos>() {
  return true;
}
template <>
inline bool IsSerializable<struct Quat>() {
  return true;
}
template <>
inline bool IsSerializable<struct Vec2>() {
  return true;
}
template <>
inline bool IsSerializable<struct Vec3>() {
  return true;
}

}  // namespace internal

// The State class is the most basic representation of a time-domain
// signal (similar to a line in a Simulink diagram).  It may be either
// a discrete or continuous signal.  Currently, this class only
// provides basic functionality such as tracking whether a signal has
// been updated in the current time-step, serializing a composite
// signal such as a Vec3 to a double array, and displaying the signal.

class StateInterface : public virtual DisplayableInterface,
                       public virtual SerializableInterface,
                       public virtual DictLoadableInterface {
 public:
  virtual ~StateInterface() __attribute__((noinline)) {}
  virtual const std::string &name() const = 0;
  virtual void Clear() = 0;
  virtual bool IsUpdated() const = 0;

 protected:
  // In nearly all use cases, a state should be immediately added to a list of
  // states of similar types, for use within the Model class.
  //
  // This behavior is implemented separately for ContinuousState and
  // DiscreteState.
  explicit StateInterface(StateInterface **ptr) {
    if (ptr != nullptr) *ptr = this;
  }

  StateInterface() {}
};

template <class T>
class State : public virtual StateInterface {
 public:
  State(StateInterface **ptr, const std::string &n)
      : StateInterface(ptr),
        val_(T()),
        updated_(false),
        name_(n),
        serializable_(internal::IsSerializable<T>()) {}

  State(StateInterface **ptr, const std::string &n, const T &v)
      : StateInterface(ptr),
        val_(v),
        updated_(true),
        name_(n),
        serializable_(internal::IsSerializable<T>()) {}

  ~State() __attribute__((noinline)) {}

  void set_name(const std::string &n) { name_ = n; }
  const std::string &name() const { return name_; }

  void set_val(const T &v) __attribute__((noinline)) {
    DCHECK(!updated_) << name_ << " was already updated.";
    val_ = v;
    updated_ = true;
  }
  const T &val() const __attribute__((noinline)) {
    DCHECK(updated_) << name_ << " was not updated.";
    return val_;
  }

  // Returns a const reference to the value without checking whether
  // the value has been updated.  This is useful for initializing
  // references in constructors.
  const T &val_unsafe() const { return val_; }

  // Clears the updated_ flag, which indicates that the value is
  // up-to-date.
  void Clear() { updated_ = false; }

  // Returns true if the value stored in the state is up-to-date.
  bool IsUpdated() const { return updated_; }

  // Returns the number of doubles that compose a serializable state.
  int32_t Size() const;

  // Conversions for composite types such as Vec3 or Quat to and from a
  // double array, which is used by the ODE solver.
  int32_t Serialize(double data[]) const;
  int32_t Deserialize(const double data[]);
  bool IsSerializable() const { return serializable_; }

  void Disp() const;

  virtual int32_t Save(DictFormatterInterface *out) const;
  virtual int32_t Load(const DictFormatterInterface &in);

 protected:
  explicit State(const std::string &n)
      : val_(T()),
        updated_(false),
        name_(n),
        serializable_(internal::IsSerializable<T>()) {}

  State(const std::string &n, const T &v)
      : val_(v),
        updated_(true),
        name_(n),
        serializable_(internal::IsSerializable<T>()) {}

  // Value stored in the state.
  T val_;

  // Flag indicating whether the state's value is up-to-date.
  bool updated_;

 private:
  // The following enum and type definitions are used in Save and Load to
  // have a single code path for loading / saving all integer and floating
  // point types respectively.
  enum Type { kIsInteger = 0, kIsFloatingPoint = 1, kIsOther = 2 };
  typedef std::integral_constant<Type, kIsInteger> IntegerType;
  typedef std::integral_constant<Type, kIsFloatingPoint> FloatingPointType;
  typedef std::integral_constant<Type, kIsOther> OtherType;

  int32_t SaveImpl(OtherType, DictFormatterInterface *out) const;
  int32_t LoadImpl(OtherType, const DictFormatterInterface &in);
  int32_t SaveImpl(IntegerType, DictFormatterInterface *out) const;
  int32_t LoadImpl(IntegerType, const DictFormatterInterface &in);
  int32_t SaveImpl(FloatingPointType, DictFormatterInterface *out) const;
  int32_t LoadImpl(FloatingPointType, const DictFormatterInterface &in);

  // Generic display function called by other display functions if the
  // state they are trying to display has not been updated.
  void DispNotUpdated() const;

  // Name of the state.
  std::string name_;

  // True if there is a method of converting the state's value into an
  // array of doubles.
  const bool serializable_;
};

template <class T>
int32_t State<T>::Size() const {
  DCHECK(serializable_) << name_ << " is not serializable.";
  return static_cast<int32_t>(sizeof(T) / sizeof(double));  // NOLINT
}

template <class T>
int32_t State<T>::Serialize(double data[]) const {
  DCHECK(serializable_) << name_ << " is not serializable.";
  DCHECK(updated_) << name_ << " was not updated.";
  memcpy(data, reinterpret_cast<const void *>(&val_), sizeof(T));
  return static_cast<int32_t>(sizeof(T) / sizeof(double));  // NOLINT
}

template <class T>
int32_t State<T>::Deserialize(const double data[]) {
  DCHECK(serializable_) << name_ << " is not serializable.";
  set_val(*reinterpret_cast<const T *>(data));
  return static_cast<int32_t>(sizeof(T) / sizeof(double));  // NOLINT
}

// Specialized Disp functions are defined in state.cc.
template <class T>
void State<T>::DispNotUpdated() const {
  printf("\n%s = <not updated>", name().c_str());
}

template <class T>
void State<T>::Disp() const {
  if (updated_) {
    const int32_t n =
        static_cast<int32_t>(sizeof(T) / sizeof(double));  // NOLINT
    std::unique_ptr<double[]> data(new double[n]);

    printf("\n%s = [", name().c_str());
    if (n > 0 && serializable_) {
      Serialize(data.get());
      printf("%f", data[0]);
      for (int32_t i = 1; i < n; ++i) printf(", %f", data[i]);
    }
    printf("]");
  } else {
    DispNotUpdated();
  }
}

// Save and Load dispatch to the SaveImpl and LoadImpl by examining
// the type T at compile time.
//
// IntegerType -- All integer types and enumerations.
// FloatingPointType -- float and double.
// OtherType -- Default implementations.
//
// Further specializations are contained in state.cc.
template <class T>
int32_t State<T>::Save(DictFormatterInterface *out) const {
  return SaveImpl(std::integral_constant < Type,
                  std::is_integral<T>::value
                      ? IntegerType::value
                      : std::is_enum<T>::value
                            ? IntegerType::value
                            : std::is_floating_point<T>::value
                                  ? FloatingPointType::value
                                  : OtherType::value > (),
                  out);
}

template <class T>
int32_t State<T>::Load(const DictFormatterInterface &in) {
  return LoadImpl(std::integral_constant < Type,
                  std::is_integral<T>::value
                      ? IntegerType::value
                      : std::is_enum<T>::value
                            ? IntegerType::value
                            : std::is_floating_point<T>::value
                                  ? FloatingPointType::value
                                  : OtherType::value > (),
                  in);
}

// Default implementation of Save and Load. The state is encoded as a base64
// string.
template <class T>
int32_t State<T>::SaveImpl(State<T>::OtherType,
                           DictFormatterInterface *out) const {
  char *b64 = Base64Encode(&val_, sizeof(val_));
  int32_t error = out->Set(name(), std::string(b64));
  Base64Free(b64);

  if (error)
    return -1;
  else
    return 0;
}

template <class T>
int32_t State<T>::LoadImpl(State<T>::OtherType,
                           const DictFormatterInterface &in) {
  const char *vals;
  uint32_t length = 0U;
  int32_t error = in.Get(name(), &vals, &length);
  if (error) return -1;

  uint32_t len;
  void *decode = Base64Decode(vals, &len);
  if (decode == NULL) return -1;

  memcpy(reinterpret_cast<void *>(&val_), decode, len);
  Base64Free(decode);
  updated_ = true;
  return 0;
}

// Save and Load implementations for integer types and enumerations.
template <class T>
int32_t State<T>::SaveImpl(State<T>::IntegerType,
                           DictFormatterInterface *out) const {
  CHECK(std::is_integral<T>::value || std::is_enum<T>::value);
  if (!(std::is_integral<T>::value || std::is_enum<T>::value)) {
    return -1;
  }

  int value = static_cast<int>(val_);
  CHECK_EQ(static_cast<T>(value), val_)
      << "Cast changed value of integer or enumerated type.";
  if (static_cast<T>(value) != val_) return -1;

  int32_t err = out->Set(name(), value);
  if (err) return -1;
  return 0;
}

template <class T>
int32_t State<T>::LoadImpl(State<T>::IntegerType,
                           const DictFormatterInterface &in) {
  CHECK(std::is_integral<T>::value || std::is_enum<T>::value);
  if (!(std::is_integral<T>::value || std::is_enum<T>::value)) {
    return -1;
  }

  int32_t value;
  int32_t error = in.Get(name(), &value);
  if (error) return -1;

  T cast_value = static_cast<T>(value);
  CHECK_EQ(static_cast<int32_t>(cast_value), value)
      << "Cast changed value of integer or enumerated type.";
  if (value != static_cast<int32_t>(cast_value)) return -1;

  set_val(cast_value);
  return 0;
}

// Save and Load implementation for floats and doubles.
template <class T>
int32_t State<T>::SaveImpl(State<T>::FloatingPointType,
                           DictFormatterInterface *out) const {
  CHECK(std::is_floating_point<T>::value);
  if (!std::is_floating_point<T>::value) {
    return -1;
  }

  double value = static_cast<double>(val_);
  // TODO: This could be done with the CHECK_DOUBLE_EQ
  // macro, but we would have to use the same comparison for returning
  // the error code.  Revisit how errors are handled for saving and
  // loading state.
  CHECK_LT(fabs(value - val_), DBL_EPSILON);
  if (fabs(value - val_) >= DBL_EPSILON) return -1;

  // Round extremely small-magnitude values -- those without an epsilon-diameter
  // representable interval around them -- to zero. Values smaller than this may
  // trigger parsing failures. (I'm not sure whether jansson rejects such values
  // altogether, or whether the parser disagrees with the encoder in what's
  // representable; both seem like reasonable possibilities.)
  if (fabs(value) < std::numeric_limits<T>::denorm_min() /
                        std::numeric_limits<T>::epsilon()) {
    value = 0.0;
  }

  int32_t err = out->Set(name(), value);
  if (err) return -1;
  return 0;
}

template <class T>
int32_t State<T>::LoadImpl(State<T>::FloatingPointType,
                           const DictFormatterInterface &in) {
  CHECK(std::is_floating_point<T>::value);
  if (!std::is_floating_point<T>::value) return -1;

  double value;
  int32_t error = in.Get(name(), &value);
  if (error) return -1;

  T cast_value = static_cast<T>(value);
  CHECK_LT(fabs(value - cast_value), DBL_EPSILON);
  if (fabs(value - cast_value) > DBL_EPSILON) return -1;

  set_val(cast_value);
  return 0;
}

// Continuous states extend the State class to also include a
// derivative (deriv_) and functions for handling the derivative.

class ContinuousStateInterface : public virtual StateInterface {
 public:
  virtual ~ContinuousStateInterface() __attribute__((noinline)) {}
  virtual int32_t SerializeDeriv(double data[]) const = 0;
  virtual void DispDeriv() const = 0;
};

template <class T>
class ContinuousState : public State<T>, public ContinuousStateInterface {
 public:
  ContinuousState(ContinuousStateInterface **ptr, const std::string &n)
      : State<T>::State(n), deriv_(nullptr, "d" + n) {
    *ptr = this;
  }
  ContinuousState(ContinuousStateInterface **ptr, const std::string &n,
                  const T &v)
      : State<T>::State(n, v), deriv_(nullptr, "d" + n) {
    *ptr = this;
  }

  ~ContinuousState() __attribute__((noinline)) {}

  void set_deriv(const T &d) { deriv_.set_val(d); }
  const T &deriv() const { return deriv_.val(); }

  int32_t SerializeDeriv(double d[]) const override {
    return deriv_.Serialize(d);
  }

  void DispDeriv() const override { deriv_.Disp(); }

  void Clear() override {
    State<T>::Clear();
    deriv_.Clear();
  }

 private:
  // Derivative of the continuous state.  This has the same type as
  // the state itself.
  State<T> deriv_;
};

// Discrete states extend the State class to handle discrete signals
// by include a sample time (ts_) and a last update time (t_z1_) as
// well as a general DiscreteUpdate function.

class DiscreteStateInterface : public virtual StateInterface {
 public:
  virtual ~DiscreteStateInterface() __attribute__((noinline)) {}
  virtual double ts() const = 0;
};

template <class T>
class DiscreteState : public State<T>, public DiscreteStateInterface {
 public:
  DiscreteState(DiscreteStateInterface **ptr, const std::string &n)
      : State<T>(n), ts_(0.0), t_z1_(-DBL_MAX) {
    *ptr = this;
  }
  DiscreteState(DiscreteStateInterface **ptr, const std::string &n, const T &v)
      : State<T>(n, v), ts_(0.0), t_z1_(-DBL_MAX) {
    *ptr = this;
  }
  DiscreteState(DiscreteStateInterface **ptr, const std::string &n, double ts__,
                const T &v)
      : State<T>(n, v), ts_(ts__), t_z1_(-DBL_MAX) {
    *ptr = this;
  }

  ~DiscreteState() __attribute__((noinline)) {}

  double ts() const { return ts_; }

  // Updates the DiscreteState value if sufficient time has elapsed.
  //
  // Updates val_ if ts_ time has elapsed since the last update (with
  // some tolerance).
  //
  // Args:
  //   t: Current simulator time.
  //   v: Value to set.
  void DiscreteUpdate(double t, const T &v) {
    double t_cmp = t * (1.0 + std::numeric_limits<double>::epsilon());
    if (t_cmp - t_z1_ >= ts_) {
      State<T>::Clear();
      State<T>::set_val(v);
      if (ts_ > 0.0) {
        t_z1_ = t_cmp - fmod(t_cmp, ts_);
      } else {
        t_z1_ = t;
      }
    }
  }

 private:
  // Don't allow setting for Discrete state.  Must use DiscreteUpdate
  // instead.
  void set_val(const T & /*v*/) { CHECK(false); }

  const double ts_;
  double t_z1_;
};

#endif  // SIM_STATE_H_
