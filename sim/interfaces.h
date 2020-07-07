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

#ifndef SIM_INTERFACES_H_
#define SIM_INTERFACES_H_

#include <stdint.h>
#include <string>

class DisplayableInterface {
 public:
  virtual ~DisplayableInterface() {}
  virtual void Disp() const = 0;
};

class SerializableInterface {
 public:
  virtual ~SerializableInterface() {}
  virtual int32_t Size() const = 0;
  virtual int32_t Serialize(double data[]) const = 0;
  virtual int32_t Deserialize(const double data[]) = 0;
  virtual bool IsSerializable() const = 0;
};

class PublishableInterface {
 public:
  virtual ~PublishableInterface() {}
  virtual void Publish() const = 0;
};

// An interface for saving and loading the state of objects in the sim.
// The structure is expected to be similar to JSON, but a bit more restrictive
// (e.g. right now only double arrays are supported, and mixed type arrays
// may never be supported).
class DictFormatterInterface {
 public:
  virtual ~DictFormatterInterface() {}

  virtual DictFormatterInterface *CreateChild(const std::string &key,
                                              int32_t *error) = 0;
  virtual int32_t Set(const std::string &key, double value) = 0;
  virtual int32_t Set(const std::string &key, bool value) = 0;
  virtual int32_t Set(const std::string &key, int32_t value) = 0;
  virtual int32_t Set(const std::string &key, const char *value) = 0;
  virtual int32_t Set(const std::string &key, const std::string &value) = 0;
  virtual int32_t Set(const std::string &key, const double *d,
                      uint32_t length) = 0;

  virtual int32_t Get(const std::string &key, double *val) const = 0;
  virtual int32_t Get(const std::string &key, int32_t *val) const = 0;
  virtual int32_t Get(const std::string &key, bool *val) const = 0;
  virtual int32_t Get(const std::string &key, double *vals,
                      uint32_t expected_length) const = 0;
  // The user is not expected to free vals, but must use it before the
  // DictFormatterInterface is destroyed.
  virtual int32_t Get(const std::string &key, const char **vals,
                      uint32_t *length) const = 0;

  virtual const DictFormatterInterface *GetChild(const std::string &key,
                                                 int32_t *error) const = 0;
};

typedef enum {
  kDictFormatterErrorNone = 0,
  kDictFormatterErrorKeyNotFound = 1,
  kDictFormatterErrorIncorrectSize = 2,
  kDictFormatterErrorIncorrectType = 3,
  kDictFormatterErrorInternal = 4,
  kDictFormatterErrorAllocation = 5
} DictFormatterError;

// An interface for objects which can be loaded and saved using a
// DictFormatterInterface.
class DictLoadableInterface {
 public:
  virtual ~DictLoadableInterface() {}
  virtual int32_t Save(DictFormatterInterface *out) const = 0;
  virtual int32_t Load(const DictFormatterInterface &in) = 0;
};

#endif  // SIM_INTERFACES_H_
