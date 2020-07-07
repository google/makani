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

#ifndef SIM_JSON_FORMATTER_H_
#define SIM_JSON_FORMATTER_H_

#include <jansson.h>
#include <stdint.h>

#include <string>
#include <vector>

#include "sim/interfaces.h"

class JsonFormatter : public virtual DictFormatterInterface {
 public:
  JsonFormatter() : root_(json_object()) {}
  explicit JsonFormatter(const JsonFormatter &loader);
  explicit JsonFormatter(const std::string &s);

  ~JsonFormatter();

  int32_t WriteFile(const std::string &path);

  DictFormatterInterface *CreateChild(const std::string &key, int32_t *error);
  int32_t Set(const std::string &key, double value);
  int32_t Set(const std::string &key, bool value);
  int32_t Set(const std::string &key, int32_t value);
  int32_t Set(const std::string &key, uint64_t value);
  int32_t Set(const std::string &key, const char *value);
  int32_t Set(const std::string &key, const std::string &value);
  int32_t Set(const std::string &key, const double *d, uint32_t length);

  const DictFormatterInterface *GetChild(const std::string &key,
                                         int32_t *error) const;
  int32_t Get(const std::string &key, double *val) const;
  int32_t Get(const std::string &key, bool *val) const;
  int32_t Get(const std::string &key, int32_t *val) const;
  int32_t Get(const std::string &key, uint64_t *val) const;
  int32_t Get(const std::string &key, const char **vals,
              uint32_t *length) const;
  int32_t Get(const std::string &key, double *vals,
              uint32_t expected_length) const;

 private:
  explicit JsonFormatter(json_t *r) : root_(r) { json_incref(r); }
  const JsonFormatter &operator=(const JsonFormatter &sl);
  int32_t SetWithChecks(json_t *object, const std::string &key, json_t *value);

  json_t *root_;
};

#endif  // SIM_JSON_FORMATTER_H_
