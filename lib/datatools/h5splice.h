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

#ifndef LIB_DATATOOLS_H5SPLICE_H_
#define LIB_DATATOOLS_H5SPLICE_H_

#include <stdint.h>
#include <hdf5.h>

#define H5SPLICE_MAX_FILES 64
#define H5SPLICE_MAX_FIELDS 64

typedef enum {
  PARSE_STATE_INPUT,
  PARSE_STATE_DATASET,
  PARSE_STATE_FIELDS,
  PARSE_STATE_OUTPUT,
  PARSE_STATE_NONE
} ParseState;

void PrintUsage(void);
int32_t ParseInputs(int32_t argc, char **argv,
                    const char **in_files, int32_t *num_in_files,
                    const char **fields, const char **dsets,
                    int32_t *num_fields, int32_t *stride,
                    const char **out_file);
const char *GetDsetName(const char *dset_path);
const char *GetGroupName(const char *dset_path, char *group_str);
int32_t MatchPatterns(const char *str,
                      const char **patterns, int32_t num_patterns);
hid_t BuildPartialType(const char *head_str,
                       const char **patterns, int32_t num_patterns,
                       hid_t type_id);
void DispType(hid_t type);
void DispTypeHelper(hid_t type, const char *spaces);

#endif  // LIB_DATATOOLS_H5SPLICE_H_
