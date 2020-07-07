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

#include "lib/datatools/h5splice.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <hdf5.h>
#include <hdf5_hl.h>

int32_t verbose_flag = 0;

int main(int argc, char **argv) {
  int32_t num_in_files, num_fields, stride_num;
  const char *in_files[H5SPLICE_MAX_FILES];
  const char *fields[H5SPLICE_MAX_FIELDS], *dsets[H5SPLICE_MAX_FIELDS];
  const char *out_file;

  int32_t ret = ParseInputs(argc, argv, in_files, &num_in_files,
                            fields, dsets, &num_fields, &stride_num, &out_file);
  if (ret) return ret;

  hsize_t stride = (hsize_t)stride_num;
  hsize_t dims[1];
  hsize_t lens[H5SPLICE_MAX_FILES], strided_lens[H5SPLICE_MAX_FILES];
  hsize_t max_len = 0, tot_strided_len = 0;
  hid_t part_type = -1;

  // Find length of each dataset
  for (int32_t i = 0; i < num_in_files; ++i) {
    if (verbose_flag) printf("\nAnalyzing %s...  ", in_files[i]);
    hid_t file_in_id = H5Fopen(in_files[i], H5F_ACC_RDONLY, H5P_DEFAULT);
    if (file_in_id < 0) {
      printf("\nCould not open file: %s", in_files[i]);
      return -1;
    }
    hid_t dset_in_id = H5Dopen(file_in_id, dsets[0], H5P_DEFAULT);
    if (dset_in_id < 0) {
      printf("\nCould not open dataset in file: %s", in_files[i]);
      return -1;
    }

    if (i == 0) {
      part_type = BuildPartialType("A", fields, num_fields,
                                   H5Dget_type(dset_in_id));
      H5Tpack(part_type);
    }
    H5Sget_simple_extent_dims(H5Dget_space(dset_in_id), dims, NULL);
    if (verbose_flag) printf("%d elements", (int32_t)dims[0]);

    lens[i] = dims[0];
    if (lens[i] > max_len)
      max_len = lens[i];
    strided_lens[i] = dims[0]/stride;
    tot_strided_len += strided_lens[i];

    H5Dclose(dset_in_id);
    H5Fclose(file_in_id);
  }

  if (part_type < 0) {
    printf("\nCould not make partial type");
    return -1;
  }
  if (verbose_flag)
    printf("\nData type size: %d", (int32_t)H5Tget_size(part_type));

  // Make buffer for reading data
  uint8_t *buf = malloc(max_len * H5Tget_size(part_type));
  if (buf == NULL) {
    printf("\nCould not allocate memory");
    return -1;
  }

  // Create output file and dataset
  hid_t file_id = H5Fcreate(out_file, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  char group_str[256];
  hid_t group_id = H5Gcreate(file_id, GetGroupName(dsets[0], group_str),
                             H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  hid_t space_id = H5Screate_simple(1, &tot_strided_len, NULL);
  hid_t dset_id = H5Dcreate(group_id, GetDsetName(dsets[0]), part_type,
                            space_id, H5P_DEFAULT,
                            H5Pcreate(H5P_DATASET_CREATE), H5P_DEFAULT);

  // Write to output file
  hsize_t offset = 0;
  for (int32_t i = 0; i < num_in_files; ++i) {
    if (verbose_flag) printf("\nProcessing %s ...", in_files[i]);
    hid_t file_in_id = H5Fopen(in_files[i], H5F_ACC_RDONLY, H5P_DEFAULT);
    hid_t dset_in_id = H5Dopen(file_in_id, dsets[0], H5P_DEFAULT);
    H5Dread(dset_in_id, part_type, H5S_ALL, H5S_ALL, H5P_DEFAULT, buf);
    H5Dclose(dset_in_id);
    H5Fclose(file_in_id);

    hid_t mem_space_id = H5Screate_simple(1, &lens[i], NULL);
    hsize_t zero_offset = 0;
    H5Sselect_hyperslab(mem_space_id, H5S_SELECT_SET, &zero_offset, &stride,
                        &strided_lens[i], NULL);
    H5Sselect_hyperslab(space_id, H5S_SELECT_SET, &offset, NULL,
                        &strided_lens[i], NULL);
    H5Dwrite(dset_id, part_type, mem_space_id, space_id, H5P_DEFAULT, buf);
    H5Sclose(mem_space_id);
    offset += strided_lens[i];
  }
  if (verbose_flag) printf("\nDone!\n\n");
  H5Fclose(file_id);
}

void PrintUsage(void) {
  printf("\nUsage: ./h5splice [options] <input files> -d <dataset> -f <fields> "
         "-o <output file>"
         "\n"
         "\nOptions:"
         "\n    -v, --verbose"
         "\n            Print what it's doing to stdout"
         "\n"
         "\n    -r N, --resample N"
         "\n            Only get every Nth datapoint"
         "\n"
         "\nExample:"
         "\n    ./h5splice 20130702*.h5 -d messages/kAioNodeControllerA"
         "/kMessageTypeControlTelemetry \n"
         "        -f C.message.time "
         "C.message.state_est.wind_g -o out.h5\n\n");
}

int32_t ParseInputs(int32_t argc, char **argv,
                    const char **in_files, int32_t *num_in_files,
                    const char **fields, const char **dsets,
                    int32_t *num_fields, int32_t *stride,
                    const char **out_file) {
  const char *dset = "";
  int32_t arg_ind = 1;
  ParseState parse_state = PARSE_STATE_INPUT;
  *num_in_files = 0;
  *num_fields = 0;
  *stride = 1;
  *out_file = NULL;

  while (arg_ind < argc) {
    if (!strncmp(argv[arg_ind], "-h", 3)
        || !strncmp(argv[arg_ind], "--help", 7)) {
      PrintUsage();
      exit(0);
    } else if (!strncmp(argv[arg_ind], "-v", 3)
               || !strncmp(argv[arg_ind], "--verbose", 10)) {
      verbose_flag = 1;
    } else if (!strncmp(argv[arg_ind], "-d", 3)
               || !strncmp(argv[arg_ind], "--dataset", 10)) {
      parse_state = PARSE_STATE_DATASET;
    } else if (!strncmp(argv[arg_ind], "-f", 3)
               || !strncmp(argv[arg_ind], "--fields", 9)) {
      parse_state = PARSE_STATE_FIELDS;
    } else if (!strncmp(argv[arg_ind], "-r", 3)
               || !strncmp(argv[arg_ind], "--resample", 11)) {
      arg_ind++;
      if (arg_ind < argc) {
        *stride = (int32_t)strtol(argv[arg_ind], NULL, 10);
        if (*stride <= 0) {
          printf("\nInvalid resampling");
          exit(1);
        }
      }
    } else if (!strncmp(argv[arg_ind], "-o", 3)
               || !strncmp(argv[arg_ind], "--output", 9)) {
      parse_state = PARSE_STATE_OUTPUT;
    } else {
      switch (parse_state) {
        case PARSE_STATE_INPUT:
          if (*num_in_files > H5SPLICE_MAX_FILES) {
            printf("\nToo many input files");
            exit(1);
          }
          in_files[*num_in_files] = (const char *)argv[arg_ind];
          (*num_in_files)++;
          break;
        case PARSE_STATE_DATASET:
          dset = (const char *)argv[arg_ind];
          break;
        case PARSE_STATE_FIELDS:
          if (*num_fields > H5SPLICE_MAX_FIELDS) {
            printf("\nToo many fields");
            exit(1);
          }
          fields[*num_fields] = (const char *)argv[arg_ind];
          dsets[*num_fields] = dset;
          (*num_fields)++;
          break;
        case PARSE_STATE_OUTPUT:
          *out_file = (const char *)argv[arg_ind];
          parse_state = PARSE_STATE_NONE;
          break;
        default:
        case PARSE_STATE_NONE:
          PrintUsage();
          return -1;
          break;
      }
    }
    arg_ind++;
  }

  if (*num_in_files == 0) {
    printf("\nError: No input files listed");
    PrintUsage();
    return -1;
  }

  if (*num_fields == 0) {
    printf("\nError: No fields listed");
    PrintUsage();
    return -1;
  }

  if (*out_file == NULL) {
    printf("\nError: No output file listed");
    PrintUsage();
    return -1;
  }

  return 0;
}

const char *GetDsetName(const char *dset_path) {
  return strrchr(dset_path, '/') + 1;
}

const char *GetGroupName(const char *dset_path, char *group_str) {
  uint32_t len = (uint32_t)(strrchr(dset_path, '/') - dset_path);
  strncpy(group_str, dset_path, len);
  group_str[len] = 0;
  return group_str;
}

int32_t MatchPatterns(const char *str,
                      const char **patterns, int32_t num_patterns) {
  for (int32_t i = 0; i < num_patterns; ++i)
    if (!strncmp(str, patterns[i], 256))
      return 1;
  return 0;
}

hid_t BuildPartialType(const char *head_str,
                       const char **patterns, int32_t num_patterns,
                       hid_t type_id) {
  char full_str[256];
  if (H5Tget_class(type_id) == H5T_COMPOUND) {
    hid_t tid = H5Tcreate(H5T_COMPOUND, H5Tget_size(type_id));
    int32_t n = H5Tget_nmembers(type_id);
    for (int32_t i = 0; i < n; ++i) {
      const char *member_name = H5Tget_member_name(type_id, (uint32_t)i);
      hid_t member_type = H5Tget_member_type(type_id, (uint32_t)i);
      size_t member_offset = H5Tget_member_offset(type_id, (uint32_t)i);

      snprintf(full_str, sizeof(full_str), "%s.%s", head_str, member_name);

      if (MatchPatterns(full_str, patterns, num_patterns)) {
        H5Tinsert(tid, member_name, member_offset, member_type);
      } else {
        hid_t partial = BuildPartialType(full_str, patterns, num_patterns,
                                         member_type);
        if (partial >= 0)
          H5Tinsert(tid, member_name, member_offset, partial);
      }
    }
    if (H5Tget_nmembers(tid) > 0)
      return tid;
  }
  return -1;
}

void DispType(hid_t type) {
  DispTypeHelper(type, "");
  printf("\n\n");
}

void DispTypeHelper(hid_t type, const char *spaces) {
  if (H5Tget_class(type) == H5T_COMPOUND) {
    int32_t n = H5Tget_nmembers(type);
    for (int32_t i = 0; i < n; ++i) {
      printf("\n%s%s", spaces, H5Tget_member_name(type, (uint32_t)i));
      char more_spaces[64] = "  ";
      snprintf(more_spaces, sizeof(more_spaces), "  %s", spaces);
      DispTypeHelper(H5Tget_member_type(type, (uint32_t)i), more_spaces);
    }
  }
}
