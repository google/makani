#!/usr/bin/python
# Copyright 2020 Makani Technologies LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""Generate ADIS16488 coefficients file."""

import csv
import sys
import textwrap

import gflags

gflags.DEFINE_string('coef_file', None,
                     'Full path to IMU coefficients file.')
gflags.DEFINE_string('source_file', None,
                     'Full path to output source file.')
gflags.DEFINE_string('header_file', None,
                     'Full path to output header file.')
FLAGS = gflags.FLAGS

_BLOCK_COEFS = 15
_COEFS_PER_BANK = 120
_COEFS_PER_PAGE = 60
_BLOCKS_PER_BANK = int(_COEFS_PER_BANK / _BLOCK_COEFS)
_BLOCKS_PER_PAGE = int(_COEFS_PER_PAGE / _BLOCK_COEFS)
_BLOCK_MOSI_SET_SIZE = 2 * _BLOCK_COEFS + 2
_BLOCK_MOSI_GET_SIZE = _BLOCK_COEFS + 2


def _get_page(bank, index):
  """Determine ADIS16488 register page for bank and coefficient index."""
  return 5 + 2 * (ord(bank) - ord('A')) + index / _COEFS_PER_PAGE


def _write_mosi_set(reader, f):
  """Write SPI MOSI data to set a given filter bank."""
  f.write(textwrap.dedent('''
      static const uint16_t kMosiSetCoef[kNumAdis16488CoefBanks][{0}][{1}] = {{
      '''.format(_BLOCKS_PER_BANK, _BLOCK_MOSI_SET_SIZE)))
  banks = 0
  for row, coefs in enumerate(reader):
    bank = chr(ord('A') + row)
    banks += 1

    # Validate coefficients.
    if len(coefs) != _COEFS_PER_BANK:
      raise Exception('Expected 120 coefficients per line.')
    if _BLOCKS_PER_PAGE * _BLOCK_COEFS != _COEFS_PER_PAGE:
      raise Exception('{0} not divisable by blocks size.'.format(
          _BLOCKS_PER_PAGE))

    f.write('  {{  // Coefficient bank {0}.\n'.format(bank))
    for index, coef in enumerate(coefs):
      block = index / _BLOCK_COEFS
      page = _get_page(bank, index)
      if index % _BLOCK_COEFS == 0:
        f.write(textwrap.dedent('''
            {0}{{ADIS16488_TURN_PAGE({1}),  // Coefficient bank {2}, block {3}.
            ''')[1:].format('   ', page, bank, block))
      f.write(textwrap.dedent('''
          {0}ADIS16488_WRITE_L(ADIS16488_PAGE{1:X}_FIR_COEF_{2}({3}), {4}),
          {0}ADIS16488_WRITE_H(ADIS16488_PAGE{1:X}_FIR_COEF_{2}({3}), {4}),
          ''')[1:].format('    ', page, bank, index, coef))
      if index % _BLOCK_COEFS == _BLOCK_COEFS - 1:
        f.write(textwrap.dedent('''
            {0}ADIS16488_TURN_PAGE(0)}},
            ''')[1:].format('    '))
    f.write('  },\n')
  f.write('};\n')
  return banks


def _write_mosi_get(reader, f):
  """Write SPI MOSI data to get from a filter bank."""
  f.write(textwrap.dedent('''
      static const uint16_t kMosiGetCoef[kNumAdis16488CoefBanks][{0}][{1}] = {{
      '''.format(_BLOCKS_PER_BANK, _BLOCK_MOSI_GET_SIZE)))
  for row, _ in enumerate(reader):
    bank = chr(ord('A') + row)
    f.write('  {{  // Coefficient bank {0}.\n'.format(bank))
    for index in range(0, _COEFS_PER_BANK):
      block = index / _BLOCK_COEFS
      page = _get_page(bank, index)
      if index % _BLOCK_COEFS == 0:
        f.write(textwrap.dedent('''
            {0}{{ADIS16488_TURN_PAGE({1}),  // Coefficient bank {2}, block {3}.
            ''')[1:].format('   ', page, bank, block))
      f.write(textwrap.dedent('''
          {0}ADIS16488_READ(ADIS16488_PAGE{1:X}_FIR_COEF_{2}({3})),
          ''')[1:].format('   ', page, bank, index))
      if index % _BLOCK_COEFS == _BLOCK_COEFS - 1:
        f.write(textwrap.dedent('''
            {0}ADIS16488_TURN_PAGE(0)}},
            ''')[1:].format('   '))
    f.write('  },\n')
  f.write('};\n')


def _write_source(csv_file, src_file):
  """Write output source file."""
  src_file.write(textwrap.dedent('''
      #include "avionics/firmware/drivers/adis16488_coef.h"

      #include <assert.h>
      #include <stdbool.h>
      #include <stddef.h>
      #include <stdint.h>

      #include "avionics/firmware/drivers/adis16488_reg.h"
      ''')[1:])
  csv_file.seek(0)
  banks = _write_mosi_set(csv.reader(csv_file), src_file)
  csv_file.seek(0)
  _write_mosi_get(csv.reader(csv_file), src_file)
  src_file.write(textwrap.dedent('''
      const uint16_t *Adis16488MosiSetCoef({0}, int32_t block) {{
        assert(0 <= bank && bank < kNumAdis16488CoefBanks);
        assert(0 <= block && block < ADIS16488_COEF_BLOCKS_PER_BANK);
        return kMosiSetCoef[bank][block];
      }}

      const uint16_t *Adis16488MosiGetCoef({0}, int32_t block) {{
        assert(0 <= bank && bank < kNumAdis16488CoefBanks);
        assert(0 <= block && block < ADIS16488_COEF_BLOCKS_PER_BANK);
        return kMosiGetCoef[bank][block];
      }}

      bool Adis16488ValidateCoef({0}, int32_t block, const uint16_t *miso) {{
        assert(0 <= bank && bank < kNumAdis16488CoefBanks);
        assert(0 <= block && block < ADIS16488_COEF_BLOCKS_PER_BANK);
        assert(miso != NULL);
        for (int32_t i = 0; i < ADIS16488_COEFS_PER_BLOCK; ++i) {{
          uint16_t mosi = (kMosiSetCoef[bank][block][2 * i + 1] & 0xFF)
              | (kMosiSetCoef[bank][block][2 * i + 2] & 0xFF) << 8;
          if (mosi != miso[i + 2]) {{
            return false;
          }}
        }}
        return true;
      }}
      '''.format('Adis16488CoefBank bank')))
  return banks


def _write_header(banks, f):
  """Write output header file."""
  guard = 'AVIONICS_TMS570_ADIS16488_COEF_H_'
  f.write(textwrap.dedent('''
      #ifndef {0}
      #define {0}

      #include <stdbool.h>
      #include <stdint.h>

      #define ADIS16488_COEFS_PER_BANK       {1}
      #define ADIS16488_COEFS_PER_PAGE       {2}
      #define ADIS16488_COEFS_PER_BLOCK      {3}
      #define ADIS16488_COEF_SET_MOSI_SIZE   {4}
      #define ADIS16488_COEF_GET_MOSI_SIZE   {5}
      #define ADIS16488_COEF_BLOCKS_PER_BANK {6}
      ''')[1:].format(guard, _COEFS_PER_BANK, _COEFS_PER_PAGE,
                      _BLOCK_COEFS, _BLOCK_MOSI_SET_SIZE,
                      _BLOCK_MOSI_GET_SIZE, _BLOCKS_PER_BANK))

  # Create Adis16488CoefBank enumeration.
  f.write(textwrap.dedent('''
      typedef enum {
        kAdis16488CoefBankInvalid = -1,
      '''))
  for row in range(0, banks):
    bank = chr(ord('A') + row)
    f.write(textwrap.dedent('''
        {0}kAdis16488CoefBank{1},
        ''')[1:].format('  ', bank))
  f.write(textwrap.dedent('''
        kNumAdis16488CoefBanks
      } Adis16488CoefBank;
      ''')[1:])

  # Create function prototypes.
  f.write(textwrap.dedent('''
      const uint16_t *Adis16488MosiSetCoef({0}, int32_t block);
      const uint16_t *Adis16488MosiGetCoef({0}, int32_t block);
      bool Adis16488ValidateCoef({0}, int32_t block, const uint16_t *miso);
      ''').format('Adis16488CoefBank bank'))

  f.write(textwrap.dedent('''
      #endif  // {0}
      ''').format(guard))


def main(argv):
  """Entry point."""
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], FLAGS)
    sys.exit(1)

  with open(FLAGS.coef_file, 'r') as csv_file:
    with open(FLAGS.source_file, 'w') as src_file:
      banks = _write_source(csv_file, src_file)
    with open(FLAGS.header_file, 'w') as hdr_file:
      _write_header(banks, hdr_file)


if __name__ == '__main__':
  gflags.MarkFlagAsRequired('coef_file')
  gflags.MarkFlagAsRequired('source_file')
  gflags.MarkFlagAsRequired('header_file')
  main(sys.argv)
