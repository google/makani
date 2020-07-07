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

#include "avionics/firmware/monitors/ltc6804.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/drivers/ltc6804.h"
#include "avionics/firmware/drivers/ltc6804_types.h"
#include "avionics/firmware/monitors/ltc6804_types.h"

#define LTC6804_WAIT_CYCLES CLOCK32_MSEC_TO_CYCLES(5000)

static struct {
  uint32_t next_read_time;
} g_ltc6804;

static int32_t CountBits(uint16_t x) {
  static const uint16_t mask[] = {
    0x5555,
    0x3333,
    0x0F0F,
    0x00FF,
  };

  for (int32_t i = 0, shift = 1; i < 4; ++i, shift *= 2) {
    x = (x & mask[i]) + ((x >> shift) & mask[i]);
  }
  return x;
}

static void CellIndexInit(const Ltc6804Monitors *config,
                          int32_t index_length, Ltc6804CellIndex *cells) {
  assert(config != NULL);
  int32_t count = 0;
  for (int32_t i = 0; i < config->num_devices; ++i) {
    count += CountBits(config->device[i].input_mask);
  }
  assert(config->device[0].num_series_cells == count);
  assert(index_length >= count);
  assert(cells != NULL);

  int32_t n = 0;
  for (int32_t i = 0; i < config->num_devices; ++i) {
    for (int32_t k = 0; k < LTC6804_NUM_CHANNELS; ++k) {
      if (1 << k & config->device[i].input_mask) {
        cells[n].index = n;
        cells[n].bms_chip = i;
        cells[n].bms_channel = k;
        cells[n].voltage = 0.0f;
        n++;
      }
    }
  }
}

static void SortCells(const Ltc6804CellIndex *cells,
                      int32_t num_cells, Ltc6804CellIndex *cells_sorted) {
  assert(cells != NULL);
  assert(cells_sorted != NULL);

  memcpy(cells_sorted, cells, num_cells * sizeof(Ltc6804CellIndex));
  // Sort in descending order.
  for (int32_t a = 1; a < num_cells; ++a)  {
    int32_t b = a;
    while (b > 0 && (cells_sorted[b].voltage > cells_sorted[b - 1].voltage)) {
      Ltc6804CellIndex buf = cells_sorted[b];
      cells_sorted[b] = cells_sorted[b - 1];
      cells_sorted[b - 1] = buf;
      b--;
    }
  }
}

static void GetCellVoltages(const Ltc6804Monitors *config,
                            const Ltc6804 *device, Ltc6804CellIndex *cells) {
  assert(config != NULL);
  assert(device != NULL);
  assert(cells != NULL);

  int32_t n = 0;
  for (int32_t i = 0; i < config->num_devices; ++i) {
    for (int32_t k = 0; k < LTC6804_NUM_CHANNELS; ++k) {
      if (1 << k & config->device[i].input_mask) {
        cells[n].voltage = device->data[i].cell_voltage[k];
        n++;
      }
    }
  }
}

static void UpdateBalancing(const Ltc6804Monitors *config, Ltc6804 *device,
                            const Ltc6804CellIndex *cells,
                            Ltc6804OutputData *data) {
  assert(config != NULL);
  assert(cells != NULL);

  int32_t num_cells = config->device[0].num_series_cells;
  Ltc6804CellIndex sorted[MAX_LTC6804_DEVICES * LTC6804_NUM_CHANNELS] = {{0}};
  uint16_t discharge[MAX_LTC6804_DEVICES] = {0};

  // Get previous state of discharge switches.
  for (int32_t i = 0; i < config->num_devices; ++i) {
    discharge[i] = device->data[i].wr_cfg.discharge_en_flag;
  }

  // Update error_count to max of those in daisy-chain to signal missing chip.
  data->error_count = 0;
  for (int32_t i = 0; i < config->num_devices; ++i) {
    if (device->data[i].error_count > data->error_count) {
      data->error_count = device->data[i].error_count;
    }
    device->data[i].error_count = 0;
  }

  // Stack voltage = sum of sum_of_cells from all series ltc6804s.
  float sum_all_cells = 0.0f;
  for (int32_t i = 0; i < num_cells; ++i) {
    sum_all_cells += cells[i].voltage;
  }
  data->stack_voltage = sum_all_cells;

  // Sort cell voltages in descending order
  SortCells(cells, num_cells, sorted);

  data->max_cell_v = sorted[0].voltage;
  data->min_cell_v = sorted[num_cells - 1].voltage;

  for (int32_t i = 0; i < num_cells; ++i) {
    // Prevent over-voltage condition.
    if (sorted[i].voltage >= config->device[0].control.over_volt_thres) {
      discharge[sorted[i].bms_chip] |= 1 << sorted[i].bms_channel;
    } else if ((i < config->device[0].num_max_simult_bal)
               && (i < num_cells - 1)) {
      // Enable balancing for cells that exceed v_balance_min volts.
      if (sorted[i].voltage > config->device[0].v_balance_min) {
        // Implement hysteresis control of discharge switches.
        if (sorted[i].voltage - sorted[num_cells - 1].voltage >
            (config->device[0].v_balance_thres +
             config->device[0].v_balance_hyst)) {
          discharge[sorted[i].bms_chip] |= 1 << sorted[i].bms_channel;
        } else if (sorted[i].voltage - sorted[num_cells - 1].voltage <
                   (config->device[0].v_balance_thres -
                    config->device[0].v_balance_hyst)) {
          discharge[sorted[i].bms_chip] &= ~(1 << sorted[i].bms_channel);
        } else {
          // Do nothing: keep the current state of the discharge switch.
        }
      } else {
        discharge[sorted[i].bms_chip] &= ~(1 << sorted[i].bms_channel);
      }
    } else {
      discharge[sorted[i].bms_chip] &= ~(1 << sorted[i].bms_channel);
    }
  }
  Ltc6804SetDischarge(discharge, device);
}

void Ltc6804MonitorInit(const Ltc6804Monitors *config, int32_t index_length,
                        Ltc6804CellIndex *cell_index, Ltc6804 *device) {
  const Ltc6804Control *ctrl = &(config->device[0].control);
  Ltc6804Init(ctrl, config->num_devices, device);
  CellIndexInit(config, index_length, cell_index);
  g_ltc6804.next_read_time = Clock32GetCycles();
}

bool Ltc6804MonitorPoll(const Ltc6804Monitors *config,
                        bool reduced_ltc6804_read_rate,
                        Ltc6804CellIndex *cell_index,
                        Ltc6804OutputFunction output_function,
                        Ltc6804 *device) {
  uint32_t now = Clock32GetCycles();
  if (CLOCK32_GT(g_ltc6804.next_read_time, now)) {
    return true;
  }
  if (Ltc6804PollData(ClockGetUs(), device)) {
    if (device->data_ready) {
      Ltc6804OutputData data;
      // Sort cell voltages in descending order and determine discharge.
      GetCellVoltages(config, device, cell_index);
      UpdateBalancing(config, device, cell_index, &data);
      // Send data to structure for AIO message and board logic.
      output_function(&data);
      g_ltc6804.next_read_time = now;
      if (reduced_ltc6804_read_rate) {
        g_ltc6804.next_read_time += LTC6804_WAIT_CYCLES;
      }
    }
    return true;
  } else {
    return false;
  }
}
