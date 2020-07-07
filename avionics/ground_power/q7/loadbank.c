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

#include "avionics/ground_power/q7/loadbank.h"

#include <math.h>
#include <modbus.h>
#include <mqueue.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/ground_power/q7/loadbank_types.h"
#include "common/macros.h"

extern mqd_t g_loadbank_command_queues[kNumLoadbanks];

static const char *kLoadbankIpAddresses[] = {"192.168.0.31", "192.168.0.32"};
static const char *kSharkKiteLoadIpAddress = "192.168.0.29";
static const char *kSharkLoadbankLoadIpAddress = "192.168.0.30";

static const float kSharkGlitchThreshold = 1.0e7f;  // [W]

// Implement hysteresis on the requested loadbank load to prevent loadbank
// relays from rapidly switching on and off due to small fluctuations in
// power readings.
static int32_t Hysteresis(int32_t input) {
  static int32_t tracking = 0;

  if (input - tracking > kLoadbankParamsHysteresis) {
    tracking = input - kLoadbankParamsHysteresis;
  }
  if (input - tracking < -kLoadbankParamsHysteresis) {
    tracking = input + kLoadbankParamsHysteresis;
  }
  return tracking;
}

// The two loadbank addresses have 7 and 8 relays, respectively, controlling
// 100 kW per relay for a sum total of 1.5MW available load.
// This function builds a mask for controlling the 15 relays based on
// required load.
static uint16_t SetLoad(int32_t n_requested_relays) {
  static uint16_t loadmask = 0;  // Would need bigger type for kNumLoadbanks>2.
  const uint16_t mask_one = 1;
  static int32_t maxbit_shift = 0;
  static int32_t minbit_shift = 0;
  static int32_t n_active_relays = 0;

  if (n_requested_relays < 0) n_requested_relays = 0;
  if (n_requested_relays > kLoadbankParamsNLoadMax) {
    n_requested_relays = kLoadbankParamsNLoadMax;
  }

  while (n_active_relays < n_requested_relays) {
    // Advance max bit.
    loadmask |= (uint16_t)(mask_one << maxbit_shift);
    maxbit_shift += 1;
    maxbit_shift %= kLoadbankParamsNLoadSteps;
    n_active_relays++;
  }
  while (n_active_relays > n_requested_relays) {
    // Advance min bit.
    loadmask &= (uint16_t)(~(mask_one << minbit_shift));
    minbit_shift += 1;
    minbit_shift %= kLoadbankParamsNLoadSteps;
    n_active_relays--;
  }
  return loadmask;
}

// Rotate data from modbus to be correct for interpreting as float.
// TODO: Revisit how to change the endianness of the shark messages.
static void SwapSharkMessageRegs(SharkMessage *shark_message) {
  uint16_t swap_value;
  // Swap power data read regs.
  swap_value = shark_message->power.raw_regs[0];
  shark_message->power.raw_regs[0] = shark_message->power.raw_regs[1];
  shark_message->power.raw_regs[1] = swap_value;
  // Swap VARs data read regs.
  swap_value = shark_message->VARs.raw_regs[0];
  shark_message->VARs.raw_regs[0] = shark_message->VARs.raw_regs[1];
  shark_message->VARs.raw_regs[1] = swap_value;
  // Swap VAs data read regs.
  swap_value = shark_message->VAs.raw_regs[0];
  shark_message->VAs.raw_regs[0] = shark_message->VAs.raw_regs[1];
  shark_message->VAs.raw_regs[1] = swap_value;
}

void LoadbankSetInit(LoadbankSet *loadbanks) {
  int32_t rc1, rc2, rc3, rc4;

  loadbanks->size = kNumLoadbanks;
  for (int32_t j = 0; j < loadbanks->size; ++j) {
    loadbanks->loadbank_array[j].mb_status = false;
    loadbanks->loadbank_array[j].id = (uint8_t)j;
    loadbanks->loadbank_array[j].mb =
        modbus_new_tcp(kLoadbankIpAddresses[j], kLoadbankParamsTcpPort);
    rc1 = modbus_set_response_timeout(
        loadbanks->loadbank_array[j].mb, 0, kLoadbankParamsMsgTimeout);
    rc2 = modbus_set_byte_timeout(
        loadbanks->loadbank_array[j].mb, 0, kLoadbankParamsCharTimeout);
    rc3 = modbus_set_error_recovery(
        loadbanks->loadbank_array[j].mb,
        MODBUS_ERROR_RECOVERY_PROTOCOL | MODBUS_ERROR_RECOVERY_LINK);
    rc4 = modbus_set_slave(loadbanks->loadbank_array[j].mb, 1);
    if (rc1 < 0 || rc2 < 0 || rc3 < 0 || rc4 < 0) {
      printf("Loadbank Adam relay control: Unable to set modbus parameters.");
    }
  }
}

void SharkInit(SharkMeters *shark_meters) {
  int32_t rc1, rc2, rc3, rc4;

  shark_meters->kite_mb_status = false;
  shark_meters->loadbank_mb_status = false;
  shark_meters->status_message.desired_net_load_kw = 0;
  shark_meters->status_message.loadbank_activated = 0;
  shark_meters->kite_load_mb =
      modbus_new_tcp(kSharkKiteLoadIpAddress, kSharkParamsTcpPort);
  rc1 = modbus_set_response_timeout(
      shark_meters->kite_load_mb, 0, kSharkParamsMsgTimeout);
  rc2 = modbus_set_byte_timeout(
      shark_meters->kite_load_mb, 0, kSharkParamsCharTimeout);
  rc3 = modbus_set_error_recovery(
      shark_meters->kite_load_mb,
      MODBUS_ERROR_RECOVERY_PROTOCOL | MODBUS_ERROR_RECOVERY_LINK);
  rc4 = modbus_set_slave(shark_meters->kite_load_mb, 1);
  if (rc1 < 0 || rc2 < 0 || rc3 < 0 || rc4 < 0) {
    printf("Kite Shark power meter: Unable to set modbus parameters.");
  }

  shark_meters->loadbank_load_mb =
      modbus_new_tcp(kSharkLoadbankLoadIpAddress, kSharkParamsTcpPort);
  rc1 = modbus_set_response_timeout(
      shark_meters->loadbank_load_mb, 0, kSharkParamsMsgTimeout);
  rc2 = modbus_set_byte_timeout(
      shark_meters->loadbank_load_mb, 0, kSharkParamsCharTimeout);
  rc3 = modbus_set_error_recovery(
      shark_meters->loadbank_load_mb,
      MODBUS_ERROR_RECOVERY_PROTOCOL | MODBUS_ERROR_RECOVERY_LINK);
  rc4 = modbus_set_slave(shark_meters->loadbank_load_mb, 1);
  if (rc1 < 0 || rc2 < 0 || rc3 < 0 || rc4 < 0) {
    printf("Loadbank Shark power meter: Unable to set modbus parameters.");
  }
}

// Send loadbank relay control messages from queue to loadbank modbus.
void *LoadbankCommandThread(void *loadbank_arg) {
  Loadbank *loadbank = (Loadbank *)loadbank_arg;
  uint8_t lb_id = loadbank->id;
  static bool connected = false;

  char command[kLoadbankParamsRelaysPerBank];

  // Once modbus_connect() runs successfully, it is not necessary to ever run it
  // again, even if the modbus slave temporarily loses connection or is power
  // cycled. Libmodbus can continually try to recover from connection problems.
  // This option was enabled by calling modbus_error_recovery().
  while (!connected) {
    if (modbus_connect(loadbank->mb) == 0) {
      loadbank->mb_status = true;
      connected = true;
    } else {
      modbus_close(loadbank->mb);
    }
  }

  while (true) {
    ssize_t bytes_read = mq_receive(g_loadbank_command_queues[lb_id], command,
                                    sizeof(command[0]) *
                                    kLoadbankParamsRelaysPerBank, 0);
    // If received message from queue, send to loadbank modbus.
    if (bytes_read > 0) {
      if (modbus_write_bits(loadbank->mb, kLoadbankParamsRelayAddress,
                            kLoadbankParamsRelaysPerBank, (uint8_t *)command)
          < 0) {
        loadbank->mb_status = false;
        perror("Adam relay controller write failed.");
      } else {
        loadbank->mb_status = true;
      }
    }
  }
}

void *LoadbankRelayCalcThread(void *shark_meters_arg) {
  SharkMeters *shark_meters = (SharkMeters *)shark_meters_arg;
  static bool connected = false;
  SharkMessage shark_message;
  int32_t n_requested_relays;

  // Once modbus_connect() runs successfully, it is not necessary to ever run it
  // again, even if the modbus slave temporarily loses connection or is power
  // cycled. Libmodbus can continually try to recover from connection problems.
  // This option was enabled by calling modbus_error_recovery().
  while (!connected) {
    if (modbus_connect(shark_meters->kite_load_mb) == 0) {
      shark_meters->kite_mb_status = true;
      connected = true;
    } else {
      modbus_close(shark_meters->kite_load_mb);
    }
  }

  while (true) {
    // If new kite load data from Shark, calculate and enqueue relay settings.
    int32_t rc = modbus_read_registers(shark_meters->kite_load_mb,
                                       kSharkParamsDataRegAddr,
                                       kSharkParamsNumDataRegs,
                                       (uint16_t *)(&shark_message));

    if (rc == kSharkParamsNumDataRegs) {  // Read was successful.
      SwapSharkMessageRegs(&shark_message);  // Rotate data to be float-ready.

      // Calculate amount of load n [*100kW] needed from loadbanks.
      float kite_load_kw = (float)(shark_message.power.float_value / 1000.0);

      if (fabsf(shark_message.power.float_value) < kSharkGlitchThreshold) {
        shark_meters->kite_mb_status = true;

        int32_t requested_load_kw =
            (shark_meters->status_message.desired_net_load_kw
             - (int32_t)kite_load_kw);
        requested_load_kw = Hysteresis(requested_load_kw);

        // Round to nearest step size if loadbank on, else open all relays.
        if (shark_meters->status_message.loadbank_activated) {
          n_requested_relays =
              (requested_load_kw + kLoadbankParamsStepSize / 2)
              / kLoadbankParamsStepSize;
        } else {
          n_requested_relays = 0;
        }

        // Get relay activation bitmasks based on requested # active relays.
        uint16_t active_relay_mask = SetLoad(n_requested_relays);

        // Split bits out into arrays of bytes -- 8 for each loadbank.
        uint8_t dest[kLoadbankParamsRelaysPerBank * kNumLoadbanks];
        modbus_set_bits_from_bytes(dest, 0, ARRAYSIZE(dest),
                                   (uint8_t *)(&active_relay_mask));

        // Send relay settings through queues.
        // Big end of mask goes to loadbank 0.
        // TODO: Swap order of loadbank IP addresses so that loadbank
        // 0 has 8 relays and loadbank 1 has 7 relays.
        for (int32_t lb_id = 0; lb_id < kNumLoadbanks; ++lb_id) {
          mq_send(g_loadbank_command_queues[lb_id],
                  (const char *)(dest + (kLoadbankParamsRelaysPerBank * lb_id)),
                  sizeof(dest[0]) * kLoadbankParamsRelaysPerBank, 0);
        }

        shark_meters->status_message.n_requested_relays = n_requested_relays;
        shark_meters->status_message.relay_mask = active_relay_mask;
      } else {
        shark_meters->kite_mb_status = false;
      }

      // Store info for LoadbankStatusMessage back to operator.
      shark_meters->status_message.kite_power_kw = kite_load_kw;
      shark_meters->status_message.kite_kvar =
          (float)(shark_message.VARs.float_value / 1000.0);
      shark_meters->status_message.kite_kva =
          (float)(shark_message.VAs.float_value / 1000.0);
    } else {
      shark_meters->kite_mb_status = false;
      perror("Kite Shark power meter read failed.");
    }
  }
}

// Read loadbank data, which will later be sent and logged over AIO network.
void *LoadbankDataThread(void *shark_meters_arg) {
  SharkMeters *shark_meters = (SharkMeters *)shark_meters_arg;
  static bool connected = false;
  SharkMessage shark_message;

  // Once modbus_connect() runs successfully, it is not necessary to ever run it
  // again, even if the modbus slave temporarily loses connection or is power
  // cycled. Libmodbus can continually try to recover from connection problems.
  // This option was enabled by calling modbus_error_recovery().
  while (!connected) {
    if (modbus_connect(shark_meters->loadbank_load_mb) == 0) {
      shark_meters->loadbank_mb_status = true;
      connected = true;
    } else {
      modbus_close(shark_meters->loadbank_load_mb);
    }
  }

  while (true) {
    // If new loadbank load data from Shark, update LoadbankStatusMessage.
    int32_t rc = modbus_read_registers(shark_meters->loadbank_load_mb,
                                       kSharkParamsDataRegAddr,
                                       kSharkParamsNumDataRegs,
                                       (uint16_t *)(&shark_message));

    if (rc == kSharkParamsNumDataRegs) {  // Read was successful.
      SwapSharkMessageRegs(&shark_message);  // Rotate data to be float-ready.

      shark_meters->loadbank_mb_status =
          (fabsf(shark_message.power.float_value) < kSharkGlitchThreshold);

      shark_meters->status_message.loadbank_power_kw =
          (float)(shark_message.power.float_value / 1000.0);
      shark_meters->status_message.loadbank_kvar =
          (float)(shark_message.VARs.float_value / 1000.0);
      shark_meters->status_message.loadbank_kva =
          (float)(shark_message.VAs.float_value / 1000.0);
    } else {
      shark_meters->loadbank_mb_status = false;
      perror("Loadbank Shark power meter read failed.");
    }
  }
}
