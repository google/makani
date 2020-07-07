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

#include "avionics/ground_power/q7/inverter.h"

#include <mqueue.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/ground_power/q7/flags.h"
#include "avionics/linux/aio.h"
#include "common/macros.h"

extern mqd_t g_inverter_data_queues[kNumInverters];
extern mqd_t g_inverter_command_queues[kNumInverters];
extern mqd_t g_inverter_ack_queues[kNumInverters];
extern InverterBank g_inverter_bank;
GroundPowerStatusMessage g_status_message[kNumInverters] = {{0}};

static const char *kInverterIpAddresses[] = {"192.168.0.16", "192.168.0.17",
                                             "192.168.0.18", "192.168.0.19",
                                             "192.168.0.20", "192.168.0.21"};

// Read these modbus parameters to construct the GroundPowerStatusMessage.
static const InverterModbusReadParam inverter_read_param_list[] = {
  {kInverterRegisterFaultData, kInverterParamsFaultDataLen},
  {kInverterRegisterVdc, kInverterParamsVIDataLen},
  {kInverterRegisterVCommonMode, kInverterParamsVCommonModeDataLen},
  {kInverterRegisterCtrlBoardAirTemp, kInverterParamsTemperatureDataLen},
  {kInverterRegisterSetFaultInductor, kInverterParamsFaultInductorDataLen},
  {kInverterRegisterTetherCompCalc, kInverterParamsTetherCompCalcDataLen},
  {kInverterRegisterMeanPower, kInverterParamsMeanPowerDataLen},
  {kInverterRegisterL1, kInverterParamsL1DataLen},
};

void InverterBankInit(InverterBank *inverters) {
  inverters->size = kNumInverters;

  for (int32_t j = 0; j < inverters->size; ++j) {
    g_status_message[j].id = (uint8_t)j;
    g_status_message[j].inverter_status = kInverterStatusInit;

    inverters->inverter_array[j].mb_status = false;
    inverters->inverter_array[j].id = (uint8_t)j;
    inverters->inverter_array[j].mb =
        modbus_new_tcp(kInverterIpAddresses[j], kInverterModbusParamsTcpPort);

    int16_t rc1 = (int16_t)modbus_set_response_timeout(
        inverters->inverter_array[j].mb, 0, kInverterModbusParamsMsgTimeout);
    int16_t rc2 = (int16_t)modbus_set_byte_timeout(
        inverters->inverter_array[j].mb, 0, kInverterModbusParamsCharTimeout);
    int16_t rc3 = (int16_t)modbus_set_error_recovery(
        inverters->inverter_array[j].mb,
        MODBUS_ERROR_RECOVERY_PROTOCOL | MODBUS_ERROR_RECOVERY_LINK);
    int16_t rc4 = (int16_t)modbus_set_slave(inverters->inverter_array[j].mb, 1);
    if (rc1 < 0 || rc2 < 0 || rc3 < 0 || rc4 < 0) {
      printf("ground_power_monitor: Unable to set modbus parameters.");
      exit(EXIT_FAILURE);
    }
  }
}

static bool DataReadAttempt(uint16_t modbus_register, uint16_t datalen,
                            uint16_t max_tries, uint16_t *data_address,
                            Inverter *inverter) {
  int32_t read_result = 0;
  int32_t error_count = 0;
  bool success = false;

  while (error_count < max_tries) {
    read_result = modbus_read_registers(inverter->mb, modbus_register, datalen,
                                        data_address);
    if (read_result > 0) {
      success = true;
      return success;
    }
    error_count++;
  }
  return success;
}

// Handle the inbound GroundPowerCommand, GroundPowerGetParam,
// GroundPowerSetParam messages.
static void HandleInverterCommand(Inverter *inverter) {
  uint8_t id = inverter->id;

  GroundPowerCommandQueueMessage message;
  bzero((char *)&message, sizeof(GroundPowerCommandQueueMessage));
  ssize_t bytes_read =
      mq_receive(g_inverter_command_queues[id], (char *)&message,
                 sizeof(GroundPowerCommandQueueMessage), 0);

  if (bytes_read > 0) {
    uint16_t param_value;
    bool read_success = false;

    switch (message.message_type) {
      case kInverterMessageTypeSetParam: {
        // Handle GroundPowerSetParamMessage
        GroundPowerSetParamMessage *set_param_message =
            &message.message_union.set_param_message;

        // Write the value to the modbus register.
        modbus_write_register(inverter->mb,
                              set_param_message->modbus_register,
                              set_param_message->value);
        read_success = DataReadAttempt(set_param_message->modbus_register,
                                       kInverterModbusParamsGetParamDataLen,
                                       kInverterModbusParamsMaxQueryRetries,
                                       &param_value, inverter);
        // On success, send GroundPowerAckParamMessage.
        if (read_success) {
          GroundPowerAckParamMessage ack_param = {
            set_param_message->id, set_param_message->modbus_register,
            set_param_message->value};

          // Enqueue ack param message to be sent over AIO network.
          mq_send(g_inverter_ack_queues[inverter->id],
                  (const char *)(&ack_param),
                  sizeof(GroundPowerAckParamMessage), 0);
        }
        break;
      }
      case kInverterMessageTypeGetParam: {
        // Handle GroundPowerGetParamMessage
        GroundPowerGetParamMessage *get_param_message =
            &message.message_union.get_param_message;

        read_success = DataReadAttempt(get_param_message->modbus_register,
                                       kInverterModbusParamsGetParamDataLen,
                                       kInverterModbusParamsMaxQueryRetries,
                                       &param_value, inverter);
        if (read_success) {
          GroundPowerAckParamMessage ack_param = {
            get_param_message->id, get_param_message->modbus_register,
            param_value};

          // Enqueue ack param message to be sent over AIO network.
          mq_send(g_inverter_ack_queues[inverter->id],
                  (const char *)(&ack_param),
                  sizeof(GroundPowerAckParamMessage), 0);
        }
        break;
      }
      case kInverterMessageTypeCommand: {
        // Handle GroundPowerCommandMessage
        GroundPowerCommandMessage *command_message =
            &message.message_union.command_message;

        switch (command_message->command) {
          case kGroundPowerCommandStart:
            // start inverter with correct id.
            modbus_write_register(inverter->mb, kInverterRegisterCommand,
                                  kInverterCommandStart);
            break;
          case kGroundPowerCommandStop:
            // stop inverter with correct id.
            modbus_write_register(inverter->mb, kInverterRegisterCommand,
                                  kInverterCommandStop);
            break;
          case kGroundPowerCommandReset:
            // reset the inverter
            modbus_write_register(inverter->mb, kInverterRegisterCommand,
                                  kInverterCommandReset);
            break;
          default:
            break;
        }
        break;
      }
      default:
        break;
    }
  }
}

// Query the modbus registers of the inverters and populate
// GroundPowerStatusMessage.
static void GetInverterData(Inverter *inverter, GroundPowerStatusMessage *msg) {
  uint16_t data_vec[kInverterParamsDataVectorSize];

  msg->id = (uint8_t)(inverter->id);
  msg->modbus_status = true;

  uint16_t inverter_status;
  int32_t offset = 0;

  for (int32_t i = 0; i < ARRAYSIZE(inverter_read_param_list); ++i) {
    InverterModbusReadParam current_params = inverter_read_param_list[i];

    if (DataReadAttempt(current_params.modbus_register,
                        current_params.modbus_datalen,
                        kInverterModbusParamsMaxQueryRetries,
                        &data_vec[offset], inverter)) {
      switch (current_params.modbus_register) {
        case kInverterRegisterFaultData:
          // TODO: Move fault_word* into an array.
          msg->fault_word1 = data_vec[kInverterParamsFaultData];
          msg->fault_word2 = data_vec[kInverterParamsFaultData + 1];
          msg->fault_word3 = data_vec[kInverterParamsFaultData + 2];
          msg->fault_word4 = data_vec[kInverterParamsFaultData + 3];
          msg->fault_word5 = data_vec[kInverterParamsFaultData + 4];
          msg->fault_word6 = data_vec[kInverterParamsFaultData + 5];
          msg->fault_word7 = data_vec[kInverterParamsFaultData + 6];
          msg->fault_word8 = data_vec[kInverterParamsFaultData + 7];

          // Set InverterStatusFlag
          inverter_status =
              (msg->fault_word1 | msg->fault_word2 | msg->fault_word3 |
               msg->fault_word4 | msg->fault_word5 | msg->fault_word6 |
               msg->fault_word7 | msg->fault_word8);

          switch (inverter_status) {
            case kInverterFaultStateRunning:
              msg->inverter_status = kInverterStatusRunning;
              break;
            case kInverterFaultStateStopped:
              msg->inverter_status = kInverterStatusStopped;
              break;
            case kInverterFaultStateEStopped:
              msg->inverter_status = kInverterStatusEStopped;
              break;
            default:
              msg->inverter_status = kInverterStatusFaulted;
          }
          break;
        case kInverterRegisterVdc:
          msg->v_dc = 0.1f * (int16_t)data_vec[kInverterParamsVIData];
          msg->i_dc = 0.1f * (int16_t)data_vec[kInverterParamsVIData + 3];

          msg->grid_v_ab = 0.1f * (int16_t)data_vec[kInverterParamsVIData + 6];
          msg->grid_v_bc = 0.1f * (int16_t)data_vec[kInverterParamsVIData + 7];
          msg->grid_v_ca = 0.1f * (int16_t)data_vec[kInverterParamsVIData + 8];
          msg->v_avg_grid =
              0.033333f * ((int16_t)(data_vec[kInverterParamsVIData + 6] +
                                     data_vec[kInverterParamsVIData + 7] +
                                     data_vec[kInverterParamsVIData + 8]));

          msg->grid_i_a = 0.1f * (int16_t)data_vec[kInverterParamsVIData + 9];
          msg->grid_i_b = 0.1f * (int16_t)data_vec[kInverterParamsVIData + 10];
          msg->grid_i_c = 0.1f * (int16_t)data_vec[kInverterParamsVIData + 11];
          msg->i_avg_grid =
              0.033333f * ((int16_t)(data_vec[kInverterParamsVIData + 9] +
                                     data_vec[kInverterParamsVIData + 10] +
                                     data_vec[kInverterParamsVIData + 11]));
          break;
        case kInverterRegisterVCommonMode:
          msg->mean_common_mode_v =
              0.1f * (int16_t)data_vec[kInverterParamsVCommonModeData];
          msg->inst_common_mode_v =
              0.1f * (int16_t)data_vec[kInverterParamsVCommonModeData + 1];
          break;
        case kInverterRegisterCtrlBoardAirTemp:
          msg->cb_air_temp =
              0.1f * (int16_t)data_vec[kInverterParamsTemperatureData];
          msg->inverter_air_temp =
              0.1f * (int16_t)data_vec[kInverterParamsTemperatureData + 1];
          msg->transformer_temp =
              0.1f * (int16_t)data_vec[kInverterParamsTemperatureData + 2];
          msg->heatsink1_temp1 =
              0.1f * (int16_t)data_vec[kInverterParamsTemperatureData + 3];
          msg->heatsink1_temp2 =
              0.1f * (int16_t)data_vec[kInverterParamsTemperatureData + 4];
          break;
        case kInverterRegisterSetFaultInductor:
          msg->fault_inductor_status =
              data_vec[kInverterParamsFaultInductorData];
          break;
        case kInverterRegisterTetherCompCalc:
          msg->tether_compensation_calc =
              0.1f * data_vec[kInverterParamsTetherCompCalcData];
          break;
        case kInverterRegisterMeanPower:
          msg->p_mean_active_dc =
              0.1f * (int16_t)data_vec[kInverterParamsMeanPowerData];
          msg->p_mean_active_ac =
              0.1f * (int16_t)data_vec[kInverterParamsMeanPowerData + 1];
          msg->p_mean_reactive_ac =
              0.1f * (int16_t)data_vec[kInverterParamsMeanPowerData + 2];
          break;
        case kInverterRegisterL1:
          msg->l1_i_a = (int16_t)data_vec[kInverterParamsL1Data];
          msg->l1_i_b = (int16_t)data_vec[kInverterParamsL1Data + 1];
          msg->l1_i_c = (int16_t)data_vec[kInverterParamsL1Data + 2];
          break;
        default:
          break;
      }
    } else {
      // Indicate failure if any register read fails.
      msg->modbus_status = false;
      msg->stale_count++;
    }

    offset += current_params.modbus_datalen;
  }

  if (msg->modbus_status) {
    msg->stale_count = 0;
  }
}

void *InverterCommunication(void *inverter_arg) {
  Inverter *inverter = (Inverter *)inverter_arg;

  GroundPowerStatusMessage *status_message = &g_status_message[inverter->id];

  // Once modbus_connect() runs successfully, it is not necessary to ever run it
  // again, even if the modbus slave temporarily loses connection or is power
  // cycled. Libmodbus can continually try to recover from connection problems.
  // This option was enabled by calling modbus_error_recovery().
  while (!inverter->mb_status) {
    GroundPowerCommandQueueMessage message;

    if (modbus_connect(inverter->mb) == 0) {
      inverter->mb_status = true;
    } else {
      // Close connection before retrying.
      modbus_close(inverter->mb);
      // Clear inverter command message queue.
      mq_receive(g_inverter_command_queues[inverter->id], (char *)&message,
                 sizeof(GroundPowerCommandQueueMessage), 0);
    }
    status_message->id = (uint8_t)(inverter->id);
    status_message->modbus_status = inverter->mb_status;

    // Place message in queue. If queue is full, return immediately.
    mq_send(g_inverter_data_queues[inverter->id],
            (const char *)status_message, sizeof(*status_message), 0);
  }

  while (true) {
    HandleInverterCommand(inverter);
    GetInverterData(inverter, status_message);
    // Place message in queue. If queue is full, return immediately.
    // TODO: We should maintain a CVT so that we guarantee that we're
    // sending the most recent data.
    mq_send(g_inverter_data_queues[inverter->id],
            (const char *)status_message, sizeof(*status_message), 0);
  }
}

