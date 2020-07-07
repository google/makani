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

#include "avionics/loadcell/firmware/bridle_junc_can.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/common/faults.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/dcan.h"
#include "avionics/loadcell/firmware/calib_params.h"
#include "avionics/loadcell/firmware/output.h"
#include "common/macros.h"

#define BRIDLE_JUNC_BUS kDcanBus1  // Dcan1 from ground test interface.
#define PIN_NODE_ID 1  // Default load pin CAN node ID.
#define PIN_CONFIG_MESSAGE_LENGTH 2
#define PIN_DATA_MESSAGE_LENGTH 4  // Configured to send float as 4 bytes.
#define ENCODER_NODE_ID 15  // Default encoder ID with all address pins open.
#define ENCODER_CONFIG_MESSAGE_LENGTH 8
#define ENCODER_DATA_MESSAGE_LENGTH 4
#define NMT_BROADCAST_LENGTH 2

#define CONFIG_RESEND_TIMEOUT CLOCK32_MSEC_TO_CYCLES(50)
#define READ_WARNING_TIMEOUT CLOCK32_MSEC_TO_CYCLES(1000)

#define LBF_TO_NEWTONS 4.44822f  // Pounds force to newtons.
#define BYTES_TO_RAD (7.6708403e-4f)  // (2 * pi / 0x1FFF).
// See docs/document/d/1QcGIPQjwP0TiAO9hV9D3bXOOytOcb6y3Fu5O9HCIHbc
// "zero roll angle" section for yoke-tether-roll-offset value.
// May change in the future if we adjust bridle lengths.
#define YOKE_TETHER_ROLL_OFFSET 4.0143e-2f  // 2.3 degrees in radians.

typedef enum {
  // Transmit mailboxes.
  kBridleJuncMailboxPinConfig     = 0,  // Configure load pin.
  kBridleJuncMailboxEncoderConfig = 1,  // Configure encoder.
  kBridleJuncMailboxNmt           = 3,  // Nmt broadcast messages.
  // Receive mailboxes.
  kBridleJuncMailboxPinRecvd      = 32,  // Load pin config verification.
  kBridleJuncMailboxEncoderRecvd  = 33,  // Encoder config verification.
  kBridleJuncMailboxPinData       = 34,  // Load pin cyclic data transmission.
  kBridleJuncMailboxEncoderData   = 35,  // Encoder cyclic data transmission.
} BridleJuncMailbox;

typedef enum {
  kLoadpinStateInit,   // Configure load pin.
  kLoadpinStateData,   // Receive data from load pin.
  kLoadpinStateError,  // Handle communication errors.
} LoadpinState;

typedef enum {
  kEncoderStateInit,      // Configure encoder.
  kEncoderStateNmtStart,  // Send NMT "start data transmission" message.
  kEncoderStateData,      // Receive data from encoder.
  kEncoderStateError,     // Handle communication errors.
} EncoderState;

// Bridle pin CAN bus state machine variables.
static struct {
  LoadpinState loadpin_state;  // Present load pin comms state.
  EncoderState encoder_state;  // Present encoder comms state.
  bool loadpin_first_entry;  // First entry into a new state-handling function.
  bool encoder_first_entry;
  uint32_t loadpin_read_timeout;
  uint32_t encoder_read_timeout;
} g_bridle_junc;

// Convert load pin data to engineering units for AIO status output.
static float BytesToFloatBe(uint8_t data_bytes[4]) {
  assert(data_bytes != NULL);
  uint32_t data_int = (data_bytes[0] << 24 | data_bytes[1] << 16 |
                       data_bytes[2] << 8  | data_bytes[3] << 0);
  float *measurement = (float*)(&data_int);
  return *measurement;
}

// Send DLCHMCAN load pin STRMON message for streaming data.
static void SendPinConfig(void) {
  uint8_t config_msg[PIN_CONFIG_MESSAGE_LENGTH] = {0x02, 0x80};
  DcanTransmit(BRIDLE_JUNC_BUS,
               kBridleJuncMailboxPinConfig,
               PIN_CONFIG_MESSAGE_LENGTH,
               config_msg);
}

// Send message configuring encoder for 10ms cyclic transmission of angle data.
static void SendEncoderConfig(void) {
  uint8_t config_msg[ENCODER_CONFIG_MESSAGE_LENGTH] =
      {0x22, 0x00, 0x62, 0x00, 0x0A, 0x00, 0x00, 0x00};
  DcanTransmit(BRIDLE_JUNC_BUS,
               kBridleJuncMailboxEncoderConfig,
               ENCODER_CONFIG_MESSAGE_LENGTH,
               config_msg);
}

// Convert encoder data to radians and subtract offset to zero correctly.
static float TetherRollRadians(uint16_t encoder_data) {
  float measurement_rads = BYTES_TO_RAD * encoder_data;
  float port_pi2 = kLoadcellCalibParams->bridle_calib.encoder_pi_over_2_port;
  float stbd_pi2 = kLoadcellCalibParams->bridle_calib.encoder_pi_over_2_stbd;
  float sign_correction = (port_pi2 - stbd_pi2 >= 0) ? 1.0f : -1.0f;
  float zero_point = (port_pi2 + stbd_pi2) / 2.0f -
      sign_correction * YOKE_TETHER_ROLL_OFFSET;
  float angle_rads = sign_correction * (measurement_rads - zero_point);
  return angle_rads;
}

// Initialize the CAN bus used for encoder and load pin comms.
static void StartCarrierCanBus(void) {
  DcanInit(BRIDLE_JUNC_BUS, kDcanBitRate125kbps);  // Start tms570 CAN bus.

  // Mailbox for configuring load pin to send cyclic transmission.
  DcanSetTransmitMailbox(BRIDLE_JUNC_BUS,
                         kBridleJuncMailboxPinConfig,
                         kDcanIdStandard,
                         DCAN_STANDARD_MASK,
                         PIN_NODE_ID,
                         PIN_CONFIG_MESSAGE_LENGTH);

  // Mailbox for configuring encoder to send cyclic transmission.
  DcanSetTransmitMailbox(BRIDLE_JUNC_BUS,
                         kBridleJuncMailboxEncoderConfig,
                         kDcanIdStandard,
                         DCAN_STANDARD_MASK,
                         ENCODER_NODE_ID + 0x600,  // SDO (rx) canopen message.
                         ENCODER_CONFIG_MESSAGE_LENGTH);

  // Mailbox where load pin will reply to configure message.
  DcanSetReceiveMailbox(BRIDLE_JUNC_BUS,
                        kBridleJuncMailboxPinRecvd,
                        kDcanIdStandard,
                        DCAN_STANDARD_MASK,
                        PIN_NODE_ID + 1,  // How the DLCHMCAN addresses replies.
                        PIN_CONFIG_MESSAGE_LENGTH);

  // Mailbox where encoder will reply to configure message.
  DcanSetReceiveMailbox(BRIDLE_JUNC_BUS,
                        kBridleJuncMailboxEncoderRecvd,
                        kDcanIdStandard,
                        DCAN_STANDARD_MASK,
                        ENCODER_NODE_ID + 0x580,  // SDO (tx) canopen message.
                        ENCODER_CONFIG_MESSAGE_LENGTH);

  // Mailbox for NMT broadcast messages (ID 0).
  DcanSetTransmitMailbox(BRIDLE_JUNC_BUS,
                         kBridleJuncMailboxNmt,
                         kDcanIdStandard,
                         DCAN_STANDARD_MASK,
                         0,  // NMT node ID.
                         NMT_BROADCAST_LENGTH);

  // Inbox for load pin cyclic transmission.
  DcanSetReceiveMailbox(BRIDLE_JUNC_BUS,
                        kBridleJuncMailboxPinData,
                        kDcanIdStandard,
                        DCAN_STANDARD_MASK,
                        0xFF,  // DLCHMCAN preset address for data messages.
                        PIN_DATA_MESSAGE_LENGTH);

  // Inbox for encoder cyclic transmission.
  DcanSetReceiveMailbox(BRIDLE_JUNC_BUS,
                        kBridleJuncMailboxEncoderData,
                        kDcanIdStandard,
                        DCAN_STANDARD_MASK,
                        ENCODER_NODE_ID + 0x180,  // PDO1 (tx) canopen message.
                        ENCODER_DATA_MESSAGE_LENGTH);
}

// Send config to bridle-tether junction load pin.
static bool HandlePinInit(LoadpinState next) {
  uint32_t now = Clock32GetCycles();
  static uint32_t last_sent = 0;  // Track when config message was sent.
  uint8_t response_msg[PIN_CONFIG_MESSAGE_LENGTH];  // Confirmation msg inbox.
  BridleJuncData *mon = LoadcellOutputGetBridleJuncData();
  if (g_bridle_junc.loadpin_first_entry) {
    SendPinConfig();  // Send "STRMON" data-streaming-on message to load pin.
    last_sent = now;
    return false;
  } else if (DcanGetMailbox(BRIDLE_JUNC_BUS, kBridleJuncMailboxPinRecvd,
                            PIN_CONFIG_MESSAGE_LENGTH, response_msg,
                            NULL)) {  // Pin verified it received message.
    SignalWarning(kBridleJuncWarningLoadPinReadTimeout, false, &mon->flags);
    g_bridle_junc.loadpin_read_timeout = now + READ_WARNING_TIMEOUT;
    g_bridle_junc.loadpin_state = next;  // Advance load pin CAN state.
    return true;
  } else if (CLOCK32_GT(now - last_sent, CONFIG_RESEND_TIMEOUT)) {
    // Resend message periodically if we did not get verification.
    SignalWarning(kBridleJuncWarningLoadPinReadTimeout, true, &mon->flags);
    SendPinConfig();
    last_sent = now;
    return true;
  }
  return false;
}

// Initial configuring of bridle-tether junction roll-measuring encoder.
static bool HandleEncoderInit(EncoderState next) {
  uint32_t now = Clock32GetCycles();
  static uint32_t last_sent = 0;
  uint8_t response_msg[ENCODER_CONFIG_MESSAGE_LENGTH];
  BridleJuncData *mon = LoadcellOutputGetBridleJuncData();
  if (g_bridle_junc.encoder_first_entry) {
    SendEncoderConfig();  // Sets encoder cyclic data transmission rate.
    last_sent = now;
    return false;
  } else if (DcanGetMailbox(BRIDLE_JUNC_BUS, kBridleJuncMailboxEncoderRecvd,
                            ENCODER_CONFIG_MESSAGE_LENGTH, response_msg,
                            NULL)) {  // Encoder verified it received config.
    SignalWarning(kBridleJuncWarningEncoderReadTimeout, false, &mon->flags);
    g_bridle_junc.encoder_read_timeout = now + READ_WARNING_TIMEOUT;
    g_bridle_junc.encoder_state = next;  // Advance encoder CAN state machine.
    return true;
  } else if (CLOCK32_GT(now - last_sent, CONFIG_RESEND_TIMEOUT)) {
    // Resend config message periodically if we did not get verification.
    SignalWarning(kBridleJuncWarningEncoderReadTimeout, true, &mon->flags);
    SendEncoderConfig();
    last_sent = now;
    return true;
  }
  return false;
}

// Send NMT broadcast message to start cyclic transmissions.
static bool HandleNmtStart(EncoderState next) {
  // Start encoder data streaming.
  uint8_t nmt_start_encoder[NMT_BROADCAST_LENGTH] = {0x01, 0x0F};
  DcanTransmit(BRIDLE_JUNC_BUS, kBridleJuncMailboxNmt,
               NMT_BROADCAST_LENGTH, nmt_start_encoder);
  // TODO: Get some sort of verification before advancing state.
  g_bridle_junc.encoder_state = next;
  return true;
}

// Get tether tension data from bridle-tether junction load pin.
static bool HandlePinData(LoadpinState next) {
  uint32_t now = Clock32GetCycles();
  uint8_t raw_load_data[PIN_DATA_MESSAGE_LENGTH];
  BridleJuncData *mon = LoadcellOutputGetBridleJuncData();
  if (DcanGetMailbox(BRIDLE_JUNC_BUS, kBridleJuncMailboxPinData,
                     PIN_DATA_MESSAGE_LENGTH, raw_load_data, NULL)) {
    // Successful data read -- reset timeout and clear warning.
    g_bridle_junc.loadpin_read_timeout = now + READ_WARNING_TIMEOUT;
    SignalWarning(kBridleJuncWarningLoadPinReadTimeout, false, &mon->flags);
    // Convert data to engineering units and store in output message struct.
    float load_measurement = LBF_TO_NEWTONS * BytesToFloatBe(raw_load_data);
    assert(mon != NULL);
    mon->junc_load = load_measurement
        - kLoadcellCalibParams->bridle_calib.loadcell_zero;
  }
  if (CLOCK32_GT(now, g_bridle_junc.loadpin_read_timeout)) {
    // Read timeout -- signal warning for webmonitors.
    // TODO: send to error state for resetting CAN bus.
    SignalWarning(kBridleJuncWarningLoadPinReadTimeout, true, &mon->flags);
  }
  g_bridle_junc.loadpin_state = next;
  return true;
}

// Get roll angle data from bridle-tether junction encoder.
static bool HandleEncoderData(EncoderState next) {
  uint32_t now = Clock32GetCycles();
  uint8_t raw_angle_data[ENCODER_DATA_MESSAGE_LENGTH];
  BridleJuncData *mon = LoadcellOutputGetBridleJuncData();
  if (DcanGetMailbox(BRIDLE_JUNC_BUS, kBridleJuncMailboxEncoderData,
                     ENCODER_DATA_MESSAGE_LENGTH, raw_angle_data, NULL)) {
    // Successful data read -- reset timeout and clear warning.
    g_bridle_junc.encoder_read_timeout = now + READ_WARNING_TIMEOUT;
    SignalWarning(kBridleJuncWarningEncoderReadTimeout, false, &mon->flags);
    // Convert data to engineering units and store in output message struct.
    uint16_t encoder_data = (raw_angle_data[1] << 8) | (raw_angle_data[0]);
    float angle_rads = TetherRollRadians(encoder_data);  // Offset & units.
    assert(mon != NULL);
    mon->junc_angle = angle_rads;  // Already zeroed.
  }
  if (CLOCK32_GT(now, g_bridle_junc.encoder_read_timeout)) {
    // Read timeout -- signal warning for webmonitors.
    // TODO: send to error state for resetting CAN bus.
    SignalWarning(kBridleJuncWarningEncoderReadTimeout, true, &mon->flags);
  }
  g_bridle_junc.encoder_state = next;
  return true;
}

// Cycle of state handling functions for communication with load pin.
static bool LoadpinCanPoll(void) {
  bool next_sensor;  // Advance to next sensor in CAN polling cycle.
  LoadpinState current = g_bridle_junc.loadpin_state;
  switch (g_bridle_junc.loadpin_state) {
    case kLoadpinStateInit:
      next_sensor = HandlePinInit(kLoadpinStateData);
      break;
    case kLoadpinStateData:
      next_sensor = HandlePinData(kLoadpinStateData);
      break;
    case kLoadpinStateError:  // Pass through.
    default:
      g_bridle_junc.loadpin_state = kLoadpinStateInit;
      next_sensor = true;
      break;
  }
  g_bridle_junc.loadpin_first_entry = (current != g_bridle_junc.loadpin_state);
  return next_sensor;
}

// Cycle of state handling functions for communication with encoder.
static bool EncoderCanPoll(void) {
  bool next_sensor;  // Advance to next sensor in CAN polling cycle.
  EncoderState current = g_bridle_junc.encoder_state;
  switch (g_bridle_junc.encoder_state) {
    case kEncoderStateInit:
      next_sensor = HandleEncoderInit(kEncoderStateNmtStart);
      break;
    case kEncoderStateNmtStart:
      next_sensor = HandleNmtStart(kEncoderStateData);
      break;
    case kEncoderStateData:
      next_sensor = HandleEncoderData(kEncoderStateData);
      break;
    case kEncoderStateError:  // Pass through.
    default:
      g_bridle_junc.encoder_state = kEncoderStateInit;
      next_sensor = true;
      break;
  }
  g_bridle_junc.encoder_first_entry = (current != g_bridle_junc.encoder_state);
  return next_sensor;
}

// Initialize bridle pin CAN monitor state variables.
void BridleJuncCanInit(void) {
  StartCarrierCanBus();
  g_bridle_junc.loadpin_state = kLoadpinStateInit;
  g_bridle_junc.encoder_state = kEncoderStateInit;
  g_bridle_junc.loadpin_first_entry = true;
  g_bridle_junc.encoder_first_entry = true;
}

// Advance CAN comms state machines for encoder and load pin.
void BridleJuncCanPoll(void) {
  static bool (*const poll[])(void) = {
    LoadpinCanPoll,
    EncoderCanPoll,
  };
  static uint32_t index = 0U;

  index %= ARRAYSIZE(poll);
  if (poll[index]()) {
    ++index;
  }
}
