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

#include "avionics/cs/firmware/tether_comms.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/pack_tether_message.h"
#include "avionics/common/tether_convert.h"
#include "avionics/common/tether_cvt.h"
#include "avionics/common/tether_cvt_op.h"
#include "avionics/common/tether_cvt_rtcm.h"
#include "avionics/common/tether_message.h"
#include "avionics/common/tether_message_types.h"
#include "avionics/common/tether_op.h"
#include "avionics/cs/firmware/tether_rtcm.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/drivers/xlr.h"
#include "avionics/firmware/drivers/xlr_config.h"
#include "avionics/firmware/monitors/cs_types.h"
#include "avionics/firmware/network/net_mon_types.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_stats.h"
#include "common/c_math/util.h"
#include "common/macros.h"

// TETHER_UP_PERIOD_US and TETHER_DOWN_PERIOD_US represent the Ethernet
// transmit period as defined by network.yaml.
COMPILE_ASSERT(TETHER_UP_PERIOD_US == TETHER_DOWN_PERIOD_US,
               TetherUp_and_TetherDown_must_operate_at_the_same_frequency);

#define TETHER_UP_PERIOD_CYCLES                         \
  CLOCK32_MSEC_TO_CYCLES(TETHER_UP_PERIOD_US / 1000)
#define TETHER_DOWN_PERIOD_CYCLES                       \
  CLOCK32_MSEC_TO_CYCLES(TETHER_DOWN_PERIOD_US / 1000)

// RADIO_PERIOD_CYCLES represents the RF transmit period. We transmit the
// TetherUpPacked and TetherDownPacked messages at a reduced rate according
// to the TETHER_RADIO_DECIMATION macro.
#define RADIO_PERIOD_CYCLES                             \
  (TETHER_DOWN_PERIOD_CYCLES * TETHER_RADIO_DECIMATION)
#define MAX_ERROR_CYCLES CLOCK32_MSEC_TO_CYCLES(1)

// RADIO_GUARD_CYCLES represents the guard (silent) time between the reception
// of TetherUp and the transmission of TetherDown over the radio link. The
// guard time accounts for variability in timing.
#define RADIO_GUARD_CYCLES CLOCK32_MSEC_TO_CYCLES(2)

// RADIO_COMMS_LINK_TIMEOUT_CYCLES defines the time before marking the comms
// link status bits to false for the long range radio.
#define RADIO_COMMS_LINK_TIMEOUT_CYCLES CLOCK32_MSEC_TO_CYCLES(500)

COMPILE_ASSERT(
    TETHER_FRAME_INDEX_ROLLOVER % TETHER_RADIO_DECIMATION == 0,
    TETHER_FRAME_INDEX_ROLLOVER_must_be_a_multiple_of_radio_decimation);

#define XLR_CONFIG (&kXlrConfigPort1)
#define PACK_FUNC(func) ((PackAioMessageFunction)func)

static const PackAioMessageFunction
kTetherOpPackFunctions[kNumTetherOpTypes] = {
  [kTetherOpTypeBattCommand] = PACK_FUNC(PackBattCommandMessage),
  [kTetherOpTypeFlightCommand] = PACK_FUNC(PackFlightCommandMessage),
  [kTetherOpTypeFpvSetState] = PACK_FUNC(PackFpvSetStateMessage),
  [kTetherOpTypeMotorAckParam] = PACK_FUNC(PackMotorAckParamMessage),
  [kTetherOpTypeMotorGetParam] = PACK_FUNC(PackMotorGetParamMessage),
  [kTetherOpTypeMotorSetParam] = PACK_FUNC(PackMotorSetParamMessage),
  [kTetherOpTypeMotorSetState] = PACK_FUNC(PackMotorSetStateMessage),
  [kTetherOpTypeMvlvCommand] = PACK_FUNC(PackMvlvCommandMessage),
  [kTetherOpTypePitotSetState] = PACK_FUNC(PackPitotSetStateMessage),
  [kTetherOpTypeServoAckParam] = PACK_FUNC(PackServoAckParamMessage),
  [kTetherOpTypeServoGetParam] = PACK_FUNC(PackServoGetParamMessage),
  [kTetherOpTypeServoSetParam] = PACK_FUNC(PackServoSetParamMessage),
  [kTetherOpTypeServoSetState] = PACK_FUNC(PackServoSetStateMessage),
  [kTetherOpTypeTetherReleaseSetState] = PACK_FUNC(
      PackTetherReleaseSetStateMessage),
};

// TetherUp. Sent from ground to wing.
static uint32_t g_up_timeout_cycles = 0U;  // Ethernet transmit timer.
static uint32_t g_up_radio_received_cycles = 0U;  // Radio receive timestamp.
static bool g_up_radio_received = false;
static TetherUpCvtState g_up_cvt;
static TetherUpMessage g_up;
static TetherUpPackedMessage g_up_packed;
static uint16_t g_up_frame_index_z1;

// TetherDown. Send from wing to ground.
static uint32_t g_down_timeout_cycles = 0U;  // Ethernet transmit timer.
static TetherDownCvtState g_down_cvt;
static TetherDownMessage g_down;
static TetherDownPackedMessage g_down_packed;
static uint16_t g_down_frame_index_z1;
static int32_t g_node_status_no_update_counts[kNumAioNodes];

// Modem driver.
static XlrDevice g_xlr;

static int32_t GetFrameIndexIncrement(uint16_t frame_index,
                                      uint16_t frame_index_z1) {
  int32_t inc = TetherCompareFrameIndex(frame_index, frame_index_z1);
  if (inc <= 0) {
    inc = INT32_MAX;
  }
  return inc;
}

static void TransmitOpQueue(TetherOpQueue *q) {
  AioNode source;
  TetherOpType op_type;
  TetherOpData *data;
  uint16_t sequence;

  while ((data = TetherOpGetTail(q, &source, &op_type, &sequence)) != NULL) {
    PackAioMessageFunction pack_func = kTetherOpPackFunctions[op_type];
    MessageType message_type = TetherOpTypeToMessageType(op_type);
    if (pack_func != NULL) {
      NetSendSpoofedAio(source, message_type, sequence, pack_func, data);
    }
    TetherOpPopTail(q);
  }
}

static void HandleGroundSideTransmit(AioNode me) {
  int64_t now_usec = ClockGetUs();

  g_up.received_signal_strength = g_down.received_signal_strength;
  g_up.frame_index = (g_up.frame_index + 1U) % TETHER_FRAME_INDEX_ROLLOVER;
  TetherUpCvtQuery(now_usec, g_up.frame_index, &g_up_cvt, &g_up);

  if (g_up.frame_index % TETHER_RADIO_DECIMATION == 0) {
    TetherUpCvtRtcmQuery(now_usec, &g_up_cvt.rtcm, &g_up);
    TetherUpCvtOpQueueQuery(&g_up_cvt.operator_command);

    // Transmit TetherUpPacked from CsGsA to CsA via modem.
    if (me == kAioNodeCsGsA) {
      PackTetherUp(&g_up_cvt.operator_command, &g_up, &g_up_packed);
      XlrSendBroadcast(XLR_CONFIG, 1, 0, sizeof(g_up_packed), &g_up_packed,
                       NULL, &g_xlr);
    }
  }

  // Transmit TetherUp from ground to wing via Ethernet. We can use the time
  // difference of arrival to measure the latency of the modem.
  NetSendAioTetherUpMessage(&g_up);
}

static void FixTetherDownNodeStatusNoUpdateCounts(int32_t inc) {
  // Increment no_update_counts.
  for (int32_t i = 0; i < ARRAYSIZE(g_node_status_no_update_counts); ++i) {
    TetherIncrementNoUpdateCount(inc, &g_node_status_no_update_counts[i]);
  }

  // This function expects g_down.node_status.no_update_count to be set to
  // INT32_MAX before calling UnpackTetherDown. UnpackTetherDown should then
  // clear g_down.node_status.no_update_count for each update.
  int32_t node = g_down.node_status.node;
  if (0 <= node && node < ARRAYSIZE(g_node_status_no_update_counts)) {
    // If updated, store new count, otherwise send old no_update_count.
    if (g_down.node_status.no_update_count != INT32_MAX) {
      g_node_status_no_update_counts[node] = g_down.node_status.no_update_count;
    } else {
      g_down.node_status.no_update_count = g_node_status_no_update_counts[node];
    }
  } else {
    // Invalid message.
    memset(&g_down.node_status, 0, sizeof(g_down.node_status));
    g_down.node_status.no_update_count = INT32_MAX;
  }
}

static void HandleGroundSideReceive(const XlrApiRxIndicator *rx) {
  // Receive TetherDownPacked from CsA, then unpack and forward to ground.
  // TODO: Implement error reporting.
  if (rx->length == sizeof(g_down_packed)) {
    memcpy(&g_down_packed, rx->data, sizeof(g_down_packed));

    uint16_t frame_index;
    if (UnpackTetherDownFrameIndex(&g_down_packed, &frame_index)) {
      int32_t inc = GetFrameIndexIncrement(frame_index, g_down_frame_index_z1);
      TetherDownIncrementNoUpdateCounts(inc, &g_down);

      // FixTetherDownNoUpdateCounts expects g_down.node_status.no_update_count
      // to be set to INT32_MAX, then optionally cleared by UnpackTetherDown.
      g_down.node_status.no_update_count = INT32_MAX;
      UnpackTetherDown(&g_down_cvt.command_reply, &g_down_packed, &g_down);
      FixTetherDownNodeStatusNoUpdateCounts(inc);

      TransmitOpQueue(&g_down_cvt.command_reply);
      NetSendAioTetherDownMessage(&g_down);
      XlrSendGetReceivedSignalStrength(XLR_CONFIG, &g_xlr);

      // Needed for computation of frame_index increment between updates.
      g_down_frame_index_z1 = g_down.frame_index;
    }
  }
}

static void HandleGroundSideReceivedSignalStrength(
    const XlrApiCommandResponse *response) {
  g_down.received_signal_strength = -response->data[0];  // [dB]
}

static void HandleWingSideTransmit(AioNode me, const CsMonitorData *cs_mon,
                                   const CoreSwitchStats *switch_stats) {
  // Transmit TetherUp reception statistics in TetherDown message.
  CoreSwitchStatsToTetherCommsStatus(
      me, switch_stats, g_up.received_signal_strength, &g_down.comms_status);

  // Transmit core switch node status in TetherDown message.
  CsMonitorDataToTetherNodeStatus(cs_mon, &g_down_cvt.node_status[me]);

  g_down.received_signal_strength = g_up.received_signal_strength;
  g_down.received_frame_index = g_up.frame_index;
  g_down.frame_index = (g_down.frame_index + 1U) % TETHER_FRAME_INDEX_ROLLOVER;
  TetherDownCvtQuery(ClockGetUs(), g_down.frame_index, &g_down_cvt, &g_down);

  // Transmit TetherDownPacked from CsA to CsGsA via modem.
  if (me == kAioNodeCsA
      && g_down.frame_index % TETHER_RADIO_DECIMATION == 0) {
    // Synchronize TetherDownPacked message with the reception of
    // TetherUpPacked to improve half-duplex performance.
    if (g_up_radio_received) {
      g_up_radio_received = false;

      // Compute start time of current Ethernet transmit period.
      uint32_t down_ethernet_cycles =
          g_down_timeout_cycles - TETHER_DOWN_PERIOD_CYCLES;

      // Compute start time of next radio transmit period. We want to
      // synchronize this time with the receive time of the TetherUpPacked
      // message.
      uint32_t down_radio_cycles = (down_ethernet_cycles + RADIO_PERIOD_CYCLES
                                    + RADIO_GUARD_CYCLES);

      // Compute error within one half period.
      int32_t error_cycles = WrapInt32(
          CLOCK32_SUBTRACT(g_up_radio_received_cycles, down_radio_cycles),
          -RADIO_PERIOD_CYCLES / 2, RADIO_PERIOD_CYCLES / 2);

      // Saturate error to prevent large jumps in output period.
      if (error_cycles > MAX_ERROR_CYCLES) {
        error_cycles = MAX_ERROR_CYCLES;
      } else if (error_cycles < -MAX_ERROR_CYCLES) {
        error_cycles = -MAX_ERROR_CYCLES;
      }

      // Update next iteration time (synchronize gradually).
      g_down_timeout_cycles += error_cycles / 10;  // Gain kp = 0.1.
    }

    // Transmit.
    TetherDownCvtOpQueueQuery(&g_down_cvt.command_reply);
    PackTetherDown(&g_down_cvt.command_reply, &g_down, &g_down_packed);
    XlrSendBroadcast(XLR_CONFIG, 1, 0, sizeof(g_down_packed), &g_down_packed,
                     NULL, &g_xlr);
  }

  // Transmit TetherDown from wing to ground via Ethernet.
  NetSendAioTetherDownMessage(&g_down);
}

static void HandleWingSideReceive(const XlrApiRxIndicator *rx) {
  // Receive TetherUpPacked from CsGsA, then unpack and forward to wing.
  // TODO: Implement error reporting.
  if (rx->length == sizeof(g_up_packed)) {
    memcpy(&g_up_packed, rx->data, sizeof(g_up_packed));

    int64_t now_usec = ClockGetUs();
    uint16_t frame_index;
    if (UnpackTetherUpFrameIndex(&g_up_packed, &frame_index)) {
      TetherUpIncrementNoUpdateCounts(
          GetFrameIndexIncrement(frame_index, g_up_frame_index_z1), &g_up);
      UnpackTetherUp(&g_up_cvt.operator_command, &g_up_packed, &g_up);
      TransmitOpQueue(&g_up_cvt.operator_command);
      NetSendAioTetherUpMessage(&g_up);
      TetherUpRtcmToNetwork(now_usec, &g_up);
      XlrSendGetReceivedSignalStrength(XLR_CONFIG, &g_xlr);

      // Store time to synchronize TetherDownPacked radio transmission.
      g_up_radio_received_cycles = Clock32GetCycles();
      g_up_radio_received = true;

      // Needed for computation of frame_index increment between updates.
      g_up_frame_index_z1 = g_up.frame_index;
    }
  }
}

static void HandleWingSideReceiveTetherUp(AioNode source) {
  TetherUpMessage in;
  int64_t timestamp_usec;
  if (CvtGetTetherUpMessage(source, &in, NULL, &timestamp_usec)) {
    TetherUpRtcmToNetwork(timestamp_usec, &in);
  }
}

static void HandleWingSideReceivedSignalStrength(
    const XlrApiCommandResponse *response) {
  g_up.received_signal_strength = -response->data[0];  // [dB]
}

static void PollModem(
    void (* const handle_receive)(const XlrApiRxIndicator *rx),
    void (* const handle_rssi)(const XlrApiCommandResponse *rx)) {
  XlrApiData data;
  if (XlrPoll(XLR_CONFIG, &g_xlr, &data)) {
    if (data.frame_type == kXlrApiFrameTypeRxIndicator) {
      handle_receive(&data.u.rx_indicator);
    } else if (data.frame_type == kXlrApiFrameTypeCommandResponse
               && data.u.command_response.command
               == kXlrAtCommandReceivedSignalStrength) {
      handle_rssi(&data.u.command_response);
    }
  }
}

void TetherCommsInit(uint16_t network_id) {
  uint32_t now_cycles = Clock32GetCycles();

  // Initialize TetherUp message state.
  g_up_timeout_cycles = now_cycles;
  g_up_radio_received_cycles = 0U;
  g_up_radio_received = false;
  TetherUpCvtInit(&g_up_cvt);
  TetherUpInit(&g_up);
  g_up_frame_index_z1 = g_up.frame_index;

  // Initialize TetherDown message state.
  g_down_timeout_cycles = now_cycles;
  TetherDownCvtInit(ClockGetUs(), &g_down_cvt);
  TetherDownInit(&g_down);
  g_down_frame_index_z1 = g_down.frame_index;

  // TetherDown cycles node_status for each wing node. We cache the state
  // to handle the no_update_counts.
  for (int32_t i = 0; i < ARRAYSIZE(g_node_status_no_update_counts); ++i) {
    g_node_status_no_update_counts[i] = INT32_MAX;
  }

  // Initialize GPS RTCM processing.
  TetherRtcmInit();

  // Initialize modem driver.
  XlrInit(&g_xlr);

  // Set network id to prevent decoding messages from different systems.
  XlrSetNetworkId(network_id, &g_xlr);
}

void TetherCommsPoll(AioNode me, const CsMonitorData *cs_mon,
                     const CoreSwitchStats *switch_stats,
                     int16_t microhard_rssi) {
  // Receive.
  if (me == kAioNodeCsGsA) {
    PollModem(HandleGroundSideReceive, HandleGroundSideReceivedSignalStrength);
  } else if (me == kAioNodeCsA) {
    PollModem(HandleWingSideReceive, HandleWingSideReceivedSignalStrength);
    HandleWingSideReceiveTetherUp(kAioNodeCsGsA);
  } else if (me == kAioNodeCsB) {
    HandleWingSideReceiveTetherUp(kAioNodeCsGsB);
  }

  // Mark received_signal_strength as invalid after timeout.
  uint32_t timeout_cycles =
      g_up_radio_received_cycles + RADIO_COMMS_LINK_TIMEOUT_CYCLES;
  if (g_up.received_signal_strength < 0
      && CLOCK32_GE(Clock32GetCycles(), timeout_cycles)) {
    g_up.received_signal_strength = 0;
  }

  if (me == kAioNodeCsB) {
    g_up.received_signal_strength = microhard_rssi;
  }

  // Transmit.
  if (me == kAioNodeCsGsA || me == kAioNodeCsGsB) {
    if (PollPeriodicCycles(TETHER_UP_PERIOD_CYCLES, &g_up_timeout_cycles)) {
      HandleGroundSideTransmit(me);
    }
  } else if (me == kAioNodeCsA || me == kAioNodeCsB) {
    if (PollPeriodicCycles(TETHER_DOWN_PERIOD_CYCLES, &g_down_timeout_cycles)) {
      HandleWingSideTransmit(me, cs_mon, switch_stats);
    }
  }
}
