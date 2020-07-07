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

#include "avionics/batt/firmware/state.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/batt/firmware/config_params.h"
#include "avionics/batt/firmware/output.h"
#include "avionics/common/avionics_messages.h"
#include "avionics/common/batt_types.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/faults.h"
#include "avionics/common/safety_codes.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/io.h"
#include "avionics/firmware/drivers/log.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/monitors/batt_mcp342x_types.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/firmware/serial/carrier_serial.h"

#define BATT_SWITCH_EN0 kIoGioPinA6
#define BATT_SWITCH_EN1 kIoGioPinA7
#define TRICKLE_CHG_EN  kIoGioPinA2
#define FAST_CHG_EN0    kIoN2het1Pin14
#define FAST_CHG_EN1    kIoN2het1Pin16
#define HEATER_EN       kIoN2het1Pin4

#define DISCONNECT_CYCLES CLOCK32_MSEC_TO_CYCLES(10000)
#define LTC6804_ADC_CYCLES CLOCK32_MSEC_TO_CYCLES(7)

typedef enum {
  kBattStateIdle,
  kBattStateCommsPaired,   // Receive & send pair status w/the other batt box.
  kBattStateRecvState,    // Receive connect/disconnect command from operator.
  kBattStateBattSwitch,   // Connect/disconnect batteries from LV bus.
  kBattStateCharger,      // Control charger based on min & max cell voltage.
  kBattStateClearErrors,
  kBattStateControlHeaters,
  kBattStateInit
} BattState;

static struct {
  BattState state;
  uint32_t ltc6804_adc_wait;
  bool first_entry;
  bool disconnect_initiated;
  uint32_t disconnect_time;
  bool charge;
  float paired_stack_voltage;
  bool other_direct_charge_detected;
} g_batt;

static struct {
  bool clear_errors;
  bool connected;
} g_operator_command;

static const float *GetBattTemps(const BattMonitorData *mon) {
  return mon->mcp342x_data;
}

static float GetBusAVoltage(const BattMonitorData *mon) {
  return mon->analog_data[kBattAnalogVoltageLvA];
}

static float GetMinCellVoltage(const BattMonitorData *mon) {
  return mon->ltc6804_data.min_cell_v;
}

static float GetMaxCellVoltage(const BattMonitorData *mon) {
  return mon->ltc6804_data.max_cell_v;
}

static float GetStackVoltage(const BattMonitorData *mon) {
  return mon->ltc6804_data.stack_voltage;
}

// Close FET switches such that batteries connect to, and power, their LV bus.
static void ConnectBattsToBus(BattMonitorData *mon) {
  IoSetValue(BATT_SWITCH_EN0, 1);
  IoSetValue(BATT_SWITCH_EN1, 1);
  SetStatus(kBattMonitorStatusConnected, 1, &mon->flags);
}

// Open FET switches such that battery box ceases to power its respective bus.
static void DisconnectBattsFromBus(BattMonitorData *mon) {
  IoSetValue(BATT_SWITCH_EN0, 0);
  IoSetValue(BATT_SWITCH_EN1, 0);
  SetStatus(kBattMonitorStatusConnected, 0, &mon->flags);
}


// Return whether the battery box is a "big box" hardware revision.
static bool IsBigBox(void) {
  int32_t rev = GetCarrierHardwareRevision();
  return (rev == kBattHardwareBigCell18V1 ||
          rev == kBattHardwareBigCell18Aa ||
          rev == kBattHardwareBigCell18Ab ||
          rev == kBattHardwareBigCell18Ac);
}

// Return whether this board's rev uses the SEPIC trickle charger.
// Revs that don't use the trickle charger use the direct charge switch.
static bool UsesTrickleCharger(void) {
  int32_t rev = GetCarrierHardwareRevision();
  return (rev == kBattHardwareSmallCell15V1 ||
          rev == kBattHardwareBigCell18V1   ||
          rev == kBattHardwareSmallCell15Aa ||
          rev == kBattHardwareSmallCell15Ab ||
          rev == kBattHardwareSmallCell17Ab ||
          rev == kBattHardwareSmallCell15Ac ||
          rev == kBattHardwareSmallCell17Ac ||
          rev == kBattHardwareSmallCell17Ad);
}

// Return true if this board's hardware revision is rev_v1 or rev_aa,
// after which we changed the charge current sensor and added heating pads.
static bool RevBeforeAb(void) {
  int32_t rev = GetCarrierHardwareRevision();
  return (rev == kBattHardwareSmallCell15V1 ||
          rev == kBattHardwareBigCell18V1   ||
          rev == kBattHardwareSmallCell15Aa ||
          rev == kBattHardwareBigCell18Aa);
}

// Return whether conditions allow charging.
static bool OkToChargeBatteries(BattMonitorData *mon) {
  // Cells must be above 2.5V to avoid charging an overdischarged pack.
  // Cells must be below 4.1V to avoid overcharging and protect cell longevity.
  bool cell_voltages_ok = (GetMaxCellVoltage(mon) < 4.1f &&
                           GetMinCellVoltage(mon) > 2.5f);

  // For revisions using the DCDC-converter trickle charger rather than the
  // direct connection switch, bus A (the charging bus) must be above 65V.
  // For the small box, this ensures you're not charging from a big box that
  // is nearing the end of its available energy (which drops rapidly below 65V).
  // For the big box, this ensures that you only charge from a connected
  // umbilical, not from the small battery box, which sits <= 63V.
  bool bus_a_sufficient = (!UsesTrickleCharger() ||
                           GetBusAVoltage(mon) > 65.0f);

  // Charging should only occur when the batteries are between 10.0 and 37.7
  // degrees Celsius (per Thunder Power LiPo specifications).
  const float *temps = GetBattTemps(mon);
  bool ok_temp = (temps[kBattMcp342xMonitorBatteries1] > 10.0f &&
                  temps[kBattMcp342xMonitorBatteries2] > 10.0f &&
                  temps[kBattMcp342xMonitorBatteries1] < 37.7f &&
                  temps[kBattMcp342xMonitorBatteries2] < 37.7f);

  return (cell_voltages_ok &&
          bus_a_sufficient &&
          ok_temp);
}


// Get GPIO value to allow battery charger circuit to turn on.
static bool TrickleChargerOnValue(void) {
  if (GetCarrierHardwareRevision() == kBattHardwareSmallCell15V1 ||
      GetCarrierHardwareRevision() == kBattHardwareBigCell18V1) {
    return 1;
  } else {
    return 0;
  }
}

// Get GPIO value to prevent battery charger circuit from turning on.
static bool ChargerOffValue(void) {
  return !TrickleChargerOnValue();
}

// Functions for receiving and verifying state commands from OPERATOR.
static bool StateSignalValid(const BattCommandMessage *msg) {
  if (msg->state_command == kBattStateCommandClearErrors) {
    return msg->batt_signal == BATT_CLEAR_ERRORS_SIGNAL;
  } else if (msg->state_command == kBattStateCommandDisconnectA) {
    return msg->batt_signal == BATT_DISCONNECT_A_SIGNAL;
  } else if (msg->state_command == kBattStateCommandDisconnectB) {
    return msg->batt_signal == BATT_DISCONNECT_B_SIGNAL;
  } else if (msg->state_command == kBattStateCommandConnect) {
    return msg->batt_signal == BATT_CONNECT_SIGNAL;
  } else {
    return true;
  }
}

static BattStateCommand QuerySetStateCommand(AioNode op) {
  BattCommandMessage msg;
  if (CvtGetBattCommandMessage(op, &msg, NULL, NULL)
      && StateSignalValid(&msg)) {
    return msg.state_command;
  }
  return kBattStateCommandNone;
}

static void BattStateQuerySetStateCommand(void) {
  BattStateCommand state_cmd = QuerySetStateCommand(kAioNodeOperator);
  switch (state_cmd) {
    case kBattStateCommandClearErrors:
      LOG_PRINTF("Received LV battery clear error signal.\n");
      g_operator_command.clear_errors = true;
      break;
    case kBattStateCommandDisconnectA:
      if (AppConfigGetAioNode() == kAioNodeBattA) {
        if (g_operator_command.connected) {
          LOG_PRINTF("Received battery A disconnect signal.\n");
        }
        g_operator_command.connected = false;
      }
      break;
    case kBattStateCommandDisconnectB:
      if (AppConfigGetAioNode() == kAioNodeBattB) {
        if (g_operator_command.connected) {
          LOG_PRINTF("Received battery B disconnect signal.\n");
        }
        g_operator_command.connected = false;
      }
      break;
    case kBattStateCommandConnect:
      if (!g_operator_command.connected) {
        LOG_PRINTF("Received battery connect signal.\n");
      }
      g_operator_command.connected = true;
      break;
    default:
      break;
  }
}

// State Handling Functions.

static void BattHandleInit(BattState next, BattMonitorData *mon) {
  IoInit();

  // Control pins for battery charger and connection switch.
  IoConfigureAsOutputOpenDrain(TRICKLE_CHG_EN, ChargerOffValue());
  IoConfigureAsOutputPushPull(FAST_CHG_EN0, 0);
  IoConfigureAsOutputPushPull(FAST_CHG_EN1, 0);
  IoConfigureAsOutputPushPull(BATT_SWITCH_EN0, 0);
  IoConfigureAsOutputPushPull(BATT_SWITCH_EN1, 0);
  IoConfigureAsOutputPushPull(HEATER_EN, 0);

  // Batteries are disconnected from LV bus by default.
  g_operator_command.connected = false;
  g_operator_command.clear_errors = false;

  // Set status bit to indicate dual_big_box config.
  if (kBattConfigParams->dual_big_box == kBattConfigTypeDualBig) {
    SetStatus(kBattMonitorStatusDualBig, true, &mon->flags);
  } else {  // Not configured with two big battery boxes -- clear status bit.
    SetStatus(kBattMonitorStatusDualBig, false, &mon->flags);
  }

  g_batt.state = next;
}

static void BattHandleIdle(BattState next) {
  g_batt.state = next;
}

// Receive pair status message from other battery box.
static void BattHandleCommsPaired(BattState next, BattMonitorData *mon) {
  // Get identity of other battery box.
  AioNode paired_node = (AppConfigGetAioNode() == kAioNodeBattB)
      ? kAioNodeBattA : kAioNodeBattB;
  // Receive paired status message and store cell stack voltage value.
  BattPairedStatusMessage recv_msg;
  if (CvtGetBattPairedStatusMessage(paired_node, &recv_msg, NULL, NULL)) {
    g_batt.paired_stack_voltage = recv_msg.cell_stack_voltage;
    g_batt.other_direct_charge_detected = recv_msg.uses_direct_charge;
    mon->paired_stack_voltage = recv_msg.cell_stack_voltage;
  }
  // Set "Misconfigured" warning if box should be dual_big but isn't.
  if (kBattConfigParams->dual_big_box != kBattConfigTypeDualBig &&
      g_batt.other_direct_charge_detected == true &&
      IsBigBox()) {
    SignalWarning(kBattMonitorWarningMisconfigured, true, &mon->flags);
  }

  g_batt.state = next;
}

static void BattHandleRecvState(BattState next) {
  BattStateQuerySetStateCommand();
  g_batt.state = next;
}

static void BattHandleBattSwitch(BattState next, BattMonitorData *mon) {
  // Connect or disconnect based on g_operator_command and min cell voltage.
  // TODO: move voltage threshold from function to struct.
  if (g_operator_command.connected) {
    // Operator has commanded connect. Now check voltage threshold.
    if (GetMinCellVoltage(mon) < 3.0f) {
      uint32_t now = Clock32GetCycles();
      // Disconnect when any cell voltage < 3.0V to prevent cell damage.
      if (!g_batt.disconnect_initiated) {
        // Start timer in case undervoltage is due to temporary current surge.
        g_batt.disconnect_initiated = true;
        g_batt.disconnect_time = now + DISCONNECT_CYCLES;
      } else if (CLOCK32_GT(now, g_batt.disconnect_time)) {
        // Undervoltage timeout - disconnect cells.
        g_batt.disconnect_initiated = false;
        DisconnectBattsFromBus(mon);
      }
    } else {
      // Voltage sufficiently high; maintain connection to bus.
      g_batt.disconnect_initiated = false;
      // Make sure not dumping excessive current into other battery box.
      // If staying disconnected to prevent overcurrent, signal to webmonitor.
      if ((kBattConfigParams->dual_big_box == kBattConfigTypeDualBig ||
           g_batt.other_direct_charge_detected == true) &&
          GetStackVoltage(mon) - g_batt.paired_stack_voltage > 2.0) {
        SignalWarning(kBattMonitorWarningOCProtect, true, &mon->flags);
        // Not connecting. Remaining disconnected if previously disconnected.
      } else {
        // Good to connect or stay connected. Clear overcurrent-protect warning.
        SignalWarning(kBattMonitorWarningOCProtect, false, &mon->flags);
        ConnectBattsToBus(mon);
      }
    }
  } else {
    // Operator has commanded disconnect. Disconnect.
    DisconnectBattsFromBus(mon);
  }
  g_batt.state = next;
}

static void BattHandleCharger(BattState next, BattMonitorData *mon) {
  // Turn on or off charger circuit depending on conditions for charging.
  if (OkToChargeBatteries(mon)) {
    // Do charge, by trickle or direct switch depending on hardware revision.
    SetStatus(kBattMonitorStatusCharging, 1, &mon->flags);
    if (UsesTrickleCharger()) {  // Use trickle charger.
      IoSetValue(TRICKLE_CHG_EN, TrickleChargerOnValue());
    } else {  // Use direct fast-charging switch.
      IoSetValue(FAST_CHG_EN0, 1);
      IoSetValue(FAST_CHG_EN1, 1);
    }
  } else {
    // Do not charge.
    SetStatus(kBattMonitorStatusCharging, 0, &mon->flags);
    IoSetValue(TRICKLE_CHG_EN, ChargerOffValue());
    IoSetValue(FAST_CHG_EN0, 0);
    IoSetValue(FAST_CHG_EN1, 0);
  }
  // Update reported charger current to ltc4151 reading if rev < ab.
  // Otherwise, update to tms570 adc value (ltc4151 unpopulated for rev > aa).
  if (RevBeforeAb()) {
    mon->charger_current =
        mon->ltc4151_data[kBattLtc4151MonitorChargerOutput].current;
  } else {
    mon->charger_current = mon->analog_data[kBattAnalogVoltageIChg];
  }
  g_batt.state = next;
}

static void BattHandleClearErrors(BattState next, BattMonitorData *mon) {
  // If received clear-error message, clear all errors.
  if (g_operator_command.clear_errors) {
    assert(mon != NULL);
    ClearErrors(&mon->flags);
    g_operator_command.clear_errors = false;
  }
  g_batt.state = next;
}

// Control battery pack heating pads on revs >= ab.
static void BattHandleControlHeaters(BattState next, BattMonitorData *mon) {
  if (!RevBeforeAb()) {
    const float *temps = GetBattTemps(mon);
    // Keep batteries above 11 Celsius. Control includes hysteresis.
    if (((temps[kBattMcp342xMonitorBatteries1] < 11.0f &&
          temps[kBattMcp342xMonitorBatteries2] < 25.0f) ||
         (temps[kBattMcp342xMonitorBatteries2] < 11.0f &&
          temps[kBattMcp342xMonitorBatteries1] < 25.0f)) &&
        temps[kBattMcp342xMonitorBatteries1] > -50.0f &&  // Don't heat in
        temps[kBattMcp342xMonitorBatteries2] > -50.0f) {  // case of open.
      IoSetValue(HEATER_EN, 1);
    } else if ((temps[kBattMcp342xMonitorBatteries1] > 15.0f &&
                temps[kBattMcp342xMonitorBatteries2] > 15.0f) ||
               temps[kBattMcp342xMonitorBatteries1] > 25.0f ||
               temps[kBattMcp342xMonitorBatteries2] > 25.0f ||
               temps[kBattMcp342xMonitorBatteries1] <= -50.0f ||
               temps[kBattMcp342xMonitorBatteries2] <= -50.0f) {
      // 25 C heating pad max (also turn off if thermistor fails open).
      IoSetValue(HEATER_EN, 0);
    }
  }
  g_batt.state = next;
}

// Send own stack voltage value to paired battery node.
void BattSendPairedStatus(void) {
  BattMonitorData *mon = BattOutputGetBattMonitors();
  BattPairedStatusMessage send_msg;
  send_msg.cell_stack_voltage = GetStackVoltage(mon);
  send_msg.uses_direct_charge = !UsesTrickleCharger();
  NetSendAioBattPairedStatusMessage(&send_msg);
}

// Init.
void BattStateInit(void) {
  memset(&g_batt, 0, sizeof(g_batt));
  g_batt.state = kBattStateInit;
  g_batt.first_entry = true;
  g_batt.disconnect_initiated = false;
  g_batt.charge = false;
  g_batt.other_direct_charge_detected = false;
}

void BattStatePoll(void) {
  BattState current = g_batt.state;
  BattMonitorData *mon = BattOutputGetBattMonitors();
  switch (current) {
    case kBattStateInit:
      BattHandleInit(kBattStateIdle, mon);
      break;
    case kBattStateIdle:
      BattHandleIdle(kBattStateCommsPaired);
      break;
    case kBattStateCommsPaired:
      BattHandleCommsPaired(kBattStateRecvState, mon);
      break;
    case kBattStateRecvState:
      BattHandleRecvState(kBattStateBattSwitch);
      break;
    case kBattStateBattSwitch:
      BattHandleBattSwitch(kBattStateCharger, mon);
      break;
    case kBattStateCharger:
      BattHandleCharger(kBattStateClearErrors, mon);
      break;
    case kBattStateClearErrors:
      BattHandleClearErrors(kBattStateControlHeaters, mon);
      break;
    case kBattStateControlHeaters:
      BattHandleControlHeaters(kBattStateIdle, mon);
      break;
    default:
      g_batt.state = kBattStateIdle;
      break;
  }
  g_batt.first_entry = (current != g_batt.state);
}
