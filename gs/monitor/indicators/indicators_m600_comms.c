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

#include "gs/monitor/indicators/indicators_m600_comms.h"

#include <assert.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/motor_thermal_types.h"
#include "avionics/common/network_config.h"
#include "avionics/firmware/drivers/si7021_types.h"
#include "avionics/network/routes.h"
#include "avionics/network/switch_links.h"
#include "common/macros.h"
#include "gs/monitor/monitor.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_types.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/indicator.h"
#include "system/labels.h"

static const char *ConvCsPortToStr(CoreSwitchLabel cs, int32_t port) {
  AioNode node = CoreSwitchLabelToCoreSwitchAioNode(cs);
  const SwitchLinkMap *map = GetSwitchLinkMap(node);
  for (int32_t i = 0; i < map->num_local_links; i++) {
    if (map->local_links[i].local_port == port) {
      if (map->local_links[i].link_name != NULL) {
        return map->local_links[i].link_name;
      }
      return map->local_links[i].remote_name;
    }
  }
  for (int32_t i = 0; i < map->num_remote_links; i++) {
    if (map->remote_links[i].local_port == port) {
      if (map->remote_links[i].link_name != NULL) {
        return map->remote_links[i].link_name;
      }
      return map->remote_links[i].remote_name;
    }
  }
  return "Unused";
}

// Indicators of the form UpdateAioStatusUpdated<node>* will give an error (red)
// if the CVT of the aio_snapshot hasn't been updated with a new <node> status
// message.
void UpdateAioStatusUpdatedControllers(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Controllers");

  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool stale[kNumControllers];
  bool any_stale = false;
  bool all_stale = true;

  for (int32_t i = 0; i < kNumControllers; ++i) {
    stale[i] = !CheckControllerComms((ControllerLabel)i);
    // TODO: Remove or revisit the following statement once
    // triplicate flight controllers are implemented.
    if ((ControllerLabel)i != kControllerA) continue;
    any_stale |= stale[i];
    all_stale &= stale[i];
  }

  MON_PRINTF(ind, "A: %d   B: %d   C: %d", !stale[kControllerA],
             !stale[kControllerB], !stale[kControllerC]);

  if (all_stale) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (any_stale) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

static void DisplayCoreSwitchStatusUpdate(Indicator *ind,
                                          const CoreSwitchLabel cs_labels[2]) {
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool stale_A = !CheckCsComms(cs_labels[0]);
  bool stale_B = !CheckCsComms(cs_labels[1]);

  const char *message;
  if (stale_A && stale_B) {
    message = "Both Down";
  } else if (stale_A) {
    message = "Only B Up";
  } else if (stale_B) {
    message = "Only A up";
  } else {
    message = "Both Up";
  }

  MON_PRINTF(ind, "%s", message);
  if (stale_A && stale_B) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (stale_A || stale_B) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateAioStatusUpdatedCoreSwitchGs(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "GS Core Switch");
  CoreSwitchLabel cs_labels[] = {kCoreSwitchGsA, kCoreSwitchGsB};
  DisplayCoreSwitchStatusUpdate(ind, cs_labels);
}

void UpdateAioStatusUpdatedCoreSwitchWing(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Wing Core Switch");
  CoreSwitchLabel cs_labels[] = {kCoreSwitchA, kCoreSwitchB};
  DisplayCoreSwitchStatusUpdate(ind, cs_labels);
}

void UpdateAioStatusUpdatedDrumSensors(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Drum Sensors");
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool stale[kNumDrums];
  bool any_stale = false;
  bool all_stale = true;

  for (int32_t i = 0; i < kNumDrums; ++i) {
    stale[i] = !CheckDrumComms((DrumLabel)i);
    any_stale |= stale[i];
    all_stale &= stale[i];
  }

  MON_PRINTF(ind, "A: %d B: %d", !stale[kDrumSensorsA], !stale[kDrumSensorsB]);

  if (all_stale) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (any_stale) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateAioStatusUpdatedFlightComputers(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Flight Computers");

  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool stale[kNumFlightComputers];
  bool any_stale = false;
  bool all_stale = true;

  for (int32_t i = 0; i < kNumFlightComputers; ++i) {
    stale[i] = !CheckFlightComputerComms((FlightComputerLabel)i);
    any_stale |= stale[i];
    all_stale &= stale[i];
  }

  MON_PRINTF(ind, "FCA: %d FCB: %d FCC: %d", !stale[kFlightComputerA],
             !stale[kFlightComputerB], !stale[kFlightComputerC]);

  if (all_stale) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (any_stale) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateAioStatusUpdatedGsGps(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "GS GPS");
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool stale = !(CheckGsGpsCompassComms() && CheckGsGpsSolutionComms());

  MON_PRINTF(ind, "%d", !stale);

  if (stale) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateAioStatusUpdatedJoystick(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Joystick");
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool stale = !CheckJoystickComms();

  bool joystick_not_present =
      CheckWarning(&aio_1->joystick.status, kJoystickWarningNotPresent);

  if (stale) {
    MON_PRINTF(ind, "%s", "No Update");
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (joystick_not_present) {
    MON_PRINTF(ind, "%s", "Joystick Not Present");
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else {
    MON_PRINTF(ind, "%s", "Up");
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateAioStatusUpdatedLoadcells(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Loadcells");
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  bool stale[kNumLoadcellNodes];
  bool any_stale = false;

  for (LoadcellNodeLabel i = 0; i < kNumLoadcellNodes; ++i) {
    stale[i] = !CheckLoadcellComms(i);
    any_stale |= stale[i];
  }

  MON_PRINTF(ind, "P1: %d P2: %d S1: %d S2: %d", !stale[kLoadcellNodePortA],
             !stale[kLoadcellNodePortB], !stale[kLoadcellNodeStarboardA],
             !stale[kLoadcellNodeStarboardB]);

  if (any_stale) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

static void DisplayMotorRowStatusUpdate(
    Indicator *ind, const MotorLabel motor_labels[NUM_ROW_MOTORS]) {
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool stale[NUM_ROW_MOTORS];
  bool any_stale = false;
  bool all_stale = true;

  for (int32_t i = 0; i < NUM_ROW_MOTORS; ++i) {
    stale[i] = !CheckMotorComms(motor_labels[i]);
    any_stale |= stale[i];
    all_stale &= stale[i];
  }

  MON_PRINTF(ind, "%s: %d %s: %d %s: %d %s: %d",
             MotorLabelToString(motor_labels[0]), !stale[0],
             MotorLabelToString(motor_labels[1]), !stale[1],
             MotorLabelToString(motor_labels[2]), !stale[2],
             MotorLabelToString(motor_labels[3]), !stale[3]);

  if (all_stale) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (any_stale) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateAioStatusUpdatedMotorsBottom(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Bottom Motors");
  MotorLabel motor_labels[] = {kMotorPbo, kMotorPbi, kMotorSbi, kMotorSbo};
  DisplayMotorRowStatusUpdate(ind, motor_labels);
}

void UpdateAioStatusUpdatedMotorsTop(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Top Motors");
  MotorLabel motor_labels[] = {kMotorPto, kMotorPti, kMotorSti, kMotorSto};
  DisplayMotorRowStatusUpdate(ind, motor_labels);
}

void UpdateAioStatusUpdatedPlatformSensors(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Platform Sensors");
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool stale[kNumPlatforms];
  bool any_stale = false;
  bool all_stale = true;

  for (int32_t i = 0; i < kNumPlatforms; ++i) {
    stale[i] = !CheckPlatformComms(i);
    any_stale |= stale[i];
    all_stale &= stale[i];
  }

  MON_PRINTF(ind, "A: %d B: %d", !stale[kPlatformSensorsA],
             !stale[kPlatformSensorsB]);

  if (all_stale) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (any_stale) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateAioStatusUpdatedSelfTest(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Self Test");
  if (!CheckAioComms()) {
    MON_PRINTF(ind, "%s", "Ready");
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  for (AioNode node = 0; node < ARRAYSIZE(aio_3->self_test); ++node) {
    const SelfTestMessage *self_test;
    if (CheckSelfTest(node, &self_test)) {
      char text[sizeof(self_test->text)];
      memcpy(text, self_test->text, sizeof(text) - 1);
      text[sizeof(text) - 1] = '\0';
      MON_PRINTF(ind, "%s: %s", AioNodeToShortString(node), text);
      indicator_set_state(ind, INDICATOR_STATE_WARNING);
      return;
    }
  }
  MON_PRINTF(ind, "%s", "Ready");
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
}

void UpdateAioStatusUpdatedServosPort(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Port Servos");

  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  ServoLabel servo_labels[] = {kServoA1, kServoA2, kServoA4};
  const int32_t kNumPortServos = ARRAYSIZE(servo_labels);
  bool stale[kNumPortServos];
  bool any_stale = false;
  bool all_stale = true;

  for (int32_t i = 0; i < kNumPortServos; ++i) {
    stale[i] = !CheckServoComms(servo_labels[i]);
    any_stale |= stale[i];
    all_stale &= stale[i];
  }

  MON_PRINTF(ind, "A1: %d A2: %d A4: %d", !stale[0], !stale[1], !stale[2]);

  if (all_stale) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (any_stale) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateAioStatusUpdatedServosStar(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Star Servos");

  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  ServoLabel servo_labels[] = {kServoA5, kServoA7, kServoA8};
  const int32_t kNumStarServos = ARRAYSIZE(servo_labels);
  bool stale[kNumStarServos];
  bool any_stale = false;
  bool all_stale = true;

  for (int32_t i = 0; i < kNumStarServos; ++i) {
    stale[i] = !CheckServoComms(servo_labels[i]);
    any_stale |= stale[i];
    all_stale &= stale[i];
  }

  MON_PRINTF(ind, "A5: %d A7: %d A8: %d", !stale[0], !stale[1], !stale[2]);

  if (all_stale) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (any_stale) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateAioStatusUpdatedServosTail(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Tail Servos");

  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  ServoLabel servo_labels[] = {kServoE1, kServoE2, kServoR1, kServoR2};
  const int32_t kNumTailServos = ARRAYSIZE(servo_labels);
  bool stale[kNumTailServos];
  bool any_stale = false;
  bool all_stale = true;

  for (int32_t i = 0; i < kNumTailServos; ++i) {
    stale[i] = !CheckServoComms(servo_labels[i]);
    any_stale |= stale[i];
    all_stale &= stale[i];
  }

  MON_PRINTF(ind, "E1: %d E2: %d R1: %d R2: %d", !stale[0], !stale[1],
             !stale[2], !stale[3]);

  if (all_stale) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (any_stale) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateAioStatusUpdatedWinchPlc(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Winch PLC");
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool stale = !CheckWinchPlcComms();

  MON_PRINTF(ind, "%d", !stale);

  if (stale) {
    // TODO: Once we have a winch PLC again, this should be an error.
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateMaxBoardTemperature(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Max Board Temp");
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  double max_temp = DBL_MIN;
  AioNode node = kAioNodeUnknown;

  // TODO: Add other node types when available.
  // TODO: Cleanup after updating TMS570 monitoring code.
  for (CoreSwitchLabel i = 0; i < kNumCoreSwitches; ++i) {
    if (CheckCsComms(i)) {
      int32_t num_si7021s =
          ARRAYSIZE(aio_1->core_switch_statuses[i].cs_mon.si7021_data);
      for (int32_t j = 0; j < num_si7021s; ++j) {
        if (aio_1->core_switch_statuses[i].cs_mon.si7021_populated &
            (1U << j)) {
          double temp =
              aio_1->core_switch_statuses[i].cs_mon.si7021_data[j].temperature;
          if (temp > max_temp) {
            max_temp = temp;
            node = CoreSwitchLabelToCoreSwitchAioNode(i);
          }
        }
      }
    }
  }

  for (MotorLabel i = 0; i < kNumMotors; ++i) {
    if (CheckMotorComms(i)) {
      double temp = aio_1->motor_statuses[i].temps[kMotorThermalChannelBoard];
      if (temp > max_temp) {
        max_temp = temp;
        node = MotorLabelToMotorAioNode(i);
      }
    }
  }

  for (ServoLabel i = 0; i < kNumServos; ++i) {
    if (CheckServoComms(i)) {
      int32_t num_si7021s =
          ARRAYSIZE(aio_1->servo_statuses[i].aio_mon.si7021_data);
      for (int32_t j = 0; j < num_si7021s; ++j) {
        if (aio_1->servo_statuses[i].aio_mon.si7021_populated & (1U << j)) {
          double temp =
              aio_1->servo_statuses[i].aio_mon.si7021_data[j].temperature;
          if (temp > max_temp) {
            max_temp = temp;
            node = ServoLabelToServoAioNode(i);
          }
        }
      }
    }
  }

  // TODO: Determine thresholds.
  if (node == kAioNodeUnknown) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else if (max_temp > 100.0) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (max_temp > 85.0) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
  if (node == kAioNodeUnknown) {
    indicator_set_value(ind, "No update");
  } else {
    MON_PRINTF(ind, "%s at %.1f C", AioNodeToShortString(node), max_temp);
  }
}

static void DisplayCoreSwitchLinkState(Indicator *ind,
                                       CoreSwitchLabel core_switch) {
  assert(0 <= core_switch);
  assert(core_switch < kNumCoreSwitches);

  const CoreSwitchSlowStatusMessage *cs_slow_status;

  if (!CheckAioComms() ||
      !CheckCoreSwitchSlowStatus(
          CoreSwitchLabelToCoreSwitchAioNode(core_switch), &cs_slow_status)) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const char legend[] = "               Rx         Tx    L Error\n";
  const int32_t kStrLineBuffer = sizeof(legend);
  char link_state_str[(NUM_CORE_SWITCH_PORTS + 1) * kStrLineBuffer];
  const char *port_str;
  int32_t index = snprintf(link_state_str, sizeof(link_state_str), legend);
  index +=
      snprintf(link_state_str + index, sizeof(link_state_str) - (size_t)index,
               "           pkts/Mbps  pkts/Mbps");
  for (int32_t i = 0; i < NUM_CORE_SWITCH_PORTS; ++i) {
    assert(index < (int32_t)sizeof(link_state_str));
    port_str = ConvCsPortToStr(core_switch, i);

    const EthernetStats *stats = &cs_slow_status->switch_stats.stats[i];
    int errors = stats->rx_fragment_errors + stats->rx_alignment_errors +
                 stats->rx_fcs_errors + stats->rx_symbol_errors +
                 stats->rx_jabber_errors + stats->rx_in_range_errors;
    const char *state_str;
    if ((cs_slow_status->switch_stats.link_status_bits & (1u << i)) != 0) {
      if ((cs_slow_status->switch_stats.segment_status_bits & (1u << i)) != 0) {
        state_str = " ";
      } else {
        state_str = "M";
      }
    } else {
      state_str = "X";
    }
    index += snprintf(
        link_state_str + index, sizeof(link_state_str) - (size_t)index,
        "\n%-9s %5d/%4.1f %5d/%4.1f %1s %3.0d", port_str,
        stats->rx_multicast_packet_rate,
        (float)stats->rx_octet_rate * 8.0f / 1000000.0f,
        stats->tx_multicast_packet_rate,
        (float)stats->tx_octet_rate * 8.0f / 1000000.0f, state_str, errors);
  }

  indicator_set_state(ind, INDICATOR_STATE_GOOD);
  indicator_set_value(ind, link_state_str);
}

void UpdateCsAPortTraffic(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "CSA Port Traffic");
  DisplayCoreSwitchLinkState(ind, kCoreSwitchA);
}

void UpdateCsBPortTraffic(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "CSB Port Traffic");
  DisplayCoreSwitchLinkState(ind, kCoreSwitchB);
}

void UpdateCsGsAPortTraffic(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "CSGSA Port Traffic");
  DisplayCoreSwitchLinkState(ind, kCoreSwitchGsA);
}

void UpdateCsGsBPortTraffic(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "CSGSB Port Traffic");
  DisplayCoreSwitchLinkState(ind, kCoreSwitchGsB);
}

static void UpdateMotorsNetworkErrors(Indicator *ind, int32_t network_index) {
  assert(network_index >= 0 && network_index <= 1);
  if (!CheckAioComms() || !CheckAnyMotorComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  static int32_t current_error_total[2], last_error_total[2];
  const int32_t kStrLineBuffer = 35;
  char link_state_str[(kNumMotors + 1) * kStrLineBuffer];

  int32_t index = snprintf(link_state_str, sizeof(link_state_str),
                           "Eth Errors |frg|alg|fcs|sym|txd");

  for (int32_t i = 0; i < kNumMotors; ++i) {
    AioNode node = MotorLabelToMotorAioNode(i);
    const EthernetStats *motor_stats =
        &aio_2->slow_statuses[node].switch_stats.stats[0];
    const char *network_str = network_index == 0 ? "A" : "B";
    index += snprintf(
        link_state_str + index, sizeof(link_state_str) - (size_t)index,
        "\n%s   %s    %3d %3d %3d %3d %3d", network_str, MotorLabelToString(i),
        motor_stats[network_index].rx_fragment_errors,
        motor_stats[network_index].rx_alignment_errors,
        motor_stats[network_index].rx_fcs_errors,
        motor_stats[network_index].rx_symbol_errors,
        motor_stats[network_index].tx_dropped_packets);
    current_error_total[network_index] +=
        motor_stats[network_index].rx_fragment_errors +
        motor_stats[network_index].rx_alignment_errors +
        motor_stats[network_index].rx_fcs_errors +
        motor_stats[network_index].rx_symbol_errors +
        motor_stats[network_index].tx_dropped_packets;
  }
  indicator_set_value(ind, link_state_str);
  if (last_error_total[network_index] == current_error_total[network_index]) {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  }
  last_error_total[network_index] = current_error_total[network_index];
}

void UpdateMotorsNetworkAErrors(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Motor Net A");
  UpdateMotorsNetworkErrors(ind, 0);
}

void UpdateMotorsNetworkBErrors(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Motor Net B");
  UpdateMotorsNetworkErrors(ind, 1);
}
