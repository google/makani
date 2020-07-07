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

#include "gs/monitor/monitor_util.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "avionics/common/gps_receiver.h"
#include "avionics/network/aio_node.h"
#include "control/common.h"
#include "control/control_telemetry.h"
#include "control/fault_detection/fault_detection_types.h"
#include "gs/aio_snapshot/aio_telemetry.h"
#include "gs/monitor/monitor.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_params.h"
#include "gs/monitor/monitor_types.h"
#include "system/labels.h"
#include "system/labels_util.h"

static bool g_should_initialze = true;

bool ShouldInitialize(void) { return g_should_initialze; }

void CompleteInitialization(void) { g_should_initialze = false; }

bool CheckAioComms(void) {
  const MonAioParams *params = &GetMonitorParams()->comms.aio;
  return !ShouldInitialize() &&
         fd->aio_telem_stale_count <= params->telemetry_timeout;
}

// These macros are only used in Check<Message>Comms() below to reduce
// repetition.  The somewhat obnoxious abbreviations and helper macros
// are used here to avoid a disagreement between two different lint
// tools.
#define AIO_CHECK_IS_NOT_STALE_HELPER(field) \
  (fd->aio_stale_counts.field < GetMonitorParams()->comms.aio.field##_timeout)
#define AIO_CHECK_IS_NOT_STALE(field) \
  (!ShouldInitialize() && AIO_CHECK_IS_NOT_STALE_HELPER(field))

#define AIO_CHECK_ELEMENT_IS_NOT_STALE_HELPER(elt, i) \
  (fd->aio_stale_counts.elt[i] < GetMonitorParams()->comms.aio.elt##_timeout)
#define AIO_CHECK_ELEMENT_IS_NOT_STALE(elt, i) \
  (!ShouldInitialize() && AIO_CHECK_ELEMENT_IS_NOT_STALE_HELPER(elt, i))

bool CheckControllerComms(ControllerLabel controller_label) {
  return AIO_CHECK_ELEMENT_IS_NOT_STALE(control_telemetries, controller_label);
}

bool CheckControllerSlowComms(ControllerLabel controller_label) {
  return AIO_CHECK_ELEMENT_IS_NOT_STALE(control_slow_telemetries,
                                        controller_label);
}

bool CheckControllerTelemetry(void) { return CheckControllerComms(fd->leader); }

bool CheckControllerSlowTelemetry(void) {
  return CheckControllerSlowComms(fd->leader);
}

bool CheckControllerRunning(void) {
  return CheckControllerTelemetry() &&
         IsControlSystemRunning((InitializationState)ct->init_state);
}

bool CheckCoreSwitchSlowStatus(
    AioNode node, const CoreSwitchSlowStatusMessage **slow_status) {
  CoreSwitchLabel label = CoreSwitchAioNodeToCoreSwitchLabel(node);
  if (AIO_CHECK_ELEMENT_IS_NOT_STALE(core_switch_slow_statuses, label)) {
    *slow_status = &aio_2->core_switch_slow_statuses[label];
    return true;
  }
  return false;
}

bool CheckCsComms(CoreSwitchLabel cs_label) {
  return AIO_CHECK_ELEMENT_IS_NOT_STALE(core_switch_statuses, cs_label);
}

bool CheckDrumComms(DrumLabel drum_label) {
  return AIO_CHECK_ELEMENT_IS_NOT_STALE(drum_sensors, drum_label);
}

bool CheckFlightComputerComms(FlightComputerLabel flight_computer_label) {
  return AIO_CHECK_ELEMENT_IS_NOT_STALE(flight_computer_sensors,
                                        flight_computer_label);
}

bool CheckGroundStationWeatherComms(void) {
  return AIO_CHECK_IS_NOT_STALE(ground_station_weather);
}

bool CheckGsGpsCompassComms(void) {
  return AIO_CHECK_IS_NOT_STALE(gs_gps_compass);
}

bool CheckGsGpsObservationsComms(void) {
  return AIO_CHECK_IS_NOT_STALE(gs_gps_observations);
}

bool CheckGsGpsSolutionComms(void) {
  return AIO_CHECK_IS_NOT_STALE(gs_gps_solution);
}

bool CheckJoystickComms(void) { return AIO_CHECK_IS_NOT_STALE(joystick); }

bool CheckJoystickMonitorComms(void) {
  return AIO_CHECK_IS_NOT_STALE(joystick_monitor);
}

bool CheckLoadcellComms(LoadcellNodeLabel loadcell_label) {
  return AIO_CHECK_ELEMENT_IS_NOT_STALE(loadcell_statuses, loadcell_label);
}

bool CheckMotorComms(MotorLabel motor_label) {
  return AIO_CHECK_ELEMENT_IS_NOT_STALE(motor_statuses, motor_label);
}

bool CheckAnyMotorComms(void) {
  bool any_motor_comms = false;
  for (int32_t i = 0; i < kNumMotors; ++i) {
    any_motor_comms |= CheckMotorComms((MotorLabel)i);
  }
  return any_motor_comms;
}

bool CheckQ7SlowStatus(AioNode node, const Q7SlowStatusMessage **slow_status) {
  if (IsControllerNode(node)) {
    ControllerLabel label = ControllerAioNodeToControllerLabel(node);
    if (AIO_CHECK_ELEMENT_IS_NOT_STALE(controller_q7_slow_statuses, label)) {
      *slow_status = &aio_2->controller_q7_slow_statuses[label];
      return true;
    }
  } else if (IsRecorderQ7Node(node)) {
    RecorderQ7Label label = RecorderQ7AioNodeToRecorderQ7Label(node);
    if (AIO_CHECK_ELEMENT_IS_NOT_STALE(recorder_q7_slow_statuses, label)) {
      *slow_status = &aio_2->recorder_q7_slow_statuses[label];
      return true;
    }
  }
  return false;
}

bool CheckPlatformComms(PlatformLabel platform_label) {
  return AIO_CHECK_ELEMENT_IS_NOT_STALE(platform_sensors, platform_label);
}

bool CheckRecorderTms570Comms(RecorderTms570Label recorder_label) {
  return AIO_CHECK_ELEMENT_IS_NOT_STALE(recorder_statuses, recorder_label);
}

bool CheckSelfTest(AioNode node, const SelfTestMessage **self_test) {
  if (AIO_CHECK_ELEMENT_IS_NOT_STALE(self_test, node)) {
    *self_test = &aio_3->self_test[node];
    return true;
  }
  return false;
}

bool CheckServoComms(ServoLabel servo_label) {
  return AIO_CHECK_ELEMENT_IS_NOT_STALE(servo_statuses, servo_label);
}

bool CheckSlowStatus(AioNode node, const SlowStatusMessage **slow_status) {
  if (AIO_CHECK_ELEMENT_IS_NOT_STALE(slow_statuses, node)) {
    *slow_status = &aio_2->slow_statuses[node];
    return true;
  }
  return false;
}

bool CheckTetherDownComms(AioNode node, const TetherDownMessage **tether_down) {
  int32_t index;
  if (node == kAioNodeCsA) {
    index = 0;
  } else if (node == kAioNodeCsGsA) {
    index = 1;
  } else {
    assert(false);
    return false;
  }
  if (AIO_CHECK_ELEMENT_IS_NOT_STALE(tether_down, index)) {
    *tether_down = &aio_1->tether_down[index];
    return true;
  }
  return false;
}

bool CheckWinchPlcComms(void) { return AIO_CHECK_IS_NOT_STALE(winch_plc); }

bool CheckWingGpsSolutionComms(WingGpsReceiverLabel label) {
  GpsReceiverType gps_type = WingGpsReceiverLabelToGpsReceiverType(label);
  if (gps_type == kGpsReceiverTypeNovAtel) {
    return AIO_CHECK_ELEMENT_IS_NOT_STALE(wing_gps_novatel_solutions, label);
  } else if (gps_type == kGpsReceiverTypeSeptentrio) {
    return AIO_CHECK_ELEMENT_IS_NOT_STALE(wing_gps_septentrio_solutions, label);
  } else {
    assert(!(bool)"Invalid GPS type.");
    return false;
  }
}

#undef AIO_CHECK_IS_NOT_STALE
#undef AIO_CHECK_ELEMENT_IS_NOT_STALE

bool CheckMotorsOn() {
  for (int32_t i = 0; i < kNumMotors; ++i)
    if (ct->control_input.rotors[i] > 1.0) return 1;
  return 0;
}

void ConvTimeSecToStr(int64_t time_sec, uint32_t len, char *hms) {
  int64_t hours = time_sec / 3600;
  int64_t minutes = (time_sec % 3600) / 60;
  int64_t seconds = time_sec % 60;
  snprintf(hms, len, "%ld:%02ld:%02ld", hours, minutes, seconds);
}

static int32_t ConvFaultMaskToStrings(const char *prefix,
                                      const FaultMask *fault,
                                      char fault_list[][32]) {
  int32_t num_faults = 0;
  for (int32_t i = 0; i < kNumFaultTypes; ++i) {
    if (HasFault(i, fault)) {
      snprintf(fault_list[num_faults], sizeof(fault_list[num_faults]), "%s%s",
               prefix, FaultTypeToString(i));
      num_faults++;
    }
  }
  return num_faults;
}

const char *GetFaultDispStr(const FaultDispInfo *fdisp, int32_t num) {
  assert(num <= 10);
  static char fault_list[10 * kNumFaultTypes][32];
  int32_t num_faults = 0;
  for (int32_t i = 0; i < num; ++i) {
    num_faults += ConvFaultMaskToStrings(fdisp[i].prefix, &fdisp[i].fault,
                                         &fault_list[num_faults]);
  }
  return fault_list[((int32_t)ct->time) % num_faults];
}
