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

#ifndef GS_MONITOR_MONITOR_UTIL_H_
#define GS_MONITOR_MONITOR_UTIL_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/network/aio_node.h"
#include "control/control_telemetry.h"
#include "gs/aio_snapshot/aio_telemetry.h"
#include "system/labels.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MON_PRINTF(_ind, _fmtstr, ...)                        \
  do {                                                        \
    char _tmpstr[1024];                                       \
    snprintf(_tmpstr, sizeof(_tmpstr), _fmtstr, __VA_ARGS__); \
    indicator_set_value(_ind, _tmpstr);                       \
  } while (0)

#define NUM_ROW_MOTORS 4
#define NUM_STACKING_LEVELS 4
#define NUM_SIDE_SERVOS 3
#define NUM_TAIL_SERVOS 2

typedef struct {
  char prefix[32];
  FaultMask fault;
} FaultDispInfo;

bool ShouldInitialize(void);
void CompleteInitialization(void);

bool CheckAioComms(void);
bool CheckControllerComms(ControllerLabel controller_label);
bool CheckControllerSlowComms(ControllerLabel controller_label);
bool CheckControllerTelemetry(void);
bool CheckControllerSlowTelemetry(void);
bool CheckControllerRunning(void);
bool CheckCoreSwitchSlowStatus(AioNode node,
                               const CoreSwitchSlowStatusMessage **slow_status);
bool CheckCsComms(CoreSwitchLabel cs_label);
bool CheckDrumComms(DrumLabel drum_label);
bool CheckFlightComputerComms(FlightComputerLabel flight_computer_label);
bool CheckGroundStationWeatherComms(void);
bool CheckGsGpsCompassComms(void);
bool CheckGsGpsObservationsComms(void);
bool CheckGsGpsSolutionComms(void);
bool CheckJoystickComms(void);
bool CheckJoystickMonitorComms(void);
bool CheckLoadcellComms(LoadcellNodeLabel loadcell_label);
bool CheckMotorComms(MotorLabel motor_label);
bool CheckAnyMotorComms(void);
bool CheckPlatformComms(PlatformLabel platform_label);
bool CheckQ7SlowStatus(AioNode node, const Q7SlowStatusMessage **slow_status);
bool CheckRecorderTms570Comms(RecorderTms570Label recorder_label);
bool CheckSelfTest(AioNode node, const SelfTestMessage **self_test);
bool CheckServoComms(ServoLabel servo_label);
bool CheckSlowStatus(AioNode node, const SlowStatusMessage **slow_status);
bool CheckTetherDownComms(AioNode node, const TetherDownMessage **tether_down);
bool CheckWingGpsSolutionComms(WingGpsReceiverLabel label);
bool CheckMotorsOn(void);
bool CheckWinchPlcComms(void);

void ConvTimeSecToStr(int64_t time_sec, uint32_t len, char *hms);
const char *GetFaultDispStr(const FaultDispInfo *fdisp, int32_t num);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // GS_MONITOR_MONITOR_UTIL_H_
