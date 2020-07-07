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

#ifndef CONTROL_AVIONICS_AVIONICS_CONVERSION_H_
#define CONTROL_AVIONICS_AVIONICS_CONVERSION_H_

#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/gill_types.h"
#include "avionics/common/imu_types.h"
#include "avionics/common/novatel_types.h"
#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/sensor_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void ConvertControllerSync(const ControllerSyncMessage *sync_message,
                           ControlSyncData *sync_data);
void ConvertImuAccGyro(const FlightComputerImuMessage *imu_in,
                       const ImuParams *params, ImuData *imu_out);
void ConvertImuMag(const FlightComputerSensorMessage *sensor_in,
                   const ImuParams *params, ImuData *imu_out);
void ConvertJoystick(const TetherJoystick *joystick_in,
                     bool tether_release_armed, const JoystickParams *params,
                     JoystickData *joystick_out);
void ConvertLoadcells(const LoadcellMessage loadcell_messages[],
                      const bool loadcell_nodes_faulted[],
                      const LoadcellParams params[], double loadcells_out[],
                      bool *tether_release_armed, bool *tether_released);
void ConvertNovAtelBestXyzToGps(const NovAtelLogBestXyz *best_xyz,
                                GpsData *gps_data);
void ConvertSeptentrioSolutionToGps(const SeptentrioSolutionMessage *solution,
                                    GpsData *gps_data);
void ConvertPitot(const PitotSensor *pitot_in, const PitotSensorParams *params,
                  PitotData *pitot_out);
void ConvertNovAtelCompass(const NovAtelCompassMessage *novatel_compass,
                           GpsCompassData *compass);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_AVIONICS_AVIONICS_CONVERSION_H_
