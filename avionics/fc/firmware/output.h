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

#ifndef AVIONICS_FC_FIRMWARE_OUTPUT_H_
#define AVIONICS_FC_FIRMWARE_OUTPUT_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/fc/firmware/config_params.h"
#include "avionics/firmware/drivers/adis16488.h"
#include "avionics/firmware/drivers/hsc.h"
#include "avionics/firmware/monitors/aio_types.h"
#include "avionics/firmware/monitors/fc_types.h"

void FcOutputInit(FlightComputerSensorMessage *sensor_message);
void FcOutputGpsFailure(bool failure,
                        FlightComputerSensorMessage *sensor_message);
void FcOutputFpvState(bool enabled,
                      FlightComputerSensorMessage *sensor_message);
void FcOutputPitotState(uint8_t status,
                        FlightComputerSensorMessage *sensor_message);
void FcOutputPitot(HscDevice dev, const PitotCalib *cal, const HscData *sdata,
                   FlightComputerSensorMessage *sensor_message, int64_t now);
void FcOutputPps(int64_t now);
void FcOutputUpdateSensorMessage(FlightComputerSensorMessage *sensor_message,
                                 int64_t now);

#endif  // AVIONICS_FC_FIRMWARE_OUTPUT_H_
