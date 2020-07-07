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

#ifndef AVIONICS_COMMON_IMU_OUTPUT_H_
#define AVIONICS_COMMON_IMU_OUTPUT_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/imu_config.h"
#include "avionics/firmware/drivers/adis16488.h"

void ImuOutputInit(FlightComputerSensorMessage *sensor_message);

bool ImuOutputData(const Adis16488OutputData *imu,
                   FlightComputerSensorMessage *sensor_message, int64_t now);

void ImuOutputSendImuMessage(const float *imu_coef, int64_t now);

void ImuOutputUpdateFailure(bool failure,
                            FlightComputerSensorMessage *sensor_message);

void ImuOutputUpdateSensorMessage(FlightComputerSensorMessage *sensor_message,
                                  int64_t now);

#endif  // AVIONICS_COMMON_IMU_OUTPUT_H_
