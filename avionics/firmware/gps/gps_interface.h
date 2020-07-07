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

#ifndef AVIONICS_FIRMWARE_GPS_GPS_INTERFACE_H_
#define AVIONICS_FIRMWARE_GPS_GPS_INTERFACE_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/gps_receiver.h"
#include "avionics/common/novatel_types.h"
#include "avionics/common/septentrio_types.h"
#include "avionics/firmware/drivers/novatel.h"
#include "avionics/firmware/drivers/septentrio.h"

typedef struct {
  const NovAtelDevice *novatel;
  const SeptentrioDevice *septentrio;
} GpsConfig;

extern const GpsConfig kGpsConfigBase;
extern const GpsConfig kGpsConfigRover;

void GpsInit(void);
void GpsPoll(const GpsConfig *gps, GpsReceiverType receiver, int64_t now);
void GpsUpdatePpsTimestamp(int64_t now);
void GpsSendSerialDebugMessage(void);
void GpsSendSolutionMessage(GpsReceiverType receiver, int64_t now);
void GpsSendObservationsMessage(GpsReceiverType receiver, int64_t now);
void GpsSendCompassMessage(GpsReceiverType receiver, int64_t now);
void GpsSendSatellitesMessage(int64_t now);
void GpsSendTimeMessage(int64_t now);
void GpsGetTimeOfWeek(int64_t now, int32_t *latency, int32_t *time_of_week);
void GpsSendRtcmMessage(uint16_t message_number, int32_t length,
                        const uint8_t *data);
void GpsInsertRtcm(const GpsConfig *gps, GpsReceiverType receiver,
                   int32_t length, const uint8_t *data);
bool GpsIsReady(GpsReceiverType receiver);

#endif  // AVIONICS_FIRMWARE_GPS_GPS_INTERFACE_H_
