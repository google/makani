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

#ifndef AVIONICS_FIRMWARE_DRIVERS_EEPROM24_H_
#define AVIONICS_FIRMWARE_DRIVERS_EEPROM24_H_

#include <stdbool.h>
#include <stdint.h>

void Eeprom24Init(void);
bool Eeprom24Poll(void);
bool Eeprom24HasError(void);
bool Eeprom24ReadAsync(int32_t addr, int32_t length, uint8_t *data);
bool Eeprom24WriteAsync(int32_t addr, int32_t length, const uint8_t *data);
bool Eeprom24ReadSync(int32_t addr, int32_t data_len, uint8_t *data);
bool Eeprom24WriteSync(int32_t addr, int32_t data_len, const uint8_t *data);

#endif  // AVIONICS_FIRMWARE_DRIVERS_EEPROM24_H_
