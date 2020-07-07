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

#include "avionics/common/imu_output.h"

#include <assert.h>
#include <limits.h>
#include <string.h>

#include "avionics/common/adis16488_types.h"
#include "avionics/common/avionics_messages.h"
#include "avionics/common/coning_sculling.h"
#include "avionics/common/imu_config.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/drivers/adis16488.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"

#define NUM_IMU_FILTER_COEFS ARRAYSIZE(((ImuConfig *)0)->fir_coefs)

// Status message.
static FlightComputerImuMessage g_imu;
static int64_t g_imu_raw_timestamp = INT32_MIN;
static int64_t g_imu_mag_timestamp = INT32_MIN;
static int64_t g_imu_pressure_timestamp = INT32_MIN;
static int64_t
    g_imu_cs_timestamp[ARRAYSIZE(((FlightComputerSensorMessage *)0)->cs)] = {0};

// IMU filter.
static struct {
  float ax[NUM_IMU_FILTER_COEFS];
  float ay[NUM_IMU_FILTER_COEFS];
  float az[NUM_IMU_FILTER_COEFS];
  float gx[NUM_IMU_FILTER_COEFS];
  float gy[NUM_IMU_FILTER_COEFS];
  float gz[NUM_IMU_FILTER_COEFS];
  int32_t index;
} g_imu_filter;

static void ImuUpdateFilter(const Adis16488OutputData *imu) {
  int32_t index = (g_imu_filter.index + 1) % NUM_IMU_FILTER_COEFS;
  g_imu_filter.ax[index] = imu->accel[0];
  g_imu_filter.ay[index] = imu->accel[1];
  g_imu_filter.az[index] = imu->accel[2];
  g_imu_filter.gx[index] = imu->gyro[0];
  g_imu_filter.gy[index] = imu->gyro[1];
  g_imu_filter.gz[index] = imu->gyro[2];
  g_imu_filter.index = index;
}

static float ImuSampleRaw(const float *data, const float *imu_coef,
                          int32_t index) {
  float sum = 0.0f;
  const int32_t num_coefs = NUM_IMU_FILTER_COEFS;
  for (int32_t i = 0; i < num_coefs; ++i) {
    int32_t data_index = (index + (num_coefs - i)) % num_coefs;
    sum += imu_coef[i] * data[data_index];
  }
  return sum;
}

static void ImuUpdateAux(const Adis16488OutputData *imu, int64_t now,
                         ImuAuxSensorData *out) {
  out->temp = imu->temp;
  if (imu->sys_e_flag & kAdis16488StatusNewMagnetometer) {
    g_imu_mag_timestamp = now;
    out->mag[0] = imu->mag[0];
    out->mag[1] = imu->mag[1];
    out->mag[2] = imu->mag[2];
  }
  if (imu->sys_e_flag & kAdis16488StatusNewBarometer) {
    g_imu_pressure_timestamp = now;
    out->pressure = imu->baro;
  }
}

static void ImuUpdateConingSculling(ImuConingScullingData *out, int64_t now) {
  float dt_m;
  Vec3f phi_m, dvsf_m, alpha_m, nu_m;
  ConingScullingSample(&dt_m, &phi_m, &dvsf_m, &alpha_m, &nu_m);
  out->timestamp = (uint32_t)now;
  out->dt = dt_m;
  out->phi[0] = phi_m.x;
  out->phi[1] = phi_m.y;
  out->phi[2] = phi_m.z;
  out->dvsf[0] = dvsf_m.x;
  out->dvsf[1] = dvsf_m.y;
  out->dvsf[2] = dvsf_m.z;
  out->alpha[0] = alpha_m.x;
  out->alpha[1] = alpha_m.y;
  out->alpha[2] = alpha_m.z;
  out->nu[0] = nu_m.x;
  out->nu[1] = nu_m.y;
  out->nu[2] = nu_m.z;
}

static bool ImuUpdateData(const Adis16488OutputData *imu,
                          FlightComputerSensorMessage *sensor_message,
                          int64_t now) {
  // Update status flags.
  g_imu.status = imu->sys_e_flag;
  g_imu.error = imu->diag_sts;
  SignalWarning(kFlightComputerWarningImuData, imu->diag_sts != 0x0,
                &sensor_message->flags);

  // Update raw data.
  g_imu_raw_timestamp = now;
  ImuUpdateFilter(imu);
  ImuUpdateAux(imu, now, &sensor_message->aux);

  // Update coning and sculling data.
  Vec3f omega_ib_b = {imu->gyro[0], imu->gyro[1], imu->gyro[2]};
  Vec3f a_sf_b = {imu->accel[0], imu->accel[1], imu->accel[2]};
  if (ConingScullingUpdateRaw(CONING_SCULLING_DECIMATION, imu->dt, &omega_ib_b,
                              &a_sf_b)) {
    // Shift data such that index 0 always contains the latest data.
    for (int32_t i = ARRAYSIZE(sensor_message->cs) - 1; i > 0; --i) {
      g_imu_cs_timestamp[i] = g_imu_cs_timestamp[i - 1];
      memcpy(&sensor_message->cs[i], &sensor_message->cs[i - 1],
             sizeof(sensor_message->cs[i]));
    }
    g_imu_cs_timestamp[0] = now;
    ImuUpdateConingSculling(&sensor_message->cs[0], now);
    return true;
  }
  return false;
}

void ImuOutputInit(FlightComputerSensorMessage *sensor_message) {
  memset(sensor_message, 0, sizeof(*sensor_message));
  // IMU output.
  ConingScullingInit();
  g_imu_raw_timestamp = INT32_MIN;
  g_imu_mag_timestamp = INT32_MIN;
  for (int32_t i = 0; i < ARRAYSIZE(g_imu_cs_timestamp); ++i) {
    g_imu_cs_timestamp[i] = INT32_MIN;
  }
  memset(&g_imu_filter, 0, sizeof(g_imu_filter));
}

bool ImuOutputData(const Adis16488OutputData *imu,
                   FlightComputerSensorMessage *sensor_message, int64_t now) {
  return ImuUpdateData(imu, sensor_message, now);
}

void ImuOutputSendImuMessage(const float *imu_coef, int64_t now) {
  g_imu.raw.acc[0] =
      ImuSampleRaw(g_imu_filter.ax, imu_coef, g_imu_filter.index);
  g_imu.raw.acc[1] =
      ImuSampleRaw(g_imu_filter.ay, imu_coef, g_imu_filter.index);
  g_imu.raw.acc[2] =
      ImuSampleRaw(g_imu_filter.az, imu_coef, g_imu_filter.index);
  g_imu.raw.gyro[0] =
      ImuSampleRaw(g_imu_filter.gx, imu_coef, g_imu_filter.index);
  g_imu.raw.gyro[1] =
      ImuSampleRaw(g_imu_filter.gy, imu_coef, g_imu_filter.index);
  g_imu.raw.gyro[2] =
      ImuSampleRaw(g_imu_filter.gz, imu_coef, g_imu_filter.index);
  g_imu.raw.latency = SaturateLatency(now - g_imu_raw_timestamp);
  NetSendAioFlightComputerImuMessage(&g_imu);
}

void ImuOutputUpdateFailure(bool failure,
                            FlightComputerSensorMessage *sensor_message) {
  SignalWarning(kFlightComputerWarningImu, failure, &sensor_message->flags);
}

void ImuOutputUpdateSensorMessage(FlightComputerSensorMessage *sensor_message,
                                  int64_t now) {
  // IMU aux sensor data latency.
  sensor_message->aux.mag_latency = SaturateLatency(now - g_imu_mag_timestamp);
  sensor_message->aux.pressure_latency =
      SaturateLatency(now - g_imu_pressure_timestamp);

  // Coning and sculling latency.
  for (int32_t i = 0; i < ARRAYSIZE(sensor_message->cs); ++i) {
    sensor_message->cs[i].latency =
        SaturateLatency(now - g_imu_cs_timestamp[i]);
  }
}
