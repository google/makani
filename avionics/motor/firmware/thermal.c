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

#include "avionics/motor/firmware/thermal.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>

#include "avionics/common/motor_thermal_types.h"
#include "avionics/common/strings.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/drivers/mcp9800_types.h"
#include "avionics/firmware/drivers/si7021.h"
#include "avionics/firmware/monitors/thermal_types.h"
#include "avionics/firmware/serial/motor_serial_params.h"
#include "avionics/motor/firmware/flags.h"
#include "avionics/motor/firmware/nhet.h"
#include "avionics/motor/monitors/motor_mcp342x_types.h"
#include "avionics/motor/monitors/motor_mcp9800_types.h"
#include "avionics/motor/monitors/motor_si7021_types.h"
#include "common/macros.h"

#define N2HET2_LRPFC 5  // N2HET Loop Resolution Prescaler Factor Code
#define N2HET2_LR (1 << N2HET2_LRPFC)  // N2HET Loop Resolution prescaler.

// HT3000 thermistor half sample frequency.
static float g_sample_freq_by_2 = 0.0f;

// Thermal limit [C] for each temperature sensor, which should signal
// a motor shutdown.
MotorThermalParams g_motor_thermal_params = {
  .thermal_limits = {0},
  .over_temp_threshold = 3,
  .temp_read_error_threshold = 3,
};

// Current set of thermal readings [C].
static MotorThermalData g_thermal_data = {
  .channel_temps = {0},
  .over_temp_count = {0},
  .warning_bitmask = 0U,
  .temp_read_errors = 0,
};

typedef struct {
  MotorThermalChannel channel;
  const ThermalSensor *calib;
} MotorThermalCalibMap;

// Single-ended YASA calibration data.
static const ThermalCalData kYasaRotorCal[] = {
  {0.0f, 11725, 0.0076752508f}
};

static const ThermalSensor kYasaRotor = {
  kYasaRotorCal,
  ARRAYSIZE(kYasaRotorCal)
};

// Heat plate (B57703M0502G040) calibration data.
static const ThermalCalData kHeatPlateCal[] = {
  {-40.0f, 32612, -0.1282051282f},
  {-30.0f, 32473, -0.0719424460f},
  {-20.0f, 32234, -0.0418410042f},
  {-10.0f, 31843, -0.0255754476f},
  {0.0f,   31230, -0.0163132137f},
  {10.0f,  30318, -0.0109649123f},
  {20.0f,  29031, -0.0077700078f},
  {30.0f,  27316, -0.0058309038f},
  {40.0f,  25170, -0.0046598322f},
  {50.0f,  22657, -0.0039793076f},
  {60.0f,  19904, -0.0036324010f},
  {70.0f,  17086, -0.0035486160f},
  {80.0f,  14383, -0.0036995930f},
  {90.0f,  11906, -0.0040371417f},
  {100.0f, 9739,  -0.0046146747f},
  {110.0f, 7904,  -0.0054495913f},
  {120.0f, 6387,  -0.0065919578f},
  {130.0f, 5165,  -0.0081833061f},
  {140.0f, 4175,  -0.0101010101f},
  {150.0f, 3386,  -0.0126742712f}
};

static const ThermalSensor kHeatPlate = {
  kHeatPlateCal,
  ARRAYSIZE(kHeatPlateCal)
};

// DC capacitor (B57540G502F) calibration data.
static const ThermalCalData kDcCapacitorCal[] = {
  {-40.0f, 32519, -0.0900900901f},
  {-30.0f, 32336, -0.0546448087f},
  {-20.0f, 32049, -0.0348432056f},
  {-10.0f, 31615, -0.0230414747f},
  {0.0f,   30985, -0.0158730159f},
  {10.0f,  30106, -0.0113765643f},
  {20.0f,  28934, -0.0085324232f},
  {30.0f,  27442, -0.0067024129f},
  {40.0f,  25630, -0.0055187638f},
  {50.0f,  23541, -0.0047869794f},
  {60.0f,  21252, -0.0043687200f},
  {70.0f,  18864, -0.0041876047f},
  {80.0f,  16489, -0.0042105263f},
  {90.0f,  14225, -0.0044169611f},
  {100.0f, 12142, -0.0048007681f},
  {110.0f, 10284, -0.0053821313f},
  {120.0f, 8665,  -0.0061766523f},
  {130.0f, 7281,  -0.0072254335f},
  {140.0f, 6113,  -0.0085616438f},
  {150.0f, 5136,  -0.0102354145f}
};

static const ThermalSensor kDcCapacitor = {
  kDcCapacitorCal,
  ARRAYSIZE(kDcCapacitorCal)
};

// NSII E1-43 (Protean) calibration data.
static const ThermalCalData kProteanStatorCal[] = {
  {-10.0f, 30755, -0.0127551020f},
  {0.0f,   29633, -0.0089126560f},
  {10.0f,  28118, -0.0066006601f},
  {20.0f,  26200, -0.0052137643f},
  {30.0f,  23922, -0.0043898156f},
  {40.0f,  21389, -0.0039478879f},
  {50.0f,  18742, -0.0037778617f},
  {60.0f,  16131, -0.0038299502f},
  {70.0f,  13681, -0.0040816327f},
  {80.0f,  11477, -0.0045372051f},
  {90.0f,  9558,  -0.0052110474f},
  {100.0f, 7929,  -0.0061387354f},
  {110.0f, 6569,  -0.0073529412f},
  {120.0f, 5447,  -0.0089126560f},
  {130.0f, 4528,  -0.0108813928f},
  {140.0f, 3778,  -0.0133333333f},
  {150.0f, 3166,  -0.0163398693f},
  {160.0f, 2667,  -0.0200400802f},
  {170.0f, 2258,  -0.0244498778f},
  {180.0f, 1922,  -0.0297619048f}
};

static const ThermalSensor kProteanStator = {
  kProteanStatorCal,
  ARRAYSIZE(kProteanStatorCal)
};

// Yasa Coolant (KTY84/130) calibration data.
static const ThermalCalData kYasaCoolantCal[] = {
  {180.0f, 15124, 0.0245098039f},
  {170.0f, 14707, 0.0239808153f},
  {160.0f, 14287, 0.0238095238f},
  {150.0f, 13859, 0.0233644860f},
  {140.0f, 13418, 0.0226757370f},
  {130.0f, 12981, 0.0228832952f},
  {120.0f, 12531, 0.0222222222f},
  {110.0f, 12075, 0.0219298246f},
  {100.0f, 11620, 0.0219780220f},
  {90.0f,  11160, 0.0217391304f},
  {80.0f,  10696, 0.0215517241f},
  {70.0f,  10229, 0.0214132762f},
  {60.0f,  9768,  0.0216919740f},
  {50.0f,  9307,  0.0216919740f},
  {40.0f,  8836,  0.0212314225f},
  {30.0f,  8386,  0.0222222222f},
  {20.0f,  7929,  0.0218818381f},
  {10.0f,  7476,  0.0220750552f},
  {0.0f,   7040,  0.0229357798f},
  {-10.0f, 6611,  0.0233100233f}
};

static const ThermalSensor kYasaCoolant = {
  kYasaCoolantCal,
  ARRAYSIZE(kYasaCoolantCal)
};

// Yasa stator (PT1000) calibration data.
static const ThermalCalData kYasaStatorCal[] = {
  {180.0f, 11784, 0.0609756097f},
  {170.0f, 11617, 0.0598802395f},
  {160.0f, 11446, 0.0584795321f},
  {150.0f, 11273, 0.0578034682f},
  {140.0f, 11095, 0.0561797752f},
  {130.0f, 10914, 0.0552486187f},
  {120.0f, 10730, 0.0543478260f},
  {110.0f, 10542, 0.0531914893f},
  {100.0f, 10350, 0.0520833333f},
  {90.0f,  10154, 0.0510204081f},
  {80.0f,  9954,  0.0500000000f},
  {70.0f,  9750,  0.0490196078f},
  {60.0f,  9541,  0.0478468899f},
  {50.0f,  9329,  0.0471698113f},
  {40.0f,  9111,  0.0458715596f},
  {30.0f,  8889,  0.0450450450f},
  {20.0f,  8661,  0.0438596491f},
  {10.0f,  8429,  0.0431034482f},
  {0.0f,   8192,  0.0421940928f},
  {-10.0f, 7949,  0.0411522633f}
};

static const ThermalSensor kYasaStator = {
  kYasaStatorCal,
  ARRAYSIZE(kYasaStatorCal)
};

// HT3000 (P0K5.232.5FC.B.M) calibration data.
static const ThermalCalData kHt3000Cal[] = {
  {-20.0f, 1406, -0.2684748053f},
  {-10.0f, 1370, -0.2769225884f},
  {0.0f,   1335, -0.2854764391f},
  {10.0f,  1301, -0.2941447807f},
  {20.0f,  1268, -0.3029431771f},
  {30.0f,  1236, -0.3118752855f},
  {40.0f,  1205, -0.3209413669f},
  {50.0f,  1175, -0.3301416883f},
  {60.0f,  1145, -0.3394765227f},
  {70.0f,  1117, -0.3489461491f},
  {80.0f,  1089, -0.3585508529f},
  {90.0f,  1062, -0.3682909256f},
  {100.0f, 1035, -0.3781666651f},
  {110.0f, 1009, -0.3881783760f},
  {120.0f, 984,  -0.3983263691f},
  {130.0f, 960,  -0.4086109622f},
  {140.0f, 936,  -0.4190324797f},
  {150.0f, 913,  -0.4295912530f},
  {160.0f, 890,  -0.4402876204f},
  {170.0f, 868,  -0.4511219274f},
  {180.0f, 846,  -0.4620945266f},
  {190.0f, 825,  -0.4732057781f},
  {200.0f, 804,  -0.4844560492f},
  {210.0f, 784,  -0.4958457150f},
  {220.0f, 764,  -0.5073751580f},
  {230.0f, 745,  -0.5190447689f},
  {240.0f, 726,  -0.5308549458f},
  {250.0f, 708,  -0.5428060953f}
};

static const ThermalSensor kHt3000 = {
  kHt3000Cal,
  ARRAYSIZE(kHt3000Cal)
};

static inline uint32_t MotorThermalChannelToWarningMask(
    MotorThermalChannel ch) {
  return (1U << MOTOR_WARNING_TEMP_OFFSET) << ch;
}

static inline void SetTemperature(MotorThermalChannel ch, float temp,
                                  bool valid) {
  assert(0 <= ch && ch < ARRAYSIZE(g_thermal_data.channel_temps));

  // Read hysteresis to mitigate errors due to read glitches.
  if (valid) {
    --g_thermal_data.temp_read_errors;
  } else {
    ++g_thermal_data.temp_read_errors;
  }
  if (g_thermal_data.temp_read_errors
      >= g_motor_thermal_params.temp_read_error_threshold) {
    g_thermal_data.temp_read_errors
        = g_motor_thermal_params.temp_read_error_threshold;
    g_thermal_data.warning_bitmask |= kMotorWarningTempReadErrors;
  } else if (g_thermal_data.temp_read_errors <= 0) {
    g_thermal_data.temp_read_errors = 0;
    g_thermal_data.warning_bitmask &= ~kMotorWarningTempReadErrors;
  }

  if (valid) {
    g_thermal_data.channel_temps[ch] = temp;

    // Over-temp hysteresis to mitigate errors due to erroneous measurements.
    if (temp >= g_motor_thermal_params.thermal_limits[ch]) {
      ++g_thermal_data.over_temp_count[ch];
    } else {
      --g_thermal_data.over_temp_count[ch];
    }
    if (g_thermal_data.over_temp_count[ch]
        >= g_motor_thermal_params.over_temp_threshold) {
      g_thermal_data.over_temp_count[ch] =
          g_motor_thermal_params.over_temp_threshold;
      g_thermal_data.warning_bitmask |= MotorThermalChannelToWarningMask(ch);
    } else if (g_thermal_data.over_temp_count[ch] <= 0) {
      g_thermal_data.over_temp_count[ch] = 0;
      g_thermal_data.warning_bitmask &= ~MotorThermalChannelToWarningMask(ch);
    }
  }
}

void MotorThermalInit(MotorType motor_type, MotorHardware controller_type) {
  // Grab a pointer to make the following code easier to read.
  MotorThermalParams *params = &g_motor_thermal_params;

  // Motor controller temperature limits are somewhat derated from the maximum
  // values indicated by Andy's Motor Controller Thermal Performance document
  // (https://goo.gl/OTbt2z).  Note that the module die temperatures are a
  // function of phase current and may need to be modified as limits change.
  params->thermal_limits[kMotorThermalChannelBoard] = 82.0f;
  params->thermal_limits[kMotorThermalChannelControllerAir] = 70.0f;
  params->thermal_limits[kMotorThermalChannelCapacitor] = 85.0f;
  params->thermal_limits[kMotorThermalChannelUnused] = INFINITY;

  if (controller_type == kMotorHardwareOzoneA1) {
    // TODO: HeatPlate1 in Ozone is a second internal air temp.
    params->thermal_limits[kMotorThermalChannelHeatPlate1] = 70.0f;
    params->thermal_limits[kMotorThermalChannelHeatPlate2] = INFINITY;
    params->thermal_limits[kMotorThermalChannelHt3000A] = 100.0f;
    params->thermal_limits[kMotorThermalChannelHt3000B] = 100.0f;
    params->thermal_limits[kMotorThermalChannelHt3000C] = 100.0f;
  } else {
    params->thermal_limits[kMotorThermalChannelHeatPlate1] = 80.0f;
    params->thermal_limits[kMotorThermalChannelHeatPlate2] = 80.0f;
  }

  switch (motor_type) {
    case kMotorTypeProtean:
      params->thermal_limits[kMotorThermalChannelRotor] = INFINITY;
      params->thermal_limits[kMotorThermalChannelStatorCore] = 140.0f;
      params->thermal_limits[kMotorThermalChannelStatorCoil] = 140.0f;
      params->thermal_limits[kMotorThermalChannelNacelleAir] = 140.0f;
      break;
    case kMotorTypeYasa:
      // Yasa motor thermal limits are based off of the thermal derating
      // schedule indicated by Yasa (https://goo.gl/0bUq6H).
      params->thermal_limits[kMotorThermalChannelRotor] = 70.0f;
      params->thermal_limits[kMotorThermalChannelStatorCore] = 100.0f;
      params->thermal_limits[kMotorThermalChannelStatorCoil] = 140.0f;
      params->thermal_limits[kMotorThermalChannelNacelleAir] = 100.0f;
      break;
    default:
      assert(false);
      break;
  }
}

void MotorThermalUpdateMcp9800(int32_t monitor, float temp) {
  switch ((MotorMcp9800Monitor)monitor) {
    case kMotorMcp9800MonitorBoard:
      SetTemperature(kMotorThermalChannelBoard, temp, true);
      break;
    case kMotorMcp9800MonitorControllerAir:
      SetTemperature(kMotorThermalChannelControllerAir, temp, true);
      break;
    case kMotorMcp9800MonitorForceSigned:
    case kNumMotorMcp9800Monitors:
    default:
      assert(false);
      break;
  }
}

void MotorThermalUpdateSi7021(int32_t monitor, const Si7021OutputData *data) {
  if (monitor == kMotorSi7021MonitorBoard) {
    SetTemperature(kMotorThermalChannelBoard, data->temperature, true);
  }
}

void MotorThermalUpdateMcp342x(int32_t monitor, int32_t t_raw, bool valid) {
  const MotorThermalCalibMap kCalibMap[kNumMotorMcp342xMonitors] = {
    [kMotorMcp342xMonitorCapacitor]
    = {kMotorThermalChannelCapacitor, &kDcCapacitor},
    [kMotorMcp342xMonitorHeatPlate1]
    = {kMotorThermalChannelHeatPlate1, &kHeatPlate},
    [kMotorMcp342xMonitorHeatPlate2]
    = {kMotorThermalChannelHeatPlate2, &kHeatPlate},
    [kMotorMcp342xMonitorProteanStator1]
    = {kMotorThermalChannelStatorCore, &kProteanStator},
    [kMotorMcp342xMonitorProteanStator2]
    = {kMotorThermalChannelStatorCoil, &kProteanStator},
    [kMotorMcp342xMonitorProteanStator3]
    = {kMotorThermalChannelNacelleAir, &kProteanStator},
    [kMotorMcp342xMonitorYasaPylonAmbient]
    = {kMotorThermalChannelNacelleAir, &kProteanStator},
    [kMotorMcp342xMonitorYasaRotor]
    = {kMotorThermalChannelRotor, &kYasaRotor},
    [kMotorMcp342xMonitorYasaStatorCoil]
    = {kMotorThermalChannelStatorCoil, &kYasaStator},
    [kMotorMcp342xMonitorYasaStatorCore]
    = {kMotorThermalChannelStatorCore, &kYasaStator},
  };
  const MotorThermalChannel ch = kCalibMap[monitor].channel;
  const ThermalSensor *calib = kCalibMap[monitor].calib;

  assert(calib != NULL);
  SetTemperature(ch, ThermalCodeToTemp(t_raw, calib), valid);
}

uint32_t MotorThermalTempWarnings(void) {
  return g_thermal_data.warning_bitmask;
}

const MotorThermalData *MotorThermalGetData(void) {
  return &g_thermal_data;
}

void MotorThermalHt3000Init(float sample_freq) {
  // Start N2HET2 clock.
  PeripheralEnable(kPeripheralN2Het2);

  // Copy blank instructions from thermal_nhet.c to N2HET2 RAM.
  vmemcpy(&HET2RAM, kHetInitThermal, sizeof(kHetInitThermal));

  // Set and save sample frequency.
  float f_vclk2 = (float)PeripheralGetClockFreq(kPeripheralN2Het2);

  assert(sample_freq >= 0.1f);
  HET2RAM.COUNT.MAX_COUNT = nearbyint(f_vclk2 / (N2HET2_LR * sample_freq)) - 1;

  g_sample_freq_by_2
      = 0.5f * f_vclk2 / (float)(N2HET2_LR * (HET2RAM.COUNT.MAX_COUNT + 1));

  // Set the loop resolution prescaler.
  N2HET(2).HETPFR.raw = 0;
  N2HET(2).HETPFR.LRPFC = N2HET2_LRPFC;

  // Ignore suspend.
  N2HET(2).HETGCR.raw = 0;
  N2HET(2).HETGCR.IS = 1;

  // Set N2HET2 to master.
  N2HET(2).HETGCR.CMS = 1;

  // Start N2HET2.
  N2HET(2).HETGCR.TO = 1;
}

void MotorThermalUpdateHt3000(void) {
  int32_t freq1 = g_sample_freq_by_2 * HET2RAM.LATCH1.DATA;
  int32_t freq2 = g_sample_freq_by_2 * HET2RAM.LATCH2.DATA;
  int32_t freq3 = g_sample_freq_by_2 * HET2RAM.LATCH3.DATA;

  SetTemperature(kMotorThermalChannelHt3000A,
                 ThermalCodeToTemp(freq1, &kHt3000), true);
  SetTemperature(kMotorThermalChannelHt3000B,
                 ThermalCodeToTemp(freq2, &kHt3000), true);
  SetTemperature(kMotorThermalChannelHt3000C,
                 ThermalCodeToTemp(freq3, &kHt3000), true);
}
