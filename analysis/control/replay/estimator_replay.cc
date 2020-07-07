// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "analysis/control/replay/estimator_replay.h"

#include <assert.h>
#include <stdint.h>

#include <string>
#include <vector>

#include "avionics/network/aio_node.h"
#include "control/avionics/avionics_conversion.h"
#include "control/avionics/avionics_faults.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "lib/hdf5_to_pcap/h5log_reader.h"
#include "system/labels_util.h"

static bool CapTimeLe(const CaptureHeader &a, const CaptureHeader &b) {
  int64_t a_time = a.tv_sec * 1000000LL + a.tv_usec;
  int64_t b_time = b.tv_sec * 1000000LL + b.tv_usec;

  return a_time <= b_time;
}

static void FixNovAtelGpsData(const h5log_reader::File &file,
                              WingGpsReceiverLabel label,
                              const std::vector<CaptureHeader> &telm_cap,
                              int64_t num_telm_messages,
                              ControlTelemetry telm_messages[]) {
  assert(0 <= label && label < kNumWingGpsReceivers);
  const FaultDetectionParams *fault_params =
      &GetControlParams()->fault_detection;

  AvionicsFaultsState faults_state;
  AvionicsFaultsInit(&faults_state);

  h5log_reader::Dataset dataset;
  std::string path = "/messages/";
  path += AioNodeToString(WingGpsReceiverLabelToAioNode(label));
  path += "/kMessageTypeNovAtelSolution";

  if (dataset.ReadData(file, path)) {
    const std::vector<NovAtelSolutionMessage> gps_messages =
        dataset.Export<NovAtelSolutionMessage>();
    const std::vector<CaptureHeader> gps_cap = dataset.Export<CaptureHeader>();

    size_t gps_i_1 = 0, gps_i = 0;
    for (int64_t telm_i = 0; telm_i < num_telm_messages; ++telm_i) {
      bool cvt_updated = false;
      for ( ; gps_i < gps_messages.size()
                && CapTimeLe(gps_cap[gps_i], telm_cap[telm_i]); ++gps_i) {
        cvt_updated = true;
        gps_i_1 = gps_i;
      }
      if (cvt_updated) {
        GpsData *out = &telm_messages[telm_i].control_input.wing_gps[label];
        ConvertNovAtelBestXyzToGps(&gps_messages[gps_i_1].best_xyz, out);

        // Clear previous fault state since it may correspond to an unrelated
        // subsystem.
        FaultMask *gps_faults =
            GetWingGpsSubsysFaults(telm_messages[telm_i].faults, label);
        memset(gps_faults, 0,
               sizeof(*gps_faults) * kNumFaultDetectionGpsSignals);

        AvionicsFaultsCheckWingGpsNovAtel(&gps_messages[gps_i_1], cvt_updated,
                                          &fault_params->wing_gps,
                                          &faults_state.wing_gps[label],
                                          gps_faults, out);
      }
    }
  }
}

static void FixMassBalanceTubeGpsData(
    const h5log_reader::File &file,
    const std::vector<CaptureHeader> &capture_header, int64_t num_messages,
    ControlTelemetry messages[]) {
  FixNovAtelGpsData(file, kWingGpsReceiverCrosswind, capture_header,
                    num_messages, messages);
  FixNovAtelGpsData(file, kWingGpsReceiverHover, capture_header, num_messages,
                    messages);
}

static void FixWingtipGpsData(
    const h5log_reader::File &file,
    const std::vector<CaptureHeader> &capture_header, int64_t num_messages,
    ControlTelemetry messages[]) {
  FixNovAtelGpsData(file, kWingGpsReceiverPort, capture_header, num_messages,
                    messages);
  FixNovAtelGpsData(file, kWingGpsReceiverStar, capture_header, num_messages,
                    messages);
}

// See b/64680344. Ignore GPS data occurring every 300 seconds on interval
// [tow_offset_ms, tow_offset_ms + duration_ms].
static void FixBadGpsUpdatesEvery300Seconds(int32_t tow_offset_ms,
                                            int32_t duration_ms,
                                            int64_t num_messages,
                                            ControlTelemetry messages[]) {
  for (int64_t i = 0; i < num_messages; ++i) {
    for (int32_t j = 0; j < kNumWingGpsReceivers; ++j) {
      GpsData *gps = &messages[i].control_input.wing_gps[j];
      int32_t period_ms = 300 * 1000;
      int32_t delta_ms = (gps->time_of_week_ms - tow_offset_ms) % period_ms;
      if (-duration_ms < delta_ms && delta_ms < duration_ms) {
        gps->new_data = false;
      }
    }
  }
}

static void FixFlightData(const char *flight, const h5log_reader::File &file,
                          const std::vector<CaptureHeader> &capture_header,
                          int64_t num_messages, ControlTelemetry messages[]) {
  if (!strcmp(flight, "rpx01") || !strcmp(flight, "rpx02")) {
    FixMassBalanceTubeGpsData(file, capture_header, num_messages, messages);
  } else if (!strcmp(flight, "rpx03")) {
    FixMassBalanceTubeGpsData(file, capture_header, num_messages, messages);
    FixBadGpsUpdatesEvery300Seconds(342899750, 1000, num_messages, messages);
  } else if (!strcmp(flight, "rpx04")) {
    FixMassBalanceTubeGpsData(file, capture_header, num_messages, messages);
    FixBadGpsUpdatesEvery300Seconds(407699700, 1000, num_messages, messages);
  } else if (!strcmp(flight, "rpx05")) {
    FixMassBalanceTubeGpsData(file, capture_header, num_messages, messages);
    FixWingtipGpsData(file, capture_header, num_messages, messages);
    FixBadGpsUpdatesEvery300Seconds(254999700, 1000, num_messages, messages);
  } else if (!strcmp(flight, "rpx06")) {
    FixMassBalanceTubeGpsData(file, capture_header, num_messages, messages);
    FixWingtipGpsData(file, capture_header, num_messages, messages);
    FixBadGpsUpdatesEvery300Seconds(177899700, 1000, num_messages, messages);
  } else {
    assert(false);
  }
}

void EstimatorReplayInit(const EstimatorParams *params, FlightMode *flight_mode,
                         EstimatorState *state) {
  *flight_mode = kFlightModePilotHover;
  EstimatorInit(GetSystemParams(), params, state);
}

void EstimatorReplayIterate(const EstimatorParams *params,
                            const ControlTelemetry *message,
                            FlightMode *flight_mode_z1,
                            EstimatorState *state_z1, StateEstimate *estimate) {
  *GetControlTelemetry() = *message;

  FlightStatus flight_status;
  flight_status.flight_mode =
      static_cast<FlightMode>(GetControlTelemetry()->flight_mode);
  flight_status.flight_mode_time = GetControlTelemetry()->flight_mode_time;
  flight_status.flight_mode_first_entry =
      (*flight_mode_z1 != flight_status.flight_mode);
  flight_status.last_flight_mode = *flight_mode_z1;

  EstimatorStep(&flight_status, &GetControlTelemetry()->control_input,
                GetSystemParams(), GetControlTelemetry()->faults,
                params, state_z1, estimate);

  *flight_mode_z1 = flight_status.flight_mode;
}

void EstimatorReplayIterateArray(const EstimatorParams *params,
                                 int64_t first_index, int64_t last_index,
                                 const ControlTelemetry messages[],
                                 FlightMode *flight_mode_z1,
                                 EstimatorState *state_z1,
                                 EstimatorState states[],
                                 StateEstimate estimates[]) {
  for (int64_t i = first_index; i <= last_index; ++i) {
    EstimatorReplayIterate(params, &messages[i], flight_mode_z1, state_z1,
                           &estimates[i]);
    states[i] = *state_z1;
  }
}

void SetControlTelemetryFaults(int64_t first_index, int64_t last_index,
                               int32_t num_subsys,
                               const SubsystemLabel subsys[],
                               int32_t fault_mask,
                               ControlTelemetry messages[]) {
  for (int64_t i = first_index; i <= last_index; ++i) {
    for (int32_t j = 0; j < num_subsys; ++j) {
      const SubsystemLabel s = subsys[j];
      assert(0 <= s && s < kNumSubsystems);
      messages[i].faults[s].code |= fault_mask;
    }
  }
}

void ClearControlTelemetryFaults(int64_t first_index, int64_t last_index,
                                 const ControlTelemetry original_messages[],
                                 int32_t num_subsys,
                                 const SubsystemLabel subsys[],
                                 int32_t fault_mask,
                                 ControlTelemetry messages[]) {
  for (int64_t i = first_index; i <= last_index; ++i) {
    for (int32_t j = 0; j < num_subsys; ++j) {
      const SubsystemLabel s = subsys[j];
      assert(0 <= s && s < kNumSubsystems);
      messages[i].faults[s].code &= ~fault_mask;
      messages[i].faults[s].code |= original_messages[i].faults[s].code;
    }
  }
}

int64_t H5GetNumMessages(const char *filename, const char *path) {
  h5log_reader::File file;
  h5log_reader::Dataset dataset;
  if (file.Open(filename) && dataset.ReadTableOfContents(file, path)) {
    return dataset.GetNumObjects();
  }
  return 0;
}

int64_t H5GetControlTelemetryMessages(const char *filename, const char *path,
                                      const char *flight, int64_t num_messages,
                                      ControlTelemetry messages[]) {
  h5log_reader::File file;
  h5log_reader::Dataset dataset;
  if (file.Open(filename) && dataset.ReadData(file, path)) {
    num_messages = dataset.Export<ControlTelemetry>(num_messages, messages);
    if (flight != nullptr) {
      std::vector<CaptureHeader> cap_header = dataset.Export<CaptureHeader>();
      FixFlightData(flight, file, cap_header, num_messages, messages);
    }
    return num_messages;
  }
  return 0;
}
