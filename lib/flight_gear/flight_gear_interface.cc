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

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <gflags/gflags.h>

#include <vector>

#include "avionics/linux/aio.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"
#include "common/c_math/coord_trans.h"
#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/control_telemetry.h"
#include "control/cvt_control_telemetry.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "lib/udpio/udpio.h"
#include "sim/cvt_sim_telemetry.h"
#include "sim/sim_params.h"
#include "sim/sim_telemetry.h"

DEFINE_bool(use_sim, true,
            "Use the simulator wing and tether telemetry.");
DEFINE_string(fg_addr, GetSystemParams()->comms.udpio.flight_gear_remote_addr,
              "FlightGear computer IP address.");
DEFINE_int32(fg_port, GetSystemParams()->comms.udpio.flight_gear_remote_port,
             "FlightGear computer port.");

namespace {

bool ValidateFlightGearPort(const char */*flag_name*/, int32_t value) {
  return 0 <= value && value < 65535;
}
bool dummy __attribute__((unused))
    = google::RegisterFlagValidator(&FLAGS_fg_port, &ValidateFlightGearPort);

}  // namespace

namespace {

#define NUM_FLIGHT_GEAR_TETHER_NODES 10

// Position [m] of the viewer, i.e. pilot, in the ground-station
// frame.
const Vec3 kPilotPosG = {20.0, 40.0, 20.0};

const double kMetersToFeet = 3.2808399;
const double kRadiansToDegrees = 180.0 / M_PI;

struct FlightGearInput {
  double latitude;
  double longitude;
  double altitude;
  double roll;
  double pitch;
  double yaw;
  double a1;
  double a2;
  double a4;
  double a5;
  double a7;
  double a8;
  double ele;
  double rud;
  double airspeed;
  double tower_latitude;
  double tower_longitude;
  double tower_altitude;
  double tower_heading;
  double tower_pitch;
  double tether_latitude[NUM_FLIGHT_GEAR_TETHER_NODES];
  double tether_longitude[NUM_FLIGHT_GEAR_TETHER_NODES];
  double tether_altitude[NUM_FLIGHT_GEAR_TETHER_NODES];
};

void BuildFlightGearInput(const Vec3 &ground_station_pos_ecef,
                          double ground_frame_heading,
                          const Vec3 &wing_pos_g,
                          const Vec3 &wing_vel_b,
                          const Mat3 &dcm_g2b,
                          const std::vector<double> &flaps,
                          const std::vector<Vec3> &tether_node_pos_g,
                          FlightGearInput *flight_gear_input) {
  Vec3 gs_pos_llh;
  EcefToLlh(&ground_station_pos_ecef, &gs_pos_llh);

  Mat3 dcm_ned2g;
  AngleToDcm(ground_frame_heading, 0.0, 0.0, kRotationOrderZyx, &dcm_ned2g);

  Vec3 wing_pos_ned;
  Mat3TransVec3Mult(&dcm_ned2g, &wing_pos_g, &wing_pos_ned);
  wing_pos_ned.z -= GetSystemParams()->ground_frame.ground_z;

  Vec3 wing_pos_llh;
  NedToLlh(&wing_pos_ned, &gs_pos_llh, &wing_pos_llh);
  flight_gear_input->latitude = wing_pos_llh.x;
  flight_gear_input->longitude = wing_pos_llh.y;
  flight_gear_input->altitude = kMetersToFeet * wing_pos_llh.z;

  Mat3 dcm_ned2b;
  Mat3Mat3Mult(&dcm_g2b, &dcm_ned2g, &dcm_ned2b);

  double roll, pitch, yaw;
  DcmToAngle(&dcm_ned2b, kRotationOrderZyx, &yaw, &pitch, &roll);
  flight_gear_input->roll = kRadiansToDegrees * roll;
  flight_gear_input->pitch = kRadiansToDegrees * pitch;
  flight_gear_input->yaw = kRadiansToDegrees * yaw;

  flight_gear_input->a1 = flaps[kFlapA1];
  flight_gear_input->a2 = flaps[kFlapA2];
  flight_gear_input->a4 = flaps[kFlapA4];
  flight_gear_input->a5 = flaps[kFlapA5];
  flight_gear_input->a7 = flaps[kFlapA7];
  flight_gear_input->a8 = flaps[kFlapA8];
  flight_gear_input->ele = flaps[kFlapEle];

  // Flight Gear measures rudder deflections in the reverse direction.
  flight_gear_input->rud = -flaps[kFlapRud];

  flight_gear_input->airspeed = kMetersToFeet * wing_vel_b.x;

  // Calculate the latitude, longitude, and altitude coordinates of
  // the tower, i.e. the view point for the visualizer.
  Vec3 pilot_pos_ned;
  Mat3TransVec3Mult(&dcm_ned2g, &kPilotPosG, &pilot_pos_ned);
  pilot_pos_ned.z -= GetSystemParams()->ground_frame.ground_z;

  Vec3 pilot_pos_llh;
  NedToLlh(&pilot_pos_ned, &gs_pos_llh, &pilot_pos_llh);
  flight_gear_input->tower_latitude = pilot_pos_llh.x;
  flight_gear_input->tower_longitude = pilot_pos_llh.y;
  flight_gear_input->tower_altitude = kMetersToFeet * pilot_pos_llh.z;

  Vec3 wing_offset_ned;
  Vec3Sub(&wing_pos_ned, &pilot_pos_ned, &wing_offset_ned);
  flight_gear_input->tower_heading = atan2(wing_offset_ned.y,
                                           wing_offset_ned.x);
  flight_gear_input->tower_pitch = atan2(-wing_offset_ned.z,
                                         Vec3XyNorm(&wing_offset_ned));

  // Calculate latitude, longitude, and altitude coordinates of the
  // tether nodes.
  for (int32_t i = 0; i < NUM_FLIGHT_GEAR_TETHER_NODES; ++i) {
    if (i < static_cast<int32_t>(tether_node_pos_g.size())) {
      Vec3 node_pos_ned;
      Mat3TransVec3Mult(&dcm_ned2g, &tether_node_pos_g[i], &node_pos_ned);
      node_pos_ned.z -= GetSystemParams()->ground_frame.ground_z;

      Vec3 node_pos_llh;
      NedToLlh(&node_pos_ned, &gs_pos_llh, &node_pos_llh);
      flight_gear_input->tether_latitude[i] = node_pos_llh.x;
      flight_gear_input->tether_longitude[i] = node_pos_llh.y;
      flight_gear_input->tether_altitude[i] = kMetersToFeet * node_pos_llh.z;
    } else {
      flight_gear_input->tether_latitude[i] = 0.0;
      flight_gear_input->tether_longitude[i] = 0.0;
      flight_gear_input->tether_altitude[i] = 0.0;
    }
  }
}

void SimTelemetryToFlightGearInput(const SimTelemetry &sim_telem,
                                   FlightGearInput *flight_gear_input) {
  Mat3 dcm_g2b;
  QuatToDcm(&sim_telem.wing.q, &dcm_g2b);

  std::vector<double> flaps(sim_telem.wing.flaps,
                            sim_telem.wing.flaps + kNumFlaps);
  std::vector<Vec3> tether_node_pos_g(sim_telem.tether.Xg_nodes,
                                      sim_telem.tether.Xg_nodes
                                      + sim_telem.tether.end_ind + 1);

  BuildFlightGearInput(GetSimParams()->ground_frame_sim.pos_ecef,
                       GetSystemParams()->ground_frame.heading,
                       sim_telem.wing.Xg, sim_telem.wing.Vb, dcm_g2b, flaps,
                       tether_node_pos_g, flight_gear_input);
}

void ControlTelemetryToFlightGearInput(const ControlTelemetry &control_telem,
                                       FlightGearInput *flight_gear_input) {
  std::vector<double> flaps(control_telem.control_output.flaps,
                            control_telem.control_output.flaps + kNumFlaps);
  std::vector<Vec3> tether_node_pos_g(NUM_FLIGHT_GEAR_TETHER_NODES);
  for (Vec3 &node_pos_g : tether_node_pos_g) {
    Vec3Scale(&control_telem.state_est.Xg,
              static_cast<double>(&node_pos_g - &tether_node_pos_g[0])
              / static_cast<double>(tether_node_pos_g.size()),
              &node_pos_g);
  }

  BuildFlightGearInput(control_telem.estimator.ground_station.pose.pos_ecef,
                       GetSystemParams()->ground_frame.heading,
                       control_telem.state_est.Xg, control_telem.state_est.Vb,
                       control_telem.state_est.dcm_g2b, flaps,
                       tether_node_pos_g, flight_gear_input);
}

int32_t WriteFlightGearInput(const FlightGearInput &fg, uint8_t *data) {
  // TODO: Switch this to a binary format.
  return snprintf(
      reinterpret_cast<char *>(data), 4096,  // NOLINT(runtime/printf)
      "%0.14f,%0.14f,%0.14f,%0.14f,%0.14f,%0.14f,"
      "%0.14f,%0.14f,%0.14f,%0.14f,%0.14f,%0.14f,"
      "%0.14f,%0.14f,"
      "%0.14f,%0.14f,%0.14f,%0.14f,"
      "%0.14f,%0.14f,"
      "%0.14f,%0.14f,%0.14f,"
      "%0.14f,%0.14f,%0.14f,"
      "%0.14f,%0.14f,%0.14f,"
      "%0.14f,%0.14f,%0.14f,"
      "%0.14f,%0.14f,%0.14f,"
      "%0.14f,%0.14f,%0.14f,"
      "%0.14f,%0.14f,%0.14f,"
      "%0.14f,%0.14f,%0.14f,"
      "%0.14f,%0.14f,%0.14f,"
      "%0.14f,%0.14f,%0.14f\n",
      fg.latitude, fg.longitude, fg.altitude, fg.roll, fg.pitch, fg.yaw,
      fg.a1, fg.a2, fg.a4, fg.a5, fg.a7, fg.a8,
      fg.ele, fg.rud,
      fg.airspeed, fg.tower_latitude, fg.tower_longitude, fg.tower_altitude,
      fg.tower_heading, fg.tower_pitch,
      fg.tether_latitude[0], fg.tether_longitude[0], fg.tether_altitude[0],
      fg.tether_latitude[1], fg.tether_longitude[1], fg.tether_altitude[1],
      fg.tether_latitude[2], fg.tether_longitude[2], fg.tether_altitude[2],
      fg.tether_latitude[3], fg.tether_longitude[3], fg.tether_altitude[3],
      fg.tether_latitude[4], fg.tether_longitude[4], fg.tether_altitude[4],
      fg.tether_latitude[5], fg.tether_longitude[5], fg.tether_altitude[5],
      fg.tether_latitude[6], fg.tether_longitude[6], fg.tether_altitude[6],
      fg.tether_latitude[7], fg.tether_longitude[7], fg.tether_altitude[7],
      fg.tether_latitude[8], fg.tether_longitude[8], fg.tether_altitude[8],
      fg.tether_latitude[9], fg.tether_longitude[9], fg.tether_altitude[9]);
}

}  // namespace

int main(int argc, char **argv) {
  google::SetUsageMessage("");
  google::ParseCommandLineFlags(&argc, &argv, true);

  udp_config flight_gear_udp;
  udp_setup_sender(&flight_gear_udp, FLAGS_fg_addr.c_str(),
                   static_cast<uint16_t>(FLAGS_fg_port), 0);

  MessageType subscribe_types[] = {
    FLAGS_use_sim ? kMessageTypeSimTelemetry : kMessageTypeControlTelemetry
  };
  AioSetup(kAioNodeVisualizer, GetSystemParams()->comms.aio_port,
           subscribe_types, ARRAYSIZE(subscribe_types));

  while (true) {
    bool is_updated;
    FlightGearInput flight_gear_input;
    if (FLAGS_use_sim) {
      CVT_GET_NEXT_UPDATE(kAioNodeSimulator, CvtGetSimTelemetry,
                          0.0, &sim_telem, &is_updated);
      if (is_updated) {
        SimTelemetryToFlightGearInput(sim_telem, &flight_gear_input);
      }
    } else {
      ControlTelemetry control_telem;
      CVT_GET_NEXT_UPDATE(kAioNodeControllerA, CvtGetControlTelemetry,
                          0.0, &control_telem, &is_updated);
      if (is_updated) {
        ControlTelemetryToFlightGearInput(control_telem, &flight_gear_input);
      }
    }

    if (is_updated) {
      uint8_t data[UDP_BUFLEN];
      int32_t len = WriteFlightGearInput(flight_gear_input, data);
      udp_send(&flight_gear_udp, data, len);
    }
  }

  AioClose();
  udp_close(&flight_gear_udp);

  return 0;
}
