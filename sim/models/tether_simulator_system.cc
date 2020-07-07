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

#include "sim/models/tether_simulator_system.h"

#include <stdint.h>

#include <limits>

#include "control/control_telemetry.h"
#include "lib/hdf5_to_pcap/h5log_reader.h"
#include "sim/sim_telemetry.h"
#include "sim/state.h"

namespace {

std::vector<ControlDebugMessage> GetControlDebugMessages(const char *filename) {
  h5log_reader::File file;
  h5log_reader::Dataset dataset;
  CHECK(file.Open(filename));
  CHECK(dataset.ReadData(
      file, "/messages/kAioNodeControllerA/kMessageTypeControlDebug"));
  return dataset.Export<ControlDebugMessage>();
}

}  // namespace

FlightLogInterpolator::FlightLogInterpolator(
    const std::vector<std::string> &filenames)
    : interp_(nullptr),
      t_(),
      Xg_(),
      Vb_(),
      omega_(),
      eulers_(),
      perch_azi_(),
      gsg_azi_(),
      gsg_ele_(),
      tether_release_time_(std::numeric_limits<double>::infinity()) {
  bool tether_release_time_set;

  for (const std::string &filename : filenames) {
    std::vector<ControlDebugMessage> messages =
        GetControlDebugMessages(filename.c_str());

    size_t prev_size = t_.size();
    // Allocate more space for the new data.
    size_t new_size = prev_size + messages.size();
    t_.resize(new_size);
    Xg_.resize(new_size);
    Vb_.resize(new_size);
    omega_.resize(new_size);
    eulers_.resize(new_size);
    perch_azi_.resize(new_size);
    gsg_azi_.resize(new_size);
    gsg_ele_.resize(new_size);

    for (uint32_t i = 0U; i < messages.size(); ++i) {
      const ControlDebugMessage &control_debug = messages[i];

      uint32_t j = static_cast<uint32_t>(prev_size) + i;
      t_[j] = control_debug.time;
      CHECK(j == 0U || t_[j] > t_[j - 1]);
      Xg_[j] = control_debug.state_est.Xg;
      Vb_[j] = control_debug.state_est.Vb_f;
      omega_[j] = control_debug.state_est.pqr;
      DcmToAngle(&control_debug.state_est.dcm_g2b, kRotationOrderZyx,
                 &eulers_[j].z, &eulers_[j].y, &eulers_[j].x);

      perch_azi_[j] =
          control_debug.control_input.perch.perch_azi[kPlatformSensorsA];

      const GsgData &gsg = control_debug.control_input.gsg[kDrumSensorsA];
      gsg_azi_[j] = gsg.azi;
      gsg_ele_[j] = gsg.ele;

      if (!tether_release_time_set &&
          control_debug.control_output.tether_release) {
        tether_release_time_set = true;
        tether_release_time_ = t_[j];
      }
    }
  }

  CHECK_GT(t_.size(), 0U);
  interp_ = new sim::TimeseriesInterpolator(t_);
}

FlightLogInterpolator::~FlightLogInterpolator() { delete interp_; }

Vec3 FlightLogInterpolator::Xg(double t) { return interp_->InterpVec3(t, Xg_); }
Vec3 FlightLogInterpolator::Vb(double t) { return interp_->InterpVec3(t, Vb_); }
Vec3 FlightLogInterpolator::omega(double t) {
  return interp_->InterpVec3(t, omega_);
}

Mat3 FlightLogInterpolator::dcm_g2b(double t) {
  Vec3 eulers = interp_->WrapInterpVec3(t, eulers_, -PI, PI);
  Mat3 out;
  AngleToDcm(eulers.z, eulers.y, eulers.x, kRotationOrderZyx, &out);
  return out;
}

double FlightLogInterpolator::perch_azi(double t) {
  return interp_->WrapInterp(t, perch_azi_, -PI, PI);
}

double FlightLogInterpolator::gsg_azi(double t) {
  return interp_->WrapInterp(t, gsg_azi_, -PI, PI);
}

double FlightLogInterpolator::gsg_ele(double t) {
  return interp_->Interp(t, gsg_ele_);
}

TetherSimulatorSystem::TetherSimulatorSystem(
    FlightLogInterpolator *log_interpolator, const SystemParams &system_params,
    const SimParams &sim_params, FaultSchedule *faults)
    : BaseSystemModel("TetherSimulatorSystem", system_params, sim_params,
                      faults),
      environment_(sim_params_.iec_sim, sim_params_.phys_sim,
                   system_params_.phys, system_params_.wind_sensor,
                   system_params_.ground_frame),
      ground_frame_(environment_.ned_frame(), system_params_.ground_frame,
                    sim_params_.ground_frame_sim),
      tether_(environment_, system_params_.tether, sim_params_.tether_sim),
      log_interpolator_(log_interpolator),
      Xg_wing_(new_derived_value(), "Xg_wing"),
      Vb_wing_(new_derived_value(), "Vb_wing"),
      omega_wing_b_(new_derived_value(), "omega_wing_b"),
      dcm_g2b_(new_derived_value(), "dcm_g2b"),
      perch_azi_(new_derived_value(), "perch_azi"),
      gsg_azi_(new_derived_value(), "gsg_azi"),
      gsg_ele_(new_derived_value(), "gsg_ele"),
      tether_release_commanded_(new_derived_value(),
                                "tether_release_commanded") {
  Init();
  SetupDone();
}

void TetherSimulatorSystem::Init() {
  sub_models_.push_back(&environment_);
  sub_models_.push_back(&tether_);

  // Initialize the tether at full length.
  double full_tether_length = system_params_.tether.length;
  tether_.Init(
      std::bind(&Wing::CalcBridlePoint, log_interpolator_->Xg(0.0),
                log_interpolator_->Vb(0.0), log_interpolator_->omega(0.0),
                log_interpolator_->dcm_g2b(0.0), system_params_.wing,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4),
      kVec3Zero, kVec3Zero, full_tether_length);

  // Pulling in data from log_intepolator_ here allows Publish() to remain
  // time-independent.
  connections_.Add(0, [this](double t) {
    Xg_wing_.set_val(log_interpolator_->Xg(t));
    Vb_wing_.set_val(log_interpolator_->Vb(t));
    omega_wing_b_.set_val(log_interpolator_->omega(t));
    dcm_g2b_.set_val(log_interpolator_->dcm_g2b(t));

    perch_azi_.set_val(log_interpolator_->perch_azi(t));
    gsg_azi_.set_val(log_interpolator_->gsg_azi(t));
    gsg_ele_.set_val(log_interpolator_->gsg_ele(t));

    tether_release_commanded_.set_val(
        log_interpolator_->TetherReleaseCommanded(t));
  });

  // Set boundary conditions at the start (ground-side termination) of the
  // tether.
  connections_.Add(2, [this, full_tether_length](double /*t*/) {
    this->tether_.set_free_length(full_tether_length);

    Vec3 tether_start_pos_g, tether_start_vel_g;
    CalcTetherStartPosVel(perch_azi_.val(), gsg_azi_.val(), gsg_ele_.val(),
                          &tether_start_pos_g, &tether_start_vel_g);
    this->tether_.set_Xg_start(tether_start_pos_g);
    this->tether_.set_Vg_start(tether_start_vel_g);
  });

  // Set boundary conditions on the end of the tether. If the kite is still
  // attached, these are determined by the kite states.
  connections_.Add(3, [this](double /*t*/) {
    if (tether_release_commanded_.val()) {
      this->tether_.set_Xg_end(this->tether_.Xg_last_node());
      this->tether_.set_Vg_end(this->tether_.Vg_last_node());
    } else {
      Vec3 pos, vel;
      Wing::CalcBridlePoint(Xg_wing_.val(), Vb_wing_.val(), omega_wing_b_.val(),
                            dcm_g2b_.val(), system_params_.wing,
                            this->tether_.Xg_last_node(),
                            this->tether_.Vg_last_node(), &pos, &vel);
      this->tether_.set_Xg_end(pos);
      this->tether_.set_Vg_end(vel);
    }
  });

  this->tether_.set_ground_surface(&this->ground_frame_.surface());

  AddAllInternalConnections(&connections_);
}

void TetherSimulatorSystem::Publish() const {
  BaseSystemModel::Publish();

  sim_telem.wing.Xg = Xg_wing_.val();
  sim_telem.wing.Vb = Vb_wing_.val();
  sim_telem.wing.omega = omega_wing_b_.val();

  Mat3 dcm_g2b = dcm_g2b_.val();
  DcmToQuat(&dcm_g2b, &sim_telem.wing.q);

  Vec3 eulers;
  DcmToAngle(&dcm_g2b, kRotationOrderZyx, &eulers.z, &eulers.y, &eulers.x);
  sim_telem.wing.eulers = eulers;

  sim_telem.perch.theta_p = perch_azi_.val();
  sim_telem.gsg.azi = gsg_azi_.val();
  sim_telem.gsg.ele = gsg_ele_.val();

  sim_telem.tether.released = tether_release_commanded_.val();
}

void TetherSimulatorSystem::CalcTetherStartPosVel(double perch_azi,
                                                  double gsg_azi,
                                                  double gsg_ele,
                                                  Vec3 *tether_start_pos_g,
                                                  Vec3 *tether_start_vel_g) {
  // Aliases for geometric parameters.
  const double R = system_params_.gsg.ele_axis_horiz_offset_g;
  const double L = system_params_.tether.gsg_ele_to_termination;
  double theta = perch_azi + gsg_azi;
  double phi = gsg_ele;

  // The GSG elevation axis is offset from the g_frame origin. The azimuth
  // axis is directly above the g-frame orign.
  //
  // Side view:
  //                                      ___---* <--- Tether
  //                   L         ___---```_          termination
  //                    ___---```          \ phi
  //           ___---```                   |
  //          X--------*  -  -  -  -  -  -  -  -
  //          ^    R   |\     L*cos(phi) - R
  //          |        | \                                // Prevent backslashes
  //       gsg ele.    |  \                               // from ending lines
  //        axis       |   gsg azi. axis
  //                   |
  //                   * <--- g-frame origin
  //
  // The x- and y-coordinates of the tether start position are determined
  // by the segment of length L*cos(phi) - R rotating about the azimuth azis.

  *tether_start_pos_g = {cos(theta) * (L * cos(phi) - R),
                         -sin(theta) * (L * cos(phi) - R),
                         system_params_.gsg.ele_axis_z_g - L * sin(phi)};

  // Use a differentiating filter to calculate velocity. fc and zeta are chosen
  // by the "copy from some other differentiating filter in the code base"
  // strategy.
  const double fc = 0.1;
  const double zeta = 1.0 / sqrt(2.0);
  Vec3 z[2] = {kVec3Zero, kVec3Zero};
  DiffLpf2Vec3(tether_start_pos_g, fc, zeta, 0.01, tether_start_vel_g, z);
}
