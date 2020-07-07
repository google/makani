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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <iostream>

#include "common/c_math/force_moment.h"
#include "common/c_math/linalg.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "common/runfiles_dir.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "lib/json_load/load_params.h"
#include "sim/physics/aero.h"
#include "sim/physics/aero_frame.h"
#include "sim/sim_params.h"
#include "sim/physics/rotor_database_3d.h"
#include "sim/sim_types.h"

DEFINE_double(v_axial, 0.0,
              "Axial velocity of rotor, m/s.");
DEFINE_double(v_edgewise, 10.0,
              "Edgewise velocity of rotor, m/s.");
DEFINE_double(omega, 150.0, "Angular velocity of rotor, rad/s.");
DEFINE_int32(dir, 1, "Spin direction about thrust axis.");
DEFINE_double(air_density, 1.10, "Air density, kg/m^3.");

namespace {

void PrintRotorCoefficients(double z_force, double side_force,
                            double roll_moment, double pitch_moment,
                            double thrust, double torque) {
  printf(
      "\n V_axial      :  %7.1f m/s"
      "\n V_edgewise   :  %7.1f m/s"
      "\n Omega        :  %7.1f rad/s \n"
      "\n Thrust     FX:  %7.1f N"
      "\n Torque     MX:  %7.1f Nm"
      "\n Z Force    FZ:  %7.1f N"
      "\n Side Force FY:  %7.1f N"
      "\n Roll Mom   MZ:  %7.1f Nm"
      "\n Pitch Mom  MY:  %7.1f Nm \n\n",
      FLAGS_v_axial, FLAGS_v_edgewise, FLAGS_omega, thrust, torque,
      z_force, side_force, roll_moment, pitch_moment);
}

}  // namespace

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Specify runfiles dir for database access.
  SetRunfilesDirFromBinaryPath(argv[0]);

  // Load runtime parameters.
  json_load::LoadSystemParams(GetSystemParamsUnsafe());
  json_load::LoadSimParams(GetSimParamsUnsafe());

  // Pull rotor database name from rotor_sim params.
  const RotorSimParams &rotor_sim_params = GetSimParams()->rotor_sim;
  RotorDatabase3d rotor3d(RunfilesDir() + "/database/" +
                          rotor_sim_params.database_3d_names[0].name);

  // Perform lookup in rotor_database_3d.cc.
  double z_force = rotor3d.CalcForceZPrime(FLAGS_omega, FLAGS_v_axial,
                                           FLAGS_v_edgewise,
                                           FLAGS_air_density);
  double side_force = rotor3d.CalcForceYPrime(FLAGS_omega, FLAGS_v_axial,
                                              FLAGS_v_edgewise,
                                              FLAGS_air_density,
                                              FLAGS_dir);
  double roll_moment = rotor3d.CalcMomZPrime(FLAGS_omega, FLAGS_v_axial,
                                             FLAGS_v_edgewise,
                                             FLAGS_air_density,
                                             FLAGS_dir);
  double pitch_moment = rotor3d.CalcMomYPrime(FLAGS_omega, FLAGS_v_axial,
                                              FLAGS_v_edgewise,
                                              FLAGS_air_density);
  double thrust = rotor3d.CalcThrust(FLAGS_omega, FLAGS_v_axial,
                                     FLAGS_v_edgewise, FLAGS_air_density);
  double torque = rotor3d.CalcTorque(FLAGS_omega, FLAGS_v_axial,
                                     FLAGS_v_edgewise, FLAGS_air_density,
                                     FLAGS_dir);

  // Print to screen.
  PrintRotorCoefficients(z_force, side_force, roll_moment, pitch_moment,
                         thrust, torque);

  return EXIT_SUCCESS;
}
