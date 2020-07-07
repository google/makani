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

#ifndef CONTROL_SENSOR_UTIL_H_
#define CONTROL_SENSOR_UTIL_H_

#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/platform_frame.h"
#include "control/sensor_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// GPS functions.

// Converts the GS GPS position reading to the position of the
// ground-station origin in ECEF and calculates the transformation
// matrix from ECEF to G.
void GsGpsPosEcefToGsPosEcef(const Vec3 *gs_gps_pos_ecef,
                             const Vec3 *gs_gps_pos_g, double g_heading,
                             Vec3 *gs_pos_ecef, Mat3 *dcm_ecef2g);

// Converts a position and velocity uncertainties from the ECEF frame
// to the ground frame.
void ConvSigmaEcefToLocal(const Vec3 *sigma_X_ecef, const Vec3 *sigma_V_ecef,
                          const Mat3 *dcm_ecef2g, Vec3 *sigma_Xg,
                          Vec3 *sigma_Vg);

// Tether force functions.

// Converts a tension, tether roll, and tether pitch to a tether force
// vector in body coordinates.
void TetherForceSphToCart(const TetherForceSph *sph, Vec3 *cart);

// Converts a tether force vector in body coordinates to a tension,
// tether roll, and tether pitch.
void TetherForceCartToSph(const Vec3 *cart, TetherForceSph *sph);

// Converts the loadcell readings into bridle force vectors, then calculates
// tether tension, roll, and pitch.
void LoadcellsToTetherForce(const double loadcells[],
                            const WingParams *wing_params,
                            const LoadcellParams loadcell_params[], Vec3 *cart,
                            TetherForceSph *sph, Vec3 *port_force_b,
                            Vec3 *star_force_b);

// Apparent wind functions.

// Converts airspeed, angle-of-attack, and sideslip angle to the
// apparent wind vector in body coordinates.
void ApparentWindSphToCart(const ApparentWindSph *sph, Vec3 *cart);

// Converts the apparent wind vector to alpha, beta, and airspeed.
void ApparentWindCartToSph(const Vec3 *cart, ApparentWindSph *sph);

// Converts the pitot measurements to angles and airspeed.
void PitotToApparentWindSph(const PitotDifferentialData *diff, const Vec3 *pqr,
                            const PitotParams *params, ApparentWindSph *sph);

// Rotates a wind speed measurement in the wind sensor frame into
// ground coordinates.
//
// Args:
//   wind_ws:   Wind measurement [m/s] in the wind sensor frame.
//   omega_g2p: Body rates of the platform with respect to ground.
//   dcm_g2p:   Platform attitude with respect to ground.
//   vel_g:     Velocity [m/s] of the platform frame origin w.r.t. ground.
//   params:    Wind sensor parameters including location of the sensor.
//   wind_g:    Output wind measurement [m/s] in the ground coordinates.
void WindWsToWindG(const Vec3 *wind_ws, const Mat3 *dcm_g2p,
                   const Vec3 *omega_g2p, const Vec3 *vel_g,
                   const WindSensorParams *params, Vec3 *wind_g);

// Converts the vessel frame heading and perch heading to the perch azimuth
// angle.
//
// Args:
//   vessel_heading: Heading [rad] of the vessel frame relative to NED.  Should
//       lie in [0, 2 pi).
//   perch_heading: Heading [rad] of the perch relative to NED.  Should be
//       lie in [0, 2 pi).
//
// Returns:
//   The perch azimuth angle [rad] in range (-pi, pi).
double VesselHeadingPerchHeadingToPerchAzi(double vessel_heading,
                                           double perch_heading);

// Converts a winch position [m] to an equivalent drum angle [rad].
double WinchPosToDrumAngle(double winch_pos, const WinchParams *params);

// Alternate GLAS-based position functions.

// Converts a position in GSG coordinates to perch coordinates.
void XgsgToXp(const GsgData *gsg, const Vec3 *X_gsg, double drum_angle,
              Vec3 *Xp);

// Converts wing position in perch coordinates to an equivalent
// GSG reading ([azi, ele]') assuming a perfect GSG, and a
// straight-line tether.
void XpToGsg(const Vec3 *Xp, double drum_angle, GsgData *gsg);

// Determines the azimuth and elevation angles that correspond to the vector
// describing the departure direction of the tether from the GSG.
void TetherDirectionWdToGsgAziEle(const Vec3 *tether_direction_wd, double *azi,
                                  double *ele);

// Returns true if levelwind should be engaged based on drum angle.
bool IsLevelwindEngaged(double drum_angle, const LevelwindParams *params);

// Compute the DCM to rotate Drum frame to Gsg0 frame.
void CalcDcmWdToGsg0(double detwist_ele, double detwist_pos, Mat3 *dcm_wd2gsg0);

// Estimates tether elevation angles, and the detwist angle that places all
// tether deflection into the GSG yoke DOF.
//
// Args:
//   dcm_g2p:         Transformation matrix from ground to platform coordinates.
//   drum_position:   Angle [rad] of the drum.
//   detwist_ele:     Elevation [rad] of the detwist above the ground frame's
//                    xy-plane.
//   detwist_angle:   Angle [rad] of the detwist servo.
//   gsg_yoke:        Angle [rad] of the GSG yoke bearing.
//   gsg_termination: Angle [rad] of the GSG termination bearing.
//   hold_cone_half_angle:  Half-angle [rad] of a cone about the detwist axis.
//                    The tether detwist angle will be held if the tether
//                    deflection is inside this cone.
//   detwist_axis_offset: Angle in radians that the axis used to compute the
//                    detwist angle is offset upwards to ensure the tether
//                    departure direction encircles the physical detwist axis.
//
// Returns:
//   tether_elevation_p:    Tether elevation [rad] relative to platform.
//   tether_elevation_g:    Tether elevation [rad] relative to ground.
//   tether_detwist_angle:  Detwist angle [rad] that places all tether
//                          deflection into the GSG yoke bearing.
//   last_detwist_angle:    Prior detwist angle [rad]. Used to compute
//                          accumulated_detwist_angle.
//   accumulated_detwist_angle: Accumulated detwist angle [rad].
void CalcTetherAnglesFromGsg(
    const Mat3 *dcm_g2p, double drum_position, double detwist_ele,
    double detwist_angle, double gsg_yoke, double gsg_termination,
    double hold_cone_half_angle, double detwist_axis_offset,
    double *tether_elevation_p, double *tether_elevation_g,
    double *tether_detwist_angle, double *last_detwist_angle,
    double *accumulated_detwist_angle);

void CalcTetherAnglesFromLevelwind(const Mat3 *dcm_ned_to_platform,
                                   double levelwind_ele,
                                   double *tether_elevation_p,
                                   double *tether_elevation_g);

// Converts static pressure to altitude (positive is up) above mean
// sea level.
double PressureToAltitude(double pressure, const PhysParams *params);

// Calculate air density [kg/m^3] given pressure [Pa] and temperature [°C],
// assuming dry air.
double CalcDryAirDensity(double pressure_pascals, double temperature_celcius);

// Calculate air density [kg/m^3] given pressure [Pa], temperature
// [°C], and relative humidity [as a fraction, from 0.0 to 1.0]. The
// validity flag, if not NULL, is set indicating whether the input
// arguments are in a meteorologically plausible range.
double CalcAirDensity(double pressure_pascals, double temperature_celcius,
                      double relative_humidity, bool *valid);

// Calculate the approximate tether anchor point without modeling the
// racetrack, wide wrap section, and levelwind position.
//
// The tether anchor point is calculated 3 different ways, depending on the
// winch angle:
//
// 1. During reel, between perched and the start of the racetrack
//    (racetrack_low), the anchor point is along the top of the drum,
//    crossfaded based on winch angle between the perched and
//    start-of-racetrack positions.
// 2. While the tether is coming off the racetrack, the anchor point is
//    crossfaded between the start-of-racetrack position and the position the
//    GSG will be in when the tether comes off the racetrack.
// 3. When the winch rotates passed the highest angle at which the tether
//    anchors on the racetrack (racetrack_high), the anchor point is the
//    GSG position.
//
// These approximations have slight effects on the exact:
// * Wrap-radius while on the racetrack.
// * X-axis location along the drum as tether is reeled.
void CalcTetherAnchorPoint(double winch_pos, const Mat3 *dcm_g2p,
                           const Vec3 *vessel_g, Vec3 *tether_anchor);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_SENSOR_UTIL_H_
