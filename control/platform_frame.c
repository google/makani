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

#include "control/platform_frame.h"

#include <math.h>

#include "common/c_math/geometry.h"

// Conversions between ground frame, vessel frame, and platform frame.
void CalcDcmVesselToPlatform(double perch_azi, Mat3 *dcm_g2p) {
  AngleToDcm(perch_azi, 0.0, 0.0, kRotationOrderZyx, dcm_g2p);
}
