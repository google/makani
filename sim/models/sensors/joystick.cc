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

#include "sim/models/sensors/joystick.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <stdint.h>

#include <limits>
#include <stack>
#include <string>
#include <vector>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/linux/cvt_util.h"
#include "common/c_math/util.h"
#include "control/system_types.h"
#include "sim/sim_messages.h"
#include "sim/sim_params.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"

namespace {

AioNode GetJoystickNode(SimJoystickType type) {
  switch (type) {
    case kSimJoystickTypeProgrammed:
      return kAioNodeUnknown;
    case kSimJoystickTypeSoftware:
      return kAioNodeSimulatedJoystick;
    case kSimJoystickTypeHardware:
      return kAioNodeJoystickA;
    default:
      LOG(FATAL) << "Invalid SimJoystickType received.";
      return kAioNodeUnknown;
  }
}

}  // namespace

Joystick::Joystick(const JoystickParams &joystick_params,
                   const SimJoystickParams &joystick_sim_params)
    : Sensor("Joystick"),
      joystick_params_(joystick_params),
      joystick_sim_params_(joystick_sim_params),
      joystick_node_(GetJoystickNode(joystick_sim_params.joystick_type)),
      updates_(),
      throttle_(new_discrete_state(), "throttle", 0.0, 0.1),
      roll_(new_discrete_state(), "roll", 0.0, 0.0),
      pitch_(new_discrete_state(), "pitch", 0.0, 0.0),
      yaw_(new_discrete_state(), "yaw", 0.0, 0.0),
      tri_switch_(new_discrete_state(), "tri_switch", 0.0,
                  kJoystickSwitchPositionMiddle),
      momentary_switch_(new_discrete_state(), "momentary_switch", 0.0,
                        kJoystickSwitchPositionDown) {
  CHECK_LE(0, joystick_sim_params.num_updates);

  for (int32_t i = joystick_sim_params.num_updates - 1; i >= 0; --i) {
    const SimJoystickUpdate *update = &joystick_sim_params.updates[i];
    CHECK(updates_.empty() || updates_.top()->t_update >= update->t_update)
        << "Programmed joystick update " << i << " is not in order ("
        << "occurs at t = " << update->t_update << " when the next update"
        << " occurs at t = " << updates_.top()->t_update << ")";
    updates_.push(update);
  }

  SetupDone();
}

void Joystick::DiscreteStepHelper(double t) {
  double throttle__ = throttle();
  double roll__ = roll();
  double pitch__ = pitch();
  double yaw__ = yaw();
  JoystickSwitchPositionLabel tri_switch__ = tri_switch();
  JoystickSwitchPositionLabel momentary_switch__ = momentary_switch();

  if (joystick_sim_params_.joystick_type == kSimJoystickTypeProgrammed) {
    // Pre-programmed, time-based joystick motions.  Runs every loop,
    // not just when a UDP packet is received.
    while (!updates_.empty() && updates_.top()->t_update <= t) {
      const SimJoystickUpdate &update = *updates_.top();

      switch (update.type) {
        case kSimJoystickUpdateThrottle:
          if (update.enum_value == kSimJoystickThrottleManual) {
            DCHECK(update.value >= 0.0 && update.value <= 1.0);
            throttle__ = update.value;
          } else {
            DCHECK(update.enum_value >= 0 &&
                   update.enum_value < kNumSimJoystickThrottles);
            throttle__ =
                joystick_sim_params_.throttle_settings[update.enum_value];
          }
          break;
        case kSimJoystickUpdateRoll:
          roll__ = update.value;
          break;
        case kSimJoystickUpdatePitch:
          pitch__ = update.value;
          break;
        case kSimJoystickUpdateYaw:
          yaw__ = update.value;
          break;
        case kSimJoystickUpdateSwitchUp:
          tri_switch__ = kJoystickSwitchPositionUp;
          break;
        case kSimJoystickUpdateSwitchMiddle:
          tri_switch__ = kJoystickSwitchPositionMiddle;
          break;
        case kSimJoystickUpdateSwitchDown:
          tri_switch__ = kJoystickSwitchPositionDown;
          break;
        case kSimJoystickUpdateReleasePulled:
          momentary_switch__ = kJoystickSwitchPositionUp;
          break;
        case kSimJoystickUpdateReleaseNotPulled:
          momentary_switch__ = kJoystickSwitchPositionDown;
          break;
        case kSimJoystickUpdateNone:
          break;
        default:
          LOG(ERROR) << "Invalid joystick update type.";
      }
      updates_.pop();
    }

    if (*g_sim.sim_opt & kSimOptTiedDown) {
      tri_switch__ = kJoystickSwitchPositionUp;
      momentary_switch__ = kJoystickSwitchPositionDown;
      throttle__ = fmin(t / 50.0, 1.0);
    }
  } else {
    JoystickStatusMessage joystick;
    if (CvtGetNextUpdate(joystick_node_, CvtGetJoystickStatusMessage, 0.0,
                         &joystick)) {
      throttle__ = ApplyCal(joystick.throttle, &joystick_params_.cal.throttle);
      roll__ = ApplyCal(joystick.roll, &joystick_params_.cal.roll);
      pitch__ = ApplyCal(joystick.pitch, &joystick_params_.cal.pitch);
      yaw__ = ApplyCal(joystick.yaw, &joystick_params_.cal.yaw);

      tri_switch__ =
          static_cast<JoystickSwitchPositionLabel>(joystick.tri_switch);
      momentary_switch__ =
          static_cast<JoystickSwitchPositionLabel>(joystick.momentary_switch);
    }
  }

  throttle_.DiscreteUpdate(t, throttle__);
  roll_.DiscreteUpdate(t, roll__);
  pitch_.DiscreteUpdate(t, pitch__);
  yaw_.DiscreteUpdate(t, yaw__);
  tri_switch_.DiscreteUpdate(t, tri_switch__);
  momentary_switch_.DiscreteUpdate(t, momentary_switch__);
}

void Joystick::UpdateSensorOutputs(SimSensorMessage *sensor_message,
                                   TetherUpMessage * /*tether_up*/) const {
  sensor_message->control_input_messages_updated.joystick = true;
  JoystickStatusMessage *joystick =
      &sensor_message->control_input_messages.joystick;

  joystick->throttle =
      static_cast<float>(InvertCal(throttle(), &joystick_params_.cal.throttle));
  joystick->roll =
      static_cast<float>(InvertCal(roll(), &joystick_params_.cal.roll));
  joystick->pitch =
      static_cast<float>(InvertCal(pitch(), &joystick_params_.cal.pitch));
  joystick->yaw =
      static_cast<float>(InvertCal(yaw(), &joystick_params_.cal.yaw));

  joystick->tri_switch = static_cast<uint8_t>(tri_switch());
  joystick->momentary_switch = static_cast<uint8_t>(momentary_switch());
}

void Joystick::Publish() const {
  sim_telem.joystick.throttle = throttle();
  sim_telem.joystick.roll = roll();
  sim_telem.joystick.pitch = pitch();
  sim_telem.joystick.yaw = yaw();
  sim_telem.joystick.tri_switch = tri_switch();
  sim_telem.joystick.momentary_switch = momentary_switch();
}
