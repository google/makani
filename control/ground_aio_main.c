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

#define _GNU_SOURCE  // Needed for sched_setaffinity().
#include <assert.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/linux/aio.h"
#include "avionics/linux/q7_slow_status.h"
#include "avionics/linux/q7_slow_status_types.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_system.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_ground.h"
#include "control/ground_telemetry.h"
#include "control/pack_control_telemetry.h"
#include "control/pack_ground_telemetry.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "sim/pack_sim_messages.h"

static const AioNode kThisNode = kAioNodeGsEstimator;

#if defined(MAKANI_HITL)

#include "control/avionics/avionics_sim.h"
#include "sim/cvt_sim_messages.h"
#include "sim/sim_messages.h"

static const MessageType *GetSubscribeTypes(int32_t *num_subscribe_types) {
  static const MessageType kRealFlightSubscribeTypes[] = {
      kMessageTypeSimSensor, kMessageTypeSimCommand};

  *num_subscribe_types = ARRAYSIZE(kRealFlightSubscribeTypes);
  return kRealFlightSubscribeTypes;
}

#else

static const MessageType *GetSubscribeTypes(int32_t *num_subscribe_types) {
  static const MessageType kRealFlightSubscribeTypes[] = {
      kMessageTypeFlightComputerImu, kMessageTypeFlightComputerSensor,
      kMessageTypeNovAtelSolution, kMessageTypeNovAtelCompass};

  *num_subscribe_types = ARRAYSIZE(kRealFlightSubscribeTypes);
  return kRealFlightSubscribeTypes;
}

#endif  // defined(MAKANI_HITL)

static GroundEstimatorState g_state;

// Function called periodically by AIO.
//
// Periodically calls GroundEstimateStep, then sends the resulting
// command, synchronization, and telemetry messages.
//
// During HITL simulations, waits until the first simulator message is
// received, then uses these messages to overwrite portions of the
// CVT.  Timeouts can cause the estimator to exit if messages are not
// being received from the simulator.
//
// Returns:
//   Whether or not the estimator should keep running.
static bool AioCallback(void *arg) {
  UNUSED(arg);

#if defined(MAKANI_HITL)

  SimCommandMessage sim_command = {0U, 0U, false};
  if (CvtGetSimCommandMessage(kAioNodeSimulator, &sim_command, NULL, NULL)) {
    if (sim_command.stop) {
      return false;
    }
  }

  SimSensorMessage sensor_message;
  uint16_t sequence;
  int64_t timestamp;
  bool updated = CvtGetSimSensorMessage(kAioNodeSimulator, &sensor_message,
                                        &sequence, &timestamp);

  // During HITL simulations, the controller will not run until the
  // first simulator message is received. Two timeouts are used to
  // terminate the controller if communication with the simulator is
  // interrupted:
  //   (1) the controller will wait at most sim_init_timeout seconds
  //       for the first simulator message,
  //   (2) once the controller is running, it will terminate if it does not hear
  //       from the simulator for sim_update_timeout seconds.
  const HitlControlParams *hitl_params = &GetControlParams()->hitl;
  static bool started = false;
  static int32_t iters_without_update = 0;
  if (updated) {
    iters_without_update = 0;
    started = true;
  } else {
    iters_without_update++;
    if (!started) {
      return iters_without_update * GetSystemParams()->ts <
             hitl_params->sim_init_timeout;
    } else if (iters_without_update * GetSystemParams()->ts >=
               hitl_params->sim_update_timeout) {
      return false;
    }
  }

  if (updated) {
    // If the simulator is running with a different HITL configuration than
    // the controller, exit.
    //
    // The sim_level only needs to get set once, but the logic stays better
    // contained this way.
    GetSystemParamsUnsafe()->hitl.config.sim_level = kSimulatorHitlLevelAsync;
    if (!HitlConfigurationsEquivalent(&sensor_message.hitl_config,
                                      &GetSystemParams()->hitl.config)) {
      assert(false);
      return false;
    }

    // Write the most recent sensor message to the CVT.
    UpdateGroundEstimatorCvtFromSimSensorMessage(&sensor_message, sequence,
                                                 timestamp);
  }

#else

  assert(!(bool)"Non-HITL flights will not run with asserts turned on.");

#endif  // defined(MAKANI_HITL)

  EstimatorGroundStep(GetSystemParams(), GetControlParams(), &g_state);

  AIO_SEND_PACKED(kMessageTypeGroundEstimate, PackGroundEstimateMessage,
                  PACK_GROUNDESTIMATEMESSAGE_SIZE, &g_state.ground_estimate);

  AIO_SEND_PACKED(kMessageTypeGroundTelemetry, PackGroundTelemetry,
                  PACK_GROUNDTELEMETRY_SIZE, GetGroundTelemetryMessage());

  static int32_t q7_slow_status_counter = 0;
  ++q7_slow_status_counter;
  if (q7_slow_status_counter >= Q7_SLOW_STATUS_DECIMATION) {
    q7_slow_status_counter = 0;
    AIO_SEND_PACKED(kMessageTypeQ7SlowStatus, PackQ7SlowStatusMessage,
                    PACK_Q7SLOWSTATUSMESSAGE_SIZE, GetQ7SlowStatusMessage());
  }

  return true;  // Always continue execution.
}

// TODO: Move rt functoins into a common lib.

// Lock us to CPU1 to avoid conflicting with IRQ handling.
static bool SetCpuAffinity(void) {
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(1, &set);
  if (sched_setaffinity(getpid(), sizeof(set), &set) == -1) {
    assert(!(bool)"Could not set CPU affinity.");
    return false;
  }
  return true;
}

// Raise our priority and set us to SCHED_FIFO.
static bool SetRealTimeSchedPriority(void) {
  struct sched_param param = {
      .sched_priority = 49,
  };
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    assert(!(bool)"Could not set real time priority/scheduling policy.");
    return false;
  }
  return true;
}

// Lock all to keep them resident.
static bool LockMemoryPages(void) {
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    assert(!(bool)"Could not lock memory pages.");
    return false;
  }
  return true;
}

// Pre fault in a large stack to avoid page allocation at runtime.
static bool PreFaultStack(void) {
  // The default stack ulimit is 8K.
  uint8_t stack_allocation[8 * 1024];

  // Touch the memory to force the kernel to fault in our pages.
  memset(stack_allocation, 0, sizeof(stack_allocation));
  return true;
}

static bool EnterRealTimeMode(void) {
  return SetCpuAffinity() && SetRealTimeSchedPriority() && LockMemoryPages() &&
         PreFaultStack();
}

// Entry point for the ground estimator when run on the ground estimator node
int main(int argc, char **argv) {
  UNUSED(argc);
  UNUSED(argv);

  int32_t num_subscribe_types;
  const MessageType *subscribe_types = GetSubscribeTypes(&num_subscribe_types);

  if (!EnterRealTimeMode()) return -1;

  EstimatorGroundInit(&g_state);
  // Initialize the slow status message.
  Q7SlowStatusContext q7_slow_status_context;
  if (!Q7SlowStatusInit(&q7_slow_status_context)) {
    if (kThisNode != q7_slow_status_context.node) {
      assert(!(bool)"IP address/AioNode mismatch.");
      return -1;
    }
  }
  Q7SlowStatusInitMessage(&q7_slow_status_context, GetQ7SlowStatusMessage());

  AioLoopStart(kThisNode, GetSystemParams()->comms.aio_port, subscribe_types,
               num_subscribe_types, AioCallback, NULL,
               (int32_t)(GetSystemParams()->ts * 1000000));
  AioClose();

  return 0;
}
