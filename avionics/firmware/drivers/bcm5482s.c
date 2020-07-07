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

#include "avionics/firmware/drivers/bcm5482s.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/io.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/cpu/sci.h"
#include "avionics/firmware/drivers/bcm.h"
#include "avionics/firmware/util/state_machine.h"
#include "common/macros.h"

typedef enum {
  kGmac0,  // Corresponds to kPageExternalPhy1
  kGmac1   // Corresponds to kPageExternalPhy2
} GmacPort;

// (Table 313 in 5328xM-DS303 describes register address mapping).
static bool PhyReadUint16(GmacPort gmac, uint8_t address, uint16_t *value) {
  return BcmReadUint16(gmac + 0xD9, address * 2, value);
}

// (Table 313 in 5328xM-DS303 describes register address mapping).
static bool PhyWriteUint16(GmacPort gmac, uint8_t address, uint16_t value) {
  return BcmWriteUint16(gmac + 0xD9, address * 2, value);
}

// kGmac0 on the PHY corresponds to Gport 1 / Port 25 on the switch.
// This port can be configured as copper (1000BaseT) or as fiber (1000BaseX).
static bool StateGmac0Init1000BaseT(FsmState *state) {
  static int32_t i = 0;
  if (state->first_entry) {
    i = 0;
  }
  switch (i) {
    case 0:
      // Mode control is at register 0x1C, shadow 1F (page 179 5482S-DS11-R).
      // Set to copper media, select copper registers for PHY register
      // table 0x00 to 0x0F. See Table 74, page 179, in 5482S-DS11-R.
      if (!PhyWriteUint16(kGmac0, 0x1C, 0xFC08)) {
        return false;
      }
      ++i;
      break;
    case 1:
      // Announce our capabilities.  See 5482S-DS11-RDS.pdf page 101.
      // Disclaim anything that's not gigabit, as we're forcing the switch chip
      // to gigabit elsewhere.
      if (!PhyWriteUint16(kGmac0, 0x04, 0x0001)) {
        return false;
      }
      ++i;
      break;
    case 2:
      // Write MII control register 0x00 for normal operation, 1000 Mbps link
      // speed, auto-negotiation enabled, power down disabled, full duplex,
      // collision test disabled. See Table 20, page 96, in 5482S-DS11-R.
      // 0x1340 would restart autonegotiation, 0x1140 wouldn't.
      if (!PhyWriteUint16(kGmac0, 0x00, 0x1140)) {
        return false;
      }
      ++i;
      break;
    case 3:
      // Write Copper/Fiber Auto-Detect Medium register.
      // See Table 73, page 177, in 5482S-DS11-R.
      // Set the Invert Fiber Signal Detect bit.
      // Write register 1Ch, shadow value 11110 bit [8] = 1.
      // Set the Fiber Signal Detect with Sync Status bit.
      // Write register 1Ch, shadow value 11110 bit [5] = 1;
      // Select copper as the default when no media is active.
      // Write register 1Ch, shadow value 11110, bit [2] = 0.
      // Select the fiber line interface when both media are active.
      // Write register 1Ch, shadow value 11110, bit [1] = 1.
      // Enable Auto-Detect mode for copper and fiber, just in case.
      // Write register 1Ch, shadow value 11110, bit [0] = 1.
      if (!PhyWriteUint16(kGmac0, 0x1C, 0xF963)) {
        return false;
      }
      ++i;
    case 4:  // Fall through.
      return true;
    default:
      assert(false);
      return true;
  }
  return false;
}

static bool StateGmacInit1000BaseX(FsmState *state) {
  static int32_t i = 0;
  static GmacPort gmac = kGmac0;
  static uint16_t mode_control = 0;
  static uint16_t mii_control = 0;
  if (state->first_entry) {
    i = 0;
    gmac = kGmac0;
  }
  switch (i) {
    case 0:
      if (gmac == kGmac0) {
        // Skip the writes that disable copper.
        i = 5;
        break;
      }
      // Mode Control Register is address 1Ch, shadow 1F.
      // (Table 74 page 179/clxxix, 5482S-DS11-R describes bit fields).
      // 1000BASE-X MII Control is address 00h.
      // (Table 78, page 187, 5482S-DS11-R describes bit fields).
      if (!PhyWriteUint16(gmac, 0x1C, 0x7C00)) {
        return false;
      }
      ++i;
      break;
    case 1:
      if (!PhyReadUint16(gmac, 0x1C, &mode_control)) {
        return false;
      }

      // Enable copper register bank to power down copper interface.
      // Write register 1Ch, shadow 1F bit [0] = 0.
      mode_control &= 0xFFFE;
      mode_control |= 0x8008;  // Write enable, bit 3 reserved-write-as-1.
      ++i;
      break;
    case 2:
      if (!PhyWriteUint16(gmac, 0x1C, mode_control)) {
        return false;
      }
      ++i;
      break;
    case 3:
      if (!PhyReadUint16(gmac, 0x00, &mii_control)) {
        return false;
      }
      mii_control |= (1 << 11);
      ++i;
    case 4:
      if (!PhyWriteUint16(gmac, 0x00, mii_control)) {
        return false;
      }
      ++i;
      break;
    case 5:
      // Configure the BCM5482S to Fiber mode:
      // write reg 1Ch shadow 1F [2:1] = 01.
      if (!PhyWriteUint16(gmac, 0x1C, 0xFC0A)) {
        return false;
      }
      ++i;
      break;
    case 6:
      // Enable primary SerDes register:
      // Write register 1Ch, shadow 1F bit [0] = 1.
      // The docs list this and the above write as 2 steps; it's not clear that
      // they couldn't be done in a single step, skipping the 0xFC0A.
      if (!PhyWriteUint16(gmac, 0x1C, 0xFC0B)) {
        return false;
      }
      ++i;
      break;
    case 7:
      // Announce our capabilities as gigabit only.
      // See 5482S-DS11-RDS.pdf page 192.
      if (!PhyWriteUint16(gmac, 0x04, 0x0020)) {
        return false;
      }
      ++i;
      break;
    case 8:
      // Copper/Fiber Auto-detect Medium Register (p.177 5482S-DS11-R)
      // Write register 1Ch, shadow value 11110.
      // Set the Invert Fiber Signal Detect bit.
      // Use SerDes for LED activity when fiber mode selected.
      // Set the Qualify Fiber Signal Detect with Sync Status bit.
      // Select fiber as the default when no media is active.
      // Select the fiber line interface when both media are active.
      // Enable Auto-Detect mode.
      if (!PhyWriteUint16(gmac, 0x1C, 0xF967)) {
        return false;
      }
      ++i;
      break;
    case 9:
      // Enable PHY Scan on switch (p.266, 5328XM-DS303-R); this only needs to
      // be done once, not once per GMAC.
      if (gmac == kGmac1 && !BcmWriteUint8(0x00, 0x02, 0x03)) {
        return false;
      }
      ++i;
      break;
    case 10:
      // MII control, now in the 1000BASE-X register set.
      // [see 5328XM-AN1-1-RDS.pdf page 4].
      // 1000BASE-X MII Control Register (p.117 5482S-DS11-R).
      // Configure the PHY SERDES interface through the MII registers.
      // Write register 00 [15]   =  0 for don't-be-in-reset.
      // Write register 00 [14]   =  0 for normal operation, not loopback mode.
      // Write register 00 [13]   =  reserved, write as 0.
      // Write register 00 [12]   =  1 to enable auto-negotiation.
      // Write register 00 [11]   =  0 to disable fiber power down.
      // Write register 00 [10]   =  0 to electrically connect PHY to RGMII.
      // Write register 00 [9]    =  0 to not restart auto-negotiation.
      // Write register 00 [8]    =  1 to operate full-duplex.
      // Write register 00 [7]    =  0 to disable collision test.
      // Write register 00 [6:0]  =  reserved; write as 0x40.
      if (!PhyWriteUint16(gmac, 0x00, 0x1140)) {
        return false;
      }
      ++i;
      break;
    case 11:
      // Override GigaPort State on the switch.
      // 0x80 to let it alone, 0xCB to force on as gigabit.
      if (!BcmWriteUint8(0x01, 0x29 + gmac, 0xCB)) {
        return false;
      }
      ++i;
    case 12:  // Fall through.
      if (gmac == kGmac0) {
        ++gmac;
        i = 0;
        return false;
      }
      return true;
    default:
      assert(false);
      return true;
  }
  return false;
}

static bool StateRgmiiDelayAdjustments(FsmState *state) {
  static int32_t i = 0;
  static GmacPort gmac = kGmac0;
  static uint16_t misc_control = 0;
  if (state->first_entry) {
    i = 0;
    gmac = kGmac0;
  }
  switch (i) {
    case 0:
      // Set internal delay on the RGMII interface of the BCM53284 for TX,
      // leave RX unchanged. (See 5328XM-DS303-R on page 277).
      if (!BcmWriteUint16(0x01, 0x04 + (gmac << 1), 0x01)) {
        return false;
      }
      ++i;
      break;
    case 1:
      // Set next read to get the misc control register [shadow value 0x7].
      if (!PhyWriteUint16(gmac, 0x18, 0x7007)) {
        return false;
      }
      ++i;
      break;
    case 2:
      if (!PhyReadUint16(gmac, 0x18, &misc_control)) {
        return false;
      }
      // Enable RGMII RXC delayed timing mode.
      misc_control |= 0xF107;
      ++i;
      break;
    case 3:
      if (!PhyWriteUint16(gmac, 0x18, misc_control)) {
        return false;
      }
      ++i;
      break;
    case 4:
      // Bypass GTXCLKdelay in 5482S (see 5482S-DS11-R pages 144).
      // Clear clock alignment control bit 9 (add 1Ch, shadow value 00011).
      if (!PhyWriteUint16(gmac, 0x1C, 0x8C00)) {
        return false;
      }
      ++i;
    case 5:  // Fall through.
      if (gmac == kGmac0) {
        ++gmac;
        i = 0;
        return false;
      }
      return true;
    default:
      assert(false);
      return true;
  }
  return false;
}

// Issue a hardware reset via general purpose I/O from TMS570.
static bool StateInit(FsmState *state) {
  static int i = 0;
  if (state->first_entry) {
    i = 0;
  }
  switch (i) {
    case 0:
      if (!PhyWriteUint16(kGmac0, 0x00, 0x8000)) {
        return false;
      }
      ++i;
      break;
    case 1:
      if (!PhyWriteUint16(kGmac1, 0x00, 0x8000)) {
        return false;
      }
      ++i;
      return true;
    default:
      assert(false);
      return true;
  }

  return false;
}

static bool StateInitN2Het1(FsmState *state) {
  (void)state;
  // Initialize N2HET1 for GPIO functionality on N2HET1[4,14],
  // which are used for the SFPs.

  IoSetDirectionAsOutput(kIoN2het1Pin4);
  IoSetDirectionAsOutput(kIoN2het1Pin14);

  // Enable SFP transmitter.
  IoSetValue(kIoN2het1Pin4, false);
  IoSetValue(kIoN2het1Pin14, false);

  return true;
}

static bool StateIdle(FsmState *state) {
  (void)state;
  return false;
}

typedef enum {
  kStateInit,
  kStateSoftwareResetDelay,
  kStateInitN2Het1,
  kStateRgmiiDelayAdjustments,
  kStateGmac0Init1000BaseT,
  kStateGmacInit1000BaseX,
  kStateReady,
} BcmState;


static const StateTransition kFsmTrans[] = {
  STATE_TRANS_DEFAULT(kStateInit, StateInit, kStateSoftwareResetDelay),
  STATE_TRANS(kStateSoftwareResetDelay, StateIdle, STATE_INVALID, STATE_INVALID,
              kStateInitN2Het1, 4),
  STATE_TRANS_DEFAULT(kStateInitN2Het1, StateInitN2Het1,
                      kStateRgmiiDelayAdjustments),
  STATE_TRANS_DEFAULT(kStateRgmiiDelayAdjustments, StateRgmiiDelayAdjustments,
                      kStateGmac0Init1000BaseT),
  STATE_TRANS_DEFAULT(kStateGmac0Init1000BaseT, StateGmac0Init1000BaseT,
                      kStateGmacInit1000BaseX),
  STATE_TRANS_DEFAULT(kStateGmacInit1000BaseX, StateGmacInit1000BaseX,
                      kStateReady),
  STATE_TRANS_DEFAULT(kStateReady, StateIdle, STATE_INVALID),
};

static const StateMachine kFsm = {
  .num_states = ARRAYSIZE(kFsmTrans),
  .init_state = kStateInit,
  .states = kFsmTrans
};

static FsmState g_state;

void Bcm5482SInit(void) {
  StateMachineInit(&kFsm, &g_state);
}

bool Bcm5482SReady(void) {
  return GetState(&g_state) == kStateReady;
}

// This state machine must run within a single state in the BCM53284 state
// machine--they can't interlace, or they'll fight over communications to the
// underlying bcm driver.
void Bcm5482SPoll() {
  StateMachinePoll(&kFsm, &g_state);
  BcmPoll();
}

