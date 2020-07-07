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

#include "control/system_types.h"

#include <assert.h>
#include <stdbool.h>

const char *FlightPlanToString(FlightPlan flight_plan) {
  switch (flight_plan) {
    case kFlightPlanDisengageEngage:
      return "DisengageEngage";
    case kFlightPlanHighHover:
      return "HighHover";
    case kFlightPlanHoverInPlace:
      return "HoverInPlace";
    case kFlightPlanLaunchPerch:
      return "LaunchPerch";
    case kFlightPlanManual:
      return "Manual";
    case kFlightPlanStartDownwind:
      return "StartDownwind";
    case kFlightPlanTurnKey:
      return "TurnKey";
    default:
    case kFlightPlanForceSigned:
    case kNumFlightPlans:
      assert(false);
      return "<Unknown>";
  }
}

const char *GroundStationModelToString(GroundStationModel gs) {
  switch (gs) {
    case kGroundStationModelGSv1:
      return "GSv1";
    case kGroundStationModelGSv2:
      return "GSv2";
    case kGroundStationModelTopHat:
      return "TopHat";
    default:
    case kGroundStationModelForceSigned:
    case kNumGroundStationModels:
      assert(false);
      return "<Unknown>";
  }
}

const char *TestSiteToString(TestSite test_site) {
  switch (test_site) {
    case kTestSiteAlameda:
      return "Alameda";
    case kTestSiteChinaLake:
      return "ChinaLake";
    case kTestSiteParkerRanch:
      return "ParkerRanch";
    case kTestSiteNorway:
      return "Norway";
    default:
    case kTestSiteForceSigned:
    case kNumTestSites:
      assert(false);
      return "<Unknown>";
  }
}

const char *WingSerialToString(WingSerial wing_serial) {
  switch (wing_serial) {
    case kWingSerial01:
      return "01";
    case kWingSerial02:
      return "02";
    case kWingSerial02Final:
      return "02Final";
    case kWingSerial03Hover:
      return "03Hover";
    case kWingSerial03Crosswind:
      return "03Crosswind";
    case kWingSerial04Hover:
      return "04Hover";
    case kWingSerial04Crosswind:
      return "04Crosswind";
    case kWingSerial05Hover:
      return "05Hover";
    case kWingSerial05Crosswind:
      return "05Crosswind";
    case kWingSerial06Hover:
      return "06Hover";
    case kWingSerial06Crosswind:
      return "06Crosswind";
    case kWingSerial07Hover:
      return "07Hover";
    case kWingSerial07Crosswind:
      return "07Crosswind";
    case kWingSerialOktoberKite01:
      return "OktoberKite01";
    default:
    case kWingSerialForceSigned:
    case kNumWingSerials:
      assert(false);
      return "<Unknown>";
  }
}

int WingSerialToModel(WingSerial wing_serial) {
  switch (wing_serial) {
    case kWingSerial01:
    case kWingSerial02:
    case kWingSerial02Final:
    case kWingSerial03Hover:
    case kWingSerial03Crosswind:
    case kWingSerial04Hover:
    case kWingSerial04Crosswind:
    case kWingSerial05Hover:
    case kWingSerial05Crosswind:
    case kWingSerial06Hover:
    case kWingSerial06Crosswind:
    case kWingSerial07Hover:
    case kWingSerial07Crosswind:
      return kWingModelYm600;
    case kWingSerialOktoberKite01:
      return kWingModelOktoberKite;
    default:
    case kWingSerialForceSigned:
    case kNumWingSerials:
      assert(false);
      return -99;
  }
}

LoadcellSensorLabel BridleAndChannelToLoadcellSensorLabel(BridleLabel bridle,
                                                          int32_t channel) {
  switch (bridle) {
    case kBridlePort:
      switch (channel) {
        case 0:
          return kLoadcellSensorPort0;
        case 1:
          return kLoadcellSensorPort1;
        default:
          assert(false);
          return kLoadcellSensorPort0;
      }
    case kBridleStar:
      switch (channel) {
        case 0:
          return kLoadcellSensorStarboard0;
        case 1:
          return kLoadcellSensorStarboard1;
        default:
          assert(false);
          return kLoadcellSensorPort0;
      }
    case kBridleLabelForceSigned:
    case kNumBridles:
    default:
      assert(false);
      return kLoadcellSensorPort0;
  }
}

bool IsLowAltitudeFlightPlan(FlightPlan flight_plan) {
  switch (flight_plan) {
    case kFlightPlanDisengageEngage:
    case kFlightPlanHoverInPlace:
    case kFlightPlanLaunchPerch:
    case kFlightPlanManual:
      return true;
    case kFlightPlanHighHover:
    case kFlightPlanTurnKey:
    case kFlightPlanStartDownwind:
      return false;
    case kFlightPlanForceSigned:
    case kNumFlightPlans:
    default:
      assert(false);
      return true;
  }
}
