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

#include "gs/monitor/layouts/layouts_controller.h"
#include "gs/monitor/layouts/layouts_util.h"

#include <gtk/gtk.h>
#include <stddef.h>

#include "gs/monitor/charts/charts_controller.h"
#include "gs/monitor/charts/charts_m600_gps.h"
#include "gs/monitor/indicators/indicators_controller.h"
#include "gs/monitor/indicators/indicators_ground_station.h"
#include "gs/monitor/indicators/indicators_headings.h"
#include "gs/monitor/indicators/indicators_m600_gps.h"
#include "gs/monitor/indicators/indicators_util.h"
#include "gs/monitor/monitor.h"
#include "gs/monitor/polar_plots/polar_plots_controller.h"
#include "gs/monitor/widgets/indicator.h"
#include "gs/monitor/widgets/indicator_chart.h"
#include "gs/monitor/widgets/polar_plot.h"
#include "gs/monitor/widgets/xy_plot.h"
#include "gs/monitor/xy_plots/xy_plots_controller.h"

void ConfigCrosswindMonitor(GtkWidget *window, MonitorConfig *mconfig_out) {
  GtkWidget *fixed = gtk_fixed_new();
  GtkWidget *hbox = gtk_hbox_new(FALSE, 0);
  GtkWidget *col_0 = gtk_vbox_new(FALSE, 0);
  GtkWidget *col_1 = gtk_vbox_new(FALSE, 0);
  GtkWidget *col_2 = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), fixed);
  OffsetGtkBox(fixed, hbox);
  gtk_box_pack_start(GTK_BOX(hbox), col_0, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(hbox), col_1, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(hbox), col_2, FALSE, FALSE, 2);

  MonitorConfig mconfig = {
      {
          // Configure indicators.
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFlightPlan},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFlightMode},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFlightModeGates},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateControlTime},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateJoystick},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateWind},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEmpty},

          {col_0, XY_PLOT_TYPE, (UpdateFuncType)&UpdateCrosswindCircle},

          // Configure charts.
          {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateWingPos},
          {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateTension},
          {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdatePitotAngles},
          {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateAirspeed},
          {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateCrosswindDeltas},

          // Configure X-Y plots.
          {col_2, XY_PLOT_TYPE, (UpdateFuncType)&UpdatePowerCurve},
          {col_2, POLAR_PLOT_TYPE, (UpdateFuncType)&UpdatePolarTransOutThresh},
      },
      {NULL}};
  *mconfig_out = mconfig;
}

void ConfigEstimatorMonitor(GtkWidget *window, MonitorConfig *mconfig_out) {
  GtkWidget *fixed = gtk_fixed_new();
  GtkWidget *hbox = gtk_hbox_new(FALSE, 0);
  GtkWidget *col_0 = gtk_vbox_new(FALSE, 0);
  GtkWidget *col_1 = gtk_vbox_new(FALSE, 0);
  GtkWidget *col_2 = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), fixed);
  OffsetGtkBox(fixed, hbox);
  gtk_box_pack_start(GTK_BOX(hbox), col_0, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(hbox), col_1, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(hbox), col_2, FALSE, FALSE, 2);

  MonitorConfig mconfig = {
      {
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFlightMode},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateControllerTiming},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateThrottle},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingImu},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEstimatorGyroBias},
          {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateHoverAngles},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingRedundantImus},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEstimatorAccBDiff},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEstimatorGyroDiff},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateEstimatorMagnetometerDiff},
          {col_0, INDICATOR_CHART_TYPE,
           (UpdateFuncType)&UpdateEstimatorAttitudeDiff},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdDisabled},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdAllActive},
          {col_1, INDICATOR_CHART_TYPE,
           (UpdateFuncType)&UpdateEstimatorTetherForce},
          {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateWingPos},
          {col_1, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateEstimatorCurrentGpsReceiver},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateEstimatorGpsDiff},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingWingGpsA},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateWingGpsACn0},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateWingGpsASolutionType},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateWingGpsASigmas},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingWingGpsB},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateWingGpsBCn0},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateWingGpsBSolutionType},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateWingGpsBSigmas},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingGsGps},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateGsGpsCn0},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateGsGpsPosVelType},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateGsGpsSigmas},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateEstimatorGsPosEcef},
          {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateGsGpsToWingDist},
          {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingPlatform},
          {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateEstimatorGsgBias},
          {col_2, XY_PLOT_TYPE, (UpdateFuncType)&UpdatePerchPosition},
          {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdatePerchAzi},
      },
      {NULL}};
  *mconfig_out = mconfig;
}

void ConfigFaultDetectionMonitor(GtkWidget *window,
                                 MonitorConfig *mconfig_out) {
  GtkWidget *fixed = gtk_fixed_new();
  GtkWidget *hbox = gtk_hbox_new(FALSE, 0);
  GtkWidget *col_0 = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), fixed);
  OffsetGtkBox(fixed, hbox);
  gtk_box_pack_start(GTK_BOX(hbox), col_0, FALSE, FALSE, 20);

  MonitorConfig mconfig = {
      {{col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdJoystick},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdGsgA},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdGsgB},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdGps},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdPerchAziA},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdPerchAziB},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdLevelwindEleA},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdLevelwindEleB},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdImuA},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdImuB},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdImuC},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdLoadcells},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdPitotHighSpeed},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdPitotLowSpeed},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdGsGps},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdProximitySensor},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdWindSensor},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdGsCompass},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdWinch},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdMotors},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdServos}},
      {NULL}};
  *mconfig_out = mconfig;
}

void ConfigHoverMonitor(GtkWidget *window, MonitorConfig *mconfig_out) {
  GtkWidget *fixed = gtk_fixed_new();
  GtkWidget *vbox = gtk_vbox_new(FALSE, 0);
  GtkWidget *row_0 = gtk_hbox_new(FALSE, 0);
  GtkWidget *row_1 = gtk_hbox_new(FALSE, 0);
  GtkWidget *col_0 = gtk_vbox_new(FALSE, 0);
  GtkWidget *col_1 = gtk_vbox_new(FALSE, 0);
  GtkWidget *col_2 = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), fixed);
  OffsetGtkBox(fixed, vbox);
  gtk_box_pack_start(GTK_BOX(vbox), row_0, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(row_0), col_0, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(row_0), col_1, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(row_0), col_2, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(vbox), row_1, FALSE, FALSE, 2);

  MonitorConfig mconfig = {
      {// Configure indicators.
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFlightPlan},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFlightMode},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFlightModeGates},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateControlTime},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateControllerTiming},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateJoystick},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateWind},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEmpty},

       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHoverGainRampScale},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHoverAngleCommand},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHoverThrustMoment},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEmpty},

       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdatePerchAzi},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEmpty},

       // Configure charts.
       {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateHoverPathErrors},
       {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateHoverTension},
       {col_1, INDICATOR_CHART_TYPE,
        (UpdateFuncType)&UpdateHoverPositionErrors},
       {col_1, INDICATOR_CHART_TYPE,
        (UpdateFuncType)&UpdateHoverVelocityErrors},
       {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateHoverAngles},

       // Configure fault indicators.
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdDisabled},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateFdAllActive},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateTensionComponents},

       // Configure X-Y plots.
       {row_1, XY_PLOT_TYPE, (UpdateFuncType)&UpdateRotorPitchYaw},
       {row_1, XY_PLOT_TYPE, (UpdateFuncType)&UpdateConstraintWindow},
       {row_1, XY_PLOT_TYPE, (UpdateFuncType)&UpdateHoverElevationCommand},
       {row_1, XY_PLOT_TYPE, (UpdateFuncType)&UpdateWinchSpeedPos}},
      {NULL}};
  *mconfig_out = mconfig;
}

void ConfigManualMonitor(GtkWidget *window, MonitorConfig *mconfig_out) {
  GtkWidget *fixed = gtk_fixed_new();
  GtkWidget *vbox = gtk_vbox_new(FALSE, 0);
  GtkWidget *row_0 = gtk_hbox_new(FALSE, 0);
  GtkWidget *col_0 = gtk_vbox_new(FALSE, 0);
  GtkWidget *col_1 = gtk_vbox_new(FALSE, 0);
  GtkWidget *row_1 = gtk_hbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), fixed);

  // Offset the monitors so they display on the large screen monitors
  // in the command trailer, without being cut-off at the edges.
  gtk_fixed_put(GTK_FIXED(fixed), vbox, 20, 20);
  gtk_box_pack_start(GTK_BOX(vbox), row_0, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(row_0), col_0, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(row_0), col_1, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(vbox), row_1, FALSE, FALSE, 2);

  MonitorConfig mconfig = {
      {// Configure indicators.
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateFlightMode},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateManualState},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateAutoglide},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateJoystick},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateManualFlaps},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateApparentWind},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeightAgl},

       {col_0, XY_PLOT_TYPE, (UpdateFuncType)&UpdateGlideslope},
       {row_1, XY_PLOT_TYPE, (UpdateFuncType)&UpdateManualPosition},
       {row_1, XY_PLOT_TYPE, (UpdateFuncType)&UpdateAeroAngles}},
      {NULL}};
  *mconfig_out = mconfig;
}
