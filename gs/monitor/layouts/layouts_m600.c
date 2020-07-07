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

#include "gs/monitor/layouts/layouts_m600.h"
#include "gs/monitor/layouts/layouts_util.h"

#include <gtk/gtk.h>

#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "common/macros.h"
#include "gs/monitor/charts/charts_comms.h"
#include "gs/monitor/charts/charts_controller.h"
#include "gs/monitor/charts/charts_m600_motors.h"
#include "gs/monitor/charts/charts_m600_servos.h"
#include "gs/monitor/charts/charts_sensors.h"
#include "gs/monitor/indicators/indicators_arbitration.h"
#include "gs/monitor/indicators/indicators_controller.h"
#include "gs/monitor/indicators/indicators_ground_station.h"
#include "gs/monitor/indicators/indicators_headings.h"
#include "gs/monitor/indicators/indicators_m600_comms.h"
#include "gs/monitor/indicators/indicators_m600_gps.h"
#include "gs/monitor/indicators/indicators_m600_motors.h"
#include "gs/monitor/indicators/indicators_m600_servos.h"
#include "gs/monitor/indicators/indicators_m600_status.h"
#include "gs/monitor/indicators/indicators_util.h"
#include "gs/monitor/monitor.h"
#include "gs/monitor/monitor_types.h"
#include "gs/monitor/widgets/heading.h"
#include "gs/monitor/widgets/xy_plot.h"
#include "gs/monitor/xy_plots/xy_plots_controller.h"

static void SetIndicator(GtkWidget *parent, const char *label_str,
                         IndicatorUpdateFunction update_func, int32_t *mindex,
                         MonitorConfig *mconfig) {
  if (0 <= *mindex && *mindex < MONITOR_MAX_WIDGETS) {
    MonitorWidgetConfig mw = {
        .parent = parent,
        .widget_type = INDICATOR_TYPE,
        .update_func = (UpdateFuncType)update_func,
    };
    mconfig->mwconfig[*mindex] = mw;
    mconfig->widgets[*mindex] = indicator_new(label_str);

    Indicator *ind = INDICATOR(mconfig->widgets[*mindex]);
    gtk_widget_set_size_request(GTK_WIDGET(ind->label), 180, 25);
    gtk_label_set_width_chars(ind->value, 45);
    gtk_label_set_max_width_chars(ind->value, 45);
    ++(*mindex);
  }
}

static void HeadingUpdateFunction(GtkWidget *widget, int32_t init) {
  (void)widget;
  (void)init;
}

static void SetHeading(GtkWidget *parent, const char *label_str,
                       int32_t *mindex, MonitorConfig *mconfig) {
  if (0 <= (*mindex) && (*mindex) < MONITOR_MAX_WIDGETS) {
    MonitorWidgetConfig mw = {
        .parent = parent,
        .widget_type = HEADING_TYPE,
        .update_func = HeadingUpdateFunction,
    };
    mconfig->mwconfig[*mindex] = mw;
    mconfig->widgets[*mindex] = heading_new(label_str);
    ++(*mindex);
  }
}

static GtkWidget *GetColumn(int32_t num_columns, int32_t rows_per_column,
                            int32_t mindex, GtkWidget **columns) {
  int32_t col = mindex / rows_per_column;
  if (col >= num_columns) {
    col = num_columns - 1;
  }
  return columns[col];
}

void ConfigArbitrationMonitor(GtkWidget *window, MonitorConfig *mconfig_out) {
  GtkWidget *fixed = gtk_fixed_new();
  GtkWidget *hbox = gtk_hbox_new(FALSE, 0);
  GtkWidget *col_0 = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), fixed);
  OffsetGtkBox(fixed, hbox);
  gtk_box_pack_start(GTK_BOX(hbox), col_0, FALSE, FALSE, 20);

  MonitorConfig mconfig = {
      {
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingArbitration},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateArbitrationMonitor},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateArbitrationControllers},
          // TODO: Add motor and servo arbitration monitors.
      },
      {NULL}};
  *mconfig_out = mconfig;
}

void ConfigFlightMonitor(GtkWidget *window, MonitorConfig *mconfig_out) {
  GtkWidget *fixed = gtk_fixed_new();
  GtkWidget *hbox = gtk_hbox_new(FALSE, 0);
  GtkWidget *col_0 = gtk_vbox_new(FALSE, 0);
  GtkWidget *col_1 = gtk_vbox_new(FALSE, 0);
  GtkWidget *col_2 = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), fixed);
  OffsetGtkBox(fixed, hbox);
  gtk_box_pack_start(GTK_BOX(hbox), col_0, FALSE, FALSE, 20);
  gtk_box_pack_start(GTK_BOX(hbox), col_1, FALSE, FALSE, 20);
  gtk_box_pack_start(GTK_BOX(hbox), col_2, FALSE, FALSE, 20);

  // Pre/Flight status.
  // TODO: Add summary indicators for the overall status of:
  // 1. Network.
  // 2. Autochecks.
  // 3. Fault Detection.
  // 4. Estimator.
  // 5. Hover controller.
  MonitorConfig mconfig = {
      {// Comms.
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingAio},
       {col_0, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateAioStatusUpdatedCoreSwitchGs},
       {col_0, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateAioStatusUpdatedCoreSwitchWing},
       {col_0, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateAioStatusUpdatedControllers},
       {col_0, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateAioStatusUpdatedFlightComputers},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateAioStatusUpdatedJoystick},
       // Avionics temperatures and voltages.
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingAvionics},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMaxBoardTemperature},
       // TODO: Update with a summary LV bus state indicator.
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateLvBusTail},
       // Flight Controller.
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingFlightController},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateVersion},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFlightPlan},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFlightMode},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFlightModeGates},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateControlTime},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateControllerTiming},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateJoystick},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHoverGainRampScale},
       // Wind.
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingWind},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateWindSensor},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateWind},
       // Fault detection.
       // TODO: Add summary indicator.
       // Wing state.
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingWingState},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateHoverAngles},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateWingPos},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateAirspeed},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateTension},
       // Wing sensors.
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingWingSensors},
       {col_1, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateEstimatorCurrentGpsReceiver},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingWingGpsA},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateWingGpsACn0},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateWingGpsASolutionType},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateWingGpsASigmas},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingWingGpsB},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateWingGpsBCn0},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateWingGpsBSolutionType},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateWingGpsBSigmas},
       // Ground Station.
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingGroundStation},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateGsGpsCn0},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateGsGpsSigmas},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateGsGpsPosVelType},
       // TODO: Add UpdateGsgElevation and UpdatePerchAzimuthEncoders
       // once flying off a tophat with functional encoders.
       // Winch.
       // TODO: Add winch indicators once flying off a ground station.
       // Servos.
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingServos},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateServosArmedTail},
       {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateServoElePosChart},
       {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateServoRudPosChart},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateServosArmedPort},
       {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateServoPortPosChart},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateServosArmedStarboard},
       {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateServoStarPosChart},
       // Motors.
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorErrors},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorWarnings},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingTopMotors},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorVoltagesTop},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorsArmedTop},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorTempsTop},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorCapTempsTop},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorBoardTempsTop},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorControllerTempsTop},
       {col_2, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateMotorSpeedsTop},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingBottomMotors},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorVoltagesBottom},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorsArmedBottom},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorTempsBottom},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorCapTempsBottom},
       {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorBoardTempsBottom},
       {col_2, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateMotorControllerTempsBottom},
       {col_2, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateMotorSpeedsBottom},
       {col_2, XY_PLOT_TYPE, (UpdateFuncType)&UpdateConstraintWindow}},
      {NULL}};
  *mconfig_out = mconfig;
}

void ConfigGroundStationMonitor(GtkWidget *window, MonitorConfig *mconfig_out) {
  GtkWidget *fixed = gtk_fixed_new();
  GtkWidget *hbox = gtk_hbox_new(FALSE, 0);
  GtkWidget *col_0 = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), fixed);
  OffsetGtkBox(fixed, hbox);
  gtk_box_pack_start(GTK_BOX(hbox), col_0, FALSE, FALSE, 20);

  MonitorConfig mconfig = {
      {{col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingAio},
       {col_0, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateAioStatusUpdatedCoreSwitchGs},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateAioStatusUpdatedGsGps},
       {col_0, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateAioStatusUpdatedPlatformSensors},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateAioStatusUpdatedWinchPlc},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEmpty},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateWindSensor},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateWeather},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateAirDensity},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEmpty},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingGsGps},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateGsGpsCn0},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateGsGpsSigmas},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateGsGpsPosVelType},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateGsGpsPos},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingGsCompass},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateGsGpsCompassHeading},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEmpty},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateLevelwindElevation},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdatePerchAzimuthEncoders},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateGsgAzimuth},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateGsgElevation},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEmpty},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateWinchArmed},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdatePlcStatus},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateDrumState},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateWinchProximitySensor}},
      {NULL}};
  *mconfig_out = mconfig;
}

void ConfigMotorsMonitor(GtkWidget *window, MonitorConfig *mconfig_out) {
  GtkWidget *fixed = gtk_fixed_new();
  GtkWidget *hbox = gtk_hbox_new(FALSE, 0);
  GtkWidget *col_0 = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), fixed);
  OffsetGtkBox(fixed, hbox);
  gtk_box_pack_start(GTK_BOX(hbox), col_0, FALSE, FALSE, 20);

  MonitorConfig mconfig = {
      {{col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingAio},
       {col_0, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateAioStatusUpdatedMotorsTop},
       {col_0, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateAioStatusUpdatedMotorsBottom},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEmpty},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorErrors},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorWarnings},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateStackBusPower},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingTopMotors},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorVoltagesTop},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorsArmedTop},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorTempsTop},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorCapTempsTop},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorBoardTempsTop},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorControllerTempsTop},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateMotorSpeedsTop},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingBottomMotors},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorVoltagesBottom},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorsArmedBottom},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorTempsBottom},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorCapTempsBottom},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorBoardTempsBottom},
       {col_0, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateMotorControllerTempsBottom},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateMotorSpeedsBottom},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFmmProxy}},
      {NULL}};
  *mconfig_out = mconfig;
}

void ConfigNetworkMonitor(GtkWidget *window, MonitorConfig *mconfig_out) {
  GtkWidget *fixed = gtk_fixed_new();
  GtkWidget *hbox = gtk_hbox_new(FALSE, 0);
  GtkWidget *col_0 = gtk_vbox_new(FALSE, 0);
  GtkWidget *col_1 = gtk_vbox_new(FALSE, 0);
  GtkWidget *col_2 = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), fixed);
  OffsetGtkBox(fixed, hbox);
  gtk_box_pack_start(GTK_BOX(hbox), col_0, FALSE, FALSE, 20);
  gtk_box_pack_start(GTK_BOX(hbox), col_1, FALSE, FALSE, 20);
  gtk_box_pack_start(GTK_BOX(hbox), col_2, FALSE, FALSE, 20);

  MonitorConfig mconfig = {
      {
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingAio},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedSelfTest},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedJoystick},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedCoreSwitchGs},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedCoreSwitchWing},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedControllers},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedFlightComputers},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedLoadcells},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedMotorsTop},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedMotorsBottom},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedServosPort},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedServosStar},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedServosTail},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateAioStatusUpdatedGsGps},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedPlatformSensors},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedDrumSensors},
          {col_0, INDICATOR_TYPE,
           (UpdateFuncType)&UpdateAioStatusUpdatedWinchPlc},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEmpty},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorsNetworkAErrors},
          {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateMotorsNetworkBErrors},
          {col_0, INDICATOR_CHART_TYPE,
           (UpdateFuncType)&UpdateWingCsPacketsDropped},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateCsGsAPortTraffic},
          {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateCsGsBPortTraffic},
          {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateCsAPortTraffic},
          {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateCsBPortTraffic},
      },
      {NULL}};
  *mconfig_out = mconfig;
}

void ConfigPilotMonitor(GtkWidget *window, MonitorConfig *mconfig_out) {
  GtkWidget *fixed = gtk_fixed_new();
  GtkWidget *hbox = gtk_hbox_new(FALSE, 0);
  GtkWidget *col_0 = gtk_vbox_new(FALSE, 0);
  GtkWidget *col_1 = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), fixed);
  OffsetGtkBox(fixed, hbox);
  gtk_box_pack_start(GTK_BOX(hbox), col_0, FALSE, FALSE, 20);
  gtk_box_pack_start(GTK_BOX(hbox), col_1, FALSE, FALSE, 20);

  MonitorConfig mconfig = {
      {// Flight Controller.
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingFlightController},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFlightPlan},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFlightMode},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateFlightModeGates},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateControlTime},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateJoystick},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHoverGainRampScale},
       // Wind.
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingWind},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateWindSensor},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateWind},
       // Wing.
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingWing},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateHoverAngles},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateWingPos},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateAirspeed},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateHoverTension},
       // Controller.
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateHoverAngleCommand},
       {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateHoverThrustMoment},
       {col_1, INDICATOR_CHART_TYPE,
        (UpdateFuncType)&UpdateHoverPositionErrors},
       {col_1, INDICATOR_CHART_TYPE,
        (UpdateFuncType)&UpdateHoverVelocityErrors},
       // Hover envelope.
       {col_1, XY_PLOT_TYPE, (UpdateFuncType)&UpdateConstraintWindow}},
      {NULL}};
  *mconfig_out = mconfig;
}

void ConfigServosMonitor(GtkWidget *window, MonitorConfig *mconfig_out) {
  GtkWidget *fixed = gtk_fixed_new();
  GtkWidget *hbox = gtk_hbox_new(FALSE, 0);
  GtkWidget *col_0 = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), fixed);
  OffsetGtkBox(fixed, hbox);
  gtk_box_pack_start(GTK_BOX(hbox), col_0, FALSE, FALSE, 20);

  MonitorConfig mconfig = {
      {{col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingAio},
       {col_0, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateAioStatusUpdatedServosPort},
       {col_0, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateAioStatusUpdatedServosStar},
       {col_0, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateAioStatusUpdatedServosTail},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEmpty},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateLvBusTail},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateServosArmedTail},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateServoElePosChart},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateServoRudPosChart},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateServosArmedPort},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateServoPortPosChart},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateServosArmedStarboard},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateServoStarPosChart}},
      {NULL}};
  *mconfig_out = mconfig;
}

void ConfigStatusMonitor(GtkWidget *window, MonitorConfig *mconfig) {
  GtkWidget *fixed = gtk_fixed_new();
  GtkWidget *hbox = gtk_hbox_new(FALSE, 0);
  GtkWidget *columns[3];  // Array size defines number of columns.

  // Initialize configuration.
  memset(mconfig, 0, sizeof(*mconfig));
  int32_t mindex = 0;

  // Programmatically determine layout according to the number of nodes.
  for (int32_t i = 0; i < ARRAYSIZE(columns); ++i) {
    columns[i] = gtk_vbox_new(FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), columns[i], FALSE, FALSE, 20);
  }
  gtk_container_add(GTK_CONTAINER(window), fixed);
  OffsetGtkBox(fixed, hbox);

  // Determine number of rows per column.
  int32_t rows_per_column = 12;  // Fudge factor to account for headings.
  for (AioNode n = 0; n < kNumAioNodes; ++n) {
    if (IsRemoteCommandNode(n) || IsGroundStationNode(n) || IsWingNode(n)) {
      ++rows_per_column;
    }
  }
  rows_per_column += ARRAYSIZE(columns) / 2;  // Handle odd number of columns.
  rows_per_column /= ARRAYSIZE(columns);

  GtkWidget *col =
      GetColumn(ARRAYSIZE(columns), rows_per_column, mindex, columns);

  // System status.

  SetHeading(col, "System status", &mindex, mconfig);
  SetIndicator(col, "Asserts", &UpdateSystemAssertStatus, &mindex, mconfig);
  SetIndicator(col, "Git hash", &UpdateSystemBuildInfoStatus, &mindex, mconfig);
  SetIndicator(col, "Monitor", &UpdateSystemMonitorStatus, &mindex, mconfig);

  // Communications status.

  SetHeading(col, "Communication link status", &mindex, mconfig);
  SetIndicator(col, "PoF", &UpdateCommsStatusPoF, &mindex, mconfig);
  SetIndicator(col, "EoP", &UpdateCommsStatusEoP, &mindex, mconfig);
  SetIndicator(col, "WiFi", &UpdateCommsStatusWiFi, &mindex, mconfig);
  SetIndicator(col, "Joystick", &UpdateCommsStatusJoystick, &mindex, mconfig);
  SetIndicator(col, "Long range", &UpdateCommsStatusLongRange, &mindex,
               mconfig);

  // Remote command nodes.

  SetHeading(col, "Remote command center", &mindex, mconfig);
  for (AioNode n = 0; n < kNumAioNodes; ++n) {
    if (IsRemoteCommandNode(n)) {
      SetIndicator(col, AioNodeToShortString(n), &UpdateNodeStatus, &mindex,
                   mconfig);
    }
    col = GetColumn(ARRAYSIZE(columns), rows_per_column, mindex, columns);
  }

  // Ground station nodes.

  SetHeading(col, "Ground station core switches", &mindex, mconfig);
  for (AioNode n = 0; n < kNumAioNodes; ++n) {
    if (IsGroundStationNode(n) && IsCoreSwitchNode(n)) {
      SetIndicator(col, AioNodeToShortString(n), &UpdateNodeStatus, &mindex,
                   mconfig);
    }
  }
  col = GetColumn(ARRAYSIZE(columns), rows_per_column, mindex, columns);

  SetHeading(col, "Ground station nodes", &mindex, mconfig);
  for (AioNode n = 0; n < kNumAioNodes; ++n) {
    if (IsGroundStationNode(n) && !IsCoreSwitchNode(n)) {
      SetIndicator(col, AioNodeToShortString(n), &UpdateNodeStatus, &mindex,
                   mconfig);
    }
    col = GetColumn(ARRAYSIZE(columns), rows_per_column, mindex, columns);
  }

  // Wing nodes.

  SetHeading(col, "Wing core switches", &mindex, mconfig);
  for (AioNode n = 0; n < kNumAioNodes; ++n) {
    if (IsWingNode(n) && IsCoreSwitchNode(n)) {
      SetIndicator(col, AioNodeToShortString(n), &UpdateNodeStatus, &mindex,
                   mconfig);
    }
  }
  col = GetColumn(ARRAYSIZE(columns), rows_per_column, mindex, columns);

  SetHeading(col, "Wing nodes", &mindex, mconfig);
  for (AioNode n = 0; n < kNumAioNodes; ++n) {
    if (IsWingNode(n) && !IsCoreSwitchNode(n)) {
      SetIndicator(col, AioNodeToShortString(n), &UpdateNodeStatus, &mindex,
                   mconfig);
    }
    col = GetColumn(ARRAYSIZE(columns), rows_per_column, mindex, columns);
  }
}

void ConfigWingSensorsMonitor(GtkWidget *window, MonitorConfig *mconfig_out) {
  GtkWidget *fixed = gtk_fixed_new();
  GtkWidget *hbox = gtk_hbox_new(FALSE, 0);
  GtkWidget *col_0 = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), fixed);
  OffsetGtkBox(fixed, hbox);
  gtk_box_pack_start(GTK_BOX(hbox), col_0, FALSE, FALSE, 20);

  MonitorConfig mconfig = {
      {{col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateHeadingAio},
       {col_0, INDICATOR_TYPE,
        (UpdateFuncType)&UpdateAioStatusUpdatedLoadcells},
       {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEmpty},
       {col_0, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateLoadcellsPort},
       {col_0, INDICATOR_CHART_TYPE,
        (UpdateFuncType)&UpdateLoadcellsStarboard}},
      {NULL}};
  *mconfig_out = mconfig;
}
