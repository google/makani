# Makani

Makani was a project to develop a commercial-scale airborne wind turbine, culminating in
a flight test of the Makani M600 off the coast of Norway. All Makani software has now been
open-sourced. This repository contains the working Makani flight simulator, controller (autopilot),
visualizer, and command center flight monitoring tools. Additionally, almost all avionics firmware
is also included, albeit potentially not in a buildable state, due to the removal of some
third-party proprietary code. We hope that this code will be inspirational and useful to the
kite-based windpower and wider communities.

## A brief tour of the code

 * `analysis` - Miscellaneous analysis scripts. The most important files here
   are `analysis/control/crosswind.py`, which generates the gains for the
   crosswind inner loop controller, and
   `analysis/control/generate_hover_controllers.m`, which generates the gains
   for the hover controller.
 * `avionics` - Firmware for the winch, ground station, motors, network, servos, batteries,
   network switches, GPS, strobe lights, etc.
 * `control` - The hover, transition-in, crosswind, and off-tether flight controllers.
 * `common`
 * `config` - A Python-based configuration system produces a JSON dict
   specifying all system, controller, and simulation parameters. This is
   translated into a read-only C structure at compile-time.
 * `database` - Aero tables, Pitot calibration tables, etc.
 * `documentation`
 * `vis` - OpenGL-based visualizer that depicts the state of the system during
   simulation and real flight.

## How to build.

This code base was originally designed to run on Linux systems running the Debian Stretch
distribution. For the convenience of future users, we are shipping this open source release
with a script to create the necessary environment within Docker.

## How to read the logs.

The Control Telemetry Users' guide, included as a PDF with this distribution, gives a full
description of log file data structures. It also gives examples for how to load and plot log data
using Python or MATLAB.