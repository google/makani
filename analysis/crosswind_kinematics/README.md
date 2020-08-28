# Crosswind Kinematics

This folder contains matlab code which implements and tests the kinematic
equations which are needed to compute the complete commanded 6-DOF kite state
from outer loop commands, as documented in the article "Crosswind Kinematics" by
Michael Abraham, included in the Makani technical documentation release.  This
code can be used to compute the consequences of outer loop commands in terms of
the resulting kite motion including angular velocities.

The core algorithm is SphereKinematics.m which computes the complete 6-DOF
aircraft state and also its linear acceleration. The inputs to the algorithm
include the instantaneous commanded kite path (using spherical coordinates), the
commanded angle of attack and sideslip, a roll angle for the kite defined about
the airspeed vector, and the wind.

TestSphereKinematics.m allows the user to see the consequences of a single set
of inputs to the core algorithm.

Test.m generates random smooth trajectories for the kite, puts those
trajectories through the algorithm, and plots the results.

If a path-following controller provides the required roll angle (about the
airspeed vector), this code can compute the resulting aircraft motion which also
meets the alpha and beta commands.
