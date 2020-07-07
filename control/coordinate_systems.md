Notation
========


Standard Rotation Matrices
--------------------------

R_x(a) := [1       0       0
           0       cos(a)  sin(a)
           0       -sin(a) cos(a)]

R_y(a) := [cos(a)  0       -sin(a)]
           0       1       0
           sin(a)  0       cos(a)]

R_z(a) := [cos(a)  sin(a) 0
           -sin(a) cos(a) 0
           0       0      1]


Coordinate Systems
==========================


Earth-centered Earth-fixed (ecef) Coordinates
---------------------------------------------

Standard definition based on WGS84 (see Chapter 2 of [1]).


North-east-down (ned) Coordinates
---------------------------------

The origin is at the center of the top of the slewing bearing that
attaches the tower to the perch platform, in its nominal position
and attitude.  For offshore configurations, the nominal position and
attitude of the platform is the one corresponding to the buoy floating
vertically, above its anchor point, at its nominal draft.
The ned coordinate axes are defined in the standard fashion using
WGS84 (see Chapter 2 of [1]).


Ground (g) Coordinates
----------------------

The ground coordinates are defined relative to NED.  The origin is the
same as the NED coordinates.  The z-axis points down, and the x-axis
points along the ground frame heading direction (see ground frame
heading in the Conventions below).


Body (b) Coordinates
--------------------

Reference coordinates defined by the airframe team's CAD models.
Origin located at the center of the main wing, at quarter-chord.
The x-axis points toward the nominal flight direction, the y-axis
points starboard.


Fly (f) Coordinates
-------------------

The origin is the same as the Body (b) origin. The x-axis points
into the kite's apparent wind in an "upwind" sense. The y-axis is
tangent to the tether sphere in a direction defined by the cross
product of the x-axis with a vector that points from the tether
anchor to the kite. The z-axis is defined in a right-handed sense
and generally points toward the inside of the tether sphere.

The "Airspeed Roll" angle is defined as the first Euler angle in an
X-Z-Y sequence that goes from the Fly (f) Coordinates to the Body
(b) Coordinates: Begin in Fly (f): x-axis along airspeed, y-axis
tangent to sphere. First roll about the airspeed vector by the
"airspeed roll angle". Next yaw about the new intermediate z-axis
by -sideslip. Finally pitch about the Body (b) y-axis by alpha.
See go/crosswind-kinematics for a detailed description.


Vessel (v) Coordinates
----------------------

### Onshore configurations

The vessel frame is identical to the ground frame.

### Offshore configurations

The vessel frame is fixed to the floating buoy. The origin of this
frame is at the center of the top of the slewing bearing that
attaches the tower to the perch platform, and it is generally not
coincident with the origin of the ground frame.  The axes are derived
from the ground frame by the buoy roll, pitch and yaw angles. When all
the buoy angles are zero, the axes of the vessel frame are aligned
with the axes of the ground frame.


Perch or Platform (p) Coordinates
---------------------------------

### Ground station v1: p="Perch"

The perch coordinates are defined relative to the vessel coordinates,
and come from the ground station team's CAD models.  The origin is at
the origin of the vessel frame.  The z-axis points along the vessel
frame z-axis and the x-axis points toward the perch panels. The perch
rotates by approximately 180 degrees from hover to crosswind (i.e. the
x-axis points approximately upwind during crosswind). See also the
Perch Azimuth in the conventions below.

### Ground station v2: p="Platform"

The platform frame shares its origin with the vessel frame. It is
rotated about the vessel z-axis by the platform azimuth angle. The
platform rotates by approximately 90 degrees between hover and
crosswind. The y-axis points toward the perch panels. The x-axis
points upwind during crosswind flight.


Levelwind (lw) Coordinates and Levelwind Elevation [rad] (levelwind_ele)
------------------------------------------------------------------------

The levelwind coordinates are defined relative to perch coordinates.
The z coordinate of the origin is the height of the axis of rotation
of the levelwind (the origin changes as the tether is wound on to the
drum).  The origin's x and y coordinates are defined by the
intersection of the axis-of-rotation of the levelwind and the line
tangent to the levelwind (at zero elevation) that passes through the
the perch axis of rotation:

                           ^ y_lw
                          /
                 ( lw )\ /
  levelwind             x   ^ x_p
  axis of rotation  -> / \  |
                          \ |
                           \|
                            x--------> y_p

The levelwind y-axis is aligned with the levelwind axis of rotation
(which is parallel to the x_p-y_p plane) and its sense is defined so
that it has a positive dot product with y_p.  When levelwind elevation
is zero, the levelwind x-axis points along the perch z-axis.  As the
levelwind elevation increases, the levelwind x-axis and z-axis rotate
around the levelwind rotation axis with the z-axis initially rotating
to align with the perch z-axis.

Winch Drum (wd) Coordinates and Drum Angle (drum_angle)
-------------------------------------------------------

### Ground station v1

The winch drum coordinates are defined relative to perch coordinates.
The origin is at the intersection of the drum rotation axis and the
x_p-y_p plane.  It is assumed the drum rotation axis is parallel to
z_p.  The winch-drum frame z axis is aligned with z_p.  The winch-drum
frame x axis is aligned with x_p when drum_angle is zero, and rotates
toward y_p as the drum_angle increases:

      x---------> x_p
      |\  ,'
      | \'  drum_angle
      |  \
      |   \
      |    v x_wd
      v y_p

### Ground station v2

This is the drum coordinate system as described in
go/makani-gs02-coordinates. It is a child of the platform frame.

The origin is provided by the ground station team's CAD models. At zero
drum angle, there is no rotation relative to the platform frame.

The drum angle describes a rotation about the wd-frame x-axis. The drum
angle increases during reel-out and decreases during reel-in. The
maximum drum angle is zero, at which point the tether is anchored to
the GSG, and the GSG is at the lowest point of the drum.


Ground-Side-Gimbal (gsg) Coordinates, GSG Elevation and GSG Azimuth
-------------------------------------------------------------------

The ground-side-gimbal coordinate origin is located on the GSG's
azimuth rotation axis at the point closest to the GSG's termination
rotation axis.  When yoke and termination angles are zero, the
ground-side-gimbal x-axis points along negative x_wd, and its z-axis
is aligned with z_wd.  The gsg to wd rotation is given by:

  R_wd2gsg = R_y(gsg_ele) R_z(gsg_azi + pi).

In particular, as the elevation angle increases from zero the gsg
starts to point up.

                 ^ y_wd
                 |
                 |
                 |
  x_wd <---------+-----
                 |\  ,'
                 | \'  gsg_azi
                 |  \
                 |   \
                 |    v x_gsg


Hover (h) coordinates
---------------------

The origin of the hover coordinates is at the wing center, coincident
with the origin of the body coordinates. The x-axis points straight
up, along the negative z-axis of the ground frame. The y-axis points
along the cross product of the wing position and the ground z axis,
increasing toward the starboard tip of the wing. The z-axis points
towards the ground-station, parallel to the ground xy-plane.


Crosswind (cw) coordinates
--------------------------
The "crosswind" coordinate system (cw) is used to describe the plane
the wing flies circles in during crosswind flight. The plane may be
moved around to accommodate shifting wind directions or various levels
of power generation and tension control. It is centered about the
center of the crosswind path, and defined relative to its parent
"ground" (g) coordinate system as:

   x: Normal to the crosswind flight plane and points towards the
      ground-station.
   y: Horizontal in crosswind flight plane, forming a right-handed
      orthogonal coordinate system.
   z: Positive downward in the crosswind flight plane.


Crosswind tangent (t) coordinates
---------------------------------
The origin of the crosswind tangent coordinate system (t) is
coincident with the origin of the kite body coordinate system (b). Two
axes lie in the crosswind plane, one of which is tangential to the
crosswind circle, pointing in the direction of flight.

   x-y plane: Co-planar with the crosswind plane.
   x: Tangential to the crosswind circle, directed in the direction of flight.
   y: Normal to the crosswind circle, directed outside of the loop.
   z: Normal to the crosswind plane, directed toward the ground station.


Mean wind (mw) coordinates
--------------------------

The origin of the mean wind frame is (0, 0, ground_z) in the ground
frame. In other words, it is at ground level directly below the ground
frame origin.

The x-axis (u-axis) is in the direction of the mean wind velocity. The
z-axis (w-axis) points "up", opposite the ground frame z-axis. The
y-axis (v-axis) completes a right-handed frame.

The mean wind direction is the angle about the z_g-axis that the
negative mean wind velocity makes with the x_g-axis.


                ^ y_g   --> y_mw
                |    --/
                | --/
                |/
  x_g <---------+-----
                |\  ,'
                | \'  mean_wind_dir
                |  \
                |   \
                |    v x_mw



Conventions
===========


Ground Frame Heading [rad] (g_heading)
--------------------------------------

The ground frame heading lies in [0, 2*pi).  When the ground frame
heading is zero, the ground frame x-axis points north.  As the
ground frame heading increases, the x-axis rotates to point east.

      x---------> x_ned (north)
      |\  ,'
      | \'  g_heading
      |  \
      |   \
      |    v x_g
      v y_ned (east)


Perch Azimuth [rad] (perch_azi)
-------------------------------

The perch azimuth lies in [-pi, pi).  When the azimuth is zero, the
perch x-axis points along the vessel frame x-axis.  As the perch
azimuth increases, the perch x-axis rotates to point along the ground
frame y-axis.


      x---------> x_g
      |\  ,'
      | \'  perch_azi
      |  \
      |   \
      |    v x_p
      v y_g


Winch Position [m] (winch_pos) and Payout [m]
---------------------------------------------

The winch position is the drum angle times the drum radius.  The drum
angle is zero when the winch drum is fully rotated into crosswind
position (1).  The drum angle rotates half a rotation before the
levelwind engages, and then continues to rotate winding the tether
onto the drum until the wing is perched.  In the current
configuration, these rotations decrease the drum angle and the winch
position.

   Flight Mode       |  drum_angle                  | winch_pos
  -------------------+------------------------------+----------------------------
   Crosswind         |  0.0                         |  0.0
   Engage/Disengage  |  -pi                         | -pi * r_drum
   Perched (approx.) |  -tether_length/r_drum - pi  | -tether_length - pi*r_drum

The perched lengths are only approximate due to variation in how the
tether lies in the drum.  Payout measures the difference in winch
position since the last time the wing was on the perch.

   Flight Mode         | payout
  ---------------------+----------------------------
   Perched             | 0.0
   Crosswind (approx.) | tether_length + pi * r_drum

(1) The drum angle being zero is an important pre-condition to the
ground station supporting crosswind loads.


Ground-side-gimbal Elevation [rad] and Azimuth [rad]
----------------------------------------------------

See the defintion of the GSG coordinates.  The GSG elevation lies in
the range [-pi/2.0, pi/2.0].  The GSG azimuth lies in the range
[-pi, pi).


TODO: Wind direction [rad] (wind_dir).
TODO: Flap deflections [rad] (flaps).
TODO: Tether roll [rad] and pitch [rad].


References
==========

[1] Groves, Paul D. Principles of GNSS, Inertial, and Multisensor
      Integrated Navigation Systems. 2nd Ed.  GNSS Technology and
      Applications Series. Boston: Artech 2013.
