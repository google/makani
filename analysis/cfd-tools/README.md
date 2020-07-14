# Overview

This code is used to automate runs of external aerodynamics analyses using
Siemens STAR-CCM+ simulation software. Only the source code files for this
tool are provided.

This README file is by no means comprehensive of what is possible using this
tool and intends only to instruct in its basic usage to help get started at
best. Users are encouraged to understand the code structure and develop
their own approaches to run any analysis.

There are two analysis types:
1. Wind tunnel airfoil analysis
2. Rotor (n-rotor) analysis

Conceptually, the code reads a file from the simulation working directory
called "aero_input.cfd" to read what kind of analysis to run and various
customized settings. Not all options are compatible. Pre-through-post can
generally be set up for a given analysis analysis type. It is recommended
that initial users of this tool attempt to first use it in steps of pre,
mesh, and then a coupled solve/post to assist in case setup and debugging.

# Wind tunnel analyses

A wind tunnel analysis is used for airfoil anaylses.
Wind tunnels can be run in 2D and 3D.

A sample 2D, one domain, two element configuration file is located in
  "aero_input.cfd.airfoil"

Conceptually, the code recognizes n-wind tunnel domains inside a single
simulation. Doing this has pros and cons. A pro is that a single sim
file can contain different flap deflection angles and will improve its
parallel solve efficiency. A major con is that with a large number of wind
tunnel domains (>5), the preprocessing setup becomes extremelly slow (1+ hr)
and the debugging or post-processin of a single simulation file can become
very cumbersome if done manually.

## Wind tunnel domain

A single wind tunnel domain is a CompositePart named WT<Name>, e.g.:  
    WTDomain
    
which requires a Tag named WT<Name>, e.g.:  
    "WTDomain".
    
The Tag must be applied to every CAD body contained within WTDomain, including
the Composite Part. The Tag must also be applied to every Coordinate system
associated with the wind tunnel domain.

A single wind tunnel domain is created with the following objects:

    A CompositePart containing all CAD bodies of interest.

    A wind tunnel domain CAD part:
        Part name:
          <FS Name>

        Surface names:
          000 <Inlet>
          001 <Outlet>
          004 <Symmetries>

    A set of wind tunnel coordinate systems:
        A Cartesian coordinate system named: WTDomain
            The X-axis is pointed at upstream alpha = 0.
            The Y-axis is oriented body down.

    A Cartesian coordinate system named: Body Csys
        The airfoil's total forces and moments are taken about the origin of
        this coordinate system.

    A Cartesian coordinate system named: CAD Body Csys
        This coordinate system keeps track of the absolute zero angle of
        attack.

    An Overset CAD body containing the airfoil or multi-element airfoil of
    interest (in this example there are 3 elements):
        Part name: OS1_OVERSET
        Surface names:
            006 <Overset>
            004 <Symmetries>
            0100 <Element 1>
            0100 <Element 1> TE
            0200 <Element 2>
            0200 <Element 2> TE
            0300 <Element 3>
            0300 <Element 3> TE

    There must be a nested Cartesian coordinate system named "OS1_OVERSET"
    with the z-axis its axis of rotation. The x-axis must be aligned with
    the zero alpha x-axis of the Body Csys coordinate system mentioned above.

    There must also be an equivalent Cartesian coordinate system for each
    element of the airfoil. The origin is where the forces and moment for
    each element is taken. The name of the coordinate system must match the
    name of the airfoil element exactly. For example, in the Surface names
    list above, there would be 3 coordinate systems required:
      Element 1
      Element 2
      Element 3

### Boundary layer probes

    Boundary layer probe CAD parts are typically used in 2D analyses and
    are comprised of CAD sheets that pierce the Z=0 plane and the surface
    of the airfoil at one position. The name of the CAD part must be:  
        BLP_<Name>
        (e.g.: BLP_XC_0p25)

    A boundary layer probe part requires a nested coordinate system under
    the WTDomain coordinate system where the Z-direction is surface normal.
    It must be named with the exact same name as the CAD part, i.e.:  
        BLP_<Name>

### Volume controls
    Any volume control CAD parts:  
        VC_<Name>_<PercentageOfBaseSize>  
        (e.g.: VC_Wake_50.0)

# Rotor analyses

  A rotor analysis may have up to any number of rotors within it.  
  A single rotor requires a CAD body, an interface body, and a
  corresponding coordinate system.

  Rotors are meshed for a constant value of an equivalent wind tunnel alpha and
  are not swept in a single simulation solution.

  A sample dual rotor configuration file is located in
    "aero_input.cfd.dual_rtr"

  A rotor's behavior (reported position, axis of rotation, rotation rate,
  number of blades, and blade radius) is controlled and defined via its
  original CAD coordinate system. A rotor rotates about the Z-coordinate of
  its corresponding coordinate system.

  Modifing the rotor rotation rates in the configuration file will allow the
  user to change the rotor rotation speed between analyses. This is useful when
  trying to start from an already initialized flow field.

  Volume controls are as in the wind tunnel analysis.

  To set up a rotor analysis, these steps can be used as a guide:

      0) Have a freestream domain CAD body:
        Part name: <FS Name>
        Surface names:
          000 <Inlet>
          001 <Outlet>
          004 <Symmetries>

      1) Have a CAD body ready to use as a rotor.
        The blades should be broken out into their own surface.
        The blade trailing edges should be in their own surface.
        Part name: <CAD Body Name>
        Surface names:
          8000 <Blade name>
          8000 <Blade name> TE

      2) Have a cylinder CAD body that encapsulates the rotor.
        Part name: INT_<Blade name>
        Surface name: 009 Sliding

      3) Create a Coordinate System named:
        <CAD Body Name>_n<number_of_blades>_b<blade_radius>_w<rad/s>
        Note: 
          Rotors can be spun in the opposite direction by using a negative
          rad/s value.
