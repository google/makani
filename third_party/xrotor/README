
General
-------
XROTOR and its plot library should compile on any Unix system 
with normal Fortran-77, C, and X-Windows support.  So far,
XROTOR has been tested on the following systems:

  DEC-5000
  Alpha
  SGI
* Sun
* RS/6000
* HP-9000
* Pentium/Linux 

The systems marked with "*" have peculiar features which require slight 
modifications to the Makefiles in the plotlib/ and bin/ directories.  
Examine these Makefiles before building the plot library and Xrotor.


Build Sequence
--------------
To install, first build the plot library in  ./plotlib  ...

 % cd plotlib
 % make libPlt.a

Then build the programs in  ./bin  ...

 % make xrotor
 % make jplot


Documentation
-------------
User Guide is in the  xrotor.doc  file.  If impatient, you can just
run XROTOR:

 % xrotor

and size a new rotor from the DESI menu with the INPU command.
The rotor can then be "operated" in the OPER menu.

You can also analyze the sample water prop in the ./runs directory:

 % xrotor ex.prop ex.cases

Various plots, parameter sweeps, etc., can be performed in the OPER menu:

 XROTOR   c>  oper

.OPER   c>  plot 1

.OPER   c>  plot 2

.OPER   c>  plot 3


The file EI.ex can be used for a structural analysis in the BEND menu:

 XROTOR   c>  bend

.BEND   c>  read ex.EI

.BEND   c>  eval

.BEND   c>  plot 1

.BEND   c>  plot 2

If you really wanted a white background for graphics you can do this...
 % setenv XPLOT11_BACKGROUND white         (graphics default to rev. video)


