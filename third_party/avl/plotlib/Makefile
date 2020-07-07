#***********************************************************************
#    Module:  Makefile
# 
#    Copyright (C) 1996 Harold Youngren, Mark Drela 
# 
#    This library is free software; you can redistribute it and/or
#    modify it under the terms of the GNU Library General Public
#    License as published by the Free Software Foundation; either
#    version 2 of the License, or (at your option) any later version.
#
#    This library is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#    Library General Public License for more details.
#
#    You should have received a copy of the GNU Library General Public
#    License along with this library; if not, write to the Free
#    Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
# 
#    Report problems to:    guppy@maine.com 
#                        or drela@mit.edu  
#***********************************************************************


#================================#
# Makefile for Xplot11 library   #
#  edit the config.make file to  #
#  set specific options for your #
#  system                        #
#================================#

# Point to your install directory
#INSTALLDIR = /home/codes/bin
#INSTALLDIR = /usr/local/lib
INSTALLDIR = .

# Use these to set default library name (overridden in config.make file) 
PLTLIB = libPlt.a
#PLTLIB = libPltDP.a


###========================================================
###  Basic plot library object files
OBJ     = plt_base.o plt_font.o plt_util.o plt_color.o \
          set_subs.o gw_subs.o ps_subs.o Xwin2.o
OBJMISC =
OBJ3D   =
OBJOLD  =

###--------------------------------------------------------
###  Uncomment to add the old plot compatibility routines
OBJOLD  = plt_old.o

###--------------------------------------------------------
###  Uncomment to add the primitive 3D-view routines
OBJ3D  = plt_3D.o

###--------------------------------------------------------
###  Uncomment for f77 compiler w/o AND() and RSHIFT/LSHIFT functions.
###   This adds some functions to duplicate these using IAND and ISHFT
###   which often appear in these offending fortran's libraries.
###   The compilers that this has affected include:
###      HPUX f77
###      Absoft f77 on Linux
###
#OBJMISC = util-ops.o


###-------------------------------------------------------------------------
### Set compiler, compiler flags, name of output object library
include ./config.make


###-------------------------------------------------------------------------
### Basic make targets - build library, test programs

$(PLTLIB):  $(OBJ) $(OBJOLD) $(OBJ3D) $(OBJMISC)
	$(AR)     $(PLTLIB) $(OBJ) $(OBJOLD) $(OBJ3D) $(OBJMISC)
	$(RANLIB) $(PLTLIB)

test:  $(PLTLIB)
	(cd examples; make test)


###-------------------------------------------------------------------------
### Utility functions - install the library, clean the directory

install:  $(PLTLIB)
	mv $(PLTLIB) $(INSTALLDIR)
	$(RANLIB)   $(INSTALLDIR)/$(PLTLIB)

clean:
	-/bin/rm $(OBJ) $(OBJOLD) $(OBJ3D) $(OBJMISC)
	-/bin/rm $(PLTLIB)
	-/bin/rm plot*.ps
	(cd examples; make clean)


###-------------------------------------------------------------------------
### compile plot package routines

plt_base.o: plt_base.f pltlib.inc
	$(FC) -c $(FFLAGS)  plt_base.f

plt_color.o: plt_color.f  pltlib.inc
	$(FC) -c $(FFLAGS)  plt_color.f

plt_font.o: plt_font.f CHAR.INC SLAN.INC MATH.INC SYMB.INC
	$(FC) -c $(FFLAGS)  plt_font.f

plt_util.o: plt_util.f
	$(FC) -c $(FFLAGS)  plt_util.f

plt_3D.o: plt_3D.f
	$(FC) -c $(FFLAGS)  plt_3D.f

plt_old.o: plt_old.f pltlib.inc
	$(FC) -c $(FFLAGS)  plt_old.f

set_subs.o: set_subs.f  pltlib.inc
	$(FC) -c $(FFLAGS)  set_subs.f

gw_subs.o: gw_subs.f  pltlib.inc
	$(FC) -c $(FFLAGS)  gw_subs.f

ps_subs.o: ps_subs.f  pltlib.inc
	$(FC) -c $(FFLAGS)  ps_subs.f

util-ops.o: util-ops.f 
	$(FC) -c $(FFLAGS)  util-ops.f

Xwin.o: Xwin.c
	$(CC) -c $(CFLAGS) Xwin.c
Xwin2.o: Xwin2.c
	$(CC) -c $(CFLAGS) Xwin2.c


### May need to specify these on a brain-dead make system
#.f.o:	$(FC) -c $(FFLAGS) $<
#.c.o:	$(CC) -c $(CFLAGS) $<



