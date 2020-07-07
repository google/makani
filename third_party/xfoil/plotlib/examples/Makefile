#***********************************************************************
#    Module:  Makefile (examples directory)
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

########################################
# makefile for Xplot11 library examples
########################################


###================================================
### This line includes your compiler/make options
### with definitions for compiler and flags

include ../config.make

###================================================

LIB  = ../$(PLTLIB)

#FC = ifort
#FFLAGS = -r8

PROGS = volts volts_old \
        symbols symbolsall \
	squares squaresdoublebuff \
	spectrum cmap2 cmap3 defmap \
        gridtest zoomtest contest mousexy mousexy2


examples:  $(PROGS)


test:  $(PROGS)

clean:
	-/bin/rm *.o
	-/bin/rm $(PROGS)
	-/bin/rm plot*.ps



#Test routines for package

volts: volts.o 
	$(FC) -o volts volts.o $(LIB) $(LINKLIB)

volts_old: volts_old.o 
	$(FC) -o volts_old volts_old.o $(LIB) $(LINKLIB)

symbols: symbols.o 
	$(FC) -o symbols symbols.o $(LIB) $(LINKLIB)

symbolsall: symbolsall.o 
	$(FC) -o symbolsall symbolsall.o $(LIB) $(LINKLIB)

squares: squares.o
	$(FC) -o squares squares.o $(LIB) $(LINKLIB)

squaresdoublebuff: squaresdoublebuff.o
	$(FC) -o squaresdoublebuff squaresdoublebuff.o $(LIB) $(LINKLIB)

spectrum: spectrum.o 
	$(FC) -o spectrum spectrum.o $(LIB) $(LINKLIB)

cmap2: cmap2.o 
	$(FC) -o cmap2 cmap2.o $(LIB) $(LINKLIB)

cmap3: cmap3.o 
	$(FC) -o cmap3 cmap3.o $(LIB) $(LINKLIB)
 
defmap: defmap.o
	$(FC) -o defmap defmap.o $(LIB) $(LINKLIB)

gridtest: gridtest.o 
	$(FC) -o gridtest gridtest.o $(LIB) $(LINKLIB)

zoomtest: zoomtest.o 
	$(FC) -o zoomtest zoomtest.o $(LIB) $(LINKLIB)

contest: contest.o 
	$(FC) -o contest contest.o $(LIB) $(LINKLIB)

mousexy: mousexy.o 
	$(FC) -o mousexy mousexy.o $(LIB) $(LINKLIB)

mousexy2: mousexy2.o 
	$(FC) -o mousexy2 mousexy2.o $(LIB) $(LINKLIB)

mousexy3: mousexy3.o 
	$(FC) -o mousexy3 mousexy3.o $(LIB) $(LINKLIB)



volts.o: volts.f
	$(FC) -c $(FFLAGS) $<

volts_old.o: volts_old.f
	$(FC) -c $(FFLAGS) $<

symbols.o: symbols.f
	$(FC) -c $(FFLAGS) $<

symbolsall.o: symbolsall.f
	$(FC) -c $(FFLAGS) $<

squares.o: squares.f
	$(FC) -c $(FFLAGS) $<

squaresdoublebuff.o: squaresdoublebuff.f
	$(FC) -c $(FFLAGS) $<

spectrum.o: spectrum.f
	$(FC) -c $(FFLAGS) $<

cmap2.o: cmap2.f
	$(FC) -c $(FFLAGS) $<

cmap3.o: cmap3.f
	$(FC) -c $(FFLAGS) $<

defmap.o: defmap.f
	$(FC) -c $(FFLAGS) $<

gridtest.o: gridtest.f
	$(FC) -c $(FFLAGS) $<

zoomtest.o: zoomtest.f
	$(FC) -c $(FFLAGS) $<

contest.o: contest.f
	$(FC) -c $(FFLAGS) $<

mousexy.o: mousexy.f
	$(FC) -c $(FFLAGS) $<

mousexy2.o: mousexy2.f
	$(FC) -c $(FFLAGS) $<

mousexy3.o: mousexy3.f
	$(FC) -c $(FFLAGS) $<

#May need to specify these on a brain-dead make system
#.f.o:	$(FC) -c $(FFLAGS) $<
#.c.o:	$(CC) -c $(CFLAGS) $<






