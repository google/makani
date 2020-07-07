#***********************************************************************
#    Module:  Makefile (Xplot/sym directory)
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

#Makefile for stroke font creation and editing programs 

FC    = f77
#FC   = g77
FFLAGS = -O1
LIBS   = ../libPlt.a  -lX11

PROGS = symgen  test

all:	$(PROGS)

clean:
	-/bin/rm *.o
	-/bin/rm *.a
	-/bin/rm $(PROGS)


symgen: symgen.o
	$(FC) -o symgen symgen.o $(LIBS)

symgen.o: symgen.f
	$(FC) -c $(FFLAGS) symgen.f

../plt_font.o: ../plt_font.f ../CHAR.INC ../SLAN.INC ../MATH.INC ../SYMB.INC
	$(FC) -c $(FFLAGS) ../plt_font.f
	mv ./plt_font.o ../



test: test.o ../plt_font.o
	$(FC) -o test test.o ../plt_font.o $(LIBS)

test.o: test.f
	$(FC) -c $(FFLAGS) test.f








