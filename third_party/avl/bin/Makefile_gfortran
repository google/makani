#
# Makefile for AVL
#

BINDIR = /home/codes/bin/
INSTALLCMD = install -s

SRC = ../src

PROGS = avl
AOBJS = avl.o aoper.o amode.o amass.o asetup.o \
amake.o ainput.o aoutput.o aero.o atrim.o atpforc.o \
aic.o cdcl.o airutil.o autil.o \
aplotvl.o aplottp.o aplotmd.o limits.o \
pltops.o hidden.o \
plsubs.o \
userio.o plutil.o arrow3d.o getvm.o \
matrix.o spline.o sgutil.o \
second.o

PLTOBJ = ../plotlib/libPlt.a
EIGOBJ = ../eispack/eispack.a
## EIGOBJ = eispack.o

###================================================
###  Default compilers and flags
FC = f77
FFLAGS = -O
PLTLIB = -lX11
FTNLIB =

#Subroutine SECOND source file
SECOND = second_g77.f

##--------------------------

# Uncomment flags for desired machine...

##--------------------------
### DEC Alpha with OSF and DEC f77/f90 compiler
#FC = f77
#FFLAGS  = -fast -O4 -tune host
##--------------------------
### SGI setup
#FC = f77
#FFLAGS  = -O2
##--------------------------
##  Uncomment for RS/6000
#FFLAGS  = -O -qextname
##--------------------------
##  Uncomment for HP-9000
#FFLAGS  = -O +ppu
#FTNLIB  = -U77
##--------------------------
### Absoft Linux f77
#FC = f77
#FFLAGS  = -O -f -s -W -B108 -N34
##--------------------------
### f2c/gcc compiler driver 
#FC = fort77
#FFLAGS  =  -O2 -fomit-frame-pointer
##--------------------------
### GNU g77
#FC = g77
#FFLAGS = -O3
##--------------------------
### Intel Fortran Compiler 8.x
FC = ifort
FFLAGS = -O 
PLTLIB = -L/usr/X11R6/lib -lX11
SECOND = second_ifc.f
#FTNLIB = -Vaxlib
##--------------------------
### double-precision option (also need double precision Eispack and libPlt)
FFLAGS = -O -r8
PLTOBJ = ../plotlib/libPltDP.a

##---------------------------------------
##  Uncomment for Mac OSX gfortran
FC = gfortran

ACHK = -fbounds-check -finit-real=inf -ffpe-trap=invalid,zero,denormal
#ACHK =


#FTNLIB = -Vaxlib
FTNLIB = 
SECOND = second_g77.f

DP =
PLTOBJ = ../plotlib/libPlt_gSP.a
EIGOBJ = ../eispack/eispack_gSP.a

#DP = -fdefault-real-8
#PLTOBJ = ../plotlib/libPltDP.a
#EIGOBJ = ../eispack/eispack_gDP.a

FFLAGS = -O -m64 $(ACHK) $(DP)
##---------------------------------------



all:	 $(PROGS)

install: $(PROGS)
	$(INSTALLCMD) $(PROGS) $(BINDIR)

clean: 
	-/bin/rm $(PROGS)
	-/bin/rm *.o

$(SRC)/AVL.INC: $(SRC)/AINDEX.INC
	touch $(SRC)/AVL.INC

avl: $(AOBJS)
	$(FC) -o avl $(AOBJS)  $(ALIBS) $(PLTOBJ) $(EIGOBJ) $(PLTLIB) $(FTNLIB)
	$(INSTALLCMD) avl $(BINDIR)

dtest: dtest.o
	$(FC) -o dtest dtest.o

avl.o: $(SRC)/avl.f $(SRC)/AVL.INC $(SRC)/AVLPLT.INC
	$(FC) -c $(FFLAGS) $(SRC)/avl.f
aoper.o: $(SRC)/aoper.f $(SRC)/AVL.INC $(SRC)/AVLPLT.INC
	$(FC) -c $(FFLAGS) $(SRC)/aoper.f
amode.o: $(SRC)/amode.f $(SRC)/AVL.INC $(SRC)/AVLPLT.INC
	$(FC) -c $(FFLAGS) $(SRC)/amode.f
amass.o: $(SRC)/amass.f $(SRC)/AVL.INC
	$(FC) -c $(FFLAGS) $(SRC)/amass.f
asetup.o: $(SRC)/asetup.f $(SRC)/AVL.INC
	$(FC) -c $(FFLAGS) $(SRC)/asetup.f
cdcl.o: $(SRC)/cdcl.f $(SRC)/AVL.INC
	$(FC) -c $(FFLAGS)  $(SRC)/cdcl.f
amake.o: $(SRC)/amake.f $(SRC)/AVL.INC
	$(FC) -c $(FFLAGS) $(SRC)/amake.f
ainput.o: $(SRC)/ainput.f $(SRC)/AVL.INC
	$(FC) -c $(FFLAGS) $(SRC)/ainput.f
aoutput.o: $(SRC)/aoutput.f $(SRC)/AVL.INC
	$(FC) -c $(FFLAGS) $(SRC)/aoutput.f
aero.o: $(SRC)/aero.f $(SRC)/AVL.INC
	$(FC) -c $(FFLAGS) $(SRC)/aero.f
getvm.o: $(SRC)/getvm.f $(SRC)/AVL.INC
	$(FC) -c $(FFLAGS) $(SRC)/getvm.f
atrim.o: $(SRC)/atrim.f $(SRC)/AVL.INC
	$(FC) -c $(FFLAGS) $(SRC)/atrim.f
atpforc.o: $(SRC)/atpforc.f $(SRC)/AVL.INC
	$(FC) -c $(FFLAGS) $(SRC)/atpforc.f
aplotvl.o: $(SRC)/aplotvl.f $(SRC)/AVL.INC $(SRC)/AVLPLT.INC $(SRC)/MASKS.INC
	$(FC) -c $(FFLAGS) $(SRC)/aplotvl.f
aplottp.o: $(SRC)/aplottp.f $(SRC)/AVL.INC $(SRC)/AVLPLT.INC $(SRC)/MASKS.INC
	$(FC) -c $(FFLAGS) $(SRC)/aplottp.f
aplotmd.o: $(SRC)/aplotmd.f $(SRC)/AVL.INC $(SRC)/AVLPLT.INC $(SRC)/MASKS.INC
	$(FC) -c $(FFLAGS) $(SRC)/aplotmd.f
limits.o: $(SRC)/limits.f $(SRC)/AVL.INC $(SRC)/AVLPLT.INC
	$(FC) -c $(FFLAGS) $(SRC)/limits.f
pltops.o: $(SRC)/pltops.f $(SRC)/AVLPLT.INC
	$(FC) -c $(FFLAGS) $(SRC)/pltops.f
plsubs.o: $(SRC)/plsubs.f $(SRC)/AINDEX.INC
	$(FC) -c $(FFLAGS) $(SRC)/plsubs.f
hidden.o: $(SRC)/hidden.f
	$(FC) -c $(FFLAGS) $(SRC)/hidden.f
matrix.o: $(SRC)/matrix.f
	$(FC) -c $(FFLAGS)  $(SRC)/matrix.f
aic.o: $(SRC)/aic.f
	$(FC) -c $(FFLAGS) $(SRC)/aic.f
userio.o: $(SRC)/userio.f
	$(FC) -c $(FFLAGS) $(SRC)/userio.f
plutil.o: $(SRC)/plutil.f $(SRC)/MASKS.INC
	$(FC) -c $(FFLAGS) $(SRC)/plutil.f
arrow3d.o: $(SRC)/arrow3d.f $(SRC)/MASKS.INC
	$(FC) -c $(FFLAGS) $(SRC)/arrow3d.f

spline.o: $(SRC)/spline.f
	$(FC) -c $(FFLAGS)  $(SRC)/spline.f
sgutil.o: $(SRC)/sgutil.f
	$(FC) -c $(FFLAGS)  $(SRC)/sgutil.f
airutil.o: $(SRC)/airutil.f
	$(FC) -c $(FFLAGS)  $(SRC)/airutil.f
autil.o: $(SRC)/autil.f
	$(FC) -c $(FFLAGS)  $(SRC)/autil.f

dtest.o: $(SRC)/dtest.f
	$(FC) -c $(FFLAGS) $(SRC)/dtest.f

second.o: $(SRC)/$(SECOND)
	cp $(SRC)/$(SECOND) $(SRC)/second.f
	$(FC) -c $(FFLAGS) $(SRC)/second.f
