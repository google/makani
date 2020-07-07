
#=======================================#
# Makefile options for Xplot11 library  #
#   Set up or select a set of compile   #
#   options for your system             # 
#=======================================#


# Set library name 
PLTLIB = libPlt_gfortran.a

# Some fortrans need trailing underscores in C interface symbols (see Xwin.c)
# This should work for most of the "unix" fortran compilers
DEFINE = -DUNDERSCORE

FC = gfortran
CC  = gcc
DP =

FFLAGS  = -O2 $(DP)
CFLAGS  = -O2 $(DEFINE)
AR = ar r
RANLIB = ranlib 
LINKLIB = -L/usr/X11R6/lib -lX11 

