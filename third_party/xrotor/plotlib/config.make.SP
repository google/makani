
#=======================================#
# Makefile options for Xplot11 library  #
#   Set up or select a set of compile   #
#   options for your system             # 
#=======================================#


# Setlibrary name 
PLTLIB = libPlt.a

# Some fortrans need trailing underscores in C interface symbols (see Xwin.c)
# This should work for most of the "unix" fortran compilers
DEFINE = -DUNDERSCORE

FC = ifort
CC  = gcc
DP =

FFLAGS  = -O3 $(DP)
CFLAGS  = -O3 $(DEFINE)
AR = ar r
RANLIB = ranlib 
LINKLIB = -L/usr/X11R6/lib -lX11

