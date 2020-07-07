
FC = gfortran
FLG = -O
AR = ar r
RAN = ranlib

DP =
EISLIB = eispack_gSP.a

#DP = -fdefault-real-8
#EISLIB = eispack_gDP.a


$(EISLIB): eispack.f
	$(FC) -c $(FLG) $(DP) eispack.f
	$(AR) $(EISLIB) eispack.o
	$(RAN) $(EISLIB)

clean:
	/bin/rm *.o *.a


