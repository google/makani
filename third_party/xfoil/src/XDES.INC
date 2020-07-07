C     INCLUDE file for XFOIL design routines...
C
C---- Statement functions used to offset and scale all plots with blowups
      XMOD(XTMP) = XSF * (XTMP - XOFF)
      YMOD(YTMP) = YSF * (YTMP - YOFF)
C
      YMODP(YTMP) = YSFP * (YTMP - YOFF)
C
