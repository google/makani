
      SUBROUTINE SECONDS(TSEC)
      REAL*8 TSEC
C
C...SECNDS is a real*4 function that returns seconds.
C   The value is modified by subtracting the supplied argument.
C   It acts as in the VMS FORTRAN Manual.
C
      REAL*4 SECNDS, TIME
      TIME = 0.0
      TSEC = SECNDS(TIME)
      END
