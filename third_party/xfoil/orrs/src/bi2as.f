      PROGRAM BI2AS
C-------------------------------------------------------
C     Converts a set of binary OS data files
C     into the equivalent ASCII OS data files.
C     The files to be converted are listed 
C     in a text file given as the argument.
C     The binary files are assumed to end with "bin".
C     The ASCII  files are assumed to end with "dat".
C-------------------------------------------------------
      PARAMETER (NMAX=257,NRX=101,NWX=91,NHX=21)
      REAL ETA(NMAX,NHX), U(NMAX,NHX), S(NMAX,NHX)
      REAL AR(NRX,NWX,NHX), AI(NRX,NWX,NHX)
      REAL RTL(NRX,NHX)
      REAL WSL(NWX,NHX)
      REAL HH(NHX)
      INTEGER N(NHX), NRP(NHX), NWP(NHX)
      CHARACTER*80 ARGP
C
      CALL GETARG(1,ARGP)
C
      CALL READOS(ARGP,0,
     &            N,NMAX,ETA,U,S,
     &            NRP,NWP,NHP,
     &            RTL,WSL,HH , AR,AI,
     &            NRX,NWX,NHX)
C
      CALL WRITOS(ARGP,1,
     &            N,NMAX,ETA,U,S,
     &            NRP,NWP,NHP,
     &            RTL,WSL,HH , AR,AI,
     &            NRX,NWX,NHX)
C
      STOP
      END


