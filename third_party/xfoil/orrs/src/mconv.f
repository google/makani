      PROGRAM MAPGEN
      PARAMETER (NMAX=257,NRX=81,NWX=81)
      REAL ETA(NMAX), F(NMAX), U(NMAX), S(NMAX)
      LOGICAL*1 FNAME(32)
      REAL AR(NRX,NWX), AI(NRX,NWX), X(NRX,NWX), Y(NRX,NWX)
      REAL RT(NRX),RTL(NRX), WS(NWX),WSL(NWX)
      CHARACTER*1 ANS
C
      READ(19) N, H
      READ(19) (ETA(I),I=1, N)
      READ(19) (U(I)  ,I=1, N)
      READ(19) (S(I)  ,I=1, N)
      READ(19) NRP, NWP
      READ(19) (RTL(IR),IR=1,NRP)
      READ(19) (WSL(IW),IW=1,NWP)
C
      DO 10 IW=NWP, 1, -1
        DO 101 IR=1, NRP
          READ(19,END=11) AR(IR,IW), AI(IR,IW)
  101   CONTINUE
   10 CONTINUE
   11 CONTINUE
C
C
      GEO = (ETA(3)-ETA(2)) / (ETA(2)-ETA(1))
      ETAE = ETA(N)
C
CCC   CALL FS(3,2,BU,H,N,ETAE,GEO,ETA,F,U,S)
C
      WRITE(6,*) 'GEO =', GEO, '   ETAE =', ETAE
      WRITE(6,*) 'H   =', H  , '   N    =', N
C
      WRITE(29) N, H
      WRITE(29) (ETA(I),I=1, N)
      WRITE(29) (U(I)  ,I=1, N)
      WRITE(29) (S(I)  ,I=1, N)
      WRITE(29)  NRP, NWP
      WRITE(29) (RTL(IR),IR=1,NRP)
      WRITE(29) (WSL(IW),IW=1,NWP)
C
      DO 20 IW=1, NWP
        WRITE(29) (AR(IR,IW),IR=1,NRP)
        WRITE(29) (AI(IR,IW),IR=1,NRP)
   20 CONTINUE
C
      STOP
      END

