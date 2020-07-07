      PROGRAM FSRUN
      PARAMETER (NMAX=256)
      DIMENSION ETA(NMAX), F(NMAX), U(NMAX), S(NMAX)
      CHARACTER*1 ANS
C
      LST = 1
      LRE = 1
C
      N = 256
      ETAE = 30.0
      GEO = 1.01
C
      IDEV = 6
      SIZE = 6.0
      IHARD = -999
C
      EWT = 1.0/ETAE
      UWT = 0.5
      CH = 0.02
C
      CALL PLINITIALIZE
      CALL PLOTS(0,IHARD,IDEV)
      CALL FACTOR(SIZE)
C
      CALL PLOT(0.7,0.1,-3)
C
      CALL NEWPEN(1)
C
      CALL PLOT(0.0,0.0,3)
      CALL PLOT(UWT*1.0,0.0,2)
      CALL PLOT(0.0,0.0,3)
      CALL PLOT(0.0,EWT*ETAE,2)
C
      WRITE(*,*) 'Enter H1, H2, dH'
      READ (*,*) H1,H2,DH
C
      NH = INT((H2-H1)/DH) + 1
C
      open(7,file='hfuns.fs',status='unknown')
c
      CALL NEWPEN(3)
      DO 10 IH=1, NH
        H = H1 + DH*FLOAT(IH-1)
        CALL FS(3,2,BU,H,N,ETAE,GEO,ETA,F,U,S,DELTA)
C---------------------
c        BU = H
c        CALL FS(1,1,BU,H,N,ETAE,GEO,ETA,F,U,S,DELTA)
C---------------------
C
        DSI = 0.0
        THI = 0.0
        TSI = 0.0
        CDN = 0.0
        DO 103 I=1, N-1
          UA = 0.5*(U(I+1) + U(I))
          DETA = ETA(I+1) - ETA(I)
C
          DSI = DSI + (1.0 -    UA)   *DETA
          THI = THI + (1.0 -    UA)*UA*DETA
          TSI = TSI + (1.0 - UA*UA)*UA*DETA
C
          CDN = CDN + (U(I+1) - U(I))**2 / DETA
 103    CONTINUE
C
        HK = DSI/THI
        HS = TSI/THI
C
        CDN = CDN *THI * 2.0/HS
        CFN = S(1)*THI
C
        DSI = DSI*DELTA        
        THI = THI*DELTA        
        TSI = TSI*DELTA        
C
        BUF = (CFN - CDN)/(HK-1.0) / THI**2
        write(*,*) H, BU, THI**2, 0.5*(BU + 1.0) * THI**2
        write(7,*) H, BU, THI**2, 0.5*(BU + 1.0) * THI**2
c
        CALL PLOTON
        CALL PLOT(UWT*U(1),EWT*ETA(1),3)
        DO 105 I=2, N
          CALL PLOT(UWT*U(I),EWT*ETA(I),2)
 105    CONTINUE
        CALL PLOTOF
 10   CONTINUE
c
      close(7)
C
      CALL PLFLUSH
      WRITE(*,*) 'Hit <cr>'
      READ (*,8000) ANS
 8000 FORMAT(A1)

      CALL PLCLOSE
      STOP
      END

