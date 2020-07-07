      PROGRAM TESTCON
      PARAMETER (NMAX=256)
      DIMENSION ETA(NMAX), F(NMAX), U(NMAX), S(NMAX)
      DIMENSION UTR(NMAX), UTI(NMAX), VTR(NMAX), VTI(NMAX), UT(NMAX)
      REAL X(50,50), Y(50,50), ZR(50,50), ZI(50,50)
      CHARACTER*1 ANS
C
      IDEV = 12
      SIZE = 6.0
      EWT = 1.0/30.0
      UWT = 0.5
      PWT = 10.0
C
C----------------------
      BU = 0.0
      H = 2.65
      ISPEC = 2
C
      N = 128
      ETAE = 10.0
      GEO = 1.02
C
      CALL FS(3,ISPEC,BU,H,N,ETAE,GEO,ETA,F,U,S)
C---------------------
C
C      CALL PLOTS(0,-999,IDEV)
C      CALL FACTOR(SIZE)
CC
C      CALL PLOT(0.1,0.1,-3)
CC
C      CALL NEWPEN(1)
C
c      CALL PLOT(0.0,0.0,3)
c      CALL PLOT(UWT*1.0,0.0,2)
c      CALL PLOT(0.0,0.0,3)
c      CALL PLOT(0.0,EWT*20.0,2)
cC
      II = 17
      JJ = 17
C
      NCON = 41
C
C      ARMIN = 0.10
C      ARMAX = 0.25
C      AIMIN = 0.00
C      AIMAX = 0.15
C
      ARMIN = 0.08
      ARMAX = 0.20
      AIMIN = -.02
      AIMAX = 0.10
C
      RESPEC = 5000.
      WRSPEC = 0.03000
C
C
      DO 100 J=1, JJ
C
      DO 10 I=1, II
C
        RE = RESPEC
        WR = WRSPEC
        WI = 0.0
C
        AR = ARMIN + (ARMAX-ARMIN) * FLOAT(I-1)/FLOAT(II-1)
        AI = AIMIN + (AIMAX-AIMIN) * FLOAT(J-1)/FLOAT(JJ-1)
C
        ITMAX = 1
        CALL ORRS(1,1,N,ETA,U,S, RE, ITMAX,
     &            AR,AI, WR,WI, UTR,UTI,VTR,VTI,RESMAX)
C
        ZR(I,J) = UTR(1)
        ZI(I,J) = UTI(1)
        WRITE(6,1050) I,J,AR,AI,ZR(I,J),ZI(I,J)
 1050   FORMAT(1X,2I4,'   alpha =', 2F10.6,'   Res =', 2E12.4)
C
C
c        DO 15 I=1, N
c          UT(I) = SQRT(UTR(I)**2 + UTI(I)**2)
c   15   CONTINUE
C
c        CALL NEWPEN(2)
c        CALL PLOT(PWT*UT(1),EWT*ETA(1),3)
c        DO 20 I=2, N
c          CALL PLOT(PWT*UT(I),EWT*ETA(I),2)
c   20   CONTINUE
cC
C        CALL PLOT(PWT*UTI(1),EWT*ETA(1),3)
C        DO 25 I=2, N
C          CALL PLOT(PWT*UTI(I),EWT*ETA(I),2)
C   25   CONTINUE
C
c        CALL PLOT(UWT*U(1),EWT*ETA(1),3)
c        DO 30 I=2, N
c          CALL PLOT(UWT*U(I),EWT*ETA(I),2)
c   30   CONTINUE
C
   10 CONTINUE
  100 CONTINUE
C
      ZRMIN = ZR(1,1)
      ZRMAX = ZR(1,1)
      ZIMIN = ZI(1,1)
      ZIMAX = ZI(1,1)
      DO 150 I=1, II
        DO 160 J=1, JJ
          ZRMIN = AMIN1(ZRMIN,ZR(I,J))
          ZRMAX = AMAX1(ZRMAX,ZR(I,J))
          ZIMIN = AMIN1(ZIMIN,ZI(I,J))
          ZIMAX = AMAX1(ZIMAX,ZI(I,J))
          X(I,J) = FLOAT(I-1)/FLOAT(II-1)
          Y(I,J) = FLOAT(J-1)/FLOAT(JJ-1)
  160   CONTINUE
  150 CONTINUE
C
      CALL PLOTS(0,0,IDEV)
      CALL FACTOR(SIZE)
C
      CALL PLOT(0.2,0.1,-3)
C
      CALL NEWPEN(1)
      CALL PLOT(0.0,0.0,3)
      CALL PLOT(1.0,0.0,2)
      CALL PLOT(1.0,1.0,2)
      CALL PLOT(0.0,1.0,2)
      CALL PLOT(0.0,0.0,2)
C
      CALL NUMBER(0.0,1.20,0.03,H ,0.0,3)
      CALL NUMBER(0.0,1.13,0.03,RE,0.0,5)
      CALL NUMBER(0.3,1.13,0.03,WR,0.0,5)
      CALL SYMBOL(0.0,1.05,0.02,'REAL',0.0,4)
      CALL NUMBER(-.03,-.03,0.02,ARMIN,0.0,4)
      CALL NUMBER(0.97,-.03,0.02,ARMAX,0.0,4)
      CALL NUMBER(-.15,-.01,0.02,AIMIN,0.0,4)
      CALL NUMBER(-.15,0.99,0.02,AIMAX,0.0,4)
C
      FCON = 0.0
      CALL NEWPEN(4)
      CALL CON1(50,50,II,JJ,X,Y,ZR,FCON,1.0,1.0)
C
      CALL NEWPEN(1)
      DO 210 ICON=1, NCON
        FRCON = ZRMIN + (ZRMAX-ZRMIN)*FLOAT(ICON-1)/FLOAT(NCON-1)
        CALL CON1(50,50,II,JJ,X,Y,ZR,FRCON,1.0,1.0)
  210 CONTINUE
C
      WRITE(6,*) 'Hit <cr>'
      READ (5,8000) ANS
      CALL PLOT(0.0,0.0,-999)
C
C
      CALL PLOTS(0,0,IDEV)
      CALL FACTOR(SIZE)
C
      CALL PLOT(0.2,0.1,-3)
C
      CALL NEWPEN(1)
      CALL PLOT(0.0,0.0,3)
      CALL PLOT(1.0,0.0,2)
      CALL PLOT(1.0,1.0,2)
      CALL PLOT(0.0,1.0,2)
      CALL PLOT(0.0,0.0,2)
C
      CALL NUMBER(0.0,1.20,0.03,H ,0.0,3)
      CALL NUMBER(0.0,1.13,0.03,WR,0.0,5)
      CALL SYMBOL(0.0,1.05,0.02,'IMAG',0.0,4)
      CALL NUMBER(-.03,-.03,0.02,ARMIN,0.0,4)
      CALL NUMBER(0.97,-.03,0.02,ARMAX,0.0,4)
      CALL NUMBER(-.15,-.01,0.02,AIMIN,0.0,4)
      CALL NUMBER(-.15,0.99,0.02,AIMAX,0.0,4)
C
      FCON = 0.0
      CALL NEWPEN(4)
      CALL CON1(50,50,II,JJ,X,Y,ZI,FCON,1.0,1.0)
C
      CALL NEWPEN(1)
      DO 220 ICON=1, NCON
        FICON = ZIMIN + (ZIMAX-ZIMIN)*FLOAT(ICON-1)/FLOAT(NCON-1)
        CALL CON1(50,50,II,JJ,X,Y,ZI,FICON,1.0,1.0)
  220 CONTINUE
C
      WRITE(6,*) 'Hit <cr>'
      READ (5,8000) ANS
 8000 FORMAT(A1)
C
      CALL PLOT(0.0,0.0,+999)
      STOP
      END

