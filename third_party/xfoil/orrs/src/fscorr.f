      PROGRAM FSCORR
      PARAMETER (NHX=200)
      REAL H(NHX), M(NHX), LSQ(NHX), FUN(NHX), FCORR(NHX)
      REAL X(NHX), Y(NHX), Z(NHX)
C
      IDEV = 6
      SIZE = 8.0
      CH = 0.02
C
      HMAX = 10.0
      DH = 1.0
C
      FMAX = 0.5
      DF = 0.1
C
      PAR = 0.75
C
      HWT = 1.0/HMAX
      FWT = PAR/FMAX
C     
      OPEN(7,FILE='hfun.fs',STATUS='OLD')
      DO 10 I=1, NHX
        READ(7,*,END=11) H(I),M(I),LSQ(I),FUN(I)
 10   CONTINUE
 11   CONTINUE
      N = I-1
      CLOSE(7)
C
      DO 20 I=1, N
        HB = 1.0/(H(I)-1.0)
ccc        F = 0.22*(1.0 - (5.0*HB-1.0)**2) + 0.5*HB - 0.05 + 3.0*HB**3
C
        F = -0.05 + 2.7*HB - 5.5*HB**2 + 3.0*HB**3
C
        HK = H(I)
ccc        TFS    =  (6.54*HK - 14.07   )/HK**2
c
        TFS  = 4.70*HB - 8.45*HB**2 + 3.41*HB**3
        AM = 2.0*F/TFS - 1.0
C
        Z(I) = M(I)
C
        Y(I) = AM
        X(I) = 10.0*HB

        FUN(I) = M(I)
        FCORR(I) = AM
C
CC        F = 0.5*(BUH + 1.0)*TFS
C
C        X(I) = 10.0*HB
CC
C        F = 0.395*(1.0 - 5.8*(HB-0.485)**2)
C        Y(I) = F
C        Z(I) = FUN(I)*4.0/H(I)
CC
C        FCORR(I) = 0.25*F*H(I)
C
 20   CONTINUE
C
C
      CALL PLOTS(0,-999,IDEV)
      CALL FACTOR(SIZE)
C
      CALL PLOT(8.0*CH,8.0*CH,-3)
C
      CALL PLOTON
C
      CALL XAXIS(0.0,0.0,1.0,DH*HWT,0.0,DH,CH,1)
      CALL YAXIS(0.0,0.0,PAR,DF*FWT,0.0,DF,CH,1)
C
      CALL XYPLOT(N,H,FUN  ,0.0,HWT,0.0,FWT,1,0.3*CH,+1)
      CALL XYPLOT(N,H,FCORR,0.0,HWT,0.0,FWT,1,0.3*CH, 0)
C
      CALL XYPLOT(N,X,Z,0.0,HWT,0.0,FWT,1,0.3*CH,+1)
      CALL XYPLOT(N,X,Y,0.0,HWT,0.0,FWT,1,0.3*CH, 0)
C
      CALL PLOTOF
C
      WRITE(*,*) 'Hit <cr>'
      READ(*,1000) ANS
 1000 FORMAT(A4)
C
      CALL PLOT(0.0,0.0,+999)
      STOP
      END

