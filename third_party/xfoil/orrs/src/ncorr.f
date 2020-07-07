      PROGRAM NCORR
      PARAMETER (NH=13)
      REAL H(NH), RT(NH),DN(NH)
C
      DATA H /  2.3,    2.4,    2.5,    2.6,    2.8,    3.0,    3.4,
     &          4.0,    5.0,    7.0,   10.0,   15.0,   20.0 /
C
      DATA RT / 4000., 1820.,   700.,   270.,   100.,   76.,    52., 
     &          34.,    26.,     14.,    9.0,   6.4,    5.0 /
C
      DATA DN / 0.0067, 0.0064, 0.0076, 0.0104, 0.0200, 0.0323, 0.0529,
     &          0.0727, 0.110 , 0.169 , 0.253 , 0.390 , 0.526  /
C
      REAL X1(NH), Y1(NH), F1(NH), G1(NH)
      REAL X2(NH), Y2(NH), F2(NH), G2(NH)
C
      IDEV = 6
      SIZE = 8.0
      CH = 0.02
C
      HMAX = 20.0
      DH = 2.0
C
      FMAX = 0.5
      DF = 0.1
C
      FMAX = 0.05
      DF = 0.005
C
      PAR = 0.75
C
      HWT = 1.0/HMAX
      FWT = PAR/FMAX
C     
      N = NH
C
      DO 20 I=1, N
        HB = 1.0/(H(I)-1.0)
C
        X1(I) = 4.0*H(I)
        Y1(I) = DN(I)
        G1(I) = 0.13 * (0.215/HB)
        F1(I) = 0.13 * (0.215/HB)
     &        - 0.0345 * EXP(-15.0*(HB-0.65)**2)
        F1(I) = 0.028*(H(I)-1.0)
     &        - 0.0345 * EXP(-(3.87/(H(I)-1.0) - 2.52)**2)
C
        X2(I) = HMAX* HB
        Y2(I) = DN(I)
        G2(I) = 0.13 * (0.215/HB)
        F2(I) = 0.13 * (0.215/HB)
     &        - 0.0345 * EXP(-15.0*(HB-0.65)**2)
        F2(I) = 0.028*(H(I)-1.0)
     &        - 0.0345 * EXP(-(3.87/(H(I)-1.0) - 2.52)**2)
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
      CALL XAXIS(0.0,-PAR,1.0,0.1,0.0,0.1,CH,1)
C
      CALL XAXIS(0.0,0.0,1.0,DH*HWT,0.0,DH,CH,1)
      CALL YAXIS(0.0,0.0,PAR,DF*FWT,0.0,DF,CH,3)
C
      CALL XYPLOT(N,X1,Y1,0.0,HWT,0.0,FWT,1,0.3*CH,+1)
      CALL XYPLOT(N,X1,F1,0.0,HWT,0.0,FWT,1,0.3*CH, 0)
      CALL XYPLOT(N,X1,G1,0.0,HWT,0.0,FWT,5,0.3*CH, 0)
C
      CALL XYPLOT(N,X2,Y2,0.0,HWT,0.0,FWT,3,0.3*CH,+5)
      CALL XYPLOT(N,X2,F2,0.0,HWT,0.0,FWT,2,0.3*CH, 0)
      CALL XYPLOT(N,X2,G2,0.0,HWT,0.0,FWT,3,0.3*CH, 0)
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

