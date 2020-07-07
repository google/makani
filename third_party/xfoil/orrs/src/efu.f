
      PROGRAM EFU
      PARAMETER(NX=2001)
      DIMENSION Y(NX), S(NX), U(NX), F(NX)
C
      PI = 4.0*ATAN(1.0)
C
      YWALL = -4.8

      YMAX  =  40.0
C
      N = NX
C
      DO I=1, N
        Y(I) = YMAX*FLOAT(I-1)/FLOAT(N-1)
      ENDDO
C
      DO I = 1, N
        S(I) = EXP(-(Y(I)+YWALL)**2)
      ENDDO
C
      I = 1
      U(I) = 0.
      F(I) = 0.
      DO I = 2, N
        DY = Y(I) - Y(I-1)
        U(I) = U(I-1) + 0.5*(S(I)+S(I-1))*DY
        F(I) = F(I-1) + 0.5*(U(I)+U(I-1))*DY
      ENDDO
      SMAX = 1.0
C
      UE = U(N)
      DO I = 1, N
        F(I) = F(I)/UE
        U(I) = U(I)/UE
        S(I) = S(I)/UE
      ENDDO
      SMAX = SMAX/UE
C
      DSUM = 0.
      TSUM = 0.
      ESUM = 0.
      DO I = 2, N
        DY =  Y(I) - Y(I-1)
        UA = (U(I) + U(I-1))*0.5
        DSUM = DSUM + (1.0 - UA   )   *DY
        TSUM = TSUM + (1.0 - UA   )*UA*DY
        ESUM = ESUM + (1.0 - UA**2)*UA*DY
      ENDDO
C
      WRITE(*,*) N, DSUM/TSUM, 1.0/(SMAX*TSUM)
      DO I = 1, N
        WRITE(*,*) Y(I)/TSUM, U(I), S(I)*TSUM
      ENDDO
C
      STOP
      END
