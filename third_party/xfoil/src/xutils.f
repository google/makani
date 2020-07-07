


      SUBROUTINE SETEXP(S,DS1,SMAX,NN)
C........................................................
C     Sets geometrically stretched array S:
C
C       S(i+1) - S(i)  =  r * [S(i) - S(i-1)]
C
C       S     (output)  array to be set  
C       DS1   (input)   first S increment:  S(2) - S(1)
C       SMAX  (input)   final S value:      S(NN)
C       NN    (input)   number of points
C........................................................
      REAL S(NN)
C
      SIGMA = SMAX/DS1
      NEX = NN-1
      RNEX = FLOAT(NEX)
      RNI = 1.0/RNEX
C
C---- solve quadratic for initial geometric ratio guess
      AAA = RNEX*(RNEX-1.0)*(RNEX-2.0) / 6.0
      BBB = RNEX*(RNEX-1.0) / 2.0
      CCC = RNEX - SIGMA
C
      DISC = BBB**2 - 4.0*AAA*CCC
      DISC = MAX( 0.0 , DISC )
C
      IF(NEX.LE.1) THEN
       STOP 'SETEXP: Cannot fill array.  N too small.'
      ELSE IF(NEX.EQ.2) THEN
       RATIO = -CCC/BBB  +  1.0
      ELSE
       RATIO = (-BBB + SQRT(DISC))/(2.0*AAA)  +  1.0
      ENDIF
C
      IF(RATIO.EQ.1.0) GO TO 11
C
C---- Newton iteration for actual geometric ratio
      DO 1 ITER=1, 100
        SIGMAN = (RATIO**NEX - 1.0) / (RATIO - 1.0)
        RES = SIGMAN**RNI - SIGMA**RNI
        DRESDR = RNI*SIGMAN**RNI
     &         * (RNEX*RATIO**(NEX-1) - SIGMAN) / (RATIO**NEX - 1.0)
C
        DRATIO = -RES/DRESDR
        RATIO = RATIO + DRATIO
C
        IF(ABS(DRATIO) .LT. 1.0E-5) GO TO 11
C
    1 CONTINUE
      WRITE(*,*) 'SETEXP: Convergence failed.  Continuing anyway ...'
C
C---- set up stretched array using converged geometric ratio
   11 S(1) = 0.0
      DS = DS1
      DO 2 N=2, NN
        S(N) = S(N-1) + DS
        DS = DS*RATIO
    2 CONTINUE
C
      RETURN
      END



      FUNCTION ATANC(Y,X,THOLD)
      IMPLICIT REAL (A-H,M,O-Z)
C---------------------------------------------------------------
C     ATAN2 function with branch cut checking.
C
C     Increments position angle of point X,Y from some previous
C     value THOLD due to a change in position, ensuring that the
C     position change does not cross the ATAN2 branch cut
C     (which is in the -x direction).  For example:
C
C       ATANC( -1.0 , -1.0 , 0.75*pi )  returns  1.25*pi , whereas
C       ATAN2( -1.0 , -1.0 )            returns  -.75*pi .
C
C     Typically, ATANC is used to fill an array of angles:
C
C        THETA(1) = ATAN2( Y(1) , X(1) )
C        DO i=2, N
C          THETA(i) = ATANC( Y(i) , X(i) , THETA(i-1) )
C        END DO
C
C     This will prevent the angle array THETA(i) from jumping by 
C     +/- 2 pi when the path X(i),Y(i) crosses the negative x axis.
C
C     Input:
C       X,Y     point position coordinates
C       THOLD   position angle of nearby point
C
C     Output:
C       ATANC   position angle of X,Y
C---------------------------------------------------------------
      DATA  PI /3.1415926535897932384/
      DATA TPI /6.2831853071795864769/
C
C---- set new position angle, ignoring branch cut in ATAN2 function for now
      THNEW = ATAN2( Y , X )
      DTHET = THNEW - THOLD
C
C---- angle change cannot exceed +/- pi, so get rid of any multiples of 2 pi 
      DTCORR = DTHET - TPI*INT( (DTHET + SIGN(PI,DTHET))/TPI )
C
C---- set correct new angle
      ATANC = THOLD + DTCORR
C
      RETURN
      END ! ATANC
 
