C***********************************************************************
C    Module:  sgutil.f
C 
C    Copyright (C) 2002 Mark Drela, Harold Youngren
C 
C    This program is free software; you can redistribute it and/or modify
C    it under the terms of the GNU General Public License as published by
C    the Free Software Foundation; either version 2 of the License, or
C    (at your option) any later version.
C
C    This program is distributed in the hope that it will be useful,
C    but WITHOUT ANY WARRANTY; without even the implied warranty of
C    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
C    GNU General Public License for more details.
C
C    You should have received a copy of the GNU General Public License
C    along with this program; if not, write to the Free Software
C    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
C***********************************************************************



      SUBROUTINE AKIMA ( X, Y, N, XX, YY, SLP )
C
C...PURPOSE    General-purpose monovariate interpolation routine
C              using a locally-fitted cubic spline. One point is
C              interpolated from the input data arrays.
C
C...INPUT:     X  An input array of abscissas in ascending or 
C                 descending value order.
C              Y  An input array of corresponding ordinates. The 
C                 function Y(X) must be single-valued.
C              N  Size for the 'X' and 'Y' arrays. 'N' is an integer.
C                 (the input arrays may be any size)
C
C             XX  Input 'X' point at which an interpolated 'Y' value is
C                 desired.
C
C...OUTPUT:   YY  Interpolated 'Y' ordinate.
C            SLP  Interpolated slope (DY/DX).
C
C...DISCUSSION This spline method produces an interpolated curve that
C              is relatively free of the oscillatory behavior normally
C              associated with cubic splines. The curve produced will
C              be continuous in 'Y' and 'DY/DX' but further derivatives
C              may be discontinuous. The interpolated slope should be
C              treated with some caution, however, due to the tendency
C              of this spline curve to concentrate changes of curvature
C              at the input points (interval ends). If only 2 points are
C              input the curve will be linear, if 3 points are given
C              the curve will be parabolic, more than 3 points will
C              produce a cubic. Extrapolation beyond the bounds of the
C              input data is done with a quadratic through the last 3
C              points at that data boundary.
C
C              This routine is intended as a replacement for B.Wainfan's
C              AKIMAD. It is shorter, twice as fast and it works on
C              input data in ascending or descending order. The calling
C              sequence is more natural and is easier to use for the
C              bulk of applications.
C
C              The coding is compatible with IBM or VAX Fortran 77 
C              and IBM Fortran IV G or H with optimization.
C
C...ORIGIN     Harold Youngren  CALAC Dept. 72-71   3/81
C
C...REFERENCE  Akima, Hiroshi, "A New Method of Interpolation and
C              Smooth Curve Fitting Based on Local Procedures",
C              Journal of the Association for Computing Machines,
C              Vol. 17, No. 4, Oct 1970, pages 589-602
C
C              Wainfan, B. S., "The Akima Subroutines:  Nonlinear
C              Interpolation by Local Polynomial Fit", LR 29244,
C              Oct 30, 1979.
C
C
      DIMENSION  X(N), Y(N), D(5), T(2)
C 
C
C...Check for a degenerate case ( X(1)=X(N) ).    
C 
      IF (X(1) .NE. X(N))  GO TO 10
         YY  = Y(1) 
         SLP = 0. 
         GO TO 70
C
C
C...Find which interval contains the point by binary search.
C...The binary search loop is terminated when the search residual
C...(NSTEP) is zero. The index 'I' will point to the input point
C...lower than or equal to the desired point for the 'X' values
C...in ascending order or to the input point greater than or equal
C...to the desired point for descending order 'X' values.       
C
   10 XORDR = 1.0
      IF (X(1) .GT. X(N))  XORDR = -1.0
C
      IBOT = 1
      ITOP = N
      XXO  = XX * XORDR
C
   20 NSTEP = ( ITOP - IBOT ) / 2
      I     = IBOT + NSTEP
      XO    = X(I) * XORDR
      IF ( XXO .GE. XO )  IBOT = I
      IF ( XXO .LT. XO )  ITOP = I
      IF ( NSTEP .NE. 0 )  GO TO 20
C
C
C...Calculate the straight line slopes between adjacent input points.
C...D(3) is the slope on the interval of interpolation. If the other
C...slopes D(1), D(2), D(4) OR D(5) are not defined they will be
C...created by quadratic extrapolation (only at start and end of data).
C
      DO 30  J = 1, 5
         K  = I + (J-2)
         IF ( ((K-1) .GE. 1) .AND. (K .LE. N) )
     *      D(J) = ( Y(K) - Y(K-1) ) / ( X(K) - X(K-1) )
   30 CONTINUE
C
C...Synthesize upper and lower slopes if required. Check for
C...single line segment input (N=2).
C
      IF (N .EQ. 2)  D(2) = D(3)
C
      IF ((I+2) .GT. N)  D(4) = 2. * D(3) - D(2)
      IF ((I+3) .GT. N)  D(5) = 2. * D(4) - D(3)
      IF ((I-1) .LT. 1)  D(2) = 2. * D(3) - D(4)
      IF ((I-2) .LT. 1)  D(1) = 2. * D(2) - D(3)
C
C
C...Calculate the slopes (T(1),T(2)) at the lower and upper
C...points bounding the interval of interpolation. If the point is
C...at an intersection of straight line segments the slope is
C...defined by the average of the adjacent segment slopes.
C
      DO 50 J = 1, 2
         A = ABS( D(J+3)  -  D(J+2) )
         B = ABS( D(J+1)  -  D(J)   )
         IF ((A + B) .NE. 0.)  GO TO 40
            A = 1.
            B = 1.
   40    T(J) = ( A*D(J+1) + B*D(J+2) ) / ( A + B )
   50 CONTINUE
C
C
C...Check if desired point is on upper point of interval. This
C...reduces error at the transition points between intervals.
C
      IF (XX .NE. X(I+1))  GO TO 60
         YY  = Y(I+1)
         SLP = T(2)
         GO TO 70
C
C...Calculate the cubic coefficients.
C
   60 XINT  =  X(I+1) - X(I)
      XDIF  =  XX     - X(I)
      P0    =  Y(I)
      P1    =  T(1)
      P2    =  ( 3.*D(3) - 2.*T(1) - T(2) ) / XINT
      P3    =  ( T(1)  + T(2)  -  2.*D(3) ) / (XINT*XINT)
C
C...Calculate the Y-value and the slope.
C
      YY  =  P0 + XDIF*( P1 + XDIF*( P2 + XDIF*P3 ) )
      SLP =  P1 + XDIF*( 2.*P2 + XDIF*( 3.*P3 ) )
C
   70 RETURN
      END ! AKIMA



      FUNCTION TRP1 (N,X,Y,XTRP)
C
C...PURPOSE  TO LINEARLY INTERPOLATE A VALUE FROM AN 
C            ARRAY OF DATA
C
C...INPUT    N         NUMBER OF POINTS IN ARRAY 
C            X,Y       ARRAYS OF ABSCISSAE AND ORDINATES
C            XTRP      COORDINATE AT WHICH YTRP IS DESIRED 
C
C...OUTPUT   TRP1      INTERPOLATED VALUE
C
C...COMMENTS   
C
      DIMENSION  X(N), Y(N)
C
      IF (N.LT.1)  THEN
        TRP1 = 0.
        RETURN
      ENDIF
      IF (N.LT.2)  THEN
        TRP1 = Y(1)
        RETURN
      ENDIF
C
C...FIND THE INTERVAL CONTAINING THE POINT
      I = 1
   10 IF (X(I+1).GT.XTRP .OR. I+1.EQ.N)  GO TO 20 
        I = I + 1
        GO TO 10
C
   20 TRP1 = Y(I) + (Y(I+1)-Y(I))*(XTRP-X(I))/(X(I+1)-X(I))
      RETURN
      END ! TRP1


      SUBROUTINE NRMLIZ (N,X)
C
C...PURPOSE  TO NORMALIZE AN ARRAY OF DATA
C
C...INPUT    N         NUMBER OF POINTS IN ARRAY 
C            X         ARRAY OF DATA IN ASCENDING OR DESCENDING ORDER
C
C...OUTPUT   X         NORMALIZED ARRAY (0<=X<=1.)
C
C...COMMENTS   
C
      DIMENSION  X(N)
C
      IF (N.LE.1)  RETURN
C
      DX = X(N) - X(1)
      IF (DX .EQ. 0.0) DX = 1.0
C
      X1 = X(1)
      DO 10 I = 1, N
        X(I) = (X(I)-X1) / DX
   10 CONTINUE
      RETURN
      END ! NRMLIZ


      SUBROUTINE SPACER (N,PSPACE,X)
C...PURPOSE     TO CALCULATE A NORMALIZED (0<=X<=1) SPACING ARRAY.
C
C...INPUT       N      =  NUMBER OF DESIRED POINTS IN ARRAY.
C               PSPACE =  SPACING PARAMETER (-3<=PSPACE<=3).
C                         DEFINES POINT DISTRIBUTION
C                         TO BE USED AS FOLLOWS:
C                 PSPACE = 0  : EQUAL SPACING
C                 PSPACE = 1  : COSINE SPACING.
C                 PSPACE = 2  : SINE SPACING
C                               (CONCENTRATING POINTS NEAR 0).
C                 PSPACE = 3  : EQUAL SPACING.
C
C                 NEGATIVE VALUES OF PSPACE PRODUCE SPACING
C                 WHICH IS REVERSED (AFFECTS ONLY SINE SPACING).
C                 INTERMEDIATE VALUES OF PSPACE WILL PRODUCE
C                 A SPACING WHICH IS A LINEAR COMBINATION
C                 OF THE CORRESPONDING INTEGER VALUES.
C     
C...OUTPUT      X      =  NORMALIZED SPACING ARRAY (0 <= X <= 1)
C                         THE FIRST ELEMENT WILL ALWAYS BE  X(1) = 0.
C                         THE LAST ELEMENT WILL ALWAYS BE   X(N) = 1.
C
      DIMENSION  X(N)
      DATA  PI / 3.1415926535 /
C
      PABS = ABS (PSPACE)
      NABS = IFIX (PABS) + 1
C
      GO TO (10,20,30,30), NABS
C
   10                 PEQU   = 1.-PABS
                      PCOS   = PABS
                      PSIN   = 0.
           GO TO 50
C
   20                 PEQU   = 0.
                      PCOS   = 2.-PABS
                      PSIN   = PABS-1.
           GO TO 50
C
   30                 PEQU   = PABS-2.
                      PCOS   = 0.
                      PSIN   = 3.-PABS
C
   50 DO 100  K = 1, N
         FRAC = FLOAT(K-1)/FLOAT(N-1)
         THETA =  FRAC * PI
         IF (PSPACE .GE. 0. )   X(K) =
     &                 PEQU * FRAC
     &       +         PCOS * ( 1. - COS ( THETA )      ) / 2.
     &       +         PSIN * ( 1. - COS ( THETA / 2. ) )
         IF (PSPACE .LE. 0. )   X(K) =
     &                 PEQU * FRAC
     &       +         PCOS * ( 1. - COS ( THETA )      ) / 2.
     &       +         PSIN * SIN ( THETA / 2. )
  100 CONTINUE
C
      RETURN
      END ! SPACER



      SUBROUTINE CSPACER(NVC,CSPACE,CLAF, XPT,XVR,XSR,XCP)
      REAL XPT(*), XVR(*), XSR(*), XCP(*)
C
      PI = 4.0*ATAN(1.0)
C
C---- set blending weights
      ACSP = ABS(CSPACE)
      NCSP = IFIX(ACSP)
      IF    (NCSP.EQ.0) THEN
       F0 = 1.0 - ACSP
       F1 = ACSP
       F2 = 0.
      ELSEIF(NCSP.EQ.1) THEN
       F0 = 0.
       F1 = 2.0 - ACSP
       F2 = ACSP - 1.0
      ELSE
       F0 = ACSP - 2.0
       F1 = 0.
       F2 = 3.0 - ACSP
      ENDIF
C
C---- cosine chordwise spacing
      DTH1 =     PI/FLOAT(4*NVC + 2)
      DTH2 = 0.5*PI/FLOAT(4*NVC + 1)
      DXC0 =    1.0/FLOAT(4*NVC)
C
      DO IVC = 1, NVC
C------ uniform
        XC0 = INT(4*IVC - 4) * DXC0
        XPT0 = XC0        
        XVR0 = XC0 +     DXC0
        XSR0 = XC0 + 2.0*DXC0
        XCP0 = XC0 +     DXC0 + 2.0*DXC0*CLAF
C
C------ cosine
        TH1 = INT(4*IVC - 3) * DTH1
        XPT1 = 0.5*(1.0 - COS(TH1         ))
        XVR1 = 0.5*(1.0 - COS(TH1+    DTH1))
        XSR1 = 0.5*(1.0 - COS(TH1+2.0*DTH1))
        XCP1 = 0.5*(1.0 - COS(TH1+    DTH1+2.0*DTH1*CLAF))
C
        IF(CSPACE .GT. 0.0) THEN
C------- sine
         TH2 = INT(4*IVC - 3) * DTH2
         XPT2 = 1.0 - COS(TH2         )
         XVR2 = 1.0 - COS(TH2+    DTH2)
         XSR2 = 1.0 - COS(TH2+2.0*DTH2)
         XCP2 = 1.0 - COS(TH2+    DTH2+2.0*DTH2*CLAF)
        ELSE
C------- -sine
         TH2 = INT(4*IVC - 4) * DTH2
         XPT2 = SIN(TH2         )
         XVR2 = SIN(TH2+    DTH2)
         XSR2 = SIN(TH2+2.0*DTH2)
         XCP2 = SIN(TH2+    DTH2+2.0*DTH2*CLAF)
        ENDIF
C
C------ blend 'em
        XPT(IVC) = F0*XPT0 + F1*XPT1 + F2*XPT2
        XVR(IVC) = F0*XVR0 + F1*XVR1 + F2*XVR2
        XSR(IVC) = F0*XSR0 + F1*XSR1 + F2*XSR2
        XCP(IVC) = F0*XCP0 + F1*XCP1 + F2*XCP2
C
      ENDDO
      XPT(1) = 0.0
      XPT(NVC+1) = 1.0
C
      RETURN
      END ! CSPACER

