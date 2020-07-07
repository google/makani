C***********************************************************************
C    Module:  xgeom.f
C 
C    Copyright (C) 2000 Mark Drela 
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

      SUBROUTINE LEFIND(SLE,X,XP,Y,YP,S,N)
      DIMENSION X(*),XP(*),Y(*),YP(*),S(*)
C------------------------------------------------------
C     Locates leading edge spline-parameter value SLE
C
C     The defining condition is
C         
C      (X-XTE,Y-YTE) . (X',Y') = 0     at  S = SLE
C
C     i.e. the surface tangent is normal to the chord
C     line connecting X(SLE),Y(SLE) and the TE point.
C------------------------------------------------------
C
C---- convergence tolerance
      DSEPS = (S(N)-S(1)) * 1.0E-5
C
C---- set trailing edge point coordinates
      XTE = 0.5*(X(1) + X(N))
      YTE = 0.5*(Y(1) + Y(N))
C
C---- get first guess for SLE
      DO 10 I=3, N-2
        DXTE = X(I) - XTE
        DYTE = Y(I) - YTE
        DX = X(I+1) - X(I)
        DY = Y(I+1) - Y(I)
        DOTP = DXTE*DX + DYTE*DY
        IF(DOTP .LT. 0.0) GO TO 11
   10 CONTINUE
C
   11 SLE = S(I)
C
C---- check for sharp LE case
      IF(S(I) .EQ. S(I-1)) THEN
ccc        WRITE(*,*) 'Sharp LE found at ',I,SLE
        RETURN
      ENDIF
C
C---- Newton iteration to get exact SLE value
      DO 20 ITER=1, 50
        XLE  = SEVAL(SLE,X,XP,S,N)
        YLE  = SEVAL(SLE,Y,YP,S,N)
        DXDS = DEVAL(SLE,X,XP,S,N)
        DYDS = DEVAL(SLE,Y,YP,S,N)
        DXDD = D2VAL(SLE,X,XP,S,N)
        DYDD = D2VAL(SLE,Y,YP,S,N)
C
        XCHORD = XLE - XTE
        YCHORD = YLE - YTE
C
C------ drive dot product between chord line and LE tangent to zero
        RES  = XCHORD*DXDS + YCHORD*DYDS
        RESS = DXDS  *DXDS + DYDS  *DYDS
     &       + XCHORD*DXDD + YCHORD*DYDD
C
C------ Newton delta for SLE 
        DSLE = -RES/RESS
C
        DSLE = MAX( DSLE , -0.02*ABS(XCHORD+YCHORD) )
        DSLE = MIN( DSLE ,  0.02*ABS(XCHORD+YCHORD) )
        SLE = SLE + DSLE
        IF(ABS(DSLE) .LT. DSEPS) RETURN
   20 CONTINUE
      WRITE(*,*) 'LEFIND:  LE point not found.  Continuing...'
      SLE = S(I)
      RETURN
      END



      SUBROUTINE XLFIND(SLE,X,XP,Y,YP,S,N)
      DIMENSION X(*),XP(*),Y(*),YP(*),S(*)
C------------------------------------------------------
C     Locates leftmost (minimum x) point location SLE
C
C     The defining condition is
C         
C      X' = 0     at  S = SLE
C
C     i.e. the surface tangent is vertical
C------------------------------------------------------
C
      DSLEN = S(N) - S(1)
C
C---- convergence tolerance
      DSEPS = (S(N)-S(1)) * 1.0E-5
C
C---- get first guess for SLE
      DO 10 I=3, N-2
        DX = X(I+1) - X(I)
        IF(DX .GT. 0.0) GO TO 11
   10 CONTINUE
C
   11 SLE = S(I)
C
C---- check for sharp LE case
      IF(S(I) .EQ. S(I-1)) THEN
ccc        WRITE(*,*) 'Sharp LE found at ',I,SLE
        RETURN
      ENDIF
C
C---- Newton iteration to get exact SLE value
      DO 20 ITER=1, 50
        DXDS = DEVAL(SLE,X,XP,S,N)
        DXDD = D2VAL(SLE,X,XP,S,N)
C
C------ drive DXDS to zero
        RES  = DXDS
        RESS = DXDD
C
C------ Newton delta for SLE 
        DSLE = -RES/RESS
C
        DSLE = MAX( DSLE , -0.01*ABS(DSLEN) )
        DSLE = MIN( DSLE ,  0.01*ABS(DSLEN) )
        SLE = SLE + DSLE
        IF(ABS(DSLE) .LT. DSEPS) RETURN
   20 CONTINUE
      WRITE(*,*) 'XLFIND:  Left point not found.  Continuing...'
      SLE = S(I)
      RETURN
      END ! XLFIND



      SUBROUTINE NSFIND(SLE,X,XP,Y,YP,S,N)
      REAL X(*),Y(*),S(*),XP(*),YP(*)
C----------------------------------------------------------
C     Finds "nose" of airfoil where curvature is a maximum
C----------------------------------------------------------
C
      PARAMETER (NMAX=500)
      DIMENSION A(NMAX), B(NMAX), C(NMAX), CV(NMAX)
C
      IF(N.GT.NMAX) STOP 'NSFIND: Local array overflow. Increase NMAX.'
C
C---- set up curvature array
      DO 3 I=1, N
        CV(I) = CURV(S(I),X,XP,Y,YP,S,N)
    3 CONTINUE
C
C---- curvature smoothing length
      SMOOL = 0.006*(S(N)-S(1))
C
C---- set up tri-diagonal system for smoothed curvatures
      SMOOSQ = SMOOL**2
      A(1) = 1.0
      C(1) = 0.0
      DO 4 I=2, N-1
        DSM = S(I) - S(I-1)
        DSP = S(I+1) - S(I)
        DSO = 0.5*(S(I+1) - S(I-1))
C
        IF(DSM.EQ.0.0 .OR. DSP.EQ.0.0) THEN
C------- leave curvature at corner point unchanged
         B(I) = 0.0
         A(I) = 1.0
         C(I) = 0.0
        ELSE
         B(I) =  SMOOSQ * (         - 1.0/DSM) / DSO
         A(I) =  SMOOSQ * ( 1.0/DSP + 1.0/DSM) / DSO  +  1.0
         C(I) =  SMOOSQ * (-1.0/DSP          ) / DSO
        ENDIF
    4 CONTINUE
      B(N) = 0.0
      A(N) = 1.0
C
      CALL TRISOL(A,B,C,CV,N)
C
C---- find max curvature index
      CVMAX = 0.
      IVMAX = 0
      DO 71 I=2, N-1
        IF(ABS(CV(I)) .GT. CVMAX) THEN
         CVMAX = ABS(CV(I))
         IVMAX = I
        ENDIF
   71 CONTINUE
C
C---- fit a parabola to the curvature at the three points near maximum
      I = IVMAX
C
      IP = I+1
      IM = I-1
      IF(S(I) .EQ. S(IP)) IP = I+2
      IF(S(I) .EQ. S(IM)) IM = I-2

      DSM = S(I) - S(IM)
      DSP = S(IP) - S(I)
C
      CVSM = (CV(I)-CV(IM))/DSM
      CVSP = (CV(IP)-CV(I))/DSP
C
C---- 1st and 2nd derivatives at i=IVMAX
      CVS = (CVSM*DSP + CVSP*DSM)/(DSP+DSM)
      CVSS = 2.0*(CVSP-CVSM)/(DSP+DSM)
C
C---- set location of arc length at maximum of parabola
      DS = -CVS/CVSS
      SLE = S(I) + DS
C
      RETURN
      END


      SUBROUTINE SOPPS(SOPP, SI, X,XP,Y,YP,S,N, SLE)
      DIMENSION X(*),XP(*),Y(*),YP(*),S(*)
C--------------------------------------------------
C     Calculates arc length SOPP of point 
C     which is opposite of point SI, on the 
C     other side of the airfoil baseline
C--------------------------------------------------
C
C---- reference length for testing convergence
      SLEN = S(N) - S(1)
C
C---- set chordline vector
      XLE = SEVAL(SLE,X,XP,S,N)
      YLE = SEVAL(SLE,Y,YP,S,N)
      XTE = 0.5*(X(1)+X(N))
      YTE = 0.5*(Y(1)+Y(N))
      CHORD = SQRT((XTE-XLE)**2 + (YTE-YLE)**2)
      DXC = (XTE-XLE) / CHORD
      DYC = (YTE-YLE) / CHORD
C
      IF(SI.LT.SLE) THEN
       IN = 1
       INOPP = N
      ELSE
       IN = N
       INOPP = 1
      ENDIF
      SFRAC = (SI-SLE)/(S(IN)-SLE)
      SOPP = SLE + SFRAC*(S(INOPP)-SLE)
C     
      IF(ABS(SFRAC) .LE. 1.0E-5) THEN
       SOPP = SLE
       RETURN
      ENDIF
C
C---- XBAR = x coordinate in chord-line axes
      XI  = SEVAL(SI , X,XP,S,N)
      YI  = SEVAL(SI , Y,YP,S,N)
      XLE = SEVAL(SLE, X,XP,S,N)
      YLE = SEVAL(SLE, Y,YP,S,N)
      XBAR = (XI-XLE)*DXC + (YI-YLE)*DYC
C
C---- converge on exact opposite point with same XBAR value
      DO 300 ITER=1, 12
        XOPP  = SEVAL(SOPP,X,XP,S,N)
        YOPP  = SEVAL(SOPP,Y,YP,S,N)
        XOPPD = DEVAL(SOPP,X,XP,S,N)
        YOPPD = DEVAL(SOPP,Y,YP,S,N)
C
        RES  = (XOPP -XLE)*DXC + (YOPP -YLE)*DYC - XBAR
        RESD =  XOPPD     *DXC +  YOPPD     *DYC
C
        IF(ABS(RES)/SLEN .LT. 1.0E-5) GO TO 305
        IF(RESD .EQ. 0.0) GO TO 303
C
        DSOPP = -RES/RESD
        SOPP = SOPP + DSOPP
C
        IF(ABS(DSOPP)/SLEN .LT. 1.0E-5) GO TO 305
 300  CONTINUE
 303  WRITE(*,*)
     &      'SOPPS: Opposite-point location failed. Continuing...'
      SOPP = SLE + SFRAC*(S(INOPP)-SLE)
C
 305  CONTINUE
      RETURN
      END ! SOPPS


 
      SUBROUTINE NORM(X,XP,Y,YP,S,N)
      DIMENSION X(*),XP(*),Y(*),YP(*),S(*)
C-----------------------------------------------
C     Scales coordinates to get unit chord
C-----------------------------------------------
C
      CALL SCALC(X,Y,S,N)
      CALL SEGSPL(X,XP,S,N)
      CALL SEGSPL(Y,YP,S,N)
C
      CALL LEFIND(SLE,X,XP,Y,YP,S,N)
C
      XMAX = 0.5*(X(1) + X(N))
      XMIN = SEVAL(SLE,X,XP,S,N)
      YMIN = SEVAL(SLE,Y,YP,S,N)
C
      FUDGE = 1.0/(XMAX-XMIN)
      DO 40 I=1, N
        X(I) = (X(I)-XMIN)*FUDGE
        Y(I) = (Y(I)-YMIN)*FUDGE
        S(I) = S(I)*FUDGE
   40 CONTINUE
C
      RETURN
      END


      SUBROUTINE GEOPAR(X,XP,Y,YP,S,N, T,
     &             SLE,CHORD,AREA,RADLE,ANGTE,
     &             EI11A,EI22A,APX1A,APX2A,
     &             EI11T,EI22T,APX1T,APX2T,
     &             THICK,CAMBR)
      DIMENSION X(*), XP(*), Y(*), YP(*), S(*), T(*)
C
      PARAMETER (IBX=600)
      DIMENSION
     &     XCAM(2*IBX), YCAM(2*IBX), YCAMP(2*IBX),
     &     XTHK(2*IBX), YTHK(2*IBX), YTHKP(2*IBX)
C------------------------------------------------------
C     Sets geometric parameters for airfoil shape
C------------------------------------------------------
      CALL LEFIND(SLE,X,XP,Y,YP,S,N)
C
      XLE = SEVAL(SLE,X,XP,S,N)
      YLE = SEVAL(SLE,Y,YP,S,N)
      XTE = 0.5*(X(1)+X(N))
      YTE = 0.5*(Y(1)+Y(N))
C
      CHSQ = (XTE-XLE)**2 + (YTE-YLE)**2
      CHORD = SQRT(CHSQ)
C
      CURVLE = CURV(SLE,X,XP,Y,YP,S,N)
C
      RADLE = 0.0
      IF(ABS(CURVLE) .GT. 0.001*(S(N)-S(1))) RADLE = 1.0 / CURVLE
C
      ANG1 = ATAN2( -YP(1) , -XP(1) )
      ANG2 = ATANC(  YP(N) ,  XP(N) , ANG1 )
      ANGTE = ANG2 - ANG1
C

      DO I=1, N
        T(I) = 1.0
      ENDDO
C
      CALL AECALC(N,X,Y,T, 1, 
     &            AREA,XCENA,YCENA,EI11A,EI22A,APX1A,APX2A)
C
      CALL AECALC(N,X,Y,T, 2, 
     &            SLEN,XCENT,YCENT,EI11T,EI22T,APX1T,APX2T)
C
C--- Old, approximate thickness,camber routine (on discrete points only)
      CALL TCCALC(X,XP,Y,YP,S,N, THICK,XTHICK, CAMBR,XCAMBR )
C
C--- More accurate thickness and camber estimates
cc      CALL GETCAM(XCAM,YCAM,NCAM,XTHK,YTHK,NTHK,
cc     &            X,XP,Y,YP,S,N )
cc      CALL GETMAX(XCAM,YCAM,YCAMP,NCAM,XCAMBR,CAMBR)
cc      CALL GETMAX(XTHK,YTHK,YTHKP,NTHK,XTHICK,THICK)
cc      THICK = 2.0*THICK
C
      WRITE(*,1000) THICK,XTHICK,CAMBR,XCAMBR
 1000 FORMAT( ' Max thickness = ',F12.6,'  at x = ',F7.3,
     &       /' Max camber    = ',F12.6,'  at x = ',F7.3)


C
      RETURN
      END ! GEOPAR


      SUBROUTINE AECALC(N,X,Y,T, ITYPE, 
     &                  AREA,XCEN,YCEN,EI11,EI22,APX1,APX2)
      DIMENSION X(*),Y(*),T(*)
C---------------------------------------------------------------
C     Calculates geometric properties of shape X,Y
C
C     Input:
C       N      number of points
C       X(.)   shape coordinate point arrays
C       Y(.)
C       T(.)   skin-thickness array, used only if ITYPE = 2
C       ITYPE  = 1 ...   integration is over whole area  dx dy
C              = 2 ...   integration is over skin  area   t ds
C
C     Output:
C       XCEN,YCEN  centroid location
C       EI11,EI22  principal moments of inertia
C       APX1,APX2  principal-axis angles
C---------------------------------------------------------------
      DATA PI / 3.141592653589793238 /
C
      SINT  = 0.0
      AINT  = 0.0
      XINT  = 0.0
      YINT  = 0.0
      XXINT = 0.0
      XYINT = 0.0
      YYINT = 0.0
C
      DO 10 IO = 1, N
        IF(IO.EQ.N) THEN
          IP = 1
        ELSE
          IP = IO + 1
        ENDIF
C
        DX =  X(IO) - X(IP)
        DY =  Y(IO) - Y(IP)
        XA = (X(IO) + X(IP))*0.50
        YA = (Y(IO) + Y(IP))*0.50
        TA = (T(IO) + T(IP))*0.50
C
        DS = SQRT(DX*DX + DY*DY)
        SINT = SINT + DS

        IF(ITYPE.EQ.1) THEN
C-------- integrate over airfoil cross-section
          DA = YA*DX
          AINT  = AINT  +       DA
          XINT  = XINT  + XA   *DA
          YINT  = YINT  + YA   *DA/2.0
          XXINT = XXINT + XA*XA*DA
          XYINT = XYINT + XA*YA*DA/2.0
          YYINT = YYINT + YA*YA*DA/3.0
        ELSE
C-------- integrate over skin thickness
          DA = TA*DS
          AINT  = AINT  +       DA
          XINT  = XINT  + XA   *DA
          YINT  = YINT  + YA   *DA
          XXINT = XXINT + XA*XA*DA
          XYINT = XYINT + XA*YA*DA
          YYINT = YYINT + YA*YA*DA
        ENDIF
C
 10   CONTINUE
C
      AREA = AINT
C
      IF(AINT .EQ. 0.0) THEN
        XCEN  = 0.0
        YCEN  = 0.0
        EI11  = 0.0
        EI22  = 0.0
        APX1 = 0.0
        APX2 = ATAN2(1.0,0.0)
        RETURN
      ENDIF
C
C
C---- calculate centroid location
      XCEN = XINT/AINT
      YCEN = YINT/AINT
C
C---- calculate inertias
      EIXX = YYINT - YCEN*YCEN*AINT
      EIXY = XYINT - XCEN*YCEN*AINT
      EIYY = XXINT - XCEN*XCEN*AINT
C
C---- set principal-axis inertias, EI11 is closest to "up-down" bending inertia
      EISQ  = 0.25*(EIXX - EIYY)**2  + EIXY**2
      SGN = SIGN( 1.0 , EIYY-EIXX )
      EI11 = 0.5*(EIXX + EIYY) - SGN*SQRT(EISQ)
      EI22 = 0.5*(EIXX + EIYY) + SGN*SQRT(EISQ)
C
      IF(EI11.EQ.0.0 .OR. EI22.EQ.0.0) THEN
C----- vanishing section stiffness
       APX1 = 0.0
       APX2 = ATAN2(1.0,0.0)
C
      ELSEIF(EISQ/(EI11*EI22) .LT. (0.001*SINT)**4) THEN
C----- rotationally-invariant section (circle, square, etc.)
       APX1 = 0.0
       APX2 = ATAN2(1.0,0.0)
C
      ELSE
C----- normal airfoil section
       C1 = EIXY
       S1 = EIXX-EI11
C
       C2 = EIXY
       S2 = EIXX-EI22
C
       IF(ABS(S1).GT.ABS(S2)) THEN
         APX1 = ATAN2(S1,C1)
         APX2 = APX1 + 0.5*PI
       ELSE
         APX2 = ATAN2(S2,C2)
         APX1 = APX2 - 0.5*PI
       ENDIF

       IF(APX1.LT.-0.5*PI) APX1 = APX1 + PI
       IF(APX1.GT.+0.5*PI) APX1 = APX1 - PI
       IF(APX2.LT.-0.5*PI) APX2 = APX2 + PI
       IF(APX2.GT.+0.5*PI) APX2 = APX2 - PI
C
      ENDIF
C
      RETURN
      END ! AECALC



      SUBROUTINE TCCALC(X,XP,Y,YP,S,N, 
     &                  THICK,XTHICK, CAMBR,XCAMBR )
      DIMENSION X(*),XP(*),Y(*),YP(*),S(*)
C---------------------------------------------------------------
C     Calculates max thickness and camber at airfoil points
C
C     Note: this routine does not find the maximum camber or 
C           thickness exactly as it only looks at discrete points
C
C     Input:
C       N      number of points
C       X(.)   shape coordinate point arrays
C       Y(.)
C
C     Output:
C       THICK  max thickness
C       CAMBR  max camber
C---------------------------------------------------------------
      CALL LEFIND(SLE,X,XP,Y,YP,S,N)
      XLE = SEVAL(SLE,X,XP,S,N)
      YLE = SEVAL(SLE,Y,YP,S,N)
      XTE = 0.5*(X(1)+X(N))
      YTE = 0.5*(Y(1)+Y(N))
      CHORD = SQRT((XTE-XLE)**2 + (YTE-YLE)**2)
C
C---- set unit chord-line vector
      DXC = (XTE-XLE) / CHORD
      DYC = (YTE-YLE) / CHORD
C
      THICK = 0.
      XTHICK = 0.
      CAMBR = 0.
      XCAMBR = 0.
C
C---- go over each point, finding the y-thickness and camber
      DO 30 I=1, N
        XBAR = (X(I)-XLE)*DXC + (Y(I)-YLE)*DYC
        YBAR = (Y(I)-YLE)*DXC - (X(I)-XLE)*DYC
C
C------ set point on the opposite side with the same chord x value
        CALL SOPPS(SOPP, S(I), X,XP,Y,YP,S,N, SLE)
        XOPP = SEVAL(SOPP,X,XP,S,N)
        YOPP = SEVAL(SOPP,Y,YP,S,N)
C
        YBAROP = (YOPP-YLE)*DXC - (XOPP-XLE)*DYC
C
        YC = 0.5*(YBAR+YBAROP)
        YT =  ABS(YBAR-YBAROP)
C
        IF(ABS(YC) .GT. ABS(CAMBR)) THEN
         CAMBR = YC
         XCAMBR = XOPP
        ENDIF
        IF(ABS(YT) .GT. ABS(THICK)) THEN
         THICK = YT
         XTHICK = XOPP
        ENDIF
   30 CONTINUE
C
      RETURN
      END ! TCCALC



      SUBROUTINE YSYM(X,XP,Y,YP,S,NX,N,ISIDE, XNEW,YNEW)
C---------------------------------------------------------
C     Makes passed-in airfoil symmetric about chord line.
C---------------------------------------------------------

      DIMENSION X(NX),XP(NX),Y(NX),YP(NX),S(NX)
      DIMENSION XNEW(NX), YNEW(NX)
C
      SREF = S(N) - S(1)
C
      CALL LEFIND(SLE,X,XP,Y,YP,S,N)
      XLE = SEVAL(SLE,X,XP,S,N)
      YLE = SEVAL(SLE,Y,YP,S,N)
      XTE = 0.5*(X(1)+X(N))
      YTE = 0.5*(Y(1)+Y(N))
      CHSQ = (XTE-XLE)**2 + (YTE-YLE)**2
C
C---- set unit chord-line vector
      DXC = (XTE-XLE) / SQRT(CHSQ)
      DYC = (YTE-YLE) / SQRT(CHSQ)
C
C---- find index of node ILE which is just before leading edge point
      DO 5 I=2, N
        DS = S(I) - S(I-1)
        IF(S(I)-SLE .GE. -0.01*DS) GO TO 6
 5    CONTINUE
 6    CONTINUE
      ILE = I-1
C
      DS = S(ILE+1) - S(ILE)
      IF(SLE-S(ILE-1) .LT. 0.1*DS) THEN
C------ point is just before LE, we will move it ahead to LE
        ILE1 = ILE - 1
        ILE2 = ILE + 1
      ELSE IF(S(ILE+1)-SLE .LT. 0.1*DS) THEN
C------ point is just after LE, we will move it back to LE
        ILE1 = ILE
        ILE2 = ILE + 2
      ELSE
C------ no point is near LE ... we will add new point
        ILE1 = ILE
        ILE2 = ILE + 1
      ENDIF
C
C---- set index limits of side which will set symmetric geometry
      IF(ISIDE.EQ.1) THEN
       IG1 = 1
       IG2 = ILE1
       IGDIR = +1
      ELSE
       IG1 = N
       IG2 = ILE2
       IGDIR = -1
      ENDIF
C
C---- set new number of points, including LE point
      NNEW = 2*(IABS(IG2-IG1) + 1) + 1
      IF(NNEW.GT.NX) STOP 'YSYM:  Array overflow on passed arrays.'
C
C---- set symmetric geometry
      DO 10 I=IG1, IG2, IGDIR
C
C------ coordinates in chord-line axes
        XBAR = (X(I)-XLE)*DXC + (Y(I)-YLE)*DYC
        YBAR = (Y(I)-YLE)*DXC - (X(I)-XLE)*DYC
C
        I1 = 1    + (I - IG1)*IGDIR
        I2 = NNEW - (I - IG1)*IGDIR
C
        XNEW(I1) = XLE + XBAR*DXC - YBAR*DYC
        XNEW(I2) = XLE + XBAR*DXC + YBAR*DYC
C
        YNEW(I1) = YLE + YBAR*DXC + XBAR*DYC
        YNEW(I2) = YLE - YBAR*DXC + XBAR*DYC
 10   CONTINUE
C
C---- set new LE point
      XNEW(NNEW/2+1) = XLE
      YNEW(NNEW/2+1) = YLE
C
C---- set geometry for returning
      N = NNEW
      DO 20 IG = 1, N
        IF(IGDIR.EQ.+1) THEN
         I = IG
        ELSE
         I = N - IG + 1
        ENDIF
        X(I) = XNEW(IG)
        Y(I) = YNEW(IG)
 20   CONTINUE
C
      CALL SCALC(X,Y,S,N)
      CALL SEGSPL(X,XP,S,N)
      CALL SEGSPL(Y,YP,S,N)
C
      RETURN
      END ! YSYM



      SUBROUTINE LERSCL(X,XP,Y,YP,S,N, DOC,RFAC, XNEW,YNEW)
C---------------------------------------------------------
C     Adjusts airfoil to scale LE radius by factor RFAC.
C     Blending of new shape is done with decay length DOC.
C---------------------------------------------------------
      DIMENSION X(*),XP(*),Y(*),YP(*),S(*)
      DIMENSION XNEW(*), YNEW(*)
C
      CALL LEFIND(SLE,X,XP,Y,YP,S,N)
      XLE = SEVAL(SLE,X,XP,S,N)
      YLE = SEVAL(SLE,Y,YP,S,N)
      XTE = 0.5*(X(1)+X(N))
      YTE = 0.5*(Y(1)+Y(N))
      CHORD = SQRT((XTE-XLE)**2 + (YTE-YLE)**2)
C
C---- set unit chord-line vector
      DXC = (XTE-XLE) / CHORD
      DYC = (YTE-YLE) / CHORD
C
      SRFAC = SQRT(ABS(RFAC))
C
C---- go over each point, changing the y-thickness appropriately
      DO 30 I=1, N
        XBAR = (X(I)-XLE)*DXC + (Y(I)-YLE)*DYC
        YBAR = (Y(I)-YLE)*DXC - (X(I)-XLE)*DYC
C
C------ set point on the opposite side with the same chord x value
        CALL SOPPS(SOPP, S(I), X,XP,Y,YP,S,N, SLE)
        XOPP = SEVAL(SOPP,X,XP,S,N)
        YOPP = SEVAL(SOPP,Y,YP,S,N)
C
        YBAROP = (YOPP-YLE)*DXC - (XOPP-XLE)*DYC
C
C------ thickness factor tails off exponentially towards trailing edge
        XOC = XBAR/CHORD
        ARG = MIN( XOC/DOC , 15.0 )
        TFAC = 1.0 - (1.0-SRFAC)*EXP(-ARG)
C
C------ set new chord x,y coordinates by changing thickness locally
        YBARCT = 0.5*(YBAR+YBAROP) + TFAC*0.5*(YBAR-YBAROP)
C
        XNEW(I) = XLE + XBAR  *DXC - YBARCT*DYC
        YNEW(I) = YLE + YBARCT*DXC + XBAR  *DYC
   30 CONTINUE
C
      RETURN
      END



      SUBROUTINE SSS(SS,S1,S2,DEL,XBF,YBF,X,XP,Y,YP,S,N,ISIDE)
      DIMENSION X(*),XP(*),Y(*),YP(*),S(*)
C----------------------------------------------------------------
C     Returns arc length points S1,S2 at flap surface break
C     locations.  S1 is on fixed airfoil part, S2 is on flap.
C     The points are defined according to two cases:
C
C
C     If DEL > 0:  Surface will be eliminated in S1 < s < S2
C
C     Returns the arc length values S1,S2 of the endpoints
C     of the airfoil surface segment which "disappears" as a
C     result of the flap deflection.  The line segments between
C     these enpoints and the flap hinge point (XBF,YBF) have
C     an included angle of DEL.  DEL is therefore the flap
C     deflection which will join up the points at S1,S2.
C     SS is an approximate arc length value near S1 and S2.
C     It is used as an initial guess for the Newton loop 
C     for S1 and S2.
C
C
C     If DEL = 0:  Surface will be created at s = S1 = S2
C
C     If DEL=0, then S1,S2 will cooincide, and will be located
C     on the airfoil surface where the segment joining the
C     point at S1,S2 and the hinge point is perpendicular to
C     the airfoil surface.  This will be the point where the
C     airfoil surface must be broken to permit a gap to open
C     as a result of the flap deflection.
C----------------------------------------------------------------
C
C---- convergence epsilon
      DATA EPS / 1.0E-5 /
C
      STOT = ABS( S(N) - S(1) )
C
      SIND = SIN(0.5*ABS(DEL))
C
      SSGN = 1.0
      IF(ISIDE.EQ.1) SSGN = -1.0
C
C---- initial guesses for S1, S2
      RSQ = (SEVAL(SS,X,XP,S,N)-XBF)**2 + (SEVAL(SS,Y,YP,S,N)-YBF)**2
      S1 = SS - (SIND*SQRT(RSQ) + EPS*STOT)*SSGN
      S2 = SS + (SIND*SQRT(RSQ) + EPS*STOT)*SSGN
C
C---- Newton iteration loop
      DO 10 ITER=1, 10
        X1  = SEVAL(S1,X,XP,S,N)
        X1P = DEVAL(S1,X,XP,S,N)
        Y1  = SEVAL(S1,Y,YP,S,N)
        Y1P = DEVAL(S1,Y,YP,S,N)
C
        X2  = SEVAL(S2,X,XP,S,N)
        X2P = DEVAL(S2,X,XP,S,N)
        Y2  = SEVAL(S2,Y,YP,S,N)
        Y2P = DEVAL(S2,Y,YP,S,N)
C
        R1SQ = (X1-XBF)**2 + (Y1-YBF)**2
        R2SQ = (X2-XBF)**2 + (Y2-YBF)**2
        R1 = SQRT(R1SQ)
        R2 = SQRT(R2SQ)
C
        RRSQ = (X1-X2)**2 + (Y1-Y2)**2
        RR = SQRT(RRSQ)
C
        IF(R1.LE.EPS*STOT .OR. R2.LE.EPS*STOT) THEN
         S1 = SS
         S2 = SS
         RETURN
        ENDIF
C
        R1_S1 = (X1P*(X1-XBF) + Y1P*(Y1-YBF))/R1
        R2_S2 = (X2P*(X2-XBF) + Y2P*(Y2-YBF))/R2
C
        IF(SIND.GT.0.01) THEN
C
         IF(RR.EQ.0.0) RETURN
C
         RR_S1 =  (X1P*(X1-X2) + Y1P*(Y1-Y2))/RR
         RR_S2 = -(X2P*(X1-X2) + Y2P*(Y1-Y2))/RR
C
C------- Residual 1: set included angle via dot product
         RS1 = ((XBF-X1)*(X2-X1) + (YBF-Y1)*(Y2-Y1))/RR - SIND*R1
         A11 = ((XBF-X1)*( -X1P) + (YBF-Y1)*( -Y1P))/RR
     &       + ((  -X1P)*(X2-X1) + (  -Y1P)*(Y2-Y1))/RR
     &       - ((XBF-X1)*(X2-X1) + (YBF-Y1)*(Y2-Y1))*RR_S1/RRSQ
     &       - SIND*R1_S1
         A12 = ((XBF-X1)*(X2P  ) + (YBF-Y1)*(Y2P  ))/RR
     &       - ((XBF-X1)*(X2-X1) + (YBF-Y1)*(Y2-Y1))*RR_S2/RRSQ
C
C------- Residual 2: set equal length segments
         RS2 = R1 - R2
         A21 = R1_S1
         A22 =    - R2_S2
C
        ELSE
C
C------- Residual 1: set included angle via small angle approximation
         RS1 = (R1+R2)*SIND + (S1 - S2)*SSGN
         A11 =  R1_S1 *SIND + SSGN
         A12 =  R2_S2 *SIND - SSGN
C
C------- Residual 2: set vector sum of line segments beteen the 
C-       endpoints and flap hinge to be perpendicular to airfoil surface.
         X1PP = D2VAL(S1,X,XP,S,N)
         Y1PP = D2VAL(S1,Y,YP,S,N)
         X2PP = D2VAL(S2,X,XP,S,N)
         Y2PP = D2VAL(S2,Y,YP,S,N)
C
         XTOT = X1+X2 - 2.0*XBF
         YTOT = Y1+Y2 - 2.0*YBF
C
         RS2 = XTOT*(X1P+X2P) + YTOT*(Y1P+Y2P)
         A21 =  X1P*(X1P+X2P) +  Y1P*(Y1P+Y2P) + XTOT*X1PP + YTOT*Y1PP
         A22 =  X2P*(X1P+X2P) +  Y2P*(Y1P+Y2P) + XTOT*X2PP + YTOT*Y2PP
C
        ENDIF
C
        DET =   A11*A22 - A12*A21
        DS1 = -(RS1*A22 - A12*RS2) / DET
        DS2 = -(A11*RS2 - RS1*A21) / DET
C
        DS1 = MIN( DS1 , 0.01*STOT )
        DS1 = MAX( DS1 , -.01*STOT )
        DS2 = MIN( DS2 , 0.01*STOT )
        DS2 = MAX( DS2 , -.01*STOT )
C
        S1 = S1 + DS1
        S2 = S2 + DS2
        IF(ABS(DS1)+ABS(DS2) .LT. EPS*STOT ) GO TO 11
   10 CONTINUE
      WRITE(*,*) 'SSS: failed to converge subtending angle points'
      S1 = SS
      S2 = SS
C
   11 CONTINUE
C
C---- make sure points are identical if included angle is zero.
      IF(DEL.EQ.0.0) THEN
       S1 = 0.5*(S1+S2)
       S2 = S1
      ENDIF
C
      RETURN
      END


      SUBROUTINE CLIS(X,XP,Y,YP,S,N)
      DIMENSION X(*), XP(*), Y(*), YP(*), S(*)
C-------------------------------------------------------------------
C     Displays curvatures at panel nodes.
C-------------------------------------------------------------------
      PI = 4.0*ATAN(1.0)
C
      CMAX = 0.0
      IMAX = 1
C
C---- go over each point, calculating curvature
      WRITE(*,1050)
      DO 30 I=1, N
        IF(I.EQ.1) THEN
         ARAD = ATAN2(-YP(I),-XP(I))
        ELSE
         ARAD = ATANC(-YP(I),-XP(I),ARAD)
        ENDIF
        ADEG = ARAD * 180.0/PI
        CV = CURV(S(I),X,XP,Y,YP,S,N)
        WRITE(*,1100) I, X(I), Y(I), S(I), ADEG, CV
        IF(ABS(CV) .GT. ABS(CMAX)) THEN
         CMAX = CV
         IMAX = I
        ENDIF
   30 CONTINUE
C
      WRITE(*,1200) CMAX, IMAX, X(IMAX), Y(IMAX), S(IMAX)
C
      RETURN
C
 1050 FORMAT(
     &    /'  i        x         y         s       theta        curv')
CCC           120   0.12134  -0.10234  -0.30234   180.024      2025.322
 1100 FORMAT(1X,I3, 3F10.5, F11.3, F12.3)
 1200 FORMAT(/' Maximum curvature =', F14.3,
     &        '   at  i,x,y,s  = ', I3, 3F9.4 )
      END ! CLIS




      SUBROUTINE PLTCRV(SBLE,XB,XBP,YB,YBP,SB,NB,CV)
C
C---- Plot the curvature on the blade
C
      DIMENSION  XB(NB),XBP(NB),YB(NB),YBP(NB),SB(NB),CV(NB)
      CHARACTER ANS*1, ANSARG*128
      LOGICAL LCVEXP, ERROR
C
      DATA LMASK0, LMASK1, LMASK2, LMASK3 / -1, -32640, -30584, -21846 /
C
      CH = 0.01
      LCVEXP = .FALSE.
      CVEXP = 1.0/3.0
C
 10   SBTOT = 0.5*(SB(NB)-SB(1))
      XTE = 0.5*(XB(NB)+XB(1))
      YTE = 0.5*(YB(NB)+YB(1))
      XLE = SEVAL(SBLE,XB,XBP,SB,NB)
      YLE = SEVAL(SBLE,YB,YBP,SB,NB)
      CVLE = CURV(SBLE,XB,XBP,YB,YBP,SB,NB) * SBTOT
      IF(LCVEXP) CVLE = CVLE**CVEXP
C
      CVMAX = CVLE
      SVMAX = SBLE
      XVMAX = XLE
      CVMIN = CVLE
      SVMIN = SBLE
      XVMIN = XLE
      DO I=1, NB
C---- set up curvature array
        CV(I) = CURV(SB(I),XB,XBP,YB,YBP,SB,NB) * SBTOT
        IF(LCVEXP) THEN
          IF(CV(I).GT.0.0) THEN
            CV(I) = CV(I)**CVEXP
           ELSEIF(CV(I).EQ.0.0) THEN
            CV(I) = 0.0
           ELSEIF(CV(I).LT.0.0) THEN
            CVSGN = SIGN(1.0,CV(I))
            CV(I) = CVSGN*(ABS(CV(I))**CVEXP)
          ENDIF
        ENDIF
        IF(CV(I).GT.CVMAX) THEN
          CVMAX = CV(I)
          SVMAX = SB(I)
          XVMAX = XB(I)
        ENDIF
        IF(CV(I).LT.CVMIN) THEN
          CVMIN = CV(I)
          SVMIN = SB(I)
          XVMIN = XB(I)
        ENDIF
        IF(SB(I).LE.SBLE) ILE = I
      END DO
C
cc    CALL SCALIT(1,CVMAX-CVMIN,0.0,CWT)
      CVMX = CVMAX
      CVMN = CVMIN
      CALL AXISADJ(CVMN,CVMX,CVSPAN,CVDEL,NCVTICS)
      CWT = 1.0/CVSPAN
      XMX = XTE
      XMN = XLE
      CALL AXISADJ(XMN,XMX,XSPAN,XDEL,NXTICS)
C--- Correct min/max for points just slightly off from a major division in X
      IF(XLE-XMN.GT.0.95*XDEL) XMN = XMN + XDEL
      IF(XMX-XTE.GT.0.95*XDEL) XMX = XMX - XDEL
      XSPAN = XMX-XMN
      XWT = 1.0/XSPAN
C
      PAR  =  0.75
      XLEN =  0.8
      YLEN =  PAR*XLEN
      XMIN =  XMN
      XMAX =  XMX
      XDEL =  XDEL
      NXG  =  (XMAX-XMIN)/XDEL
      XSF  =  XLEN/(XMAX-XMIN)
      XOF  =  XMN
      YMIN =  CVMN
      YMAX =  CVMX
      YDEL =  CVDEL
      NYG  =  (YMAX-YMIN)/YDEL
      YSF  =  YLEN/(YMAX-YMIN)
      YOF  =  0.0
C
      CALL PLTINI
      CALL PLOT(0.14,0.1+YLEN*(-YMIN/(YMAX-YMIN)),-3)
C
C--- X axis (x/c)
      CALL NEWPEN(2)
      XLN = XLEN
      IF(XMIN.EQ.0.0) XLN = -XLN
      CALL XAXIS(0.0,0.0,XLN,XSF*XDEL,XMIN,XDEL,CH,1)  
      XC = XSF*3.5*XDEL -0.5*1.2*CH
      YC =              -3.5*1.2*CH
      CALL PLCHAR(XC,YC,1.2*CH,'X',0.0,1)
C
C--- Y axis (curvature)
      CALL YAXIS(0.0,YSF*YMIN,YLEN,YSF*YDEL,YMIN,YDEL,CH,1)
      XC =              -4.5*1.2*CH
      YC = YSF*(YMAX-0.5*YDEL) - 0.5*1.2*CH
      IF(LCVEXP) THEN
        CALL PLCHAR(XC-4.5*1.2*CH,YC,1.2*CH,'CV^',0.0,3)
        CALL PLNUMB(XC-1.5*1.2*CH,YC+0.5*CH,CH,CVEXP,0.0,2)
       ELSE
        CALL PLCHAR(XC,YC,1.2*CH,'CV',0.0,2)
      ENDIF
C
      CALL PLGRID(0.0,YSF*YMIN, NXG,XSF*XDEL, NYG,YSF*YDEL, LMASK2)
      XC = 0.0
      YC = YSF*YMAX + 1.0*1.2*CH
      IF(LCVEXP) THEN
        CALL PLCHAR(XC,YC,1.2*CH,'Curvature^n vs X',0.0,16)
       ELSE
        CALL PLCHAR(XC,YC,1.2*CH,'Curvature vs X',0.0,16)
      ENDIF
C
C--- Upper surface curvature
      CALL GETCOLOR(ICOL0)
      CALL NEWCOLORNAME('yellow')
      CALL XYLINE(ILE,XB,CV,XOF,XSF,YOF,YSF,1)
      XC = XSF*(XB(2*ILE/3)-XOF)
      YC = YSF*(CV(2*ILE/3)-YOF)
      CALL PLCHAR(XC+0.5*CH,YC+0.5*CH,CH,'Upper',0.0,5)
C
C--- LE curvature
      CALL NEWCOLORNAME('red')
      XC = XSF*(XLE-XOF)
      YC = YSF*(CVLE-YOF)
      CALL PLSYMB(XC,YC,CH,3,0.0,0)
      CALL PLCHAR(XC+1.0*CH,YC-0.5*CH,CH,'LE',0.0,2)
C
C--- Lower surface curvature
      CALL NEWCOLORNAME('cyan')
      CALL XYLINE(NB-ILE+1,XB(ILE),CV(ILE),XOF,XSF,YOF,YSF,2)
      XC = XSF*(XB((ILE+NB)/2)-XOF)
      YC = YSF*(CV((ILE+NB)/2)-YOF)
      CALL PLCHAR(XC+0.5*CH,YC+0.5*CH,CH,'Lower',0.0,5)
C
      CALL NEWCOLOR(ICOL0)
      CALL PLFLUSH
C
C
 20   WRITE(*,*) ' '
      WRITE(*,*) 'Airfoil curvature (yellow-upper, cyan-lower) '
      IF(LCVEXP) THEN
       WRITE(*,*) ' Range compressed using CV=(curvature)^n with n =',
     &              CVEXP
      ENDIF
      WRITE(*,*) ' CVLE  = ',CVLE, ' at S = ',SBLE, ' at X = ',XLE
      WRITE(*,*) ' CVmax = ',CVMAX,' at S = ',SVMAX,' at X = ',XVMAX
      WRITE(*,*) ' CVmin = ',CVMIN,' at S = ',SVMIN,' at X = ',XVMIN
C
      WRITE(*,*) ' '
      WRITE(*,*) 'Enter C for curvature    plot'
      WRITE(*,*) 'Enter N for curvature**N plot'
      WRITE(*,*) 'Hit <cr> to exit'
      ANSARG = ' '
      CALL ASKC('..CPLO^',ANS,ANSARG)
      IF(ANS.EQ.' ') RETURN
C
      RINPUT = 0.0
      NINPUT = 1
      CALL GETFLT(ANSARG,RINPUT,NINPUT,ERROR)
C
      IF(ANS.EQ.'n' .OR. ANS.EQ.'N') THEN
        IF(NINPUT.GE.1) THEN
          CVEXP = RINPUT
         ELSE
          CVEXP = 0.3
          CALL ASKR('Enter curvature exponent (default 0.3)^',CVEXP)
        ENDIF
        LCVEXP = .TRUE.
        GO TO 10
      ENDIF 
      IF(ANS.EQ.'c' .OR. ANS.EQ.'C') THEN
        LCVEXP = .FALSE.
        GO TO 10
      ENDIF 
      GO TO 20
C
 1000 FORMAT(A)
      END




      SUBROUTINE CANG(X,Y,N,IPRINT, IMAX,AMAX)
      DIMENSION X(*), Y(*)
C-------------------------------------------------------------------
C     IPRINT=2:   Displays all panel node corner angles
C     IPRINT=1:   Displays max panel node corner angle
C     IPRINT=0:   No display... just returns values
C-------------------------------------------------------------------
C
      AMAX = 0.0
      IMAX = 1
C
C---- go over each point, calculating corner angle
      IF(IPRINT.EQ.2) WRITE(*,1050)
      DO 30 I=2, N-1
        DX1 = X(I) - X(I-1)
        DY1 = Y(I) - Y(I-1)
        DX2 = X(I) - X(I+1)
        DY2 = Y(I) - Y(I+1)
C
C------ allow for doubled points
        IF(DX1.EQ.0.0 .AND. DY1.EQ.0.0) THEN
         DX1 = X(I) - X(I-2)
         DY1 = Y(I) - Y(I-2)
        ENDIF
        IF(DX2.EQ.0.0 .AND. DY2.EQ.0.0) THEN
         DX2 = X(I) - X(I+2)
         DY2 = Y(I) - Y(I+2)
        ENDIF
C
        CROSSP = (DX2*DY1 - DY2*DX1)
     &         / SQRT((DX1**2 + DY1**2) * (DX2**2 + DY2**2))
        ANGL = ASIN(CROSSP)*(180.0/3.1415926)
        IF(IPRINT.EQ.2) WRITE(*,1100) I, X(I), Y(I), ANGL
        IF(ABS(ANGL) .GT. ABS(AMAX)) THEN
         AMAX = ANGL
         IMAX = I
        ENDIF
   30 CONTINUE
C
      IF(IPRINT.GE.1) WRITE(*,1200) AMAX, IMAX, X(IMAX), Y(IMAX)
C
      RETURN
C
 1050 FORMAT(/'  i       x        y      angle')
CCC             120   0.2134  -0.0234   25.322
 1100 FORMAT(1X,I3, 2F9.4, F9.3)
 1200 FORMAT(/' Maximum panel corner angle =', F7.3,
     &        '   at  i,x,y  = ', I3, 2F9.4 )
      END ! CANG



      SUBROUTINE INTER(X0,XP0,Y0,YP0,S0,N0,SLE0,
     &                 X1,XP1,Y1,YP1,S1,N1,SLE1,
     &                 X,Y,N,FRAC)
C     .....................................................................
C
C     Interpolates two source airfoil shapes into an "intermediate" shape.
C
C     Procedure:
C        The interpolated x coordinate at a given normalized spline 
C        parameter value is a weighted average of the two source 
C        x coordinates at the same normalized spline parameter value.
C        Ditto for the y coordinates. The normalized spline parameter 
C        runs from 0 at the leading edge to 1 at the trailing edge on 
C        each surface.
C     .....................................................................
C
      REAL X0(N0),Y0(N0),XP0(N0),YP0(N0),S0(N0)
      REAL X1(N1),Y1(N1),XP1(N1),YP1(N1),S1(N1)
      REAL X(*),Y(*)
C
C---- number of points in interpolated airfoil is the same as in airfoil 0
      N = N0
C
C---- interpolation weighting fractions
      F0 = 1.0 - FRAC
      F1 = FRAC
C
C---- top side spline parameter increments
      TOPS0 = S0(1) - SLE0
      TOPS1 = S1(1) - SLE1
C
C---- bottom side spline parameter increments
      BOTS0 = S0(N0) - SLE0
      BOTS1 = S1(N1) - SLE1
C
      DO 50 I=1, N
C
C------ normalized spline parameter is taken from airfoil 0 value
        IF(S0(I).LT.SLE0) SN = (S0(I) - SLE0) / TOPS0    ! top side
        IF(S0(I).GE.SLE0) SN = (S0(I) - SLE0) / BOTS0    ! bottom side
C
C------ set actual spline parameters
        ST0 = S0(I)
        IF(ST0.LT.SLE0) ST1 = SLE1 + TOPS1 * SN
        IF(ST0.GE.SLE0) ST1 = SLE1 + BOTS1 * SN
C
C------ set input coordinates at common spline parameter location
        XT0 = SEVAL(ST0,X0,XP0,S0,N0)
        YT0 = SEVAL(ST0,Y0,YP0,S0,N0)
        XT1 = SEVAL(ST1,X1,XP1,S1,N1)
        YT1 = SEVAL(ST1,Y1,YP1,S1,N1)
C
C------ set interpolated x,y coordinates
        X(I) = F0*XT0 + F1*XT1
        Y(I) = F0*YT0 + F1*YT1
C
   50 CONTINUE
C
      RETURN
      END ! INTER



      SUBROUTINE INTERX(X0,XP0,Y0,YP0,S0,N0,SLE0,
     &                  X1,XP1,Y1,YP1,S1,N1,SLE1,
     &                  X,Y,N,FRAC)
C     .....................................................................
C
C     Interpolates two source airfoil shapes into an "intermediate" shape.
C
C     Procedure:
C        The interpolated x coordinate at a given normalized spline 
C        parameter value is a weighted average of the two source 
C        x coordinates at the same normalized spline parameter value.
C        Ditto for the y coordinates. The normalized spline parameter 
C        runs from 0 at the leading edge to 1 at the trailing edge on 
C        each surface.
C     .....................................................................
C
      REAL X0(N0),Y0(N0),XP0(N0),YP0(N0),S0(N0)
      REAL X1(N1),Y1(N1),XP1(N1),YP1(N1),S1(N1)
      REAL X(N),Y(N)
C
C---- number of points in interpolated airfoil is the same as in airfoil 0
      N = N0
C
C---- interpolation weighting fractions
      F0 = 1.0 - FRAC
      F1 = FRAC
C
      XLE0 = SEVAL(SLE0,X0,XP0,S0,N0)
      XLE1 = SEVAL(SLE1,X1,XP1,S1,N1)
C
      DO 50 I=1, N
C
C------ normalized x parameter is taken from airfoil 0 value
        IF(S0(I).LT.SLE0) XN = (X0(I) - XLE0) / (X0( 1) - XLE0)
        IF(S0(I).GE.SLE0) XN = (X0(I) - XLE0) / (X0(N0) - XLE0)
C
C------ set target x and initial spline parameters
        XT0 = X0(I)
        ST0 = S0(I)
        IF(ST0.LT.SLE0) THEN
         XT1 = XLE1 + (X1( 1) - XLE1) * XN
         ST1 = SLE1 + (S1( 1) - SLE1) * XN
        ELSE
         XT1 = XLE1 + (X1(N1) - XLE1) * XN
         ST1 = SLE1 + (S1(N1) - SLE1) * XN
        ENDIF
C
        CALL SINVRT(ST0,XT0,X0,XP0,S0,N0)
        CALL SINVRT(ST1,XT1,X1,XP1,S1,N1)
C
C------ set input coordinates at common spline parameter location
        XT0 = SEVAL(ST0,X0,XP0,S0,N0)
        YT0 = SEVAL(ST0,Y0,YP0,S0,N0)
        XT1 = SEVAL(ST1,X1,XP1,S1,N1)
        YT1 = SEVAL(ST1,Y1,YP1,S1,N1)
C
C------ set interpolated x,y coordinates
        X(I) = F0*XT0 + F1*XT1
        Y(I) = F0*YT0 + F1*YT1
C
   50 CONTINUE
C
      RETURN
      END ! INTERX





      SUBROUTINE BENDUMP(N,X,Y)
      REAL X(*), Y(*)
C
      PEX = 16.0
      CALL IJSECT(N,X,Y, PEX,
     &  AREA, SLEN, 
     &  XMIN, XMAX, XEXINT,
     &  YMIN, YMAX, YEXINT,
     &  XC , YC , 
     &  XCT, YCT, 
     &  AIXX , AIYY , 
     &  AIXXT, AIYYT,
     &  AJ   , AJT    )
c      CALL IJSECT(N,X,Y, PEX,
c     &    AREA, SLEN, 
c     &    XC, XMIN, XMAX, XEXINT,
c     &    YC, YMIN, YMAX, YEXINT,
c     &    AIXX, AIXXT,
c     &    AIYY, AIYYT,
c     &    AJ  , AJT   )
C
      WRITE(*,*) 
      WRITE(*,1200) 'Area =', AREA
      WRITE(*,1200) 'Slen =', SLEN
      WRITE(*,*)
      WRITE(*,1200) 'X-bending parameters(solid):'
      WRITE(*,1200) '        Xc =', XC
      WRITE(*,1200) '  max X-Xc =', XMAX-XC
      WRITE(*,1200) '  min X-Xc =', XMIN-XC
      WRITE(*,1200) '       Iyy =', AIYY
      XBAR = MAX( ABS(XMAX-XC) , ABS(XMIN-XC) )
      WRITE(*,1200) ' Iyy/(X-Xc)=', AIYY /XBAR
      WRITE(*,*)
      WRITE(*,1200) 'Y-bending parameters(solid):'
      WRITE(*,1200) '        Yc =', YC
      WRITE(*,1200) '  max Y-Yc =', YMAX-YC
      WRITE(*,1200) '  min Y-Yc =', YMIN-YC
      WRITE(*,1200) '       Ixx =', AIXX
      YBAR = MAX( ABS(YMAX-YC) , ABS(YMIN-YC) )
      WRITE(*,1200) ' Ixx/(Y-Yc)=', AIXX /YBAR
      WRITE(*,*)
      WRITE(*,1200) '       J   =', AJ
C
      WRITE(*,*)
      WRITE(*,*)
      WRITE(*,1200) 'X-bending parameters(skin):'
      WRITE(*,1200) '         Xc =', XCT
      WRITE(*,1200) '   max X-Xc =', XMAX-XCT
      WRITE(*,1200) '   min X-Xc =', XMIN-XCT
      WRITE(*,1200) '      Iyy/t =', AIYYT
      XBART = MAX( ABS(XMAX-XCT) , ABS(XMIN-XCT) )
      WRITE(*,1200) ' Iyy/t(X-Xc)=', AIYYT /XBART
      WRITE(*,*)
      WRITE(*,1200) 'Y-bending parameters(skin):'
      WRITE(*,1200) '         Yc =', YCT
      WRITE(*,1200) '   max Y-Yc =', YMAX-YCT
      WRITE(*,1200) '   min Y-Yc =', YMIN-YCT
      WRITE(*,1200) '      Ixx/t =', AIXXT
      YBART = MAX( ABS(YMAX-YCT) , ABS(YMIN-YCT) )
      WRITE(*,1200) ' Ixx/t(Y-Yc)=', AIXXT /YBART
      WRITE(*,*)
      WRITE(*,1200) '      J/t   =', AJT
C
c      WRITE(*,*)
c      WRITE(*,1200) '  power-avg X-Xc =', XEXINT
c      WRITE(*,1200) '  power-avg Y-Yc =', YEXINT
C
      RETURN
C
 1200 FORMAT(1X,A,G14.6)
      END ! BENDUMP



      SUBROUTINE BENDUMP2(N,X,Y,T)
      REAL X(*), Y(*), T(*)
C
      DTR = ATAN(1.0) / 45.0
C
      PEX = 16.0
      CALL IJSECT(N,X,Y, PEX,
     &  AREA, SLEN, 
     &  XMIN, XMAX, XEXINT,
     &  YMIN, YMAX, YEXINT,
     &  XC , YC , 
     &  XCT, YCT, 
     &  AIXX , AIYY , 
     &  AIXXT, AIYYT,
     &  AJ   , AJT    )
c      CALL IJSECT(N,X,Y, PEX,
c     &    AREA, SLEN, 
c     &    XC, XMIN, XMAX, XEXINT,
c     &    YC, YMIN, YMAX, YEXINT,
c     &    AIXX, AIXXT,
c     &    AIYY, AIYYT,
c     &    AJ  , AJT   )
C
C
      CALL AECALC(N,X,Y,T, 1, 
     &            AREA,XCENA,YCENA,EI11A,EI22A,APX1A,APX2A)
C
      CALL AECALC(N,X,Y,T, 2, 
     &            SLEN,XCENT,YCENT,EI11T,EI22T,APX1T,APX2T)
C

      WRITE(*,*) 
      WRITE(*,1200) 'Area =', AREA
      WRITE(*,1200) 'Slen =', SLEN
      WRITE(*,*)
      WRITE(*,1200) 'X-bending parameters:'
      WRITE(*,1200) 'solid centroid Xc=', XCENA
      WRITE(*,1200) 'skin  centroid Xc=', XCENT
      WRITE(*,1200) ' solid max X-Xc  =', XMAX-XCENA
      WRITE(*,1200) ' solid min X-Xc  =', XMIN-XCENA
      WRITE(*,1200) ' skin  max X-Xc  =', XMAX-XCENT
      WRITE(*,1200) ' skin  min X-Xc  =', XMIN-XCENT
      WRITE(*,1200) '     solid Iyy   =', EI22A
      WRITE(*,1200) '     skin  Iyy/t =', EI22T
      XBARA = MAX( ABS(XMAX-XCENA) , ABS(XMIN-XCENA) )
      XBART = MAX( ABS(XMAX-XCENT) , ABS(XMIN-XCENT) )
      WRITE(*,1200) ' solid Iyy/(X-Xc)=', EI22A/XBARA
      WRITE(*,1200) ' skin Iyy/t(X-Xc)=', EI22T/XBART
C
      WRITE(*,*)
      WRITE(*,1200) 'Y-bending parameters:'
      WRITE(*,1200) 'solid centroid Yc=', YCENA
      WRITE(*,1200) 'skin  centroid Yc=', YCENT
      WRITE(*,1200) ' solid max Y-Yc  =', YMAX-YCENA
      WRITE(*,1200) ' solid min Y-Yc  =', YMIN-YCENA
      WRITE(*,1200) ' skin  max Y-Yc  =', YMAX-YCENT
      WRITE(*,1200) ' skin  min Y-Yc  =', YMIN-YCENT
      WRITE(*,1200) '     solid Ixx   =', EI11A
      WRITE(*,1200) '     skin  Ixx/t =', EI11T
      YBARA = MAX( ABS(YMAX-YCENA) , ABS(YMIN-YCENA) )
      YBART = MAX( ABS(YMAX-YCENT) , ABS(YMIN-YCENT) )
      WRITE(*,1200) ' solid Ixx/(Y-Yc)=', EI11A/YBARA
      WRITE(*,1200) ' skin Ixx/t(Y-Yc)=', EI11T/YBART
C
      WRITE(*,*)
      WRITE(*,1200) ' solid principal axis angle (deg ccw) =', APX1A/DTR
      WRITE(*,1200) ' skin  principal axis angle (deg ccw) =', APX1T/DTR

c      WRITE(*,*)
c      WRITE(*,1200) '  power-avg X-Xc =', XEXINT
c      WRITE(*,1200) '  power-avg Y-Yc =', YEXINT
C
      WRITE(*,*)
      WRITE(*,1200) '    solid J     =', AJ
      WRITE(*,1200) '    skin  J/t   =', AJT
      RETURN
C
 1200 FORMAT(1X,A,G14.6)
      END ! BENDUMP2



      SUBROUTINE IJSECT(N,X,Y, PEX,
     &  AREA, SLEN, 
     &  XMIN, XMAX, XEXINT,
     &  YMIN, YMAX, YEXINT,
     &  XC , YC , 
     &  XCT, YCT, 
     &  AIXX , AIYY , 
     &  AIXXT, AIYYT,
     &  AJ   , AJT    )
      DIMENSION X(*), Y(*)
C
      XMIN = X(1)
      XMAX = X(1)
      YMIN = Y(1)
      YMAX = Y(1)
C
      DX = X(1) - X(N)
      DY = Y(1) - Y(N)
      DS = SQRT(DX*DX + DY*DY)
      XAVG = 0.5*(X(1) + X(N))
      YAVG = 0.5*(Y(1) + Y(N))
C
      X_DY   = DY * XAVG
      XX_DY  = DY * XAVG**2
      XXX_DY = DY * XAVG**3
      X_DS   = DS * XAVG
      XX_DS  = DS * XAVG**2
C
      Y_DX   = DX * YAVG
      YY_DX  = DX * YAVG**2
      YYY_DX = DX * YAVG**3
      Y_DS   = DS * YAVG
      YY_DS  = DS * YAVG**2
C
      C_DS   = DS
C
      DO 10 I = 2, N
        DX = X(I) - X(I-1)
        DY = Y(I) - Y(I-1)
        DS = SQRT(DX*DX + DY*DY)
        XAVG = 0.5*(X(I) + X(I-1))
        YAVG = 0.5*(Y(I) + Y(I-1))
C
        X_DY   = X_DY   + DY * XAVG
        XX_DY  = XX_DY  + DY * XAVG**2
        XXX_DY = XXX_DY + DY * XAVG**3
        X_DS   = X_DS   + DS * XAVG
        XX_DS  = XX_DS  + DS * XAVG**2
C
        Y_DX   = Y_DX   + DX * YAVG
        YY_DX  = YY_DX  + DX * YAVG**2
        YYY_DX = YYY_DX + DX * YAVG**3
        Y_DS   = Y_DS   + DS * YAVG
        YY_DS  = YY_DS  + DS * YAVG**2
C
        C_DS   = C_DS   + DS
C
        XMIN = MIN(XMIN,X(I))
        XMAX = MAX(XMAX,X(I))
        YMIN = MIN(YMIN,Y(I))
        YMAX = MAX(YMAX,Y(I))
 10   CONTINUE
C
      AREA = -Y_DX
      SLEN =  C_DS
C
      IF(AREA.EQ.0.0) RETURN
C
      XC = XX_DY / (2.0*X_DY)
      XCT = X_DS / C_DS
      AIYY  =  XXX_DY/3.0 - XX_DY*XC      + X_DY*XC**2
      AIYYT =   XX_DS     -  X_DS*XCT*2.0 + C_DS*XCT**2
C
      YC = YY_DX / (2.0*Y_DX)
      YCT = Y_DS / C_DS
      AIXX  = -YYY_DX/3.0 + YY_DX*YC      - Y_DX*YC**2
      AIXXT =   YY_DS     -  Y_DS*YCT*2.0 + C_DS*YCT**2
C
C
      SINT = 0.
      XINT = 0.
      YINT = 0.
C
      DO 20 I=2, N
        DX = X(I) - X(I-1)
        DY = Y(I) - Y(I-1)
        DS = SQRT(DX*DX + DY*DY)
        XAVG = 0.5*(X(I) + X(I-1)) - XC
        YAVG = 0.5*(Y(I) + Y(I-1)) - YC
C
        SINT = SINT + DS
cc        XINT = XINT + DS * ABS(XAVG)**PEX
cc        YINT = YINT + DS * ABS(YAVG)**PEX
 20   CONTINUE
C
      DO I=1, N-1
        IF(X(I+1) .GE. X(I)) GO TO 30
      ENDDO
      IMID = N/2
 30   IMID = I
C
      AJ = 0.0
      DO I = 2, IMID
        XAVG = 0.5*(X(I) + X(I-1))
        YAVG = 0.5*(Y(I) + Y(I-1))
        DX = X(I-1) - X(I)
C
        IF(XAVG.GT.X(N)) THEN
         YOPP = Y(N)
         GO TO 41
        ENDIF
        IF(XAVG.LE.X(IMID)) THEN
         YOPP = Y(IMID)
         GO TO 41
        ENDIF
C
        DO J = N, IMID, -1
          IF(XAVG.GT.X(J-1) .AND. XAVG.LE.X(J)) THEN
            FRAC = (XAVG - X(J-1))
     &           / (X(J) - X(J-1))
            YOPP = Y(J-1) + (Y(J)-Y(J-1))*FRAC
            GO TO 41
          ENDIF
        ENDDO
 41     CONTINUE
C
        AJ = AJ + ABS(YAVG-YOPP)**3 * DX / 3.0
      ENDDO
C
      AJT = 4.0*AREA**2/SLEN
C
cc      XEXINT = (XINT/SINT)**(1.0/PEX)
cc      YEXINT = (YINT/SINT)**(1.0/PEX)
C
      RETURN
      END ! IJSECT
C


      SUBROUTINE AREFINE(X,Y,S,XS,YS,N, ATOL, 
     &                   NDIM,NNEW,XNEW,YNEW,X1,X2)
C-------------------------------------------------------------
C     Adds points to a x,y spline contour wherever 
C     the angle between adjacent segments at a node 
C     exceeds a specified threshold.  The points are 
C     added 1/3 of a segment before and after the 
C     offending node.
C
C     The point adding is done only within X1..X2.
C
C     Intended for doubling the number of points
C     of Eppler and Selig airfoils so that they are
C     suitable for clean interpolation using Xfoil's
C     arc-length spline routines.
C------------------------------------------------------
      REAL X(*), Y(*), S(*), XS(*), YS(*)
      REAL XNEW(NDIM), YNEW(NDIM)
      LOGICAL LREF
C
      ATOLR = ATOL * 3.14159/180.0
C
      K = 1
      XNEW(K) = X(1)
      YNEW(K) = Y(1)
C
      DO 10 I = 2, N-1
        IM = I-1
        IP = I+1
C
        DXM = X(I) - X(I-1)
        DYM = Y(I) - Y(I-1)
        DXP = X(I+1) - X(I)
        DYP = Y(I+1) - Y(I)
C
        CRSP = DXM*DYP - DYM*DXP
        DOTP = DXM*DXP + DYM*DYP
        IF(CRSP.EQ.0.0 .AND. DOTP.EQ.0.0) THEN
         ASEG = 0.0
        ELSE
         ASEG = ATAN2( CRSP , DOTP )
        ENDIF
C
        LREF = ABS(ASEG) .GT. ATOLR
C
        IF(LREF) THEN
C------- add extra point just before this node
         SMID = S(I) - 0.3333*(S(I)-S(I-1))
         XK = SEVAL(SMID,X,XS,S,N)
         YK = SEVAL(SMID,Y,YS,S,N)
         IF(XK.GE.X1 .AND. XK.LE.X2) THEN
          K = K + 1
          IF(K .GT. NDIM) GO TO 90
          XNEW(K) = XK
          YNEW(K) = YK
         ENDIF
        ENDIF
C
C------ add the node itself
        K = K + 1
        IF(K .GT. NDIM) GO TO 90
        XNEW(K) = X(I)
        YNEW(K) = Y(I)
C
        IF(LREF) THEN
C------- add extra point just after this node
         SMID = S(I) + 0.3333*(S(I+1)-S(I))
         XK = SEVAL(SMID,X,XS,S,N)
         YK = SEVAL(SMID,Y,YS,S,N)
         IF(XK.GE.X1 .AND. XK.LE.X2) THEN
          K = K + 1
          IF(K .GT. NDIM) GO TO 90
          XNEW(K) = XK
          YNEW(K) = YK
         ENDIF
        ENDIF
 10   CONTINUE
C
      K = K + 1
      IF(K .GT. NDIM) GO TO 90
      XNEW(K) = X(N)
      YNEW(K) = Y(N)
C
      NNEW = K
      RETURN
C
 90   CONTINUE
      WRITE(*,*) 'SDOUBLE:  Arrays will overflow.  No action taken.'
      NNEW = 0
      RETURN
C
      END ! AREFINE


      SUBROUTINE SCHECK(X,Y,N, STOL, LCHANGE)
C-------------------------------------------------------------
C     Removes points from an x,y spline contour wherever 
C     the size of a segment between nodes falls below a 
C     a specified threshold of the adjacent segments.  
C     The two node points defining the short segment are
C     replaced with a single node at their midpoint.
C     Note that the number of nodes may be altered by 
C     this routine.
C
C     Intended for eliminating odd "micro" panels 
C     that occur when blending a flap to a foil.
C     If LCHANGE is set on return the airfoil definition 
C     has been changed and resplining should be done.
C
C     The recommended value for STOL is 0.05 (meaning 
C     segments less than 5% of the length of either adjoining 
C     segment are removed).  4/24/01 HHY
C------------------------------------------------------
      REAL X(*), Y(*)
      LOGICAL LCHANGE
C
      LCHANGE = .FALSE.
C--- Check STOL for sanity
      IF(STOL.GT.0.3) THEN
       WRITE(*,*) 'SCHECK:  Bad value for small panels (STOL > 0.3)'
       RETURN
      ENDIF
C
 10   DO 20 I = 2, N-2
        IM1 = I-1
        IP1 = I+1
        IP2 = I+2
C
        DXM1 = X(I) - X(I-1)
        DYM1 = Y(I) - Y(I-1)
        DSM1 = SQRT(DXM1*DXM1 + DYM1*DYM1)
C
        DXP1 = X(I+1) - X(I)
        DYP1 = Y(I+1) - Y(I)
        DSP1 = SQRT(DXP1*DXP1 + DYP1*DYP1)
C
        DXP2 = X(I+2) - X(I+1)
        DYP2 = Y(I+2) - Y(I+1)
        DSP2 = SQRT(DXP2*DXP2 + DYP2*DYP2)
C
C------- Don't mess with doubled points (slope breaks)
        IF(DSP1.EQ.0.0) GO TO 20
C
        IF(DSP1.LT.STOL*DSM1 .OR. DSP1.LT.STOL*DSP2) THEN
C------- Replace node I with average of I and I+1
         X(I) = 0.5*(X(I)+X(I+1))
         Y(I) = 0.5*(Y(I)+Y(I+1))
C------- Remove node I+1
         DO L = I+1, N
           X(L) = X(L+1)
           Y(L) = Y(L+1)
         END DO         
         N = N - 1
         LCHANGE = .TRUE.
         WRITE(*,*) 'SCHECK segment removed at ',I
         GO TO 10
        ENDIF
C
 20   CONTINUE
C
      RETURN
      END ! SCHECK



      SUBROUTINE HALF(X,Y,S,N)
C-------------------------------------------------
C     Halves the number of points in airfoil
C-------------------------------------------------
      REAL X(*), Y(*), S(*)
C
      K = 1
      INEXT = 3
      DO 20 I=2, N-1
C------ if corner is found, preserve it.
        IF(S(I) .EQ. S(I+1)) THEN
          K = K+1
          X(K) = X(I)
          Y(K) = Y(I)
          K = K+1
          X(K) = X(I+1)
          Y(K) = Y(I+1)
          INEXT = I+3
        ENDIF
C
        IF(I.EQ.INEXT) THEN
          K = K+1
          X(K) = X(I)
          Y(K) = Y(I)
          INEXT = I+2
        ENDIF
C
   20 CONTINUE
      K = K+1
      X(K) = X(N)
      Y(K) = Y(N)
C
C---- set new number of points
      N = K
C
      RETURN
      END ! HALF


