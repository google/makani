C***********************************************************************
C    Module:  airutil.f
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

      SUBROUTINE READBL(FNAME,IBX,NBX,XB,YB,IIB,NBL,
     &                  NAME,XINL,XOUT,YBOT,YTOP)
      CHARACTER*(*) FNAME
      DIMENSION XB(IBX,NBX) ,YB(IBX,NBX)
      DIMENSION IIB(NBX)
      CHARACTER*(*) NAME
C----------------------------------------------
C     Reads in blade.xxx dataset
C----------------------------------------------
C
      CHARACTER*80 LINE
      LOGICAL ERROR
C
      DIMENSION AINPUT(20)
C
      CHARACTER*19 NCHARS
      DATA NCHARS / '0123456789-+.,EDed ' /
C
C---- default top/bottom flow area ratio for "old" blade.xxx files
      DATA YRAT / 1.3 /
C
C---- first assume that there will be a read error
      NBL = 0
C
      NF = INDEX(FNAME,' ') + 1
C
      LU = 3
C
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=98)
C
      READ(LU,1000) LINE
C
C---- if first line has any non-numeric character, go treat it as the name
      DO K=1, 80
        IF(INDEX(NCHARS,LINE(K:K)) .EQ. 0) GO TO 20
      ENDDO
C
C---- plain unlabeled file: rewind, and just read in X,Y coordinates
      NAME = ' '
      NINPUT = 0
C
      REWIND(LU)
cc      WRITE(*,*)
cc      WRITE(*,*) 'Reading plain coordinate file'
      GO TO 40
C
C---- first line interpreted as label string
   20 READ(LINE,1000) NAME
C
C---- read and decode second line --- grid domain limits will be set later
      READ(LU,1000) LINE
      NINPUT = 4
      CALL GETFLT(LINE,AINPUT,NINPUT,ERROR)
C
      IF(ERROR) GO TO 99
C
      IF(NINPUT.LT.4) THEN
C------ no domain parameters: re-read name string and then read X,Y coordinates
        REWIND(LU)
        READ(LU,1000) LINE
      ENDIF
C
cc      WRITE(*,1010) NAME
cc 1010 FORMAT(/1X,'Reading in coordinate file for: ',A/)
C
 40   CONTINUE
C
C---- read in airfoil coordinates
      DO 55 N=1, NBX+1
        IB = 1
 50     CONTINUE
          READ(LU,*,END=56,ERR=99) XBT, YBT
          IF(XBT.EQ.999.0) THEN
           IIB(N) = IB-1
           GO TO 55
          ENDIF
          IF(N.GT.NBX) THEN
           WRITE(*,*) '*** READBL: Too many elements. Increase NBX to',N
           STOP
          ENDIF
          IBLIM = MIN( IB , IBX )
          XB(IBLIM,N) = XBT
          YB(IBLIM,N) = YBT
C
          IB = IB + 1
          GO TO 50
   55 CONTINUE
      N = NBX
C
   56 CONTINUE
      IF(IB.EQ.1) THEN
C----- coordinate file has "999.0 999.0" at the end ...
       NBL = N-1
      ELSE
C----- coordinate file has no ending line (single element ISES file)
       NBL = N
       IIB(N) = IB-1
      ENDIF
C
      CLOSE(LU)
C
C
      DO 80 N = 1, NBL
        IF(IIB(N).GT.IBX) THEN
         WRITE(*,*)
     &    '*** READBL: Too many airfoil points. Increase IBX to', IIB(N)
         STOP
        ENDIF
C
C------ calculate airfoil element area
        AREA = 0.
        DO 802 IB=1, IIB(N)-1
          RX = XB(IB+1,N) + XB(IB,N)
          RY = YB(IB+1,N) + YB(IB,N)
          DX = XB(IB+1,N) - XB(IB,N)
          DY = YB(IB+1,N) - YB(IB,N)
          DA = 0.25*(RX*DY - RY*DX)
          AREA = AREA + DA
 802    CONTINUE
C
        IF(AREA.LT.0.0) THEN
C------- if area is negative (clockwise order), reverse coordinate order
         DO 804 IB=1, IIB(N)/2
           IBACK = IIB(N) - IB + 1
           XTMP = XB(IB,N)
           YTMP = YB(IB,N)
           XB(IB,N) = XB(IBACK,N)
           YB(IB,N) = YB(IBACK,N)
           XB(IBACK,N) = XTMP
           YB(IBACK,N) = YTMP
 804     CONTINUE
        ENDIF
C
 80   CONTINUE
C
      IF     (NINPUT.LT.4) THEN
C------ plain or labeled airfoil file -- no domain parameters specified
        XINL = 0.
        XOUT = 0.
        YBOT = 0.
        YTOP = 0.
      ELSEIF (NINPUT.EQ.4) THEN
C------ for "new" blade.xxx file, grid size is input directly
        XINL = AINPUT(1)
        XOUT = AINPUT(2)
        YBOT = AINPUT(3)
        YTOP = AINPUT(4)
      ELSE
C------ "old" blade.xxx file, grid size is implied from airfoil limits
        CHINL = AINPUT(3)
        CHOUT = AINPUT(4)
        CHWID = AINPUT(5)
C
        XMIN = XB(1,1) + 1.0
        XMAX = XB(1,1) - 1.0
        DO 84 N=1, NBL
          DO 842 IB=1, IIB(N)
            IF(XB(IB,N).LE.XMIN) THEN
             XMIN = XB(IB,N)
             YMIN = YB(IB,N)
            ENDIF
            IF(XB(IB,N).GE.XMAX) THEN
             XMAX = XB(IB,N)
             YMAX = YB(IB,N)
            ENDIF
 842      CONTINUE
 84     CONTINUE
C
        XINL = XMIN - CHINL
        XOUT = XMAX + CHOUT
        YBOT = YMIN - CHWID *  1.0/(1.0 + YRAT)
        YTOP = YMAX + CHWID * YRAT/(1.0 + YRAT)
      ENDIF
C
C
      RETURN
C
   98 CONTINUE
      WRITE(*,1050) FNAME(1:NF)
      NBL = 0
      RETURN
C
   99 CONTINUE
      WRITE(*,1100) FNAME(1:NF)
      RETURN
C...............................................................
 1000 FORMAT(A)
 1050 FORMAT(/' File OPEN error:  ', A)
 1100 FORMAT(/' File READ error:  ', A)
      END ! READBL



      SUBROUTINE GETCAM(X,Y,N,XC,YC,TC,NC,LNORM)
C
C---Takes airfoil X,Y surface points and returns the camber defined
C   in XC,YC at NC points
C
      REAL X(*), Y(*), XC(*), YC(*), TC(*)
      LOGICAL LNORM
C
      PARAMETER(NSIZ=500)
      REAL XP(NSIZ), YP(NSIZ), S(NSIZ)
C
      PI = 4.0*ATAN(1.0)
      IF(N.GT.NSIZ) THEN
       WRITE(*,*) '*** GETCAM: Array overflow. Increase NSIZ to', N
       STOP
      ENDIF
C
C---- spline coordinates
      CALL SCALC(X,Y,S,N)
      CALL SEGSPL(X,XP,S,N)
      CALL SEGSPL(Y,YP,S,N)
C
C---- find arc length position of leading edge
      CALL LEFIND(SLE,X,XP,Y,YP,S,N)
C
C---- normalize airfoil and its spline data to unit chord
      IF(LNORM) CALL NORMIT(SLE,X,XP,Y,YP,S,N)
C
      XLE = SEVAL(SLE,X,XP,S,N)
      YLE = SEVAL(SLE,Y,YP,S,N)
      XTE = 0.5*(X(1)+X(N))
      YTE = 0.5*(Y(1)+Y(N))
C
C---- Number of output points defaults to 30
      IF(NC.LE.0) NC = 30
C
      SU = SLE - 0.01
      SL = SLE + 0.01
      FNC1 = FLOAT(NC-1)
      XC(1) = XLE
      YC(1) = YLE
      TC(1) = 0.0
      DO I = 2, NC
        XOUT = XLE + (XTE-XLE)*0.5*(1.0 - COS(PI*FLOAT(I-1)/FNC1))
        CALL SINVRT(SU,XOUT,X,XP,S,N)
        YU = SEVAL(SU,Y,YP,S,N)
        CALL SINVRT(SL,XOUT,X,XP,S,N)
        YL = SEVAL(SL,Y,YP,S,N)
        XC(I) = XOUT
        YC(I) = 0.5*(YU+YL)
        TC(I) = YU-YL
      END DO
C
      RETURN
      END


      SUBROUTINE LEFIND(SLE,X,XP,Y,YP,S,N)
C     .......................................
C
C     Finds the spline parameter value SLE
C     at the leftmost point of the airfoil
C     (i.e. the leading edge)
C     .......................................
C
      REAL X(N),Y(N),XP(N),YP(N),S(N)
C
C---- initial guess for SLE = leftmost point
      DO I = 2, N
        IF(X(I).GT.X(I-1)) GO TO 6
      END DO
    6 SLE = S(I-1)
C
C---- Newton solution for the exact SLE value (at least to within machine zero)
      SREF = S(N) - S(1)
      DO ITER = 1, 20
        RES  = DEVAL(SLE,X,XP,S,N)
        RESP = D2VAL(SLE,X,XP,S,N)
        DSLE = -RES/RESP
        SLE = SLE + DSLE
        IF(ABS(DSLE)/SREF .LT. 1.0E-5) RETURN
      END DO
      WRITE(*,*) '** LEFIND: Leading edge not found.  Continuing...'
      SLE = S(I-1)
      RETURN
      END ! LEFIND
 
 

 
      SUBROUTINE NORMIT(SLE,X,XP,Y,YP,S,N)
C     .................................................
C
C     Normalizes airfoil coordinates and their spline
C     parameter to unit chord. The x coordinates are
C     also offset so that the leading edge is at x = 0.
C     .................................................
C
      REAL X(N),Y(N),XP(N),YP(N),S(N)
C
C---- leading edge and trailing edge x coordinates
      XLE = SEVAL(SLE,X,XP,S,N)
      XTE = 0.5*(X(1)+X(N))
C
C---- normalizing factor
      DNORM = 1.0/(XTE-XLE)
      DO I = 1, N
        X(I) = (X(I)-XLE)*DNORM
        Y(I) = Y(I)*DNORM
        S(I) = S(I)*DNORM
      END DO
      SLE = SLE*DNORM
C
      RETURN
      END ! NORMIT
