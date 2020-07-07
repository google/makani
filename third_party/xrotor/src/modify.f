C***********************************************************************
C    Module:  modify.f
C 
C    Copyright (C) 2011 Mark Drela 
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

      SUBROUTINE CRSMOD(N,X,Y,YP,
     &                  XOFF,XSF,YOFF,YSF, SSIZ, NSPLT,
     &                  LSLOPE, IMOD1,IMOD2 )
      DIMENSION X(N),Y(N),YP(N)
      LOGICAL LSLOPE
C--------------------------------------------------
C     Modifies current y(x) from cursor input.
C
C     The extent of the modification is returned 
C     in the two indices:    IMOD1 < i < IMOD2
C
C--------------------------------------------------
      LOGICAL LABORT, LZOOM, LUNZOOM
      CHARACTER*1 KCHAR
C
      LOGICAL LSLOP1, LSLOP2
C
      PARAMETER (NWX=100)
      DIMENSION W1(NWX), W2(NWX), W3(NWX)
C
      XMOD(XARG) = XSF*(XARG-XOFF)
      YMOD(YARG) = YSF*(YARG-YOFF)
C
C---- save current color and set new color
      CALL GETCOLOR(ICOL0)
C
      DO I=1, NWX
        W1(I) = 0.
        W2(I) = 0.
        W3(I) = 0.
      ENDDO
C
C---- first assume no changes will be made
      IMOD1 = 1
      IMOD2 = 0
C
      CALL PUNZOOM
      CALL PZOOM
      CALL PABORT
      CALL PLFLUSH
C
      CALL NEWCOLORNAME('violet')
C
      WRITE(*,*)
      WRITE(*,*) 'Input y(x) values'
      WRITE(*,*) 'Terminate last entry with 3 clicks on one point'
      WRITE(*,*) 'Hot keys:  z zooms, u unzooms, q aborts '
      WRITE(*,*)
C
C---- read first Qspec point plot coordinates
      K = 1
 5    CALL GETCURSORXY(W1(K),W2(K),KCHAR)
C
C---- return with no changes ?
      IF(LABORT(W1(K),W2(K)).OR.INDEX('Qq',KCHAR).NE.0) GO TO 90
C
C---- zoom?
      IF(LZOOM(W1(K),W2(K)).OR.INDEX('Zz',KCHAR).NE.0) THEN
        CALL USETZOOM(.FALSE.,.TRUE.)
        CALL REPLOT(IDEV)
        GO TO 5
      ENDIF
C
C---- unzoom?
      IF(LUNZOOM(W1(K),W2(K)).OR.INDEX('Uu',KCHAR).NE.0) THEN
        CALL CLRZOOM
        CALL REPLOT(IDEV)
        GO TO 5
      ENDIF
C
C
C---- convert plot coordinates W1,W2 to actual x,y
      W1(K) = W1(K)/XSF + XOFF
      W2(K) = W2(K)/YSF + YOFF
C------ reset first point from spline, draw "o" symbol there
      IF(W1(K).LT.X(1)) W1(K) = X(1)
      IF(W1(K).GT.X(N)) W1(K) = X(N)
      IF(LSLOPE) W2(K) = SEVAL(W1(K),Y,YP,X,N)
      CALL PLSYMB(XMOD(W1(K)),YMOD(W2(K)),SSIZ,1,0.0,0)
      CALL PLFLUSH
C
C---- read in subsequent points
      DO K=2, NWX
 8       CALL GETCURSORXY(W1(K),W2(K),KCHAR)
C
C------ return with no changes ?
        IF(LABORT(W1(K),W2(K)).OR.INDEX('Qq',KCHAR).NE.0) GO TO 90
C
C------ zoom?
        IF(LZOOM(W1(K),W2(K)).OR.INDEX('Zz',KCHAR).NE.0) THEN
          CALL USETZOOM(.FALSE.,.TRUE.)
          CALL REPLOT(IDEV)
          GO TO 8
        ENDIF
C
C------ unzoom?
        IF(LUNZOOM(W1(K),W2(K)).OR.INDEX('Uu',KCHAR).NE.0) THEN
          CALL CLRZOOM
          CALL REPLOT(IDEV)
          GO TO 8
        ENDIF
C
CCCC------ draw pixel (zero-length vector) at cursor location
CCC        CALL PLOT(W1(K),W2(K),3)
CCC        CALL PLOT(W1(K),W2(K),2)
C
C------ draw a "+" symbol at cursor location
        CALL PLSYMB(W1(K),W2(K),SSIZ,3,0.0,0)
        CALL PLFLUSH
C
C------ remove offset and scaling to get true incompressible Q,s values
        W1(K) = W1(K)/XSF + XOFF
        W2(K) = W2(K)/YSF + YOFF
C
C------ if at least three points exist, check if this is last point
        IF(K.GE.3) THEN
         IF(W1(K).EQ.W1(K-1) .AND. W1(K).EQ.W1(K-2)) GO TO 11
        ENDIF
      END DO
C
 11   IF(LSLOPE) THEN
        IF(W1(K).LT.X(1)) W1(K) = X(1)
        IF(W1(K).GT.X(N)) W1(K) = X(N)
C------ reset last point from spline, draw "o" symbol there
        W2(K) = SEVAL(W1(K),Y,YP,X,N)
        CALL PLSYMB(XMOD(W1(K)),YMOD(W2(K)),SSIZ,1,0.0,0)
        CALL PLFLUSH
      ENDIF
C
C---- set number of input points
      KK = K
C
C---- sort points, removing identical pairs
      CALL SORT(KK,W1,W2)
C
      IF(KK.LT.2) THEN
       WRITE(*,*)
       WRITE(*,*) '***  Need at least 2 points    ***'
       WRITE(*,*) '***     NO CHANGES MADE        ***'
       WRITE(*,*)
       GO TO 90
      ENDIF
C
C---- default natural (zero 3rd derivative) end conditions
      YP1 = -999.0
      YP2 = -999.0
C
C---- set spline endpoint derivatives to match current Qspec's
      LSLOP1 = LSLOPE .AND. W1(1 ).GE.X(1) .AND. W1(1 ).LE.X(N)
      LSLOP2 = LSLOPE .AND. W1(KK).GE.X(1) .AND. W1(KK).LE.X(N)
      IF(LSLOP1) YP1 = DEVAL(W1(1) ,Y,YP,X,N)
      IF(LSLOP2) YP2 = DEVAL(W1(KK),Y,YP,X,N)
C
C---- set suitable number of plotting sub-intervals
      DX  = (X(N)  -X(1) )/FLOAT( N-1)
      DW1 = (W1(KK)-W1(1))/FLOAT(KK-1)
      NT = INT( 10.0 * MAX(DW1/DX,1.0) )
C
C---- strip out any remaining double points
      K = 1
 20   K = K+1
 21   IF(K.GT.KK) GO TO 25
      IF(W1(K) .EQ. W1(K-1)) THEN
C------ eliminate point K by pulling down all points after it
        DO KT=K, KK-1
          W1(KT) = W1(KT+1)
          W2(KT) = W2(KT+1)
          W3(KT) = W3(KT+1)
        END DO
        KK = KK-1
        GO TO 21
      ELSE
C------ check next point
        GO TO 20
      ENDIF
C
 25   CONTINUE
      IF(KK.LT.2) THEN
       WRITE(*,*) '***  Corrupted input detected  ***'
       WRITE(*,*) '***       NO CHANGES MADE      ***'
       GO TO 90
      ENDIF
C
      CALL NEWCOLORNAME('orange')
C
C---- spline new Qspec segment coordinates
      CALL SPLIND(W2,W3,W1,KK,YP1,YP2)
C
C---- plot segment as piecewise linear or with spline, depending on NT
      DO 30 K=2, KK
        DX = W1(K) - W1(K-1)
C
        XPLT = W1(K-1)
        YPLT = W2(K-1)
        CALL PLOT(XMOD(XPLT),YMOD(YPLT),3)
C
        DO 310 IT=1, NSPLT
          XPLT = W1(K-1) + DX*FLOAT(IT)/FLOAT(NSPLT)
          YPLT = SEVAL(XPLT,W2,W3,W1,KK)
          CALL PLOT(XMOD(XPLT),YMOD(YPLT),2)
  310   CONTINUE
   30 CONTINUE
      CALL PLFLUSH
C
C---- set y(x) array inside modified interval W1(1) < s < W1(KK)
      DO 40 I=1, N
        IF(X(I).LT.W1(1)) THEN
         IMOD1 = I+1
         GO TO 40
        ELSE IF(X(I).LE.W1(KK)) THEN
C------- set new y(x)
         Y(I) = SEVAL(X(I),W2,W3,W1,KK)
         IMOD2 = I
        ELSE
         GO TO 41
        ENDIF
   40 CONTINUE
   41 CONTINUE
C
 90   CONTINUE
C
C---- restore saved color and zoom parameters
      CALL NEWCOLOR(ICOL0)
      CALL CLRZOOM
C
      RETURN
      END



      SUBROUTINE SORT(NW,S,W)
      IMPLICIT REAL (A-H,O-Z)
      DIMENSION S(NW), W(NW)
      LOGICAL DONE
C
C---- sort arrays
      DO 10 IPASS=1, 500
        DONE = .TRUE.
        DO 101 N=1, NW-1
          NP = N+1
          IF(S(NP).GE.S(N)) GO TO 101
           TEMP = S(NP)
           S(NP) = S(N)
           S(N) = TEMP
           TEMP = W(NP)
           W(NP) = W(N)
           W(N) = TEMP
           DONE = .FALSE.
  101   CONTINUE
        IF(DONE) GO TO 11
   10 CONTINUE
      STOP 'SORT failed'
C
C---- search for duplicate pairs and eliminate each one
   11 NWS = NW
      DO 20 N=1, NWS
        IF(N.GE.NW) RETURN
        IF(S(N).NE.S(N+1)) GO TO 20
C------- eliminate pair
         NW = NW-2
         DO 201 NT=N, NW
           S(NT) = S(NT+2)
           W(NT) = W(NT+2)
  201    CONTINUE
   20 CONTINUE
C
      RETURN
      END ! SORT
