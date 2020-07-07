C***********************************************************************
C    Module:  jputil.f
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


      SUBROUTINE CONPLT(IX,JX,II,JJ,X,Y,JLOW,JUPP,F,FCON,
     &                  XOFF,YOFF,XWT,YWT)
      DIMENSION X(IX,JX), Y(IX,JX), JLOW(IX), JUPP(IX)
      DIMENSION F(IX,JX)
C--------------------------------------------------------------------------
C
C     Plots a contour of a discrete function F on a logically 
C     rectangular grid.
C
C      IX JX   dimensions of arrays   X, Y, F
C      II JJ   array limits of arrays X, Y, F
C      X(i,j)  independent coordinates of point (i,j)
C      Y(i,j)  
C      F(i,j)  function value at point (i,j)
C      JLOW(i) smallest value of j for which F(i,j) is defined (normally 1)
C      JUPP(i) largest  value of j for which F(i,j) is defined (normally JJ)
C      FCON    value of F on the contour to be drawn
C      XWT     scaling factor for X
C      YWT     scaling factor for Y
C--------------------------------------------------------------------------
C
      LOGICAL FOUND
C
      XMOD(XX) = (XX-XOFF)*XWT
      YMOD(YY) = (YY-YOFF)*YWT
C
C---- go over all cells and draw contour in any cell which contains the contour
      DO 10 IO=1, II-1
        IP = IO+1
C
C------ go over only those cells which have F defined at all four corners
        JLO = MAX0( JLOW(IO) , JLOW(IP) )
        JHI = MIN0( JUPP(IO) , JUPP(IP) )
        DO 110 JO=JLO, JHI-1
          JP = JO+1
C
          FOUND = .FALSE.
C
C           op    3    pp
C
C           4           2
C
C           oo    1    po
C
          XOO = X(IO,JO)
          XOP = X(IO,JP)
          XPO = X(IP,JO)
          XPP = X(IP,JP)
C
          YOO = Y(IO,JO)
          YOP = Y(IO,JP)
          YPO = Y(IP,JO)
          YPP = Y(IP,JP)
C
          FOO = F(IO,JO)
          FOP = F(IO,JP)
          FPO = F(IP,JO)
          FPP = F(IP,JP)
C
C-------- bottom edge (side 1)
          IF(FCON.GE.FOO .AND. FCON.LT.FPO .OR.
     &       FCON.LT.FOO .AND. FCON.GE.FPO     ) THEN
           XCON = XOO + (FCON-FOO)*(XPO-XOO)/(FPO-FOO)
           YCON = YOO + (FCON-FOO)*(YPO-YOO)/(FPO-FOO)
           IF(FOUND) THEN
            CALL PLOT(XMOD(XCON),YMOD(YCON),2)
           ELSE
            CALL PLOT(XMOD(XCON),YMOD(YCON),3)
           ENDIF 
           FOUND = .NOT.FOUND
          ENDIF
C
C-------- left edge (side 4)
          IF(FCON.GE.FOO .AND. FCON.LT.FOP .OR.
     &       FCON.LT.FOO .AND. FCON.GE.FOP     ) THEN
           XCON = XOO + (FCON-FOO)*(XOP-XOO)/(FOP-FOO)
           YCON = YOO + (FCON-FOO)*(YOP-YOO)/(FOP-FOO)
           IF(FOUND) THEN
            CALL PLOT(XMOD(XCON),YMOD(YCON),2)
           ELSE
            CALL PLOT(XMOD(XCON),YMOD(YCON),3)
           ENDIF 
           FOUND = .NOT.FOUND
          ENDIF
C
C-------- right edge (side 2)
          IF(FCON.GE.FPO .AND. FCON.LT.FPP .OR.
     &       FCON.LT.FPO .AND. FCON.GE.FPP     ) THEN
           XCON = XPO + (FCON-FPO)*(XPP-XPO)/(FPP-FPO)
           YCON = YPO + (FCON-FPO)*(YPP-YPO)/(FPP-FPO)
           IF(FOUND) THEN
            CALL PLOT(XMOD(XCON),YMOD(YCON),2)
           ELSE
            CALL PLOT(XMOD(XCON),YMOD(YCON),3)
           ENDIF 
           FOUND = .NOT.FOUND
          ENDIF
C
C-------- top edge (side 3)
          IF(FCON.GE.FOP .AND. FCON.LT.FPP .OR.
     &       FCON.LT.FOP .AND. FCON.GE.FPP     ) THEN
           XCON = XOP + (FCON-FOP)*(XPP-XOP)/(FPP-FOP)
           YCON = YOP + (FCON-FOP)*(YPP-YOP)/(FPP-FOP)
           IF(FOUND) THEN
            CALL PLOT(XMOD(XCON),YMOD(YCON),2)
           ELSE
            CALL PLOT(XMOD(XCON),YMOD(YCON),3)
           ENDIF 
           FOUND = .NOT.FOUND
          ENDIF
C
  110   CONTINUE
   10 CONTINUE
C
      RETURN
      END ! CONPLT



      SUBROUTINE CONLAB(IX,JX,II,JJ,X,Y,F,FCON,XOFF,YOFF,XWT,YWT,
     &           CH,NDIG,ISIDE)
      DIMENSION X(IX,JX), Y(IX,JX)
      DIMENSION F(IX,JX)
C---------------------------------------------------------------------
C     Puts numerical labels on the contour with value FCON at the
C     edge of the domain specified by ISIDE.  The PLNUMB of digits
C     in the label(s) after the decimal point is given by NDIG.
C---------------------------------------------------------------------
      LOGICAL LABEL 
C
      DATA PI, RTOD / 3.141592654, 57.2957795 /
C
      XMOD(XX) = (XX-XOFF)*XWT
      YMOD(YY) = (YY-YOFF)*YWT
C
      IF(ISIDE.EQ.1) THEN
       JO = 1
       JP = 2
       KLO = 1
       KHI = II-1
      ELSE IF(ISIDE.EQ.2) THEN
       IO = II-1
       IP = II
       KLO = 1
       KHI = JJ-1
      ELSE IF(ISIDE.EQ.3) THEN
       JO = JJ-1
       JP = JJ
       KLO = 1
       KHI = II-1
      ELSE IF(ISIDE.EQ.4) THEN
       IO = 1
       IP = 2
       KLO = 1
       KHI = JJ-1
      ENDIF
C
C---- check domain edge specified by ISIDE if the contour touches it
      DO 10 K=KLO, KHI
C
        IF(ISIDE.EQ.1) THEN
         IO = K
         IP = K+1
        ELSE IF(ISIDE.EQ.2) THEN
         JO = K
         JP = K+1
        ELSE IF(ISIDE.EQ.3) THEN
         IO = K
         IP = K+1
        ELSE IF(ISIDE.EQ.4) THEN
         JO = K
         JP = K+1
        ENDIF
C
C------ flag indicating if contour crosses current cell
        LABEL = .FALSE.
C
C           op    3    pp
C
C           4           2
C
C           oo    1    po
C
        XOO = X(IO,JO)
        XOP = X(IO,JP)
        XPO = X(IP,JO)
        XPP = X(IP,JP)
C
        YOO = Y(IO,JO)
        YOP = Y(IO,JP)
        YPO = Y(IP,JO)
        YPP = Y(IP,JP)
C
        FOO = F(IO,JO)
        FOP = F(IO,JP)
        FPO = F(IP,JO)
        FPP = F(IP,JP)
C
C------ bottom edge (side 1)
        IF(FCON.GE.FOO .AND. FCON.LT.FPO .OR.
     &     FCON.LT.FOO .AND. FCON.GE.FPO     ) THEN
         IF(ISIDE.EQ.1) THEN
          XCON2 = XOO + (FCON-FOO)*(XPO-XOO)/(FPO-FOO)
          YCON2 = YOO + (FCON-FOO)*(YPO-YOO)/(FPO-FOO)
          LABEL = .TRUE.
         ELSE
          XCON1 = XOO + (FCON-FOO)*(XPO-XOO)/(FPO-FOO)
          YCON1 = YOO + (FCON-FOO)*(YPO-YOO)/(FPO-FOO)
         ENDIF 
        ENDIF
C
C------ left edge (side 4)
        IF(FCON.GE.FOO .AND. FCON.LT.FOP .OR.
     &     FCON.LT.FOO .AND. FCON.GE.FOP     ) THEN
         IF(ISIDE.EQ.4) THEN
          XCON2 = XOO + (FCON-FOO)*(XOP-XOO)/(FOP-FOO)
          YCON2 = YOO + (FCON-FOO)*(YOP-YOO)/(FOP-FOO)
          LABEL = .TRUE.
         ELSE
          XCON1 = XOO + (FCON-FOO)*(XOP-XOO)/(FOP-FOO)
          YCON1 = YOO + (FCON-FOO)*(YOP-YOO)/(FOP-FOO)
         ENDIF 
        ENDIF
C
C------ right edge (side 2)
        IF(FCON.GE.FPO .AND. FCON.LT.FPP .OR.
     &     FCON.LT.FPO .AND. FCON.GE.FPP     ) THEN
         IF(ISIDE.EQ.2) THEN
          XCON2 = XPO + (FCON-FPO)*(XPP-XPO)/(FPP-FPO)
          YCON2 = YPO + (FCON-FPO)*(YPP-YPO)/(FPP-FPO)
          LABEL = .TRUE.
         ELSE
          XCON1 = XPO + (FCON-FPO)*(XPP-XPO)/(FPP-FPO)
          YCON1 = YPO + (FCON-FPO)*(YPP-YPO)/(FPP-FPO)
         ENDIF 
        ENDIF
C
C------ top edge (side 3)
        IF(FCON.GE.FOP .AND. FCON.LT.FPP .OR.
     &     FCON.LT.FOP .AND. FCON.GE.FPP     ) THEN
         IF(ISIDE.EQ.3) THEN
          XCON2 = XOP + (FCON-FOP)*(XPP-XOP)/(FPP-FOP)
          YCON2 = YOP + (FCON-FOP)*(YPP-YOP)/(FPP-FOP)
          LABEL = .TRUE.
         ELSE
          XCON1 = XOP + (FCON-FOP)*(XPP-XOP)/(FPP-FOP)
          YCON1 = YOP + (FCON-FOP)*(YPP-YOP)/(FPP-FOP)
         ENDIF 
        ENDIF
C
        IF(LABEL) THEN
C
C------- a contour reaching the domain edge has been found - set coordinates
C        of contour on the cell edges
         X1 = XMOD(XCON1)
         X2 = XMOD(XCON2)
         Y1 = YMOD(YCON1)
         Y2 = YMOD(YCON2)
C
         DX = X2 - X1
         DY = Y2 - Y1
C
C------- contour angle
         ACON = ATAN2( DY , DX )
         SA = SIN(ACON)
         CA = COS(ACON)
C
C------- if contour points to the right ...
         IF(ABS(ACON) .LT. 0.5*PI) THEN
C
C-------- set angle and lower left coordinates of number
          ANUM = RTOD*ACON
          XN = X2 + CH*CA + 0.5*CH*SA
          YN = Y2 + CH*SA - 0.5*CH*CA
C
C------- if contour points to the left ...
         ELSE
C
C-------- add +/- 180 degrees to number angle to make it read right to left
          ANUM = RTOD*ACON - SIGN(180.0,ACON)
C
C-------- set total number of characters in numeral, including decimal point
          GAFCON = 0.0
          IF(FCON.NE.0.0) GAFCON = LOG10( ABS(FCON) )
          RDIG = FLOAT(2+NDIG)
          RDIG = RDIG + MAX( AINT(GAFCON) , 0.0 )
          IF(FCON.LT.0.0) RDIG = RDIG + 1.0
C
C-------- set lower left coordinates of number
          XN = X2 + CH*(RDIG+0.7)*CA - 0.55*CH*SA
          YN = Y2 + CH*(RDIG+0.7)*SA + 0.55*CH*CA
C
         ENDIF
C
C------- draw number
         CALL PLNUMB(XN,YN,CH,FCON,ANUM,NDIG)
C
        ENDIF
C
   10 CONTINUE
C
      RETURN
      END ! CONLAB
