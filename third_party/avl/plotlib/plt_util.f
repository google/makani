C***********************************************************************
C    Module:  plt_util.f
C 
C    Copyright (C) 1996 Harold Youngren, Mark Drela 
C 
C    This library is free software; you can redistribute it and/or
C    modify it under the terms of the GNU Library General Public
C    License as published by the Free Software Foundation; either
C    version 2 of the License, or (at your option) any later version.
C
C    This library is distributed in the hope that it will be useful,
C    but WITHOUT ANY WARRANTY; without even the implied warranty of
C    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
C    Library General Public License for more details.
C
C    You should have received a copy of the GNU Library General Public
C    License along with this library; if not, write to the Free
C    Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
C 
C    Report problems to:    guppy@maine.com 
C                        or drela@mit.edu  
C***********************************************************************
C
***********************************************************************
C --- Xplot11 utility routines
C
C     Version 4.46 11/28/01
C
C     Note:  These are additional routines that supply additional plot
C            functionality.  Included are routines for axis scaling, 
C            axis plotting, line or curve plotting, contours, etc.
C***********************************************************************


      subroutine XAXIS(X1,Y1,XAXT,DXANN,FANN,DANN,CHT,NDIG)
C.......................................................
C
C     X1,Y1  starting point of x axis
C     XAXT   length of x axis   ( - = suppress zero annotation)
C     DXANN  distance between annotations
C     FANN   first annotation value
C     DANN   delta annotation value
C     CHT    character width   ( - = annotation above axis)
C     NDIG   number of digits to right of decimal point
C            = -1   no decimal point
C            = -2   number of digits determined internally
C           <= -3   no axis annotation (just hash marks)
C.......................................................
C
      XAX = ABS(XAXT)
      CH = ABS(CHT)
C
      IF(NDIG.LE.-2) THEN
        ADANN = ABS(DANN)
        ND = MAX( 1 , INT(-LOG10(ADANN)) )
        IF(ADANN*10**ND - AINT(ADANN*10**ND+0.01) .GT. 0.01) ND = ND + 1
        IF(ADANN*10**ND - AINT(ADANN*10**ND+0.01) .GT. 0.01) ND = ND + 1
      ELSE
        ND = NDIG
      ENDIF
C
      CALL GETFACTORS(XFAC,YFAC)
      CHX = CH
      CHY = CH*XFAC/YFAC
C
C---- x-axis
      CALL PLOT(X1,Y1,3)
      CALL PLOT(X1+XAX,Y1,2)
C
C---- annotate x-axis
      DO 10 NT=1, 12345
        XT = X1 + DXANN*FLOAT(NT-1)
        IF(XT-X1.GT.XAX+0.5*DXANN) GO TO 11
C---- hash marks
         CALL PLOT(XT,Y1-0.2*CHY,3)
         CALL PLOT(XT,Y1+0.2*CHY,2)
C---- Numeric annotations
         IF(NDIG.LE.-3) GO TO 10
         RN = FANN + DANN*FLOAT(NT-1)
         IF(ABS(RN).LT.1.0E-5 .AND. XAXT.LT.0.0) GO TO 10
         GRN = 0.
         IF(RN.NE.0.0) GRN = LOG10(ABS(RN)+0.5/10.0**MAX(0,ND))
         GRN = MAX(GRN,0.0)
         NABC = INT(GRN) + 2 + ND
         WIDTH = CHX*FLOAT(NABC)
         IF(RN.LT.0.0) WIDTH = WIDTH + CHX
         XNUM = XT - 0.5*WIDTH + 0.1*CHX
         YNUM = Y1 - 2.1*CHY
         IF(CHT.LT.0.0) YNUM = Y1 + 0.9*CHY
         CALL PLNUMB(XNUM,YNUM,CH,RN,0.0,ND)
   10 CONTINUE
   11 CONTINUE
C
      RETURN
      END
 
 
      subroutine YAXIS(X1,Y1,YAXT,DYANN,FANN,DANN,CHT,NDIG)
C.......................................................
C
C     X1,Y1  starting point of y axis
C     YAXT   length of y axis   ( - = suppress zero annotation)
C     DYANN  distance between annotations
C     FANN   first annotation value
C     DANN   delta annotation value
C     CHT    character width   ( - = annotation on right side )
C     NDIG   number of digits to right of decimal point
C            = -1   no decimal point
C            = -2   number of digits determined internally
C           <= -3   no axis annotation (just hash marks)
C.......................................................
C
      YAX = ABS(YAXT)
      CH = ABS(CHT)
C
      IF(NDIG.LE.-2) THEN
        ADANN = ABS(DANN)
        ND = MAX( 1 , INT(-LOG10(ADANN)) )
        IF(ADANN*10**ND - AINT(ADANN*10**ND+0.01) .GT. 0.01) ND = ND + 1
        IF(ADANN*10**ND - AINT(ADANN*10**ND+0.01) .GT. 0.01) ND = ND + 1
      ELSE
        ND = NDIG
      ENDIF
C
      CALL GETFACTORS(XFAC,YFAC)
      CHX = CH
      CHY = CH*XFAC/YFAC
C
C---- y-axis
      CALL PLOT(X1,Y1,3)
      CALL PLOT(X1,Y1+YAX,2)
C
C---- annotate y-axis
      DO 10 NT=1, 12345
        YT = Y1 + DYANN*FLOAT(NT-1)
        IF(YT-Y1.GT.YAX+0.5*DYANN) GO TO 11
C---- hash marks
         CALL PLOT(X1-0.2*CHX,YT,3)
         CALL PLOT(X1+0.2*CHX,YT,2)
C---- Numeric annotations
         IF(NDIG.LE.-3) GO TO 10
         RN = FANN + DANN*FLOAT(NT-1)
         IF(ABS(RN).LT.1.0E-5 .AND. YAXT.LT.0.0) GO TO 10
         GRN = 0.
         IF(RN.NE.0.0) GRN = LOG10(ABS(RN)+0.5/10.0**MAX(0,ND))
         GRN = MAX(GRN,0.0)
         NABC = INT(GRN) + 2 + ND
         WIDTH = CHX*FLOAT(NABC)
         IF(RN.LT.0.0) WIDTH = WIDTH + CHX
         XT = X1 - (0.6*CHX + WIDTH)
         IF(CHT.LT.0.0) XT = X1 + CHX
         CALL PLNUMB(XT,YT-0.5*CHY,CH,RN,0.0,ND)
   10 CONTINUE
   11 CONTINUE
C
      RETURN
      END



      subroutine XYLINE(N,X,Y,XOFF,XWT,YOFF,YWT,ILIN)
C....................................................................
C
C...General XY polyline plotting routine with offsets and scaling
C
C...INPUT    X, Y     Input arrays of length N
C            XOFF,XWT Offset and scale factor for X array...
C            YOFF,YWT Offset and scale factor for Y array...
C                       Xplot = XWT*(X-XOFF)
C                       Yplot = YWT*(Y-YOFF)
C            ILIN     Selects line pattern
C
C...8 line patterns are available (repeat for ILIN>8)
C     1  *****************************  SOLID
C     2  **** **** **** **** **** ****  LONG DASHED
C     3  ** ** ** ** ** ** ** ** ** **  SHORT DASHED
C     4  * * * * * * * * * * * * * * *  DOTTED
C     5  ***** * ***** * ***** * *****  DASH-DOT
C     6  ***** * * ***** * * ***** * *  DASH-DOT-DOT
C     7  ***** * * * ***** * * * *****  DASH-DOT-DOT-DOT
C     8  **** **** * * **** **** * *    DASH-DASH-DOT-DOT
C
C....................................................................
      DIMENSION X(N), Y(N)
C
      DIMENSION  NMOV(7), SMOV(8,7)
C
      DATA  NPAT / 8 /
      DATA  SCL1  / 0.125 /
      DATA  NMOV / 2,  2,  2,  4,  6,  8,  8 /
      DATA  SMOV /1.2,  -.4,   0.,   0.,   0.,  0.,  0.,  0.,
     &             .5,  -.4,   0.,   0.,   0.,  0.,  0.,  0.,
     &             .2,  -.4,   0.,   0.,   0.,  0.,  0.,  0.,
     &            1.4,  -.4,  .2,  -.4,    0.,  0.,  0.,  0.,
     &            1.4,  -.4,  .2,  -.4,   .2,  -.4,  0.,  0.,
     &            1.4,  -.4,  .2,  -.4,   .2,  -.4,  .2, -.4,
     &            1.2,  -.4, 1.2,  -.4,   .2,  -.4,  .2, -.4   /
C 
      IF(N.LE.1) RETURN
C
C---- set line pattern scale based on current user scaling factors
      CALL GETFACTORS(XSCALE,YSCALE)
      SCL = SCL1 / SQRT(XSCALE*YSCALE)
C
      NLIN = MAX(ILIN,1)
      IPAT = MOD(NLIN-1,NPAT) + 1
C
      X2 = XWT*(X(1)-XOFF)
      Y2 = YWT*(Y(1)-YOFF)
      CALL PLOT(X2,Y2,3)
C
      IF (IPAT.EQ.1) THEN
C...Plot using continuous line 
       DO 10 I=2, N
         X1 = X2
         Y1 = Y2
         X2 = XWT*(X(I)-XOFF)
         Y2 = YWT*(Y(I)-YOFF)
         CALL PLOT(X2,Y2,2)
   10  CONTINUE
C
      ELSE
C...Plot using stored patterns for lines
       I  = 1
       S1 = 0.
       S2 = 0.
       S0 = 0.
C
   20  DO 40 II=1, 99999
C
C...Pattern specifies pen up or down
        IM = MOD(II+1,NMOV(IPAT-1)) + 1 
        IPEN = 3
        IF(SMOV(IM,IPAT-1).GT.0.) IPEN = 2
C
        DS = SCL*ABS(SMOV(IM,IPAT-1))
        SPAT = S0 + DS
C
C...Find data interval containing pattern point
   30   IF (SPAT.GE.S2 .AND. I+1.LE.N) THEN
          I = I + 1
          CALL PLOT(X2,Y2,IPEN)
          X1 = X2
          Y1 = Y2
          S1 = S2
          X2 = XWT*(X(I)-XOFF)
          Y2 = YWT*(Y(I)-YOFF)
          DS = SQRT((X2-X1)**2 + (Y2-Y1)**2)
          S2 = S1 + DS
          GO TO 30
        ENDIF
C
C...Find point on interval using linear interpolation
        IF (SPAT.GT.S2) SPAT = S2
        IF(S2 .EQ. S1) THEN
          FRAC = 0.0
        ELSE
          FRAC = (SPAT-S1)/(S2-S1)
        ENDIF
        XX = X1 + FRAC*(X2-X1)
        YY = Y1 + FRAC*(Y2-Y1)
C
C...Move to new point using stored pattern to specify pen up or down
        CALL PLOT(XX,YY,IPEN)
        IF (I.GE.N .AND. SPAT.GE.S2) GO TO 50
        S0 = SPAT
   40  CONTINUE
C
   50 CONTINUE
      ENDIF
C
      RETURN
      END



      subroutine XYSYMB(N,X,Y,XOFF,XWT,YOFF,YWT,SH,ISYM)
C.............................................................
C
C...GENERAL XY MULTIPLE-SYMBOL PLOTTING ROUTINE
C   (useful for overplotting XYLINE plot with point symbols)
C
C...INPUT    X, Y     Input arrays of length N
C            XOFF,XWT Offset and scale factor for X array
C            YOFF,YWT Offset and scale factor for Y array
C            SH       Symbol size
C            ISYM     Selects symbol type
C                     if ISYM < 0 ... no plotting
C.............................................................
C
      DIMENSION X(N), Y(N)
C
      IF(ISYM.LT.0) RETURN
C
      DO 10 I=1, N
        XPLT = XWT*(X(I)-XOFF)
        YPLT = YWT*(Y(I)-YOFF)
        CALL PLSYMB(XPLT,YPLT,SH,ISYM,0.0,0)
 10   CONTINUE
C
      RETURN
      END


      subroutine CONT_GRID(IX,JX,II,JJ,X,Y,F,FCON,XOFF,YOFF,XWT,YWT)
      DIMENSION X(IX,JX), Y(IX,JX), F(IX,JX)
      CALL CONTGRID(IX,JX,II,JJ,X,Y,F,FCON,XOFF,YOFF,XWT,YWT)
      RETURN
      END

      subroutine CONTGRID(IX,JX,II,JJ,X,Y,F,FCON,XOFF,YOFF,XWT,YWT)
      DIMENSION X(IX,JX), Y(IX,JX), F(IX,JX)
C--------------------------------------------------------------------------
C
C     Plots one contour of a function F on a logically rectangular grid.
C     (normally called repeatedly if a number of contours is to be drawn)
C
C      IX JX   dimensions of arrays   X, Y, F
C      II JJ   array limits of arrays X, Y, F
C      X(i,j)  independent coordinates of point (i,j)
C      Y(i,j)  
C      F(i,j)  function value at point (i,j)
C      FCON    value of F on the contour to be drawn
C      XOFF    offset for X
C      YOFF    offset for Y
C      XWT     scaling factor for X
C      YWT     scaling factor for Y
C
C        XPLOT = (X - XOFF)*XWT
C        YPLOT = (Y - YOFF)*YWT
C--------------------------------------------------------------------------
C
      LOGICAL FOUND
C
C---- go over all cells and draw contour in any cell which contains the contour
      DO 10 IO=1, II-1
        IP = IO+1
C
        DO 110 JO=1, JJ-1
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
            CALL PLOT(XWT*(XCON-XOFF),YWT*(YCON-YOFF),2)
           ELSE
            CALL PLOT(XWT*(XCON-XOFF),YWT*(YCON-YOFF),3)
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
            CALL PLOT(XWT*(XCON-XOFF),YWT*(YCON-YOFF),2)
           ELSE
            CALL PLOT(XWT*(XCON-XOFF),YWT*(YCON-YOFF),3)
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
            CALL PLOT(XWT*(XCON-XOFF),YWT*(YCON-YOFF),2)
           ELSE
            CALL PLOT(XWT*(XCON-XOFF),YWT*(YCON-YOFF),3)
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
            CALL PLOT(XWT*(XCON-XOFF),YWT*(YCON-YOFF),2)
           ELSE
            CALL PLOT(XWT*(XCON-XOFF),YWT*(YCON-YOFF),3)
           ENDIF 
           FOUND = .NOT.FOUND
          ENDIF
C
  110   CONTINUE
   10 CONTINUE
C
      RETURN
      END




      subroutine CONTQUAD(X,Y,F,
     &                     FUPR,FLWR,
     &                     NCU,XCU,YCU,
     &                     NCL,XCL,YCL,
     &                     NA,NE,NV,XP,YP)
C--------------------------------------------------------------------------
C  Contour a single quadrilateral element between an upper and 
C  a lower contour limit.  The output from this routine is both the 
C  line segments defining the upper and lower contours and a set of
C  polygons that define the area of the element lying between the 
C  contour limits.  These may be used to shade the area on a plot.
C
C    X, Y       Arrays containing quadrilateral coordinate data
C    F          Array containing the quantity F(x,y) to be contoured, 
C                ie. F(i) is defined at X(i),Y(i), i=1->4
C    FUPR, FLWR Upper and lower contour limits
C    NCU        Number of upper limit contour line points
C    XCU, YCU   Arrays of x,y points (in pairs) on upper contour
C    NCL        Number of lower limit contour line points
C    XCL, YCL   Arrays of x,y points (in pairs) on lower contour
C    NA         Number of polygon areas in XP, YP, NE arrays
C    NE         Array of numbers of points in each polygon area
C    NV         Vertex count of points in XP,YP arrays
C    XP, YP     Coordinate points of contour polygons
C
C  Note: the output contour line points on the upper contour (XCU,YCU)
C        or lower contour (XCL,YCL) are only valid in point pairs, ie. 
C        points 1 and 2 define a segment, points 3 and 4 define the next,
C        etc.  In general, there is no guarantee that the cross-pair points
C        (like points 2 and 3) will be contiguous. DO NOT PLOT THESE AS A 
C        CONTIGUOUS ARRAY OF POINTS.
C
C  Note: the output areas (NA polygon areas) are stored with the points
C        for all polygons in one big XP,YP array.  The number of points
C        in each polygon are stored in NE, i.e. NE(1) is the number of
C        points stored in XP,YP for the first polygon, NE(2) is the number
C        of vertices stored following those for the second polygon, etc.  
C        The total number of vertices is NV=sum[NE(1)+NE(2)..+NE(NA)]. 
C
C  Note:  the NCU,NCL,NA,NV counters are cumulative in this routine!
C         If you want to contour each quadrilateral without accumulating
C         contour points or polygons reset NCU=0,NCL=0,NA=0,NV=0 before
C         each call to CONTQUAD.
C
C--------------------------------------------------------------------------
      DIMENSION X(4), Y(4), F(4)
      DIMENSION XP(1), YP(1), NE(1)
      DIMENSION XCL(1), YCL(1), XCU(1), YCU(1)
C
      DIMENSION IANG(4)
      DIMENSION XTMP(3), YTMP(3), FTMP(3)
C
C----  NCU tracks the number of upper contour points
C      NCL tracks the number of lower contour points
C      NA tracks the number of polygon areas
C      NV tracks the total number of stored vertices in the XP,YP arrays
C---- Uncomment these if you want to reset the counters each time you 
C     contour a quadrilateral
C      NA = 0
C      NV = 0
C      NCU = 0
C      NCL = 0
C
      FHI = FUPR
      FLO = FLWR
      IF (FHI.LT.FLO) THEN
        FHI = FLWR
        FLO = FUPR
      ENDIF
C
C---- Extrema
      FMAX = AMAX1(F(1),F(2),F(3),F(4))
      FMIN = AMIN1(F(1),F(2),F(3),F(4))
C
C---- If cell is above contour band or below contour band skip it
      IF (FMAX.LE.FLO .OR. FMIN.GE.FHI) RETURN
C
C---- If cell is totally within contour band there are no contour lines,
C     and cell can be shaded directly
      IF (FMAX.LE.FHI .AND. FMIN.GE.FLO) THEN
        DO 2 I = 1, 4
          NV = NV + 1
          XP(NV) = X(I)
          YP(NV) = Y(I)
    2   CONTINUE
        NA = NA + 1
        NE(NA) = 4
        GO TO 100
      ENDIF
C
C----Check for convex or concave quadrilaterals
      ISUM = 0
      DO 3 J = 1, 4
        JM = MOD(J+2,4) + 1
        JP = MOD(J,  4) + 1
        IANG(J) = 1
        IF ( (X(J)-X(JM))*(Y(JP)-Y(J)) .LT.
     &       (X(JP)-X(J))*(Y(J)-Y(JM)) ) IANG(J) = -1
        ISUM = ISUM + IANG(J)
    3 CONTINUE
      DO 4 J = 1, 4
        IF (IANG(J)*ISUM.LT.0) GO TO 10
    4 CONTINUE
C
C----All angles < 180 deg., split into 4 triangles with average center pt
      XTMP(3) = 0.25*(X(1)+X(2)+X(3)+X(4))
      YTMP(3) = 0.25*(Y(1)+Y(2)+Y(3)+Y(4))
      FTMP(3) = 0.25*(F(1)+F(2)+F(3)+F(4))
C
      DO 5 I = 1, 4
        IP = MOD(I,4) + 1
        XTMP(1) = X(I)
        YTMP(1) = Y(I)
        FTMP(1) = F(I)
        XTMP(2) = X(IP)
        YTMP(2) = Y(IP)
        FTMP(2) = F(IP)
        CALL CONTTRI(XTMP,YTMP,FTMP,FHI,FLO,
     &                NCU,XCU,YCU,NCL,XCL,YCL,NA,NE,NV,XP,YP)      
    5 CONTINUE
      GO TO 100
C
C----Quadrilaterals with an angle > 180, two triangles
   10 XTMP(3) = X(J)
      YTMP(3) = Y(J)
      FTMP(3) = F(J)
C
      JP1 = MOD(J,4) + 1
      XTMP(1) = X(JP1)
      YTMP(1) = Y(JP1)
      FTMP(1) = F(JP1)
      JP2 = MOD(JP1,4) + 1
      XTMP(2) = X(JP2)
      YTMP(2) = Y(JP2)
      FTMP(2) = F(JP2)
      CALL CONTTRI(XTMP,YTMP,FTMP,FHI,FLO,
     &              NCU,XCU,YCU,NCL,XCL,YCL,NA,NE,NV,XP,YP)      
C
      JM2 = MOD(JM+1,4) + 1
      XTMP(1) = X(JM2)
      YTMP(1) = Y(JM2)
      FTMP(1) = F(JM2)
      JM1 = MOD(JM2,4) + 1
      XTMP(2) = X(JM1)
      YTMP(2) = Y(JM1)
      FTMP(2) = F(JM1)
      CALL CONTTRI(XTMP,YTMP,FTMP,FHI,FLO,
     &              NCU,XCU,YCU,NCL,XCL,YCL,NA,NE,NV,XP,YP)      
C
  100 RETURN
      END



      subroutine CONTTRI(X,Y,F,
     &                    FUPR,FLWR,
     &                    NCU,XCU,YCU,
     &                    NCL,XCL,YCL,
     &                    NA,NE,NV,XP,YP)      
C
C  Contour a single triangular element between an upper and 
C  a lower contour limit.  The output from this routine is both the line
C  segments defining the upper and lower contours and a set of
C  polygons that define the area of the element lying between the 
C  contour limits.  These may be used to shade the area on a plot.
C
C    X, Y,      Arrays containing triangular element points
C    F          Array containing the quantity F(x,y) to be contoured, 
C                ie. F(i) is defined at X(i),Y(i), i=1->3
C    FUPR, FLWR Upper and lower contour limits
C    NCU        Number of upper limit contour line points
C    XCU, YCU   Arrays of x,y points on upper contour
C    NCL        Number of lower limit contour line points
C    XCL, YCL   Arrays of x,y points on lower contour
C    NA         Number of polygon areas in XP, YP, NE arrays
C    NE         Array of numbers of points in each polygon area
C    NV         Vertex count of points in XP,YP arrays
C    XP, YP     Coordinate points of contour polygons
C
C  Note: the output contour line points on the upper contour (XCU,YCU)
C        or lower contour (XCL,YCL) are only valid in point pairs, ie. 
C        points 1 and 2 define a segment, points 3 and 4 define the next,
C        etc.  In general, there is no guarantee that the cross-pair points
C        (like points 2 and 3) will be contiguous.  DO NOT PLOT THESE AS A 
C        CONTIGUOUS ARRAY OF POINTS.
C
C  Note: the output areas (NA polygons) are stored with the points
C        for all polys in one big XP,YP array.  The number of points
C        in each polygon are stored in NE, i.e. NE(1) is the number of
C        points stored in XP,YP for the first polygon, NE(2) is the number
C        of vertices stored following those for the second polygon, etc.  
C        The total number of vertices is NV=sum[NE(1)+NE(2)..+NE(NA)]. 
C
C  Note:  the NCU,NCL,NA,NV counters are cumulative in this routine!
C         If you want to contour each triangle without accumulating
C         contour points or polygons reset NCU=0,NCL=0,NA=0,NV=0 before
C         each call to CONTTRI.
C
      DIMENSION X(3), Y(3), F(3), FH(3), FL(3),
     &          XCU(1), YCU(1), XCL(1), YCL(1), XP(1), YP(1), NE(1)
C
C----  NCU tracks the number of upper contour points
C      NCL tracks the number of lower contour points
C      NA tracks the number of polygon areas
C      NV tracks the total number of stored vertices in the XP,YP arrays
C---- Uncomment these if you want to reset the counters each time you 
C     contour a triangle
C      NA = 0
C      NV = 0
C      NCU = 0
C      NCL = 0
C
      FHI = FUPR
      FLO = FLWR
      IF (FHI.LT.FLO) THEN
        FHI = FLWR
        FLO = FUPR
      ENDIF

      NVFRST = NV
C
      EPS = 0.0001*(FHI-FLO)
C
C---- Temporary values
      DO 1 I = 1, 3
        FH(I) = F(I) - FHI
        FL(I) = F(I) - FLO
        IF (FH(I).GE.0. .AND. FH(I).LT. EPS) FH(I) =  EPS
        IF (FH(I).LE.0. .AND. FH(I).GT.-EPS) FH(I) = -EPS
        IF (FL(I).GE.0. .AND. FL(I).LT. EPS) FL(I) =  EPS
        IF (FL(I).LE.0. .AND. FL(I).GT.-EPS) FL(I) = -EPS
    1 CONTINUE
C
C----Check point by point for points in contour limits
      DO 50 I = 1, 3
C
C----Inside contour limits
        IF (FH(I).LT.0. .AND. FL(I).GT.0.) THEN
          NV = NV + 1
          XP(NV) = X(I)
          YP(NV) = Y(I)
C
        ELSE
C
          IF (FH(I).GE.0.) THEN
C----Check for intersections with previous and next point
            IP = MOD(I,3)   + 1
            IM = MOD(I+1,3) + 1
            IF (FH(IM).LT.0.) THEN
              ETA = -(FH(IM)+FH(I))/(FH(IM)-FH(I))
              NCU = NCU + 1
              XCU(NCU) = 0.5*(X(I) + X(IM) + ETA*(X(IM)-X(I)) )
              YCU(NCU) = 0.5*(Y(I) + Y(IM) + ETA*(Y(IM)-Y(I)) )
              NV = NV + 1
              XP(NV) = XCU(NCU)
              YP(NV) = YCU(NCU)
            ENDIF
            IF (FH(IP).LT.0.) THEN
              ETA = -(FH(IP)+FH(I))/(FH(IP)-FH(I))
              NCU = NCU + 1
              XCU(NCU) = 0.5*(X(I) + X(IP) + ETA*(X(IP)-X(I)) )
              YCU(NCU) = 0.5*(Y(I) + Y(IP) + ETA*(Y(IP)-Y(I)) )
              NV = NV + 1
              XP(NV) = XCU(NCU)
              YP(NV) = YCU(NCU)
            ENDIF
          ENDIF
C
          IF (FL(I).LE.0.) THEN
C----Check for intersections with previous and next point
            IP = MOD(I,3)   + 1
            IM = MOD(I+1,3) + 1
            IF (FL(IM).GT.0.) THEN
              ETA = -(FL(IM)+FL(I))/(FL(IM)-FL(I))
              NCL = NCL + 1
              XCL(NCL) = 0.5*(X(I) + X(IM) + ETA*(X(IM)-X(I)) )
              YCL(NCL) = 0.5*(Y(I) + Y(IM) + ETA*(Y(IM)-Y(I)) )
              NV = NV + 1
              XP(NV) = XCL(NCL)
              YP(NV) = YCL(NCL)
            ENDIF
            IF (FL(IP).GT.0.) THEN
              ETA = -(FL(IP)+FL(I))/(FL(IP)-FL(I))
              NCL = NCL + 1
              XCL(NCL) = 0.5*(X(I) + X(IP) + ETA*(X(IP)-X(I)) )
              YCL(NCL) = 0.5*(Y(I) + Y(IP) + ETA*(Y(IP)-Y(I)) )
              NV = NV + 1
              XP(NV) = XCL(NCL)
              YP(NV) = YCL(NCL)
            ENDIF
          ENDIF
C     
        ENDIF
C
   50 CONTINUE
C
      IF (NV.GT.NVFRST+2) THEN
        NA = NA + 1
        NE(NA) = NV - NVFRST
      ENDIF
C
      RETURN
      END



      subroutine AXISADJ(xmin,xmax,xspan,deltax,ntics)
C...Make scaled axes with engineering increments between tics
C
C   Input:    xmin, xmax - input range for which scaled axis is desired
C
C   Output:   xmin, xmax - adjusted range for scaled axis
C             xspan      - adjusted span of scaled axis
C             deltax     - increment to be used for scaled axis
C             nincr      - number of tics to be used on axis
C                          note that ntics=1+(xspan/deltax)
C
      real    xmin,xmax,xspan,deltax,xinc,xinctbl(5)
      integer ntics,i
      data    xinctbl / 0.1, 0.2, 0.25, 0.5, 1. /
c
      xspan1 = xmax-xmin
      if (xspan1.eq.0.) xspan1 = 1.
c
      xpon = ifix(log10(xspan1))
      xspan = xspan1 / 10.**xpon
c
      do i = 1, 5
        xinc = xinctbl(i)
        ntics = 1 + ifix(xspan/xinc + 0.1)
        if (ntics.LE.6) go to 1
      end do
c
   1  deltax = xinc*10.**xpon
      xmin = deltax*  ifloor(xmin/deltax)
      xmax = deltax*iceiling(xmax/deltax)
      xspan = xmax - xmin
      ntics = 1 + ifix(xspan/deltax + 0.1)
      return
      end

      function iceiling(x)
c--- returns next highest integer value if fraction is non-zero 
      integer iceiling
      real x
      i = ifix(x)
      if(x-i.GT.0.) i = i+1
      iceiling = i
      return
      end

      function ifloor(x)
c--- returns next lowest integer value if fraction is negative, non-zero
      integer ifloor
      real x
      i = ifix(x)
      if(x-i.LT.0.) i = i-1
      ifloor = i
      return
      end


 


      subroutine ANNOT(CH)
C------------------------------------------------------
C     Interactive annotation menu for adding custom 
C     ornaments to an active plot (before PLEND call).
C------------------------------------------------------
      CHARACTER*80 AA
      CHARACTER*1 OPT, KCHAR
C
      SAVE CHF, ISYMB
      DATA CHF, ISYMB / 1.0, 0 /
C
 900  CONTINUE
C
 1000 FORMAT(A)
 1010 FORMAT(A,$)
 1020 FORMAT(A,F7.3,A,$)
 1030 FORMAT(A,I2  ,A,$)
C
      WRITE(*,1050)
 1050 FORMAT(/'   C haracters           |       '
     &       /'   S lant characters     |       '
     &       /'   M ath characters      |  plot '
     &       /'   P oint symbol         |       '
     &       /'   L ine                 |       '
     &       /'   A rrow                |       '
     &       /' '
     &       /'   W idth of characters  | modify'
     &       /'   T ype of point symbol |       ')
C
 905  WRITE(*,*)
      WRITE(*,1010) ' Select option or <return>:  '
      READ(*,1000) OPT
      IF(OPT.EQ.' ') RETURN
C
      CHI = CHF*CH
C
C-------------------------------------------------------------
      IF(INDEX('CcSsMm',OPT).NE.0) THEN
C
      WRITE(*,*) 'Click on lower left point of character string...'
      CALL GETCURSORXY(XX,YY,KCHAR)
      WRITE(*,1010) ' Enter character string:  '
      READ (*,1000) AA
C
C---- find index of last non-blank character
      DO 112 NA=80, 1, -1
        IF(AA(NA:NA).NE.' ') GO TO 113
  112 CONTINUE
  113 CONTINUE
C
      CALL NEWPEN(3)
      IF(INDEX('Cc',OPT).NE.0) CALL PLCHAR(XX,YY,CHI,AA,0.0,NA)
      IF(INDEX('Ss',OPT).NE.0) CALL PLSLAN(XX,YY,CHI,AA,0.0,NA)
      IF(INDEX('Mm',OPT).NE.0) CALL PLMATH(XX,YY,CHI,AA,0.0,NA)
      CALL PLFLUSH
C
C-------------------------------------------------------------
      ELSE IF(INDEX('Pp',OPT).NE.0) THEN
C
      WRITE(*,*) 'Click on symbol locations ...'
      CALL GETCURSORXY(XX,YY,KCHAR)
      CALL NEWPEN(2)
      CALL PLSYMB(XX,YY,CHI,ISYMB,0.0,0)
      CALL PLFLUSH
C
C-------------------------------------------------------------
      ELSE IF(INDEX('LlAa',OPT).NE.0) THEN
C
      WRITE(*,*) 'Click on line points, twice on last point...'
C
      CALL NEWPEN(1)
      CALL GETCURSORXY(XXM,YYM,KCHAR)
      CALL PLOT(XXM,YYM,3)
      CALL PLOT(XXM,YYM,2)
      XXL = XXM
      YYL = YYM
      CALL PLFLUSH
      DO 131 IP=1, 12345
        CALL GETCURSORXY(XX,YY,KCHAR)
        CALL PLOT(XX,YY,2)

        CALL PLFLUSH

        IF(XXM.EQ.XX .AND. YYM.EQ.YY) GO TO 132
         XXL = XXM
         YYL = YYM
         XXM = XX
         YYM = YY
 131  CONTINUE
 132  CONTINUE
C
      IF(INDEX('Aa',OPT).NE.0) THEN
C------ add arrowhead
        DX = XX - XXL
        DY = YY - YYL
        DS = SQRT(DX**2 + DY**2)
        ARLEN = 1.5*CHI
        HAR = 0.1
cc        IF(DS .GT. ARLEN) THEN
          CALL PLOT(XX-ARLEN*(DX+HAR*DY)/DS,YY-ARLEN*(DY-HAR*DX)/DS,2)
          CALL PLOT(XX-ARLEN*(DX-HAR*DY)/DS,YY-ARLEN*(DY+HAR*DX)/DS,2)
          CALL PLOT(XX,YY,2)
cc        ENDIF
      ENDIF
C
      XX = XX + 0.7*CHI
      YY = YY - 0.5*CHI
      CALL PLFLUSH
C
C-------------------------------------------------------------
      ELSE IF(INDEX('Ww',OPT).NE.0) THEN
C
 140  WRITE(*,1020) 
     &   ' Enter new character width factor (currently =',CHF,'):  '
      READ (*,*,ERR=140) CHF
C
C-------------------------------------------------------------
      ELSE IF(INDEX('Tt',OPT).NE.0) THEN
C
      WRITE(*,*)
      WRITE(*,*) '  0  square         7  Y         '
      WRITE(*,*) '  1  circle         8  flipped Y '
      WRITE(*,*) '  2  triangle       9  *         '
      WRITE(*,*) '  3  +             10  flipped * '
      WRITE(*,*) '  4  x             11  hourglass '
      WRITE(*,*) '  5  diamond       12  bowtie    '
      WRITE(*,*) '  6  yield sign    13  star      ' 
      WRITE(*,*)
 160  WRITE(*,1030) ' Enter new symbol type (currently =',ISYMB,'):  '
      READ (*,*,ERR=160) ISYMB
C
C-------------------------------------------------------------
      ELSE
C
       GO TO 900
C
      ENDIF
C
      GO TO 905
      END ! ANNOT
