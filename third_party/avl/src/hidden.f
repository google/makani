C***********************************************************************
C    Module:  hidden.f
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

      SUBROUTINE TRIINIT(ID,NROWS,NCOLS,PTS, NTRI,TRI)
C...PURPOSE:  Set up triangle data for hidden line routine from quadrilateral
C             input polygons.
C
C          This routine takes a grid of quadrilaterals and defines
C          two triangles per quad and puts them into the triangle 
C          arrays.  The input grid of quads is in the form of NROWSxNCOLS
C          of quads, defined by NROWS+1 x NCOLS+1 points in the PTS array. 
C          The ID index corresponds to the first quad in the lattice and 
C          is incremented as TRIINIT steps through the quad array to define
C          triangles.  Thus the output value of ID is ID + NROWSxNCOLS - 1.
C          This allows the user to use the ID option in the hidden line 
C          routines to avoid visibility tests for input features that  
C          lie on triangles with the same ID index.
C
      REAL PTS(3,*)
      REAL XYZLIM(3,2), TRI(16,*)
C
      IT = NTRI
      IDX = ID - 1
C
C--- For a rectangular grid of quadrilaterals defined by NROWS, NCOLS
C    polygons, find the vertices of each quadrilateral and divide it into
C    two triangles.
C
      DO 110 J = 1, NCOLS
        DO 1110 K = 1, NROWS
C          
C--- Increment ID# to get ID for next element
          IDX  = IDX + 1
C
          IP1 = (NROWS+1)*(J-1) + K
          IP2 = IP1 + 1
          IP3 = IP1 + NROWS+1
          IP4 = IP3 + 1
C
C          1  = x1
C          2  = y1
C          3  = z1
C          4  = x2
C          5  = y2
C          6  = z2
C          7  = x3
C          8  = y3
C          9  = z3
C         10  = min(x)
C         11  = min(y)
C         12  = min(z)
C         13  = max(x)
C         14  = max(y)
C         15  = max(z)
C         16  = ID reference number
C
C--- First triangle
          IT = IT+1
C 
          TRI(1,IT) = PTS(1,IP1)
          TRI(2,IT) = PTS(2,IP1)
          TRI(3,IT) = PTS(3,IP1)
C         
          TRI(4,IT) = PTS(1,IP2)
          TRI(5,IT) = PTS(2,IP2)
          TRI(6,IT) = PTS(3,IP2)
C         
          TRI(7,IT) = PTS(1,IP3)
          TRI(8,IT) = PTS(2,IP3)
          TRI(9,IT) = PTS(3,IP3)
C         
          TRI(10,IT) = MIN( TRI(1,IT) , TRI(4,IT) , TRI(7,IT) )
          TRI(11,IT) = MIN( TRI(2,IT) , TRI(5,IT) , TRI(8,IT) )
          TRI(12,IT) = MIN( TRI(3,IT) , TRI(6,IT) , TRI(9,IT) )
          TRI(13,IT) = MAX( TRI(1,IT) , TRI(4,IT) , TRI(7,IT) )
          TRI(14,IT) = MAX( TRI(2,IT) , TRI(5,IT) , TRI(8,IT) )
          TRI(15,IT) = MAX( TRI(3,IT) , TRI(6,IT) , TRI(9,IT) )
C
          TRI(16,IT) = FLOAT(IDX)
C
C--- Second triangle          
          IT = IT+1
C         
          TRI(1,IT) = PTS(1,IP3)
          TRI(2,IT) = PTS(2,IP3)
          TRI(3,IT) = PTS(3,IP3)
C         
          TRI(4,IT) = PTS(1,IP2)
          TRI(5,IT) = PTS(2,IP2)
          TRI(6,IT) = PTS(3,IP2)
C         
          TRI(7,IT) = PTS(1,IP4)
          TRI(8,IT) = PTS(2,IP4)
          TRI(9,IT) = PTS(3,IP4)
C         
          TRI(10,IT) = MIN( TRI(1,IT) , TRI(4,IT) , TRI(7,IT) )
          TRI(11,IT) = MIN( TRI(2,IT) , TRI(5,IT) , TRI(8,IT) )
          TRI(12,IT) = MIN( TRI(3,IT) , TRI(6,IT) , TRI(9,IT) )
          TRI(13,IT) = MAX( TRI(1,IT) , TRI(4,IT) , TRI(7,IT) )
          TRI(14,IT) = MAX( TRI(2,IT) , TRI(5,IT) , TRI(8,IT) )
          TRI(15,IT) = MAX( TRI(3,IT) , TRI(6,IT) , TRI(9,IT) )
C
          TRI(16,IT) = FLOAT(IDX)
C         
 1110   CONTINUE
  110 CONTINUE
C
      NTRI = IT
      ID = IDX 
C
      RETURN
      END ! TRIINIT



      SUBROUTINE HIDLIN(XYZLIN,ID,NGRP,ILSTGRP,XYZGRP,
     &                   NTRI,XYZTRI,NSEG,ALFSEG)
C
C      This is a hidden line routine.  Given a set of triangles, and a
C      single line as input, it computes the line segments (if any)
C      which are visible.
C
C      XYZLIN = array of data about input line (unchanged by routine)
C          1,1  = x1
C          2,1  = y1
C          3,1  = z1
C          1,2  = x2
C          2,2  = y2
C          3,2  = z2
C      ID = ID index for input line 
C           May be used to designate triangle or collection of triangles 
C           that contain this line and should be skipped for visibility 
C           tests.  If ID=0 no triangles will be skipped.
C
C      NGRP = number of triangle groups 
C             (if NGRP<=0 no group minmax tests will be done)
C      ILSTGRP(n) = index of last triangle in group n
C      XYZGRP(.n) = x,y group limits
C             1   = min(x)
C             2   = min(y)
C             3   = min(z)
C             4   = max(x)
C             5   = max(y)
C             6   = max(z)
C
C      NTRI = number of triangles (used if NGRP=0)
C      XYZTRI = array of triangle data
C          1  = x1
C          2  = y1
C          3  = z1
C          4  = x2
C          5  = y2
C          6  = z2
C          7  = x3
C          8  = y3
C          9  = z3
C         10  = min(x)
C         11  = min(y)
C         12  = min(z)
C         13  = max(x)
C         14  = max(y)
C         15  = max(z)
C         16  = ID field, ID associated with this triangle 
C
C      NSEG   = (input)  dimension of ALFSEG = max. no. of segments
C               (output) number of visible line segments
C      ALFSEG = output values describing the visible line segments as
C               fractions of the original line
C          1  = beginning of segment
C          2  =       end of segment
C
       REAL XYZLIN(3,2), XYZTRI(16,*), ALFSEG(2,NSEG)
       REAL XYZGRP(6,NGRP)
       INTEGER ILSTGRP(NGRP)
C
       DATA EPS/1.0E-7/
C
       NMXSEG = NSEG
       NSEG = 0
C
C----- Initialize a few things
       X1 = XYZLIN(1,1)
       Y1 = XYZLIN(2,1)
       Z1 = XYZLIN(3,1)
       X2 = XYZLIN(1,2)
       Y2 = XYZLIN(2,2)
       Z2 = XYZLIN(3,2)
C
       XMIN = MIN(X1,X2)
       YMIN = MIN(Y1,Y2)
       ZMIN = MIN(Z1,Z2)
       XMAX = MAX(X1,X2)
       YMAX = MAX(Y1,Y2)
       ZMAX = MAX(Z1,Z2)
C---Check for degenerate line
       IF(XMAX.EQ.XMIN .AND. YMAX.EQ.YMIN .AND. ZMAX.EQ.ZMIN) RETURN
C
       NSEG = 1
       ALFSEG(1,1) = 0.0
       ALFSEG(2,1) = 1.0
C
C----- Main processing loop
       KLAST = 0
       NNGRP = MAX(1,NGRP)
C
       DO 100 IGRP=1, NNGRP
C
       KT1 = KLAST + 1
       KLAST = NTRI
C
       IF(NGRP.GE.1) THEN   
C--- Check for group with no triangles (last index <= 0)
       IF(ILSTGRP(IGRP).LE.0) GO TO 100
        KLAST = ILSTGRP(IGRP)       
C..... preliminary purge of whole group
        IF( (XYZGRP(1,IGRP).GE.XMAX .OR. 
     &       XYZGRP(4,IGRP).LE.XMIN) .OR.
     &      (XYZGRP(2,IGRP).GE.YMAX .OR. 
     &       XYZGRP(5,IGRP).LE.YMIN) ) GO TO 100
C     &      (XYZGRP(6,IGRP).LE.ZMIN) ) GO TO 100
       ENDIF
C
       DO 1 KT=KT1, KLAST
C
C....... second purge of most cells
         IF( (XYZTRI(10,KT).GE.XMAX .OR. 
     &        XYZTRI(13,KT).LE.XMIN)   .OR.
     &       (XYZTRI(11,KT).GE.YMAX .OR. 
     &        XYZTRI(14,KT).LE.YMIN)   .OR.
     &       (XYZTRI(15,KT).LE.ZMIN) ) GO TO 1
C
C....... if ID of this line matches a triangle ID skip visibility tests
         IF(ID.NE.0) THEN
          IDTRI = IFIX(XYZTRI(16,KT))
          IF(ID.EQ.IDTRI) GO TO 1
         ENDIF
C
C---Check for line as a side of masking triangle
         NSAME=0
         DO IS=1,2
           DO L=1, 3
             IF( ABS(XYZLIN(1,IS)-XYZTRI(1+3*(L-1),KT)).LT.EPS .AND.
     &           ABS(XYZLIN(2,IS)-XYZTRI(2+3*(L-1),KT)).LT.EPS .AND.
     &           ABS(XYZLIN(3,IS)-XYZTRI(3+3*(L-1),KT)).LT.EPS ) THEN
               NSAME=NSAME+1
             ENDIF
           END DO
         END DO
         IF(NSAME.GE.2) THEN
          GO TO 1
         ENDIF
C
C--- Find where cell nodes are relative to line
         N1 = 0
           IF( (XYZTRI(1,KT)-X1)*(Y2-Y1) .GT. 
     &         (XYZTRI(2,KT)-Y1)*(X2-X1) )    N1 = N1+1
           IF( (XYZTRI(4,KT)-X1)*(Y2-Y1) .GT. 
     &         (XYZTRI(5,KT)-Y1)*(X2-X1) )    N1 = N1+2
           IF( (XYZTRI(7,KT)-X1)*(Y2-Y1) .GT. 
     &         (XYZTRI(8,KT)-Y1)*(X2-X1) )    N1 = 3-N1
C
C--- Skip tests if triangle points all on one side of line
         IF(N1.EQ.0) GO TO 1
C
C--- Set up temporary variables in canonical orientation where the line
C    passes through sides 1-2 and 1-3 of triangle (point 1 is common point)
         N2 = MOD(N1,3) + 1           
         N3 = MOD(N2,3) + 1           
C
         XT1 = XYZTRI(3*N1-2,KT)
         YT1 = XYZTRI(3*N1-1,KT)
         ZT1 = XYZTRI(3*N1  ,KT)
         XT2 = XYZTRI(3*N2-2,KT)
         YT2 = XYZTRI(3*N2-1,KT)
         ZT2 = XYZTRI(3*N2  ,KT)
         XT3 = XYZTRI(3*N3-2,KT)
         YT3 = XYZTRI(3*N3-1,KT)
         ZT3 = XYZTRI(3*N3  ,KT)
C
C--- Compute xi, eta, zeta values for line endpoints in triangle coords
         DET = ( (XT2-XT1)*(YT3-YT1) - (XT3-XT1)*(YT2-YT1) )
         IF(DET.EQ.0.0) GO TO 1
C
         DETINV = 1.0 / DET
         XI1 = DETINV * ( (X1 -XT1)*(YT3-YT1) - (XT3-XT1)*(Y1 -YT1) )
         ET1 = DETINV * ( (XT2-XT1)*(Y1 -YT1) - (X1 -XT1)*(YT2-YT1) )
         XI2 = DETINV * ( (X2 -XT1)*(YT3-YT1) - (XT3-XT1)*(Y2 -YT1) )
         ET2 = DETINV * ( (XT2-XT1)*(Y2 -YT1) - (X2 -XT1)*(YT2-YT1) )
C
C--- Check for line parallel to triangle sides (skip hide tests)
         IF( (ABS(XI1).LT.EPS .AND. ABS(XI2).LT.EPS) .OR.
     &       (ABS(ET1).LT.EPS .AND. ABS(ET2).LT.EPS) ) GO TO 1
C
C--- Get Z coordinates relative to triangle plane
         IF(ABS(XI1-1.0).LT.EPS) XI1 = 1.0
         IF(ABS(ET1-1.0).LT.EPS) ET1 = 1.0
         ZE1 = (Z1-ZT1) - XI1*(ZT2-ZT1) - ET1*(ZT3-ZT1)
C
         IF(ABS(XI2-1.0).LT.EPS) XI2 = 1.0
         IF(ABS(ET2-1.0).LT.EPS) ET2 = 1.0
         ZE2 = (Z2-ZT1) - XI2*(ZT2-ZT1) - ET2*(ZT3-ZT1)
C
C--- Skip this triangle test if line is outside XI=0,ETA=0 or XI+ETA-1 edges
         IF(MAX(XI1,XI2).LE.0.0 .OR.
     &      MAX(ET1,ET2).LE.0.0 .OR.
     &      MIN(XI1+ET1,XI2+ET2) .GT. 1.0) GO TO 1
C
C--- Check for line completely above triangle plane
         IF(MIN(ZE1,ZE2).GE.0.) GO TO 1
C
         ALF1 = 0.0
         ALF2 = 1.0
C
C--- Intersect the line with sides 1-2 and 1-3 
C    This defines the ALF (relative coordinate on line 0->1)
         DENBET = (X2-X1)*(YT2-YT1) - (Y2-Y1)*(XT2-XT1)
         DENGAM = (X2-X1)*(YT3-YT1) - (Y2-Y1)*(XT3-XT1)
         IF( ABS(DENBET).LT.EPS .OR.
     &       ABS(DENGAM).LT.EPS ) GO TO 1

         BET = ( (0.5*(XT1+XT2)-X1)*(YT2-YT1) -
     &           (0.5*(YT1+YT2)-Y1)*(XT2-XT1) ) / DENBET
C
         GAM = ( (0.5*(XT1+XT3)-X1)*(YT3-YT1) -
     &           (0.5*(YT1+YT3)-Y1)*(XT3-XT1) ) / DENGAM
C
         ALF1 = MAX(ALF1,MIN(BET,GAM))
         ALF2 = MIN(ALF2,MAX(BET,GAM))
C
C--- If line endpoint Z values straddle the triangle plane, also 
C    check the line-plane intersection point
         IF(ZE1.GT.0.0 .AND. ZE2.LT.0.0) THEN
           ALF1 = MAX(ALF1, ZE1/(ZE1-ZE2))
          ELSEIF(ZE1.LT.0.0 .AND. ZE2.GT.0.0) THEN
           ALF2 = MIN(ALF2, ZE1/(ZE1-ZE2))
         ENDIF
C
C
C--- Skip to next triangle if no part is hidden
         IF(ALF1.GE.ALF2) GO TO 1
         IF(ALF1 .LT.     EPS) ALF1 = 0.0
         IF(ALF2 .GT. 1.0-EPS) ALF2 = 1.0
C
C....... combine new hidden part with previous visible bits
         NSEG2 = NSEG
C
         DO NS = NSEG2, 1, -1
           IF(ALF1.GE.ALFSEG(2,NS) .OR. ALF2.LE.ALFSEG(1,NS)) THEN
           ELSE IF(ALF1.GT.ALFSEG(1,NS) .AND. ALF2.LT.ALFSEG(2,NS)) THEN
            IF(NSEG.EQ.NMXSEG) THEN
             WRITE(*,*) '** HIDLIN: too many segments'
             GO TO 100
            ENDIF
            NSEG = NSEG+1
            ALFSEG(1,NSEG) = ALF2
            ALFSEG(2,NSEG) = ALFSEG(2,NS)
            ALFSEG(2,NS) = ALF1
           ELSE IF(ALF1.LE.ALFSEG(1,NS) .AND. ALF2.GE.ALFSEG(2,NS)) THEN
            ALFSEG(1,NS) = ALFSEG(1,NSEG)
            ALFSEG(2,NS) = ALFSEG(2,NSEG)
            NSEG = NSEG-1
           ELSE IF(ALF1.LE.ALFSEG(1,NS) .AND. ALF2.LT.ALFSEG(2,NS)) THEN
            ALFSEG(1,NS) = ALF2
           ELSE IF(ALF1.GT.ALFSEG(1,NS) .AND. ALF2.GE.ALFSEG(2,NS)) THEN
            ALFSEG(2,NS) = ALF1
           ENDIF
         END DO
C
         IF(NSEG.EQ.0) RETURN
 1     CONTINUE
C
 100   CONTINUE
       RETURN
       END ! HIDLIN


       SUBROUTINE HIDPNT(XYZPT,ID,NGRP,ILSTGRP,XYZGRP,NTRI,XYZTRI,LVIS)
C
C      This routine determines the visibility of a single point.
C      Given a set of triangles it computes visibility of an input point
C
C      XYZPT = test point coordinate array (X,Y,Z)
C      ID = ID index for input point 
C           May be used to designate triangle or collection of triangles 
C           that contain this point and should be skipped for visibility 
C           tests.  If ID=0 no triangles will be skipped.
C
C      NGRP = number of triangle groups 
C             (if NGRP<=0 no group minmax tests will be done)
C      ILSTGRP(n) = index of last triangle in group n
C      XYZGRP(.n) = x,y group limits
C             1   = min(x)
C             2   = min(y)
C             3   = min(z)
C             4   = max(x)
C             5   = max(y)
C             6   = max(z)
C
C      NTRI = number of triangles (used if NGRP=0)
C      XYZTRI = array of triangle data
C          1  = x1
C          2  = y1
C          3  = z1
C          4  = x2
C          5  = y2
C          6  = z2
C          7  = x3
C          8  = y3
C          9  = z3
C         10  = min(x)
C         11  = min(y)
C         12  = min(z)
C         13  = max(x)
C         14  = max(y)
C         15  = max(z)
C         16  = ID field, ID associated with this triangle 
C
C      LVIS = logical flag  LVIS=.true.  if the point is visible
C
       REAL XYZPT(3), XYZTRI(16,*), XYZGRP(6,NGRP)
       INTEGER ILSTGRP(NGRP)
       LOGICAL LVIS
C
       DATA EPS/1.0E-7/
C
C----- Initialize a few things
       X1 = XYZPT(1)
       Y1 = XYZPT(2)
       Z1 = XYZPT(3)
       LVIS=.TRUE.
C
C----- Main processing loop
       KLAST = 0
       NNGRP = MAX(1,NGRP)
C
       DO 100 IGRP=1, NNGRP
C
       KT1 = KLAST + 1
       KLAST = NTRI
C
       IF(NGRP.GE.1) THEN
C--- Check for group with no triangles (last index <= 0)
        IF(ILSTGRP(IGRP).LE.0) GO TO 100
        KLAST = ILSTGRP(IGRP)       
C--- Preliminary purge of whole group
        IF( (XYZGRP(1,IGRP).GE.X1 .OR. 
     &       XYZGRP(4,IGRP).LE.X1) .OR.
     &      (XYZGRP(2,IGRP).GE.Y1 .OR. 
     &       XYZGRP(5,IGRP).LE.Y1) ) GO TO 100
C     &      (XYZGRP(6,IGRP).LE.Z1) ) GO TO 100
       ENDIF
C
       DO 1 KT=KT1, KLAST
C
C--- Second purge of most cells
         IF( (XYZTRI(10,KT).GE.X1 .OR. 
     &        XYZTRI(13,KT).LE.X1)   .OR.
     &       (XYZTRI(11,KT).GE.Y1 .OR. 
     &        XYZTRI(14,KT).LE.Y1)   .OR.
     &       (XYZTRI(15,KT).LE.Z1) ) GO TO 1
C
C--- If ID of this point matches a triangle ID skip visibility tests
         IF(ID.NE.0) THEN
          IDTRI = IFIX(XYZTRI(16,KT))
          IF(ID.EQ.IDTRI) GO TO 1
         ENDIF
C
         NSAME=0
         DO L=1, 3
           IF( ABS(X1-XYZTRI(1+3*(L-1),KT)).LT.EPS .AND.
     &         ABS(Y1-XYZTRI(2+3*(L-1),KT)).LT.EPS .AND.
     &         ABS(Z1-XYZTRI(3+3*(L-1),KT)).LT.EPS )  NSAME=NSAME+1
         END DO
         IF(NSAME.GE.1) GO TO 1
C
         N1 = 1
C--- Set up temporary variables in canonical orientation where the line
C    passes through sides 1-2 and 1-3 of triangle (point 1 is common point)
         N2 = MOD(N1,3) + 1           
         N3 = MOD(N2,3) + 1           
C
         XT1 = XYZTRI(3*N1-2,KT)
         YT1 = XYZTRI(3*N1-1,KT)
         ZT1 = XYZTRI(3*N1  ,KT)
         XT2 = XYZTRI(3*N2-2,KT)
         YT2 = XYZTRI(3*N2-1,KT)
         ZT2 = XYZTRI(3*N2  ,KT)
         XT3 = XYZTRI(3*N3-2,KT)
         YT3 = XYZTRI(3*N3-1,KT)
         ZT3 = XYZTRI(3*N3  ,KT)
C
C--- Compute xi, eta, zeta values for point in triangle coords
         DET = ( (XT2-XT1)*(YT3-YT1) - (XT3-XT1)*(YT2-YT1) )
         IF(DET.EQ.0.0) GO TO 1
C
         DETINV = 1.0 / DET
         XI1 = DETINV * ( (X1 -XT1)*(YT3-YT1) - (XT3-XT1)*(Y1 -YT1) )
         ET1 = DETINV * ( (XT2-XT1)*(Y1 -YT1) - (X1 -XT1)*(YT2-YT1) )
C
         IF(ABS(XI1-1.0).LT.EPS) XI1 = 1.0
         IF(ABS(ET1-1.0).LT.EPS) ET1 = 1.0
         ZE1 = (Z1-ZT1) - XI1*(ZT2-ZT1) - ET1*(ZT3-ZT1)
C
C---point is visible if it is outside triangle
         IF(XI1.LE.0. .OR. ET1.LE.0. .OR. (XI1+ET1).GT.1.0) GO TO 1
C---Or above it...
         IF(ZE1.GT.0.) GO TO 1
C
         LVIS=.FALSE.
         RETURN
 1     CONTINUE
C
 100   CONTINUE
       RETURN
       END ! HIDPNT


