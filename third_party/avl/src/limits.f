C***********************************************************************
C    Module:  limits.f
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

      SUBROUTINE GLIMS(XYZMIN,XYZMAX, LPROJ)
C--------------------------------------------------------------
C     Finds geometry limits
C     If LPROJ=T, does projection before finding limits
C--------------------------------------------------------------
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
      REAL XYZMIN(3), XYZMAX(3)
      LOGICAL LPROJ
C
      REAL PTS(3,2)
C
      XYZMIN(1) =  1.0E20
      XYZMIN(2) =  1.0E20
      XYZMIN(3) =  1.0E20
      XYZMAX(1) = -1.0E20
      XYZMAX(2) = -1.0E20
      XYZMAX(3) = -1.0E20
C
      DO N = 1, NSURF
       IF(LPLTSURF(N)) THEN
C
        J1 = JFRST(N)
        JN = JFRST(N) + NJ(N)-1
        JDEL = 1  !  MAX( (JN-J1)/5 , 1)
C
        DO J = J1, JN, JDEL
          PTS(1,1) = RLE1(1,J)
          PTS(2,1) = RLE1(2,J)
          PTS(3,1) = RLE1(3,J)
          PTS(1,2) = RLE1(1,J) + CHORD1(J)
          PTS(2,2) = RLE1(2,J)
          PTS(3,2) = RLE1(3,J)
          IF(LPROJ) CALL VIEWPROJ(PTS,2,PTS)
C
          DO K = 1, 3
            XYZMIN(K) = MIN( XYZMIN(K), PTS(K,1), PTS(K,2) )
            XYZMAX(K) = MAX( XYZMAX(K), PTS(K,1), PTS(K,2) )
          ENDDO
        ENDDO
       ENDIF
      END DO
C
C
      DO N = 1, NBODY
       IF(LPLTBODY(N)) THEN
C
        L1 = LFRST(N)
        LN = LFRST(N) + NL(N)-1
C
        PTS(1,1) = RL(1,L1)
        PTS(2,1) = RL(2,L1)
        PTS(3,1) = RL(3,L1)
        PTS(1,2) = RL(1,LN)
        PTS(2,2) = RL(2,LN)
        PTS(3,2) = RL(3,LN)
        IF(LPROJ) CALL VIEWPROJ(PTS,2,PTS)
C
        DO K = 1, 3
          XYZMIN(K) = MIN( XYZMIN(K), PTS(K,1), PTS(K,2) )
          XYZMAX(K) = MAX( XYZMAX(K), PTS(K,1), PTS(K,2) )
        ENDDO
C
       ENDIF
      END DO
C
      RETURN
      END ! GLIMS



      SUBROUTINE GRLIMS(XYZMIN,XYZMAX, LPROJ, TT,XYZR,DXYZ)
C--------------------------------------------------------------
C     Finds rotated,translated geometry limits
C     If LPROJ=T, does plot projection before finding limits
C--------------------------------------------------------------
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
      REAL XYZMIN(3), XYZMAX(3)
      LOGICAL LPROJ
      REAL TT(3,3), XYZR(3), DXYZ(3)
C
C
      REAL PTS(3,2)
C
      XYZMIN(1) =  1.0E20
      XYZMIN(2) =  1.0E20
      XYZMIN(3) =  1.0E20
      XYZMAX(1) = -1.0E20
      XYZMAX(2) = -1.0E20
      XYZMAX(3) = -1.0E20
C
      DO N = 1, NSURF
       IF(LPLTSURF(N)) THEN
C
        J1 = JFRST(N)
        JN = JFRST(N) + NJ(N)-1
C
        PTS(1,1) = RLE1(1,J1)
        PTS(2,1) = RLE1(2,J1)
        PTS(3,1) = RLE1(3,J1)
        PTS(1,2) = RLE1(1,J1) + CHORD1(J1)
        PTS(2,2) = RLE1(2,J1)
        PTS(3,2) = RLE1(3,J1)
        CALL TETRAN(PTS(1,1),TT,XYZR,DXYZ)
        CALL TETRAN(PTS(1,2),TT,XYZR,DXYZ)
        IF(LPROJ) CALL VIEWPROJ(PTS,2,PTS)
C
        DO K = 1, 3
          XYZMIN(K) = MIN( XYZMIN(K), PTS(K,1), PTS(K,2) )
          XYZMAX(K) = MAX( XYZMAX(K), PTS(K,1), PTS(K,2) )
        ENDDO
C
        PTS(1,1) = RLE2(1,JN)
        PTS(2,1) = RLE2(2,JN)
        PTS(3,1) = RLE2(3,JN)
        PTS(1,2) = RLE2(1,JN) + CHORD2(JN)
        PTS(2,2) = RLE2(2,JN)
        PTS(3,2) = RLE2(3,JN)
        CALL TETRAN(PTS(1,1),TT,XYZR,DXYZ)
        CALL TETRAN(PTS(1,2),TT,XYZR,DXYZ)
        IF(LPROJ) CALL VIEWPROJ(PTS,2,PTS)
C
        DO K = 1, 3
          XYZMIN(K) = MIN( XYZMIN(K), PTS(K,1), PTS(K,2) )
          XYZMAX(K) = MAX( XYZMAX(K), PTS(K,1), PTS(K,2) )
        ENDDO
C
       ENDIF
      END DO
C
C
      DO N = 1, NBODY
       IF(LPLTBODY(N)) THEN
C
        L1 = LFRST(N)
        LN = LFRST(N) + NL(N)-1
C
        PTS(1,1) = RL(1,L1)
        PTS(2,1) = RL(2,L1)
        PTS(3,1) = RL(3,L1)
        PTS(1,2) = RL(1,LN)
        PTS(2,2) = RL(2,LN)
        PTS(3,2) = RL(3,LN)
        CALL TETRAN(PTS(1,1),TT,XYZR,DXYZ)
        CALL TETRAN(PTS(1,2),TT,XYZR,DXYZ)
        IF(LPROJ) CALL VIEWPROJ(PTS,2,PTS)
C
        DO K = 1, 3
          XYZMIN(K) = MIN( XYZMIN(K), PTS(K,1), PTS(K,2) )
          XYZMAX(K) = MAX( XYZMAX(K), PTS(K,1), PTS(K,2) )
        ENDDO
C
       ENDIF
      END DO
C
      RETURN
      END ! GRLIMS



      SUBROUTINE AXLIMS
C---------------------------------------------------------
C     Sets min,max,delta annotations for geometry axes
C---------------------------------------------------------
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
C
      DO K = 1, 3
        AXMIN(K) = MIN( GMIN(K) , 0.0 )
        AXMAX(K) = MAX( GMAX(K) , 0.0 )
      ENDDO
C
      AXDMAX = MAX( AXMAX(1)-AXMIN(1),
     &              AXMAX(2)-AXMIN(2),
     &              AXMAX(3)-AXMIN(3) )
      DO K = 1, 3
        IF(AXMAX(K)-AXMIN(K) .LT. 0.25*AXDMAX) THEN
         AXMIN(K) = MIN( AXMIN(K) , -0.125*AXDMAX )
         AXMAX(K) = MAX( AXMAX(K) ,  0.125*AXDMAX )
        ENDIF
        CALL AXISADJ(AXMIN(K),AXMAX(K),AXSPAN(K),AXDEL(K),NAXANN(K))
      ENDDO
C
      RETURN
      END ! AXLIMS



      SUBROUTINE HIDINIT(LRESET)
C-------------------------------------------------------------
C     Sets up surface triangle data for hidden line routine
C
C     If LRESET=T resets NTRI=0 to start accumulating 
C     a new set of triangles
C-------------------------------------------------------------
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
      LOGICAL LRESET
C
C
      REAL PTS(3,4)
C
      IF(LRESET) NTRI = 0
C
      DO N = 1, NSURF
       IF(LPLTSURF(N)) THEN
C
        J1 = JFRST(N)
        JN = JFRST(N) + NJ(N)-1
        JINC = 1
C
        DO J = J1, JN, JINC
          ID = J
          PTS(1,1) = RLE1(1,J)
          PTS(2,1) = RLE1(2,J)
          PTS(3,1) = RLE1(3,J)
          PTS(1,2) = RLE1(1,J) + CHORD1(J)
          PTS(2,2) = RLE1(2,J)
          PTS(3,2) = RLE1(3,J)
C
          PTS(1,3) = RLE2(1,J)
          PTS(2,3) = RLE2(2,J)
          PTS(3,3) = RLE2(3,J)
          PTS(1,4) = RLE2(1,J) + CHORD2(J)
          PTS(2,4) = RLE2(2,J)
          PTS(3,4) = RLE2(3,J)
C
          CALL VIEWPROJ(PTS,4,PTS)
          CALL TRIINIT(ID,1,1,PTS, NTRI,TRI)
C
        END DO
C
       ENDIF
      END DO
C
C
c      DO N = 1, NBODY
c       IF(LPLTBODY(N)) THEN
cC
c        L1 = LFRST(N)
c        LN = LFRST(N) + NL(N)-1
cC
c        PTS(1,1) = RL(1,L1)
c        PTS(2,1) = RL(2,L1)
c        PTS(3,1) = RL(3,L1)
c        PTS(1,2) = RL(1,LN)
c        PTS(2,2) = RL(2,LN)
c        PTS(3,2) = RL(3,LN)
cC
c        CALL VIEWPROJ(PTS,2,PROJPTS)
cC
c       ENDIF
c      END DO
C
      RETURN
      END ! HIDINIT



      SUBROUTINE HIDINITE(LRESET, ANG,POS,XYZR)
C-------------------------------------------------------------
C     Sets up surface triangle data for hidden line routine
C
C     If LRESET=T resets NTRI=0 to start accumulating 
C     a new set of triangles
C-------------------------------------------------------------
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
      LOGICAL LRESET
      REAL ANG(3), POS(3), XYZR(3)
C
      REAL PTS(3,4)
      REAL TT(3,3), TT_ANG(3,3,3)
C
      IF(LRESET) NTRI = 0
C
      CALL ROTENS3(ANG,TT,TT_ANG)
C
      DO N = 1, NSURF
       IF(LPLTSURF(N)) THEN
C
        J1 = JFRST(N)
        JN = JFRST(N) + NJ(N)-1
        JINC = 1
C
        DO J = J1, JN, JINC
          ID = J
          PTS(1,1) = RLE1(1,J)
          PTS(2,1) = RLE1(2,J)
          PTS(3,1) = RLE1(3,J)
          PTS(1,2) = RLE1(1,J) + CHORD1(J)
          PTS(2,2) = RLE1(2,J)
          PTS(3,2) = RLE1(3,J)
C
          PTS(1,3) = RLE2(1,J)
          PTS(2,3) = RLE2(2,J)
          PTS(3,3) = RLE2(3,J)
          PTS(1,4) = RLE2(1,J) + CHORD2(J)
          PTS(2,4) = RLE2(2,J)
          PTS(3,4) = RLE2(3,J)
C
          CALL TETRAN(PTS(1,1),TT,XYZR,POS)
          CALL TETRAN(PTS(1,2),TT,XYZR,POS)
          CALL TETRAN(PTS(1,3),TT,XYZR,POS)
          CALL TETRAN(PTS(1,4),TT,XYZR,POS)
C
          CALL VIEWPROJ(PTS,4,PTS)
          CALL TRIINIT(ID,1,1,PTS, NTRI,TRI)
C
        END DO
C
       ENDIF
      END DO
C
C
c      DO N = 1, NBODY
c       IF(LPLTBODY(N)) THEN
cC
c        L1 = LFRST(N)
c        LN = LFRST(N) + NL(N)-1
cC
c        PTS(1,1) = RL(1,L1)
c        PTS(2,1) = RL(2,L1)
c        PTS(3,1) = RL(3,L1)
c        PTS(1,2) = RL(1,LN)
c        PTS(2,2) = RL(2,LN)
c        PTS(3,2) = RL(3,LN)
cC
c        CALL VIEWPROJ(PTS,2,PROJPTS)
cC
c       ENDIF
c      END DO
C
      RETURN
      END ! HIDINIT
