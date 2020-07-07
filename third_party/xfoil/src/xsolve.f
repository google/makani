C***********************************************************************
C    Module:  xsolve.f
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


      SUBROUTINE GAUSS(NSIZ,NN,Z,R,NRHS)
C     *******************************************************
C     *                                                     *
C     *   Solves general NxN system in NN unknowns          *
C     *    with arbitrary number (NRHS) of righthand sides. *
C     *   Assumes system is invertible...                   *
C     *    ...if it isn't, a divide by zero will result.    *
C     *                                                     *
C     *   Z is the coefficient matrix...                    *
C     *     ...destroyed during solution process.           *
C     *   R is the righthand side(s)...                     *
C     *     ...replaced by the solution vector(s).          *
C     *                                                     *
C     *                              Mark Drela  1984       *
C     *******************************************************
C
      DIMENSION Z(NSIZ,NSIZ), R(NSIZ,NRHS)
C
      DO 1 NP=1, NN-1
        NP1 = NP+1
C
C------ find max pivot index NX
        NX = NP
        DO 11 N=NP1, NN
          IF(ABS(Z(N,NP))-ABS(Z(NX,NP))) 11,11,111
  111      NX = N
   11   CONTINUE
C
        PIVOT = 1.0/Z(NX,NP)
C
C------ switch pivots
        Z(NX,NP) = Z(NP,NP)
C
C------ switch rows & normalize pivot row
        DO 12 L=NP1, NN
          TEMP = Z(NX,L)*PIVOT
          Z(NX,L) = Z(NP,L)
          Z(NP,L) = TEMP
   12   CONTINUE
C
        DO 13 L=1, NRHS
          TEMP = R(NX,L)*PIVOT
          R(NX,L) = R(NP,L)
          R(NP,L) = TEMP
   13   CONTINUE
C
C------ forward eliminate everything
        DO 15 K=NP1, NN
          ZTMP = Z(K,NP)
C
C          IF(ZTMP.EQ.0.0) GO TO 15
C
          DO 151 L=NP1, NN
            Z(K,L) = Z(K,L) - ZTMP*Z(NP,L)
  151     CONTINUE
          DO 152 L=1, NRHS
            R(K,L) = R(K,L) - ZTMP*R(NP,L)
  152     CONTINUE
   15   CONTINUE
C
    1 CONTINUE
C
C---- solve for last row
      DO 2 L=1, NRHS
        R(NN,L) = R(NN,L)/Z(NN,NN)
    2 CONTINUE
C
C---- back substitute everything
      DO 3 NP=NN-1, 1, -1
        NP1 = NP+1
        DO 31 L=1, NRHS
          DO 310 K=NP1, NN
            R(NP,L) = R(NP,L) - Z(NP,K)*R(K,L)
  310     CONTINUE
   31   CONTINUE
    3 CONTINUE
C
      RETURN
      END ! GAUSS


      SUBROUTINE CGAUSS(NSIZ,NN,Z,R,NRHS)
C********************************************
C     Solves general complex linear systems.
C********************************************
      COMPLEX Z(NSIZ,NSIZ), R(NSIZ,NRHS)
      COMPLEX PIVOT, TEMP, ZTMP
C
      DO 1 NP=1, NN-1
        NP1 = NP+1
C
C------ find max pivot index NX
        NX = NP
        DO 11 N=NP1, NN
          IF(ABS(Z(N,NP))-ABS(Z(NX,NP))) 11,11,111
  111      NX = N
   11   CONTINUE
C
        PIVOT = (1.0,0.0)/Z(NX,NP)
C
C------ switch pivots
        Z(NX,NP) = Z(NP,NP)
C
C------ switch rows & normalize pivot row
        DO 12 L=NP1, NN
          TEMP = Z(NX,L)*PIVOT
          Z(NX,L) = Z(NP,L)
          Z(NP,L) = TEMP
   12   CONTINUE
C
        DO 13 L=1, NRHS
          TEMP = R(NX,L)*PIVOT
          R(NX,L) = R(NP,L)
          R(NP,L) = TEMP
   13   CONTINUE
C
C------ forward eliminate everything
        DO 15 K=NP1, NN
          ZTMP = Z(K,NP)
C
C          IF(ZTMP.EQ.0.0) GO TO 15
C
          DO 151 L=NP1, NN
            Z(K,L) = Z(K,L) - ZTMP*Z(NP,L)
  151     CONTINUE
          DO 152 L=1, NRHS
            R(K,L) = R(K,L) - ZTMP*R(NP,L)
  152     CONTINUE
   15   CONTINUE
C
    1 CONTINUE
C
C---- solve for last row
      DO 2 L=1, NRHS
        R(NN,L) = R(NN,L)/Z(NN,NN)
    2 CONTINUE
C
C---- back substitute everything
      DO 3 NP=NN-1, 1, -1
        NP1 = NP+1
        DO 31 L=1, NRHS
          DO 310 K=NP1, NN
            R(NP,L) = R(NP,L) - Z(NP,K)*R(K,L)
  310     CONTINUE
   31   CONTINUE
    3 CONTINUE
C
      RETURN
      END ! CGAUSS



      SUBROUTINE LUDCMP(NSIZ,N,A,INDX)
C     *******************************************************
C     *                                                     *
C     *   Factors a full NxN matrix into an LU form.        *
C     *   Subr. BAKSUB can back-substitute it with some RHS.*
C     *   Assumes matrix is non-singular...                 *
C     *    ...if it isn't, a divide by zero will result.    *
C     *                                                     *
C     *   A is the matrix...                                *
C     *     ...replaced with its LU factors.                *
C     *                                                     *
C     *                              Mark Drela  1988       *
C     *******************************************************
C
      DIMENSION A(NSIZ,NSIZ), INDX(NSIZ)
C
      PARAMETER (NVX=500)
      DIMENSION VV(NVX)
C
      IF(N.GT.NVX) STOP 'LUDCMP: Array overflow. Increase NVX.'
C
      DO 12 I=1, N
        AAMAX = 0.
        DO 11 J=1, N
          AAMAX = MAX( ABS(A(I,J)) , AAMAX )
   11   CONTINUE
        VV(I) = 1.0/AAMAX
   12 CONTINUE
C
      DO 19 J=1, N
        DO 14 I=1, J-1
          SUM = A(I,J)
          DO 13 K=1, I-1
            SUM = SUM - A(I,K)*A(K,J)
   13     CONTINUE
          A(I,J) = SUM
   14   CONTINUE
C
        AAMAX = 0.
        DO 16 I=J, N
          SUM = A(I,J)
          DO 15 K=1, J-1
            SUM = SUM - A(I,K)*A(K,J)
   15     CONTINUE
          A(I,J) = SUM
C
          DUM = VV(I)*ABS(SUM)
          IF(DUM.GE.AAMAX) THEN
           IMAX = I
           AAMAX = DUM
          ENDIF
   16   CONTINUE
C
        IF(J.NE.IMAX) THEN
         DO 17 K=1, N
           DUM = A(IMAX,K)
           A(IMAX,K) = A(J,K)
           A(J,K) = DUM
   17    CONTINUE
         VV(IMAX) = VV(J)
        ENDIF
C
        INDX(J) = IMAX
        IF(J.NE.N) THEN
         DUM = 1.0/A(J,J)
         DO 18 I=J+1, N
           A(I,J) = A(I,J)*DUM
   18    CONTINUE
        ENDIF
C
   19 CONTINUE
C
      RETURN
      END ! LUDCMP


      SUBROUTINE BAKSUB(NSIZ,N,A,INDX,B)
      DIMENSION A(NSIZ,NSIZ), B(NSIZ), INDX(NSIZ)
C
      II = 0
      DO 12 I=1, N
        LL = INDX(I)
        SUM = B(LL)
        B(LL) = B(I)
        IF(II.NE.0) THEN
         DO 11 J=II, I-1
           SUM = SUM - A(I,J)*B(J)
   11    CONTINUE
        ELSE IF(SUM.NE.0.0) THEN
         II = I
        ENDIF
        B(I) = SUM
   12 CONTINUE
C
      DO 14 I=N, 1, -1
        SUM = B(I)
        IF(I.LT.N) THEN
         DO 13 J=I+1, N
           SUM = SUM - A(I,J)*B(J)
   13    CONTINUE
        ENDIF
        B(I) = SUM/A(I,I)
   14 CONTINUE
C
      RETURN
      END ! BAKSUB



      SUBROUTINE BLSOLV
C-----------------------------------------------------------------
C      Custom solver for coupled viscous-inviscid Newton system:
C
C        A  |  |  .  |  |  .  |    d       R       S
C        B  A  |  .  |  |  .  |    d       R       S
C        |  B  A  .  |  |  .  |    d       R       S
C        .  .  .  .  |  |  .  |    d   =   R - dRe S
C        |  |  |  B  A  |  .  |    d       R       S
C        |  Z  |  |  B  A  .  |    d       R       S
C        .  .  .  .  .  .  .  |    d       R       S
C        |  |  |  |  |  |  B  A    d       R       S
C
C       A, B, Z  3x3  blocks containing linearized BL equation coefficients
C       |        3x1  vectors containing mass defect influence 
C                     coefficients on Ue
C       d        3x1  unknown vectors (Newton deltas for Ctau, Theta, m)
C       R        3x1  residual vectors
C       S        3x1  Re influence vectors
C-----------------------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
      IVTE1 = ISYS(IBLTE(1),1)
C
      VACC1 = VACCEL
      VACC2 = VACCEL * 2.0 / (S(N) - S(1))
      VACC3 = VACCEL * 2.0 / (S(N) - S(1))
C
      DO 1000 IV=1, NSYS
C
        IVP = IV + 1
C
C====== Invert VA(IV) block
C
C------ normalize first row
        PIVOT = 1.0 / VA(1,1,IV)
        VA(1,2,IV) = VA(1,2,IV) * PIVOT
        DO 10 L=IV, NSYS
          VM(1,L,IV) = VM(1,L,IV)*PIVOT
   10   CONTINUE
        VDEL(1,1,IV) = VDEL(1,1,IV)*PIVOT
        VDEL(1,2,IV) = VDEL(1,2,IV)*PIVOT
C
C------ eliminate lower first column in VA block
        DO 15 K=2, 3
          VTMP = VA(K,1,IV)
          VA(K,2,IV) = VA(K,2,IV) - VTMP*VA(1,2,IV)
          DO 150 L=IV, NSYS
            VM(K,L,IV) = VM(K,L,IV) - VTMP*VM(1,L,IV)
  150     CONTINUE
          VDEL(K,1,IV) = VDEL(K,1,IV) - VTMP*VDEL(1,1,IV)
          VDEL(K,2,IV) = VDEL(K,2,IV) - VTMP*VDEL(1,2,IV)
   15   CONTINUE
C
C
C------ normalize second row
        PIVOT = 1.0 / VA(2,2,IV)
        DO 20 L=IV, NSYS
          VM(2,L,IV) = VM(2,L,IV)*PIVOT
   20   CONTINUE
        VDEL(2,1,IV) = VDEL(2,1,IV)*PIVOT
        VDEL(2,2,IV) = VDEL(2,2,IV)*PIVOT
C
C------ eliminate lower second column in VA block
        K = 3
        VTMP = VA(K,2,IV)
        DO 250 L=IV, NSYS
          VM(K,L,IV) = VM(K,L,IV) - VTMP*VM(2,L,IV)
  250   CONTINUE
        VDEL(K,1,IV) = VDEL(K,1,IV) - VTMP*VDEL(2,1,IV)
        VDEL(K,2,IV) = VDEL(K,2,IV) - VTMP*VDEL(2,2,IV)
C
C
C------ normalize third row
        PIVOT = 1.0/VM(3,IV,IV)
        DO 350 L=IVP, NSYS
          VM(3,L,IV) = VM(3,L,IV)*PIVOT
  350   CONTINUE
        VDEL(3,1,IV) = VDEL(3,1,IV)*PIVOT
        VDEL(3,2,IV) = VDEL(3,2,IV)*PIVOT
C
C
C------ eliminate upper third column in VA block
        VTMP1 = VM(1,IV,IV)
        VTMP2 = VM(2,IV,IV)
        DO 450 L=IVP, NSYS
          VM(1,L,IV) = VM(1,L,IV) - VTMP1*VM(3,L,IV)
          VM(2,L,IV) = VM(2,L,IV) - VTMP2*VM(3,L,IV)
  450   CONTINUE
        VDEL(1,1,IV) = VDEL(1,1,IV) - VTMP1*VDEL(3,1,IV)
        VDEL(2,1,IV) = VDEL(2,1,IV) - VTMP2*VDEL(3,1,IV)
        VDEL(1,2,IV) = VDEL(1,2,IV) - VTMP1*VDEL(3,2,IV)
        VDEL(2,2,IV) = VDEL(2,2,IV) - VTMP2*VDEL(3,2,IV)
C
C------ eliminate upper second column in VA block
        VTMP = VA(1,2,IV)
        DO 460 L=IVP, NSYS
          VM(1,L,IV) = VM(1,L,IV) - VTMP*VM(2,L,IV)
  460   CONTINUE
        VDEL(1,1,IV) = VDEL(1,1,IV) - VTMP*VDEL(2,1,IV)
        VDEL(1,2,IV) = VDEL(1,2,IV) - VTMP*VDEL(2,2,IV)
C
C
        IF(IV.EQ.NSYS) GO TO 1000
C
C====== Eliminate VB(IV+1) block, rows  1 -> 3
        DO 50 K=1, 3
          VTMP1 = VB(K, 1,IVP)
          VTMP2 = VB(K, 2,IVP)
          VTMP3 = VM(K,IV,IVP)
          DO 510 L=IVP, NSYS
            VM(K,L,IVP) = VM(K,L,IVP)
     &        - (  VTMP1*VM(1,L,IV)
     &           + VTMP2*VM(2,L,IV)
     &           + VTMP3*VM(3,L,IV) )
  510     CONTINUE
          VDEL(K,1,IVP) = VDEL(K,1,IVP)
     &        - (  VTMP1*VDEL(1,1,IV)
     &           + VTMP2*VDEL(2,1,IV)
     &           + VTMP3*VDEL(3,1,IV) )
          VDEL(K,2,IVP) = VDEL(K,2,IVP)
     &        - (  VTMP1*VDEL(1,2,IV)
     &           + VTMP2*VDEL(2,2,IV)
     &           + VTMP3*VDEL(3,2,IV) )
   50   CONTINUE
C
        IF(IV.EQ.IVTE1) THEN
C------- eliminate VZ block
         IVZ = ISYS(IBLTE(2)+1,2)
C
         DO 55 K=1, 3
           VTMP1 = VZ(K,1)
           VTMP2 = VZ(K,2)
           DO 515 L=IVP, NSYS
             VM(K,L,IVZ) = VM(K,L,IVZ)
     &         - (  VTMP1*VM(1,L,IV)
     &            + VTMP2*VM(2,L,IV) )
  515      CONTINUE
           VDEL(K,1,IVZ) = VDEL(K,1,IVZ)
     &         - (  VTMP1*VDEL(1,1,IV)
     &            + VTMP2*VDEL(2,1,IV) )
           VDEL(K,2,IVZ) = VDEL(K,2,IVZ)
     &         - (  VTMP1*VDEL(1,2,IV)
     &            + VTMP2*VDEL(2,2,IV) )
   55    CONTINUE
        ENDIF
C
        IF(IVP.EQ.NSYS) GO TO 1000
C
C====== Eliminate lower VM column
        DO 60 KV=IV+2, NSYS
          VTMP1 = VM(1,IV,KV)
          VTMP2 = VM(2,IV,KV)
          VTMP3 = VM(3,IV,KV)
C
          IF(ABS(VTMP1).GT.VACC1) THEN
          DO 610 L=IVP, NSYS
            VM(1,L,KV) = VM(1,L,KV) - VTMP1*VM(3,L,IV)
  610     CONTINUE
          VDEL(1,1,KV) = VDEL(1,1,KV) - VTMP1*VDEL(3,1,IV)
          VDEL(1,2,KV) = VDEL(1,2,KV) - VTMP1*VDEL(3,2,IV)
          ENDIF
C
          IF(ABS(VTMP2).GT.VACC2) THEN
          DO 620 L=IVP, NSYS
            VM(2,L,KV) = VM(2,L,KV) - VTMP2*VM(3,L,IV)
  620     CONTINUE
          VDEL(2,1,KV) = VDEL(2,1,KV) - VTMP2*VDEL(3,1,IV)
          VDEL(2,2,KV) = VDEL(2,2,KV) - VTMP2*VDEL(3,2,IV)
          ENDIF
C
          IF(ABS(VTMP3).GT.VACC3) THEN
          DO 630 L=IVP, NSYS
            VM(3,L,KV) = VM(3,L,KV) - VTMP3*VM(3,L,IV)
  630     CONTINUE
          VDEL(3,1,KV) = VDEL(3,1,KV) - VTMP3*VDEL(3,1,IV)
          VDEL(3,2,KV) = VDEL(3,2,KV) - VTMP3*VDEL(3,2,IV)
          ENDIF
C
   60   CONTINUE
C
 1000 CONTINUE
C
C
C
      DO 2000 IV=NSYS, 2, -1
C
C------ eliminate upper VM columns
        VTMP = VDEL(3,1,IV)
        DO 81 KV=IV-1, 1, -1
          VDEL(1,1,KV) = VDEL(1,1,KV) - VM(1,IV,KV)*VTMP
          VDEL(2,1,KV) = VDEL(2,1,KV) - VM(2,IV,KV)*VTMP
          VDEL(3,1,KV) = VDEL(3,1,KV) - VM(3,IV,KV)*VTMP
   81   CONTINUE
C
        VTMP = VDEL(3,2,IV)
        DO 82 KV=IV-1, 1, -1
          VDEL(1,2,KV) = VDEL(1,2,KV) - VM(1,IV,KV)*VTMP
          VDEL(2,2,KV) = VDEL(2,2,KV) - VM(2,IV,KV)*VTMP
          VDEL(3,2,KV) = VDEL(3,2,KV) - VM(3,IV,KV)*VTMP
   82   CONTINUE
C
 2000 CONTINUE
C
      RETURN
      END
