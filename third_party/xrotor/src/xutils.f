C***********************************************************************
C    Module:  xutils.f
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

      SUBROUTINE SETEXP(S,SMAX,NN)
C........................................................
C     Sets geometrically stretched array S:
C
C       S(i+1) - S(i)  =  r * [S(i) - S(i-1)]   ;  r = geometric ratio
C
C       S(1),S(2)  (input)   give first S increment:  S(2) - S(1)
C       S(I)       (output)  array to be set for I > 2
C       SMAX       (input)   final S value:      S(NN)
C       NN         (input)   number of points
C........................................................
      REAL S(NN)
C
      DS1 = S(2) - S(1)
      SIGMA = (SMAX-S(1))/DS1
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
      DISC = AMAX1( 0.0 , DISC )
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
   11 DS = DS1
      DO 2 N=2, NN
        S(N) = S(N-1) + DS
        DS = DS*RATIO
    2 CONTINUE
C
      RETURN
      END ! SETEXP



      SUBROUTINE GAUSS(NSIZ,NN,Z,R,NRHS)
C     *******************************************************
C     *                                                     *
C     *   Solves general NxN system in N unknowns           *
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
      REAL VV(250)
C
      DO 12 I=1, N
        AAMAX = 0.
        DO 11 J=1, N
          AAMAX = AMAX1( ABS(A(I,J)) , AAMAX )
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

