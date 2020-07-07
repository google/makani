C***********************************************************************
C    Module:  matrix.f
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

      SUBROUTINE LUDCMP(NSIZ,N,A,INDX,WORK)
C     *******************************************************
C     *   Factors a full NxN matrix into an LU form.        *
C     *   Subr. BAKSUB can back-substitute it with some RHS.*
C     *   Assumes matrix is non-singular...                 *
C     *    ...if it isn't, a divide by zero will result.    *
C     *                                                     *
C     *   A is the matrix...                                *
C     *     ...replaced with its LU factors.                *
C     *                                                     *
C     *   Stolen from Numerical Recipes.                    *
C     *******************************************************
C
      REAL A(NSIZ,NSIZ), WORK(NSIZ)
      INTEGER INDX(NSIZ)
C
      DO 12 I=1, N
        AAMAX = 0.
        DO 11 J=1, N
          AAMAX = MAX( ABS(A(I,J)) , AAMAX )
   11   CONTINUE
        WORK(I) = 1.0/AAMAX
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
          DUM = WORK(I)*ABS(SUM)
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
         WORK(IMAX) = WORK(J)
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
      REAL A(NSIZ,NSIZ), B(NSIZ)
      INTEGER INDX(NSIZ)
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

