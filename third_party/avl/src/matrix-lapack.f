
      SUBROUTINE LUDCMP(NSIZ,N,A,INDX,WORK)
C     *******************************************************
C     *   Factors a full NxN matrix into an LU form.        *
C     *   Uses LINPACK routines for linear algebra          *
C     *******************************************************
C
      REAL A(NSIZ,NSIZ), WORK(NSIZ)
      INTEGER INDX(NSIZ)
C
      CALL SGETRF(N,NSIZ,A,NSIZ,INDX,INFO)
C
      RETURN
      END ! LUDCMP


      SUBROUTINE BAKSUB(NSIZ,N,A,INDX,B)
      DIMENSION A(NSIZ,NSIZ), B(NSIZ), INDX(NSIZ)
C     *******************************************************
C     *   BAKSUB does back-substitution with RHS using      *
C     *   stored LU decomposition.                          *
C     *   Uses LINPACK routines for linear algebra          *
C     *******************************************************
C
      MRHS = 1
      CALL SGETRS('N',N,MRHS,A,NSIZ,INDX,B,NSIZ,INFO)
C
      RETURN
      END ! BAKSUB



